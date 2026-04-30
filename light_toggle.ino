// Waveshare ESP32-S3-Touch-LCD-1.69 — Nova "everything" light toggle.
//
// What it does:
//   - Connects to WiFi (see secrets.h).
//   - Subscribes to nova.local /api/events SSE stream and renders the
//     "everything" dashboard zone full-screen with LVGL.
//   - Tap anywhere to POST /api/zone with an on/off action. Optimistic update
//     is applied immediately; the SSE push corrects within ~100 ms.
//   - When isOn=false the backlight is dropped to zero via LEDC, but the
//     CST816 touch input remains polled so a tap still toggles back on.
//
// Cores:
//   - core 0: NetTask (WiFi + SSE reader + toggle POSTs)
//   - core 1: Arduino loop() runs LVGL + touch polling
//
// Pinout, board target, and other hardware notes live in the project memory
// file `project_waveshare_esp32s3_touch_lcd_169.md` — never drive GPIO 0/3/45/46.

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "secrets.h"

// ---------- Pins (from Waveshare BSP) -----------------------------------------
#define LCD_SCK   6
#define LCD_MOSI  7
#define LCD_CS    5
#define LCD_DC    4
#define LCD_RST   8
#define LCD_BL   15

#define I2C_SDA  11
#define I2C_SCL  10
#define TP_RST   13
#define TP_INT   14
#define CST816_ADDR 0x15

// ---------- Display -----------------------------------------------------------
static const uint16_t SCREEN_W = 240;
static const uint16_t SCREEN_H = 280;

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, /*MISO*/ -1, FSPI);

Arduino_GFX *gfx = new Arduino_ST7789(
  bus, LCD_RST, /*rotation*/ 0, /*IPS*/ true,
  SCREEN_W, SCREEN_H, 0, 20, 0, 0);

// ---------- LVGL --------------------------------------------------------------
// 1/8-screen draw buffer in PSRAM is plenty for solid fills + a label.
static const uint32_t LV_BUF_PIXELS = SCREEN_W * (SCREEN_H / 8);
static lv_color_t *lv_buf1 = nullptr;
static lv_color_t *lv_buf2 = nullptr;
static lv_display_t *lv_disp = nullptr;
static lv_indev_t *lv_indev = nullptr;

static lv_obj_t *bg_obj = nullptr;
static lv_obj_t *status_label = nullptr;
static lv_obj_t *wifi_label = nullptr;

// ---------- Shared state (cross-core) -----------------------------------------
struct LightState {
  bool   have_data   = false;
  bool   is_on       = false;
  int    brightness  = 0;          // 0..100
  uint8_t rgb[3]     = {255, 147, 41};
};

static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static LightState  g_state;
static volatile bool g_state_dirty = true;
static volatile bool g_wifi_connected = false;
static volatile bool g_sse_connected  = false;
static volatile bool g_toggle_request = false;
static const uint32_t TOUCH_HOLD_MS = 500;
static uint32_t g_touch_press_started_ms = 0;
static bool g_touch_action_fired = false;

static void publishState(const LightState &s) {
  portENTER_CRITICAL(&state_mux);
  g_state = s;
  g_state_dirty = true;
  portEXIT_CRITICAL(&state_mux);
}

static LightState snapshotState() {
  LightState s;
  portENTER_CRITICAL(&state_mux);
  s = g_state;
  portEXIT_CRITICAL(&state_mux);
  return s;
}

// ---------- Backlight ---------------------------------------------------------
static const uint32_t BL_FREQ_HZ = 5000;
static const uint8_t  BL_RES_BITS = 10;          // duty 0..1023

static void backlightInit() {
  ledcAttach(LCD_BL, BL_FREQ_HZ, BL_RES_BITS);
  ledcWrite(LCD_BL, 0);
}

static void backlightSet(uint8_t pct) {
  if (pct > 100) pct = 100;
  uint32_t duty = (1023u * pct) / 100u;
  ledcWrite(LCD_BL, duty);
}

// ---------- LVGL display flush ------------------------------------------------
static void lv_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  // LVGL 9 with LV_COLOR_DEPTH=16 hands us native RGB565 little-endian; the
  // ST7789 wants big-endian bytes, so swap before pushing.
  uint16_t *px = (uint16_t *)px_map;
  for (uint32_t i = 0; i < w * h; i++) {
    uint16_t v = px[i];
    px[i] = (v >> 8) | (v << 8);
  }
  gfx->draw16bitRGBBitmap(area->x1, area->y1, px, w, h);
  lv_display_flush_ready(disp);
}

// ---------- CST816 touch driver -----------------------------------------------
// Note: returning a struct from an .ino function trips arduino-cli's prototype
// generator (it forward-declares before the struct exists), so use out params.
static bool cst816Read(int16_t &x, int16_t &y) {
  Wire.beginTransmission(CST816_ADDR);
  Wire.write(0x01);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)CST816_ADDR, (uint8_t)6) != 6) return false;
  uint8_t buf[6];
  for (int i = 0; i < 6; i++) buf[i] = Wire.read();
  uint8_t points = buf[1] & 0x0F;
  if (points == 0) return false;
  x = ((uint16_t)(buf[2] & 0x0F) << 8) | buf[3];
  y = ((uint16_t)(buf[4] & 0x0F) << 8) | buf[5];
  return true;
}

static void lv_indev_read_cb(lv_indev_t *, lv_indev_data_t *data) {
  int16_t x, y;
  if (cst816Read(x, y)) {
    data->point.x = constrain((int)x, 0, SCREEN_W - 1);
    data->point.y = constrain((int)y, 0, SCREEN_H - 1);
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// ---------- LVGL tick from millis ---------------------------------------------
static uint32_t lv_tick_millis() { return millis(); }

// ---------- UI ----------------------------------------------------------------
static lv_obj_t *createTapTarget() {
  lv_obj_t *t = lv_obj_create(lv_screen_active());
  lv_obj_remove_style_all(t);
  lv_obj_set_size(t, SCREEN_W, SCREEN_H);
  lv_obj_set_pos(t, 0, 0);
  // Catch presses but let the bg color show through.
  lv_obj_add_flag(t, LV_OBJ_FLAG_CLICKABLE);
  return t;
}

static void triggerToggle() {
  // Optimistic flip + nudge net task to fire the HTTP toggle.
  LightState s = snapshotState();
  s.is_on = !s.is_on;
  if (!s.is_on) {
    s.brightness = 0;
  } else if (s.brightness == 0) {
    s.brightness = 86;            // dashboard's default candlelight pct
  }
  s.have_data = true;             // make sure first-tap shows feedback
  publishState(s);
  g_toggle_request = true;
}

static void onTouchEvent(lv_event_t *event) {
  lv_event_code_t code = lv_event_get_code(event);
  if (code == LV_EVENT_PRESSED) {
    g_touch_press_started_ms = millis();
    g_touch_action_fired = false;
    return;
  }

  if (code == LV_EVENT_PRESSING &&
      !g_touch_action_fired &&
      g_touch_press_started_ms != 0 &&
      millis() - g_touch_press_started_ms >= TOUCH_HOLD_MS) {
    g_touch_action_fired = true;
    triggerToggle();
    return;
  }

  if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
    g_touch_press_started_ms = 0;
    g_touch_action_fired = false;
  }
}

static void buildUi() {
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  bg_obj = lv_obj_create(scr);
  lv_obj_remove_style_all(bg_obj);
  lv_obj_set_size(bg_obj, SCREEN_W, SCREEN_H);
  lv_obj_set_pos(bg_obj, 0, 0);
  lv_obj_set_style_bg_color(bg_obj, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(bg_obj, LV_OPA_COVER, 0);
  lv_obj_clear_flag(bg_obj, LV_OBJ_FLAG_SCROLLABLE);

  status_label = lv_label_create(scr);
  lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(status_label, &lv_font_montserrat_24, 0);
  lv_label_set_text(status_label, "...");
  lv_obj_align(status_label, LV_ALIGN_CENTER, 0, 0);

  wifi_label = lv_label_create(scr);
  lv_obj_set_style_text_color(wifi_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(wifi_label, &lv_font_montserrat_14, 0);
  lv_label_set_text(wifi_label, "WiFi: ...");
  lv_obj_align(wifi_label, LV_ALIGN_TOP_MID, 0, 6);

  lv_obj_t *tap = createTapTarget();
  lv_obj_add_event_cb(tap, onTouchEvent, LV_EVENT_PRESSED, nullptr);
  lv_obj_add_event_cb(tap, onTouchEvent, LV_EVENT_PRESSING, nullptr);
  lv_obj_add_event_cb(tap, onTouchEvent, LV_EVENT_RELEASED, nullptr);
  lv_obj_add_event_cb(tap, onTouchEvent, LV_EVENT_PRESS_LOST, nullptr);
}

// Render the latest shared state into the LVGL widgets.
static void renderState() {
  LightState s = snapshotState();

  if (!s.have_data || !s.is_on) {
    lv_obj_set_style_bg_color(bg_obj, lv_color_black(), 0);
    lv_label_set_text(status_label, s.have_data ? "OFF" : "...");
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    backlightSet(s.have_data ? 0 : 50);   // keep BL on while waiting for first event
    return;
  }

  // ON: scale dominant RGB by brightnessPct so dim lights look dim on-watch.
  float k = (float)s.brightness / 100.0f;
  if (k < 0.05f) k = 0.05f;               // floor so we don't go fully black
  uint8_t r = (uint8_t)(s.rgb[0] * k);
  uint8_t g = (uint8_t)(s.rgb[1] * k);
  uint8_t b = (uint8_t)(s.rgb[2] * k);
  lv_color_t c = lv_color_make(r, g, b);
  lv_obj_set_style_bg_color(bg_obj, c, 0);

  // Pick contrasting label color
  uint16_t luma = (r * 299 + g * 587 + b * 114) / 1000;
  lv_color_t label_c = (luma > 128) ? lv_color_black() : lv_color_white();
  lv_obj_set_style_text_color(status_label, label_c, 0);
  char buf[24];
  snprintf(buf, sizeof(buf), "ON  %d%%", s.brightness);
  lv_label_set_text(status_label, buf);

  backlightSet(100);
}

static void updateWifiLabel() {
  const char *t = "WiFi: down";
  if (g_wifi_connected && g_sse_connected)      t = "Nova: connected";
  else if (g_wifi_connected)                    t = "WiFi: ok";
  lv_label_set_text(wifi_label, t);
}

// ---------- Network task (core 0) --------------------------------------------
//
// Minimal SSE client: maintain a TCP connection to /api/events, parse
// `event: state` / `data: {...}` lines, update shared state. Reconnect with
// backoff if the stream dies.
//
// The dashboard currently emits full DashboardState JSON. We only extract the
// `everything` zone and the first RGB-ish color in that zone.
static const size_t MAX_SSE_LINE = 49152;

static bool jsonExtractBoolFrom(const String &json, int from, const char *key, bool &out) {
  if (from < 0) from = 0;
  String pat = String("\"") + key + "\":";
  int idx = json.indexOf(pat, from);
  if (idx < 0) return false;
  idx += pat.length();
  while (idx < (int)json.length() && (json[idx] == ' ' || json[idx] == '\t')) idx++;
  if (json.startsWith("true", idx))  { out = true;  return true; }
  if (json.startsWith("false", idx)) { out = false; return true; }
  return false;
}

static bool jsonExtractIntFrom(const String &json, int from, const char *key, int &out) {
  if (from < 0) from = 0;
  String pat = String("\"") + key + "\":";
  int idx = json.indexOf(pat, from);
  if (idx < 0) return false;
  idx += pat.length();
  while (idx < (int)json.length() && (json[idx] == ' ' || json[idx] == '\t')) idx++;
  int start = idx;
  if (idx < (int)json.length() && (json[idx] == '-' || json[idx] == '+')) idx++;
  while (idx < (int)json.length() && isDigit(json[idx])) idx++;
  if (idx == start) return false;
  out = json.substring(start, idx).toInt();
  return true;
}

static bool jsonExtractRgbArrayFrom(const String &json, int from, const char *key, uint8_t out[3]) {
  if (from < 0) from = 0;
  String pat = String("\"") + key + "\":[";
  int idx = json.indexOf(pat, from);
  if (idx < 0) return false;
  idx += pat.length();
  for (int ch = 0; ch < 3; ch++) {
    while (idx < (int)json.length() && !isDigit(json[idx]) && json[idx] != '-') idx++;
    int start = idx;
    while (idx < (int)json.length() && isDigit(json[idx])) idx++;
    if (idx == start) return false;
    int v = json.substring(start, idx).toInt();
    out[ch] = (uint8_t)constrain(v, 0, 255);
  }
  return true;
}

static void applyCompactHomePayload(const String &payload) {
  if (payload.indexOf("\"event\":\"home\"") < 0) return;
  LightState s = snapshotState();
  bool b; int i; uint8_t rgb[3];
  if (jsonExtractBoolFrom(payload, 0, "isOn", b))         s.is_on = b;
  if (jsonExtractIntFrom(payload, 0, "brightnessPct", i)) s.brightness = constrain(i, 0, 100);
  if (jsonExtractRgbArrayFrom(payload, 0, "rgb", rgb))    memcpy(s.rgb, rgb, 3);
  s.have_data = true;
  publishState(s);
}

static void applyDashboardStatePayload(const String &payload) {
  int zoneIdx = payload.indexOf("\"id\":\"everything\"");
  if (zoneIdx < 0) return;

  LightState s = snapshotState();
  bool changed = false;
  bool b; int i; uint8_t rgb[3];

  if (jsonExtractBoolFrom(payload, zoneIdx, "isOn", b)) {
    s.is_on = b;
    changed = true;
  }
  if (jsonExtractIntFrom(payload, zoneIdx, "brightnessPct", i)) {
    s.brightness = constrain(i, 0, 100);
    changed = true;
  }
  if (jsonExtractRgbArrayFrom(payload, zoneIdx, "rgb_color", rgb) ||
      jsonExtractRgbArrayFrom(payload, zoneIdx, "rgb", rgb)) {
    memcpy(s.rgb, rgb, 3);
    changed = true;
  }

  if (!changed) return;
  s.have_data = true;
  publishState(s);
}

static void applySseDataLine(const String &payload, const String &eventName) {
  if (eventName == "state") {
    applyDashboardStatePayload(payload);
    return;
  }

  // Backward-compatible with the compact event shape from the first firmware.
  applyCompactHomePayload(payload);
}

static void doToggle() {
  LightState s = snapshotState();
  const char *action = s.is_on ? "on" : "off";
  String body = String("{\"zoneId\":\"everything\",\"action\":\"") + action + "\"";
  if (s.is_on) {
    body += String(",\"brightnessPct\":") + s.brightness;
  }
  body += "}";

  HTTPClient http;
  String url = String("http://") + NOVA_HOST + ":" + NOVA_PORT + "/api/zone";
  if (!http.begin(url)) return;
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(2000);
  int code = http.POST(body);
  Serial.printf("toggle POST -> %d\n", code);
  http.end();
}

static void sseTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t backoff_ms = 1000;
  for (;;) {
    if (g_toggle_request) {
      g_toggle_request = false;
      doToggle();
    }

    if (WiFi.status() != WL_CONNECTED) {
      g_wifi_connected = false;
      g_sse_connected  = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    g_wifi_connected = true;

    WiFiClient client;
    if (!client.connect(NOVA_HOST, NOVA_PORT)) {
      Serial.printf("SSE connect failed; retry in %lums\n", (unsigned long)backoff_ms);
      vTaskDelay(pdMS_TO_TICKS(backoff_ms));
      backoff_ms = min<uint32_t>(backoff_ms * 2, 30000);
      continue;
    }

    client.printf(
      "GET /api/events HTTP/1.1\r\n"
      "Host: %s\r\n"
      "Accept: text/event-stream\r\n"
      "Cache-Control: no-cache\r\n"
      "Connection: keep-alive\r\n"
      "\r\n", NOVA_HOST);

    // Skip headers
    bool ok = false;
    String first = client.readStringUntil('\n');
    if (first.startsWith("HTTP/1.1 200")) ok = true;
    while (client.connected()) {
      String h = client.readStringUntil('\n');
      if (h == "\r" || h.length() <= 1) break;
    }

    if (!ok) {
      Serial.println("SSE non-200; backing off");
      client.stop();
      vTaskDelay(pdMS_TO_TICKS(backoff_ms));
      backoff_ms = min<uint32_t>(backoff_ms * 2, 30000);
      continue;
    }

    g_sse_connected = true;
    backoff_ms = 1000;
    Serial.println("SSE connected");

    String eventName;
    eventName.reserve(32);
    String line;
    line.reserve(MAX_SSE_LINE);
    uint32_t lastByte = millis();

    while (client.connected() && WiFi.status() == WL_CONNECTED) {
      // Service toggles even while inside the SSE read loop.
      if (g_toggle_request) {
        g_toggle_request = false;
        doToggle();
      }

      while (client.available()) {
        char c = (char)client.read();
        lastByte = millis();
        if (c == '\r') continue;
        if (c == '\n') {
          if (line.startsWith("event:")) {
            eventName = line.substring(6);
            eventName.trim();
          } else if (line.startsWith("data:")) {
            line.remove(0, 5);
            line.trim();
            if (line.length()) applySseDataLine(line, eventName);
          } else if (line.length() == 0) {
            eventName = "";
          }
          line = "";
        } else {
          if (line.length() < MAX_SSE_LINE) line += c;
        }
      }
      if (millis() - lastByte > 30000) {
        Serial.println("SSE silent >30s; reconnecting");
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    g_sse_connected = false;
    client.stop();
    Serial.println("SSE disconnected");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ---------- Setup / loop ------------------------------------------------------
void setup() {
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, LOW);
  Serial.begin(115200);
  delay(200);
  Serial.println("light_toggle boot");

  // Display
  gfx->begin();
  gfx->fillScreen(RGB565_BLACK);
  backlightInit();
  backlightSet(50);

  // I2C for touch
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, LOW);  delay(5);
  digitalWrite(TP_RST, HIGH); delay(50);

  // LVGL init
  lv_init();
  lv_tick_set_cb(lv_tick_millis);

  size_t buf_bytes = LV_BUF_PIXELS * sizeof(lv_color_t);
  lv_buf1 = (lv_color_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  lv_buf2 = (lv_color_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!lv_buf1 || !lv_buf2) {
    Serial.println("LVGL DMA buffer alloc failed");
  }

  lv_disp = lv_display_create(SCREEN_W, SCREEN_H);
  lv_display_set_flush_cb(lv_disp, lv_flush_cb);
  lv_display_set_buffers(lv_disp, lv_buf1, lv_buf2, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev = lv_indev_create();
  lv_indev_set_type(lv_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(lv_indev, lv_indev_read_cb);

  buildUi();
  renderState();

  xTaskCreatePinnedToCore(sseTask, "net", 8192, nullptr, 5, nullptr, 0);
}

void loop() {
  static uint32_t last_render = 0;
  uint32_t now = millis();

  lv_timer_handler();

  if (g_state_dirty || (now - last_render) > 500) {
    g_state_dirty = false;
    renderState();
    updateWifiLabel();
    last_render = now;
  }

  delay(5);
}
