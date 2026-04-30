// Waveshare ESP32-S3-Touch-LCD-1.69 - Nova dashboard client.
//
// What it does:
//   - Connects to WiFi (see secrets.h).
//   - Subscribes to nova.local /api/events SSE stream for the "everything"
//     dashboard zone and shared dashboard theme.
//   - Polls /api/nova-load and renders the Nova avatar as additive arcs.
//   - Hold touch anywhere for 500 ms to POST /api/zone with an on/off action.
//   - If WiFi drops, the avatar dims and settles into a non-spinning idle state.
//
// Cores:
//   - core 0: NetTask (WiFi + SSE reader + toggle POSTs)
//   - core 1: Arduino loop() runs LVGL + touch polling
//
// Pinout, board target, and other hardware notes live in the project memory
// file `project_waveshare_esp32s3_touch_lcd_169.md` - never drive GPIO 0/3/45/46.

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include "secrets.h"

#if __has_include("avatar_config.local.h")
#include "avatar_config.local.h"
#endif

// ---------- Pins (from Waveshare BSP) -----------------------------------------
#define LCD_SCK 6
#define LCD_MOSI 7
#define LCD_CS 5
#define LCD_DC 4
#define LCD_RST 8
#define LCD_BL 15

#define I2C_SDA 11
#define I2C_SCL 10
#define TP_RST 13
#define TP_INT 14
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
// 1/8-screen draw buffer is enough because the avatar itself lives in a canvas.
static const uint32_t LV_BUF_PIXELS = SCREEN_W * (SCREEN_H / 8);
static lv_color_t *lv_buf1 = nullptr;
static lv_color_t *lv_buf2 = nullptr;
static lv_display_t *lv_disp = nullptr;
static lv_indev_t *lv_indev = nullptr;

static lv_obj_t *bg_obj = nullptr;
static lv_obj_t *avatar_canvas = nullptr;
static uint16_t *avatar_buf = nullptr;

// ---------- Shared state (cross-core) -----------------------------------------
struct LightState
{
  bool have_data = false;
  bool is_on = false;
  int brightness = 0; // 0..100
  uint8_t rgb[3] = {255, 147, 41};
};

struct Rgb
{
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

struct AvatarTheme
{
  Rgb gradient_center = {60, 0, 71}; // Nova avatar default: #d800ff at 28%
  Rgb gradient_outer = {0, 0, 0};
  Rgb line[3] = {
      {80, 130, 255},
      {180, 95, 240},
      {60, 220, 240},
  };
};

struct AvatarSegment
{
  float radius_frac = 0.5f;
  float width_frac = 0.01f;
  float angle = 0.0f;
  float angular_vel = 0.0f;
  float target_vel = 0.0f;
  float size = 0.02f;
  float target_size = 0.02f;
  float next_resample_at = 0.0f;
  uint8_t color_index = 0;
};

#ifndef NOVA_AVATAR_ARC_COUNT
#define NOVA_AVATAR_ARC_COUNT 32
#endif

#if NOVA_AVATAR_ARC_COUNT < 1 || NOVA_AVATAR_ARC_COUNT > 60
#error "NOVA_AVATAR_ARC_COUNT must be between 1 and 60"
#endif

static const uint16_t AVATAR_SIZE = 196;
static const uint8_t AVATAR_SEGMENT_COUNT = NOVA_AVATAR_ARC_COUNT;
static const uint32_t AVATAR_FRAME_MS = 40; // 25 fps target
static const uint32_t NOVA_LOAD_POLL_MS = 1000;
static const float AVATAR_RADIUS_MIN = 0.14f;
static const float AVATAR_RADIUS_MAX = 0.95f;
static const float AVATAR_LINE_WIDTH_MIN = 0.006f;
static const float AVATAR_LINE_WIDTH_MAX = 0.030f;
static const float AVATAR_IDLE_SIZE_MIN = 0.010f;
static const float AVATAR_IDLE_SIZE_MAX = 0.040f;
static const float AVATAR_FULL_SIZE = 0.72f;
static const float TWO_PI_F = 6.28318530718f;

static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static LightState g_state;
static AvatarTheme g_avatar_theme;
static AvatarSegment g_avatar_segments[AVATAR_SEGMENT_COUNT];
static volatile bool g_state_dirty = true;
static volatile bool g_avatar_theme_dirty = true;
static volatile bool g_wifi_connected = false;
static volatile bool g_sse_connected = false;
static volatile bool g_toggle_request = false;
static float g_avatar_target_load = 0.0f;
static float g_avatar_current_load = 0.0f;
static bool g_avatar_segments_ready = false;
static const uint32_t TOUCH_HOLD_MS = 500;
static uint32_t g_touch_press_started_ms = 0;
static bool g_touch_action_fired = false;

static void publishState(const LightState &s)
{
  portENTER_CRITICAL(&state_mux);
  g_state = s;
  g_state_dirty = true;
  portEXIT_CRITICAL(&state_mux);
}

static LightState snapshotState()
{
  LightState s;
  portENTER_CRITICAL(&state_mux);
  s = g_state;
  portEXIT_CRITICAL(&state_mux);
  return s;
}

static float clamp01(float v)
{
  if (v < 0.0f)
    return 0.0f;
  if (v > 1.0f)
    return 1.0f;
  return v;
}

static void publishAvatarLoad(float load)
{
  portENTER_CRITICAL(&state_mux);
  g_avatar_target_load = clamp01(load);
  portEXIT_CRITICAL(&state_mux);
}

static float snapshotAvatarTargetLoad()
{
  float load;
  portENTER_CRITICAL(&state_mux);
  load = g_avatar_target_load;
  portEXIT_CRITICAL(&state_mux);
  return load;
}

static void publishAvatarTheme(const AvatarTheme &theme)
{
  portENTER_CRITICAL(&state_mux);
  g_avatar_theme = theme;
  g_avatar_theme_dirty = true;
  portEXIT_CRITICAL(&state_mux);
}

static AvatarTheme snapshotAvatarTheme()
{
  AvatarTheme theme;
  portENTER_CRITICAL(&state_mux);
  theme = g_avatar_theme;
  portEXIT_CRITICAL(&state_mux);
  return theme;
}

// ---------- Backlight ---------------------------------------------------------
static const uint32_t BL_FREQ_HZ = 5000;
static const uint8_t BL_RES_BITS = 10; // duty 0..1023

static void backlightInit()
{
  ledcAttach(LCD_BL, BL_FREQ_HZ, BL_RES_BITS);
  ledcWrite(LCD_BL, 0);
}

static void backlightSet(uint8_t pct)
{
  if (pct > 100)
    pct = 100;
  uint32_t duty = (1023u * pct) / 100u;
  ledcWrite(LCD_BL, duty);
}

// ---------- LVGL display flush ------------------------------------------------
static void lv_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  // Arduino_GFX's SPI bus writes RGB565 values as big-endian panel bytes.
  // Keep the LVGL buffer in native RGB565; pre-swapping turns yellow purple.
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
  lv_display_flush_ready(disp);
}

// ---------- CST816 touch driver -----------------------------------------------
// Note: returning a struct from an .ino function trips arduino-cli's prototype
// generator (it forward-declares before the struct exists), so use out params.
static bool cst816Read(int16_t &x, int16_t &y)
{
  Wire.beginTransmission(CST816_ADDR);
  Wire.write(0x01);
  if (Wire.endTransmission(false) != 0)
    return false;
  if (Wire.requestFrom((uint8_t)CST816_ADDR, (uint8_t)6) != 6)
    return false;
  uint8_t buf[6];
  for (int i = 0; i < 6; i++)
    buf[i] = Wire.read();
  uint8_t points = buf[1] & 0x0F;
  if (points == 0)
    return false;
  x = ((uint16_t)(buf[2] & 0x0F) << 8) | buf[3];
  y = ((uint16_t)(buf[4] & 0x0F) << 8) | buf[5];
  return true;
}

static void lv_indev_read_cb(lv_indev_t *, lv_indev_data_t *data)
{
  int16_t x, y;
  if (cst816Read(x, y))
  {
    data->point.x = constrain((int)x, 0, SCREEN_W - 1);
    data->point.y = constrain((int)y, 0, SCREEN_H - 1);
    data->state = LV_INDEV_STATE_PRESSED;
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// ---------- LVGL tick from millis ---------------------------------------------
static uint32_t lv_tick_millis() { return millis(); }

// ---------- Nova avatar renderer ----------------------------------------------
static float frand(float min_value, float max_value)
{
  float u = (float)random(0, 10000) / 10000.0f;
  return min_value + (max_value - min_value) * u;
}

static uint16_t pack565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((uint16_t)(r & 0xF8) << 8) |
         ((uint16_t)(g & 0xFC) << 3) |
         ((uint16_t)b >> 3);
}

static void unpack565(uint16_t c, uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = (uint8_t)(((c >> 11) & 0x1F) * 255 / 31);
  g = (uint8_t)(((c >> 5) & 0x3F) * 255 / 63);
  b = (uint8_t)((c & 0x1F) * 255 / 31);
}

static void setAvatarPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
  if (!avatar_buf || x < 0 || y < 0 || x >= AVATAR_SIZE || y >= AVATAR_SIZE)
    return;
  avatar_buf[(uint32_t)y * AVATAR_SIZE + x] = pack565(r, g, b);
}

static void addAvatarPixel(int x, int y, const Rgb &color, uint8_t alpha)
{
  if (!avatar_buf || alpha == 0 || x < 0 || y < 0 || x >= AVATAR_SIZE || y >= AVATAR_SIZE)
    return;

  uint16_t &dst = avatar_buf[(uint32_t)y * AVATAR_SIZE + x];
  uint8_t r, g, b;
  unpack565(dst, r, g, b);

  int nr = r + ((int)color.r * alpha) / 255;
  int ng = g + ((int)color.g * alpha) / 255;
  int nb = b + ((int)color.b * alpha) / 255;
  dst = pack565((uint8_t)min(nr, 255), (uint8_t)min(ng, 255), (uint8_t)min(nb, 255));
}

static void drawAvatarBackground(const AvatarTheme &theme)
{
  const float cx = (AVATAR_SIZE - 1) * 0.5f;
  const float cy = (AVATAR_SIZE - 1) * 0.5f;
  const float radius = AVATAR_SIZE * 0.485f;
  const float radius2 = radius * radius;

  for (int y = 0; y < AVATAR_SIZE; y++)
  {
    float dy = y - cy;
    for (int x = 0; x < AVATAR_SIZE; x++)
    {
      float dx = x - cx;
      float d2 = dx * dx + dy * dy;
      if (d2 > radius2)
      {
        setAvatarPixel(x, y, 0, 0, 0);
        continue;
      }

      float t = d2 / radius2;
      uint8_t r = (uint8_t)(theme.gradient_center.r + (theme.gradient_outer.r - theme.gradient_center.r) * t);
      uint8_t g = (uint8_t)(theme.gradient_center.g + (theme.gradient_outer.g - theme.gradient_center.g) * t);
      uint8_t b = (uint8_t)(theme.gradient_center.b + (theme.gradient_outer.b - theme.gradient_center.b) * t);
      setAvatarPixel(x, y, r, g, b);
    }
  }
}

static void drawAvatarDisk(int cx, int cy, int radius, const Rgb &color, uint8_t alpha)
{
  if (radius < 1)
    radius = 1;
  int r2 = radius * radius;
  float ac = (AVATAR_SIZE - 1) * 0.5f;
  float clip = AVATAR_SIZE * 0.486f;
  float clip2 = clip * clip;

  for (int y = cy - radius; y <= cy + radius; y++)
  {
    for (int x = cx - radius; x <= cx + radius; x++)
    {
      int dx = x - cx;
      int dy = y - cy;
      if (dx * dx + dy * dy > r2)
        continue;

      float odx = x - ac;
      float ody = y - ac;
      if (odx * odx + ody * ody > clip2)
        continue;
      addAvatarPixel(x, y, color, alpha);
    }
  }
}

static void drawAdditiveArc(float radius_px, float width_px, float start_angle,
                            float arc_fraction, const Rgb &color, uint8_t alpha)
{
  float span = arc_fraction * TWO_PI_F;
  if (span < 0.010f)
    return;

  int steps = (int)(span * radius_px * 0.85f);
  steps = constrain(steps, 4, 120);
  int dot_radius = max(1, (int)(width_px * 0.5f));
  float cx = (AVATAR_SIZE - 1) * 0.5f;
  float cy = (AVATAR_SIZE - 1) * 0.5f;

  for (int i = 0; i <= steps; i++)
  {
    float a = start_angle + span * ((float)i / (float)steps);
    int x = (int)roundf(cx + cosf(a) * radius_px);
    int y = (int)roundf(cy + sinf(a) * radius_px);
    drawAvatarDisk(x, y, dot_radius, color, alpha);
  }
}

static void resampleAvatarSegment(AvatarSegment &s, float load, float now_sec)
{
  float idle = frand(AVATAR_IDLE_SIZE_MIN, AVATAR_IDLE_SIZE_MAX);
  s.target_size = idle + (AVATAR_FULL_SIZE - idle) * load;

  float speed = frand(0.15f, 0.75f) + load * frand(0.2f, 1.1f);
  if (random(0, 2) == 0)
    speed = -speed;
  s.target_vel = speed;
  s.next_resample_at = now_sec + frand(0.8f, 1.4f);
}

static void initAvatarSegments()
{
  randomSeed((uint32_t)micros());
  for (uint8_t i = 0; i < AVATAR_SEGMENT_COUNT; i++)
  {
    AvatarSegment &s = g_avatar_segments[i];
    s.radius_frac = frand(AVATAR_RADIUS_MIN, AVATAR_RADIUS_MAX);
    s.width_frac = frand(AVATAR_LINE_WIDTH_MIN, AVATAR_LINE_WIDTH_MAX);
    s.angle = frand(0.0f, TWO_PI_F);
    s.angular_vel = frand(-0.45f, 0.45f);
    s.color_index = i % 3;
    resampleAvatarSegment(s, 0.0f, 0.0f);
  }
  g_avatar_segments_ready = true;
}

static void updateAvatarSegments(float load, float dt, float now_sec)
{
  for (uint8_t i = 0; i < AVATAR_SEGMENT_COUNT; i++)
  {
    AvatarSegment &s = g_avatar_segments[i];
    if (now_sec >= s.next_resample_at)
    {
      resampleAvatarSegment(s, load, now_sec);
    }

    float size_ease = min(dt * 3.0f, 1.0f);
    float vel_ease = min(dt * 2.0f, 1.0f);
    s.size += (s.target_size - s.size) * size_ease;
    s.angular_vel += (s.target_vel - s.angular_vel) * vel_ease;
    s.angle += s.angular_vel * dt;
    if (s.angle > TWO_PI_F)
      s.angle -= TWO_PI_F;
    if (s.angle < 0.0f)
      s.angle += TWO_PI_F;
  }
}

static void settleAvatarSegmentsIdle(float dt)
{
  float ease = min(dt * 2.0f, 1.0f);
  for (uint8_t i = 0; i < AVATAR_SEGMENT_COUNT; i++)
  {
    AvatarSegment &s = g_avatar_segments[i];
    s.target_vel = 0.0f;
    s.angular_vel = 0.0f;
    s.size += (AVATAR_IDLE_SIZE_MIN - s.size) * ease;
  }
}

static void dimTheme(AvatarTheme &theme, uint8_t pct)
{
  theme.gradient_center.r = (uint8_t)((uint16_t)theme.gradient_center.r * pct / 100);
  theme.gradient_center.g = (uint8_t)((uint16_t)theme.gradient_center.g * pct / 100);
  theme.gradient_center.b = (uint8_t)((uint16_t)theme.gradient_center.b * pct / 100);
  theme.gradient_outer.r = (uint8_t)((uint16_t)theme.gradient_outer.r * pct / 100);
  theme.gradient_outer.g = (uint8_t)((uint16_t)theme.gradient_outer.g * pct / 100);
  theme.gradient_outer.b = (uint8_t)((uint16_t)theme.gradient_outer.b * pct / 100);

  for (uint8_t i = 0; i < 3; i++)
  {
    theme.line[i].r = (uint8_t)((uint16_t)theme.line[i].r * pct / 100);
    theme.line[i].g = (uint8_t)((uint16_t)theme.line[i].g * pct / 100);
    theme.line[i].b = (uint8_t)((uint16_t)theme.line[i].b * pct / 100);
  }
}

static void renderAvatarFrame(uint32_t now_ms)
{
  if (!avatar_buf || !avatar_canvas)
    return;
  if (!g_avatar_segments_ready)
    initAvatarSegments();

  static uint32_t last_ms = 0;
  if (last_ms == 0)
    last_ms = now_ms;
  float dt = (now_ms - last_ms) / 1000.0f;
  if (dt <= 0.0f || dt > 0.20f)
    dt = AVATAR_FRAME_MS / 1000.0f;
  last_ms = now_ms;

  bool online = g_wifi_connected;
  float target = online ? snapshotAvatarTargetLoad() : 0.0f;
  g_avatar_current_load += (target - g_avatar_current_load) * min(dt * 2.5f, 1.0f);
  float load = clamp01(g_avatar_current_load);
  float now_sec = now_ms / 1000.0f;

  AvatarTheme theme = snapshotAvatarTheme();
  if (!online)
  {
    dimTheme(theme, 28);
  }
  drawAvatarBackground(theme);
  if (online)
  {
    updateAvatarSegments(load, dt, now_sec);
  }
  else
  {
    settleAvatarSegmentsIdle(dt);
  }

  uint8_t alpha = online ? (uint8_t)(135 + load * 90) : 42;
  for (uint8_t i = 0; i < AVATAR_SEGMENT_COUNT; i++)
  {
    const AvatarSegment &s = g_avatar_segments[i];
    float radius_px = (AVATAR_SIZE * 0.5f) * s.radius_frac;
    float width_px = AVATAR_SIZE * s.width_frac;
    drawAdditiveArc(radius_px, width_px, s.angle, s.size, theme.line[s.color_index], alpha);
  }

  Rgb white = {255, 255, 255};
  drawAdditiveArc(AVATAR_SIZE * 0.31f, 2.0f, -2.25f, 0.18f, white, online ? 32 : 10);
  drawAdditiveArc(AVATAR_SIZE * 0.41f, 1.0f, 0.80f, 0.08f, white, online ? 18 : 6);

  g_avatar_theme_dirty = false;
  lv_obj_invalidate(avatar_canvas);
}

// ---------- UI ----------------------------------------------------------------
static lv_obj_t *createTapTarget()
{
  lv_obj_t *t = lv_obj_create(lv_screen_active());
  lv_obj_remove_style_all(t);
  lv_obj_set_size(t, SCREEN_W, SCREEN_H);
  lv_obj_set_pos(t, 0, 0);
  // Catch presses but let the bg color show through.
  lv_obj_add_flag(t, LV_OBJ_FLAG_CLICKABLE);
  return t;
}

static void triggerToggle()
{
  // Optimistic flip + nudge net task to fire the HTTP toggle.
  LightState s = snapshotState();
  s.is_on = !s.is_on;
  if (!s.is_on)
  {
    s.brightness = 0;
  }
  else if (s.brightness == 0)
  {
    s.brightness = 86; // dashboard's default candlelight pct
  }
  s.have_data = true; // make sure first-tap shows feedback
  publishState(s);
  g_toggle_request = true;
}

static void onTouchEvent(lv_event_t *event)
{
  lv_event_code_t code = lv_event_get_code(event);
  if (code == LV_EVENT_PRESSED)
  {
    g_touch_press_started_ms = millis();
    g_touch_action_fired = false;
    return;
  }

  if (code == LV_EVENT_PRESSING &&
      !g_touch_action_fired &&
      g_touch_press_started_ms != 0 &&
      millis() - g_touch_press_started_ms >= TOUCH_HOLD_MS)
  {
    g_touch_action_fired = true;
    triggerToggle();
    return;
  }

  if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
  {
    g_touch_press_started_ms = 0;
    g_touch_action_fired = false;
  }
}

static void buildUi()
{
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

  avatar_canvas = lv_canvas_create(scr);
  size_t avatar_bytes = (size_t)AVATAR_SIZE * AVATAR_SIZE * sizeof(uint16_t);
  avatar_buf = (uint16_t *)heap_caps_malloc(avatar_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!avatar_buf)
  {
    avatar_buf = (uint16_t *)heap_caps_malloc(avatar_bytes, MALLOC_CAP_8BIT);
  }
  if (avatar_buf)
  {
    memset(avatar_buf, 0, avatar_bytes);
    lv_canvas_set_buffer(avatar_canvas, avatar_buf, AVATAR_SIZE, AVATAR_SIZE, LV_COLOR_FORMAT_RGB565);
    lv_obj_align(avatar_canvas, LV_ALIGN_CENTER, 0, 0);
  }
  else
  {
    Serial.println("avatar canvas buffer alloc failed");
  }

  lv_obj_t *tap = createTapTarget();
  lv_obj_add_event_cb(tap, onTouchEvent, LV_EVENT_PRESSED, nullptr);
  lv_obj_add_event_cb(tap, onTouchEvent, LV_EVENT_PRESSING, nullptr);
  lv_obj_add_event_cb(tap, onTouchEvent, LV_EVENT_RELEASED, nullptr);
  lv_obj_add_event_cb(tap, onTouchEvent, LV_EVENT_PRESS_LOST, nullptr);
}

// Render the latest shared state into the LVGL widgets.
static void renderState()
{
  lv_obj_set_style_bg_color(bg_obj, lv_color_black(), 0);
  backlightSet(g_wifi_connected ? 100 : 28);
}

static void updateWifiLabel()
{
  // Visual state is carried by the avatar itself.
}

// ---------- Network task (core 0) --------------------------------------------
//
// Minimal SSE client: maintain a TCP connection to /api/events, parse
// `event: state` / `data: {...}` lines, update shared state. Reconnect with
// backoff if the stream dies.
//
// The dashboard currently emits full DashboardState JSON. We extract the
// `everything` zone plus the shared theme colors used by the avatar.
static const size_t MAX_SSE_LINE = 49152;

static bool jsonExtractBoolFrom(const String &json, int from, const char *key, bool &out)
{
  if (from < 0)
    from = 0;
  String pat = String("\"") + key + "\":";
  int idx = json.indexOf(pat, from);
  if (idx < 0)
    return false;
  idx += pat.length();
  while (idx < (int)json.length() && (json[idx] == ' ' || json[idx] == '\t'))
    idx++;
  if (json.startsWith("true", idx))
  {
    out = true;
    return true;
  }
  if (json.startsWith("false", idx))
  {
    out = false;
    return true;
  }
  return false;
}

static bool jsonExtractIntFrom(const String &json, int from, const char *key, int &out)
{
  if (from < 0)
    from = 0;
  String pat = String("\"") + key + "\":";
  int idx = json.indexOf(pat, from);
  if (idx < 0)
    return false;
  idx += pat.length();
  while (idx < (int)json.length() && (json[idx] == ' ' || json[idx] == '\t'))
    idx++;
  int start = idx;
  if (idx < (int)json.length() && (json[idx] == '-' || json[idx] == '+'))
    idx++;
  while (idx < (int)json.length() && isDigit(json[idx]))
    idx++;
  if (idx == start)
    return false;
  out = json.substring(start, idx).toInt();
  return true;
}

static bool jsonExtractFloatFrom(const String &json, int from, const char *key, float &out)
{
  if (from < 0)
    from = 0;
  String pat = String("\"") + key + "\":";
  int idx = json.indexOf(pat, from);
  if (idx < 0)
    return false;
  idx += pat.length();
  while (idx < (int)json.length() && (json[idx] == ' ' || json[idx] == '\t'))
    idx++;
  int start = idx;
  if (idx < (int)json.length() && (json[idx] == '-' || json[idx] == '+'))
    idx++;
  while (idx < (int)json.length() && (isDigit(json[idx]) || json[idx] == '.'))
    idx++;
  if (idx == start)
    return false;
  out = json.substring(start, idx).toFloat();
  return true;
}

static bool jsonExtractRgbArrayFrom(const String &json, int from, const char *key, uint8_t out[3])
{
  if (from < 0)
    from = 0;
  String pat = String("\"") + key + "\":[";
  int idx = json.indexOf(pat, from);
  if (idx < 0)
    return false;
  idx += pat.length();
  for (int ch = 0; ch < 3; ch++)
  {
    while (idx < (int)json.length() && !isDigit(json[idx]) && json[idx] != '-')
      idx++;
    int start = idx;
    while (idx < (int)json.length() && isDigit(json[idx]))
      idx++;
    if (idx == start)
      return false;
    int v = json.substring(start, idx).toInt();
    out[ch] = (uint8_t)constrain(v, 0, 255);
  }
  return true;
}

static void applyIntensityToRgb(const uint8_t rgb[3], int intensity, int min_intensity, Rgb &out)
{
  intensity = constrain(intensity, min_intensity, 100);
  out.r = (uint8_t)((uint16_t)rgb[0] * intensity / 100);
  out.g = (uint8_t)((uint16_t)rgb[1] * intensity / 100);
  out.b = (uint8_t)((uint16_t)rgb[2] * intensity / 100);
}

static bool jsonExtractThemeColorFrom(const String &json, int themeIdx, const char *key,
                                      int min_intensity, Rgb &out)
{
  if (themeIdx < 0)
    return false;
  String pat = String("\"") + key + "\":{";
  int objIdx = json.indexOf(pat, themeIdx);
  if (objIdx < 0)
    return false;

  uint8_t rgb[3];
  if (!jsonExtractRgbArrayFrom(json, objIdx, "rgb", rgb))
    return false;

  int intensity = 100;
  jsonExtractIntFrom(json, objIdx, "intensity", intensity);
  applyIntensityToRgb(rgb, intensity, min_intensity, out);
  return true;
}

static void applySharedThemePayload(const String &payload)
{
  int preferencesIdx = payload.indexOf("\"preferences\":");
  if (preferencesIdx < 0)
    return;
  int themeIdx = payload.indexOf("\"theme\":{", preferencesIdx);
  if (themeIdx < 0)
    return;

  AvatarTheme theme = snapshotAvatarTheme();
  bool changed = false;
  Rgb color;

  if (jsonExtractThemeColorFrom(payload, themeIdx, "highlight", 35, color))
  {
    theme.gradient_center = color;
    theme.line[1] = color;
    changed = true;
  }
  if (jsonExtractThemeColorFrom(payload, themeIdx, "background", 0, color))
  {
    theme.gradient_outer = color;
    changed = true;
  }
  if (jsonExtractThemeColorFrom(payload, themeIdx, "accent", 65, color))
  {
    theme.line[0] = color;
    changed = true;
  }
  if (jsonExtractThemeColorFrom(payload, themeIdx, "border", 55, color))
  {
    theme.line[2] = color;
    changed = true;
  }

  if (changed)
  {
    publishAvatarTheme(theme);
  }
}

static void applyCompactHomePayload(const String &payload)
{
  if (payload.indexOf("\"event\":\"home\"") < 0)
    return;
  LightState s = snapshotState();
  bool b;
  int i;
  uint8_t rgb[3];
  if (jsonExtractBoolFrom(payload, 0, "isOn", b))
    s.is_on = b;
  if (jsonExtractIntFrom(payload, 0, "brightnessPct", i))
    s.brightness = constrain(i, 0, 100);
  if (jsonExtractRgbArrayFrom(payload, 0, "rgb", rgb))
    memcpy(s.rgb, rgb, 3);
  s.have_data = true;
  publishState(s);
}

static void applyDashboardStatePayload(const String &payload)
{
  applySharedThemePayload(payload);

  int zoneIdx = payload.indexOf("\"id\":\"everything\"");
  if (zoneIdx < 0)
    return;

  LightState s = snapshotState();
  bool changed = false;
  bool b;
  int i;
  uint8_t rgb[3];

  if (jsonExtractBoolFrom(payload, zoneIdx, "isOn", b))
  {
    s.is_on = b;
    changed = true;
  }
  if (jsonExtractIntFrom(payload, zoneIdx, "brightnessPct", i))
  {
    s.brightness = constrain(i, 0, 100);
    changed = true;
  }
  if (jsonExtractRgbArrayFrom(payload, zoneIdx, "rgb_color", rgb) ||
      jsonExtractRgbArrayFrom(payload, zoneIdx, "rgb", rgb))
  {
    memcpy(s.rgb, rgb, 3);
    changed = true;
  }

  if (!changed)
    return;
  s.have_data = true;
  publishState(s);
}

static void applySseDataLine(const String &payload, const String &eventName)
{
  if (eventName == "state")
  {
    applyDashboardStatePayload(payload);
    return;
  }

  // Backward-compatible with the compact event shape from the first firmware.
  applyCompactHomePayload(payload);
}

static void doToggle()
{
  LightState s = snapshotState();
  const char *action = s.is_on ? "on" : "off";
  String body = String("{\"zoneId\":\"everything\",\"action\":\"") + action + "\"";
  if (s.is_on)
  {
    body += String(",\"brightnessPct\":") + s.brightness;
  }
  body += "}";

  HTTPClient http;
  String url = String("http://") + NOVA_HOST + ":" + NOVA_PORT + "/api/zone";
  if (!http.begin(url))
    return;
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(2000);
  int code = http.POST(body);
  Serial.printf("toggle POST -> %d\n", code);
  http.end();
}

static void pollNovaLoad()
{
  HTTPClient http;
  String url = String("http://") + NOVA_HOST + ":" + NOVA_PORT + "/api/nova-load";
  if (!http.begin(url))
  {
    publishAvatarLoad(0.0f);
    return;
  }
  http.setTimeout(800);
  int code = http.GET();
  if (code == 200)
  {
    String payload = http.getString();
    float load;
    if (jsonExtractFloatFrom(payload, 0, "load", load))
    {
      publishAvatarLoad(load);
    }
  }
  else
  {
    publishAvatarLoad(0.0f);
  }
  http.end();
}

static void maybePollNovaLoad(uint32_t &last_poll_ms)
{
  if (WiFi.status() != WL_CONNECTED)
    return;

  uint32_t now = millis();
  if (last_poll_ms != 0 && now - last_poll_ms < NOVA_LOAD_POLL_MS)
    return;
  last_poll_ms = now;
  pollNovaLoad();
}

static void sseTask(void *)
{
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t backoff_ms = 1000;
  uint32_t last_load_poll_ms = 0;
  for (;;)
  {
    if (g_toggle_request)
    {
      g_toggle_request = false;
      doToggle();
    }

    if (WiFi.status() != WL_CONNECTED)
    {
      g_wifi_connected = false;
      g_sse_connected = false;
      publishAvatarLoad(0.0f);
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    g_wifi_connected = true;
    maybePollNovaLoad(last_load_poll_ms);

    WiFiClient client;
    if (!client.connect(NOVA_HOST, NOVA_PORT))
    {
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
        "\r\n",
        NOVA_HOST);

    // Skip headers
    bool ok = false;
    String first = client.readStringUntil('\n');
    if (first.startsWith("HTTP/1.1 200"))
      ok = true;
    while (client.connected())
    {
      String h = client.readStringUntil('\n');
      if (h == "\r" || h.length() <= 1)
        break;
    }

    if (!ok)
    {
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

    while (client.connected() && WiFi.status() == WL_CONNECTED)
    {
      // Service toggles even while inside the SSE read loop.
      if (g_toggle_request)
      {
        g_toggle_request = false;
        doToggle();
      }
      maybePollNovaLoad(last_load_poll_ms);

      while (client.available())
      {
        char c = (char)client.read();
        lastByte = millis();
        if (c == '\r')
          continue;
        if (c == '\n')
        {
          if (line.startsWith("event:"))
          {
            eventName = line.substring(6);
            eventName.trim();
          }
          else if (line.startsWith("data:"))
          {
            line.remove(0, 5);
            line.trim();
            if (line.length())
              applySseDataLine(line, eventName);
          }
          else if (line.length() == 0)
          {
            eventName = "";
          }
          line = "";
        }
        else
        {
          if (line.length() < MAX_SSE_LINE)
            line += c;
        }
      }
      if (millis() - lastByte > 30000)
      {
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
void setup()
{
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
  digitalWrite(TP_RST, LOW);
  delay(5);
  digitalWrite(TP_RST, HIGH);
  delay(50);

  // LVGL init
  lv_init();
  lv_tick_set_cb(lv_tick_millis);

  size_t buf_bytes = LV_BUF_PIXELS * sizeof(lv_color_t);
  lv_buf1 = (lv_color_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  lv_buf2 = (lv_color_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!lv_buf1 || !lv_buf2)
  {
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

void loop()
{
  static uint32_t last_render = 0;
  static uint32_t last_avatar = 0;
  uint32_t now = millis();

  if (last_avatar == 0 || now - last_avatar >= AVATAR_FRAME_MS)
  {
    renderAvatarFrame(now);
    last_avatar = now;
  }

  lv_timer_handler();

  if (g_state_dirty || (now - last_render) > 500)
  {
    g_state_dirty = false;
    renderState();
    last_render = now;
  }

  delay(5);
}
