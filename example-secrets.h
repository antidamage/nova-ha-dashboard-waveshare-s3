#pragma once

// Copy this file to secrets.h and fill in values for your local network.
//
// secrets.h is ignored by git. Keep real WiFi credentials, hostnames, and
// addresses out of committed files.

// WiFi credentials for the network the watch should join.
#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASSWORD "your-wifi-password"

// Nova HA Dashboard host and port.
//
// NOVA_HOST can be an mDNS name such as "nova.local" or a LAN IP address such
// as "192.168.1.50". Do not include "http://" or a trailing slash.
#define NOVA_HOST "nova.local"
#define NOVA_PORT 80
