#pragma once

// Keep local Wi-Fi credentials out of version control.
// Copy config_local.example.h to config_local.h and fill in your own values.
#if defined(__has_include)
#  if __has_include("config_local.h")
#    include "config_local.h"
#  else
#    define WIFI_SSID ""
#    define WIFI_PASSWORD ""
#  endif
#else
#  define WIFI_SSID ""
#  define WIFI_PASSWORD ""
#endif

#ifndef AGRO_VERSION
#define AGRO_VERSION "V11"
#endif
