#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include <stdint.h>

// Uncomment to persist settings in external SPI flash (P25Q16H)
// Requires Adafruit_SPIFlash. The 'flash' object must be initialised before
// calling readSettings() / writeSettings().
#define SETTINGS_USE_EXTERNAL_FLASH

// Address of the settings page in external flash (last 4 KB sector)
#define SETTINGS_FLASH_ADDR   0x1FF000UL

#define MAGIC     0xDEADBEEF
#define VERSION   10000

#define DEFAULT_GEAR1 370
#define DEFAULT_GEAR2 477
#define DEFAULT_GEAR3 600
#define DEFAULT_GEAR4 720
#define DEFAULT_GEAR5 810
#define DEFAULT_GEAR6 867




typedef struct { 
  uint32_t magic;
  uint32_t version;
  uint16_t gear1;
  uint16_t gear2;
  uint16_t gear3;
  uint16_t gear4;
  uint16_t gear5;
  uint16_t gear6;
} settings_t;

extern settings_t   g_settings;

bool initSettingsStorage();
bool readSettings();
bool writeSettings();
void resetSettings();

#endif