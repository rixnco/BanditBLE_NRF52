#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include <stdint.h>
#include "config.h"

#define MAGIC     0xDEADBEEF
#define VERSION   10000




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