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
  int16_t imu_x;  // Accelerometer X offset (milli-g units)
  int16_t imu_y;  // Accelerometer Y offset (milli-g units)
  int16_t imu_z;  // Accelerometer Z offset (milli-g units)
} settings_t;

// Returns a settings_t populated with the compile-time defaults.
settings_t settingsDefaults();

#endif