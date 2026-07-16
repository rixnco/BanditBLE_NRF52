#include "settings.h"

settings_t settingsDefaults()
{
  settings_t s = {
    .magic   = MAGIC,
    .version = VERSION,
    .gear1   = DEFAULT_GEAR1,
    .gear2   = DEFAULT_GEAR2,
    .gear3   = DEFAULT_GEAR3,
    .gear4   = DEFAULT_GEAR4,
    .gear5   = DEFAULT_GEAR5,
    .gear6   = DEFAULT_GEAR6,
    .imu_x   = 0,
    .imu_y   = 0,
    .imu_z   = 0
  };
  return s;
}