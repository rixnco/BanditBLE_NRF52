#include <Arduino.h>
#include "settings.h"



//static BLEBondStore store(0);

settings_t   g_settings __attribute(( aligned(4) )) = {
    .magic = MAGIC,
    .version = VERSION,
    .gear1 = DEFAULT_GEAR1,
    .gear2 = DEFAULT_GEAR2,
    .gear3 = DEFAULT_GEAR3,
    .gear4 = DEFAULT_GEAR4,
    .gear5 = DEFAULT_GEAR5,
    .gear6 = DEFAULT_GEAR6
  };

bool readSettings()
{
  // if(store.hasData()) {
  //   store.getData((uint8_t*)&g_settings, 0 , sizeof(settings_t));
  //   return (g_settings.magic==MAGIC) && (g_settings.version == VERSION);
  // }
  return false;
}


bool writeSettings()
{
  // store.putData((const uint8_t*)&g_settings, 0, sizeof(settings_t));
  return true;
}


void resetSettings()
{
  g_settings= {
    .magic = MAGIC,
    .version = VERSION,
    .gear1 = DEFAULT_GEAR1,
    .gear2 = DEFAULT_GEAR2,
    .gear3 = DEFAULT_GEAR3,
    .gear4 = DEFAULT_GEAR4,
    .gear5 = DEFAULT_GEAR5,
    .gear6 = DEFAULT_GEAR6
  };
}