#include <Arduino.h>
#include "settings.h"

#ifdef SETTINGS_USE_EXTERNAL_FLASH
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include "flash_config.h"

// Flash chip descriptor — P25Q16H (Seeed XIAO nRF52840)
// https://files.seeedstudio.com/wiki/github_weiruanexample/Flash_P25Q16H-UXH-IR_Datasheet.pdf
static SPIFlash_Device_t const p25q16h{
  .total_size            = (1UL << 21), // 2 MiB
  .start_up_time_us      = 10000,
  .manufacturer_id       = 0x85,
  .memory_type           = 0x60,
  .capacity              = 0x15,
  .max_clock_speed_mhz   = 55,
  .quad_enable_bit_mask  = 0x02,
  .has_sector_protection = 1,
  .supports_fast_read    = 1,
  .supports_qspi         = 1,
  .supports_qspi_writes  = 1,
  .write_status_register_split = 1,
  .single_status_byte    = 0,
  .is_fram               = 0,
};

static Adafruit_SPIFlash flash(&flashTransport);

#else
#include <bluefruit.h>
static BLEBondStore store(0);
#endif

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

bool initSettingsStorage()
{
#ifdef SETTINGS_USE_EXTERNAL_FLASH
  if (!flash.begin(&p25q16h, 1)) {
    Serial.println("#Flash...KO");
    return false;
  }
  Serial.print("#Flash JEDEC ID: 0x");
  Serial.println(flash.getJEDECID(), HEX);
  Serial.print("#Flash size: ");
  Serial.print(flash.size() / 1024);
  Serial.println(" KB");
#endif
  return true;
}

bool readSettings()
{
#ifdef SETTINGS_USE_EXTERNAL_FLASH
  settings_t tmp;
  if (flash.readBuffer(SETTINGS_FLASH_ADDR, (uint8_t*)&tmp, sizeof(settings_t)) != sizeof(settings_t))
    return false;
  if (tmp.magic != MAGIC || tmp.version != VERSION)
    return false;
  g_settings = tmp;
  return true;
#else
  store.getData((uint8_t*)&g_settings, 0, sizeof(settings_t));
  return (g_settings.magic == MAGIC) && (g_settings.version == VERSION);
#endif
}


bool writeSettings()
{
#ifdef SETTINGS_USE_EXTERNAL_FLASH
  // Erase the 4 KB sector that holds the settings page
  if (!flash.eraseSector(SETTINGS_FLASH_ADDR / 4096))
    return false;
  if (flash.writeBuffer(SETTINGS_FLASH_ADDR, (const uint8_t*)&g_settings, sizeof(settings_t)) != sizeof(settings_t))
    return false;
  return true;
#else
  store.putData((const uint8_t*)&g_settings, 0, sizeof(settings_t));
  return true;
#endif
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