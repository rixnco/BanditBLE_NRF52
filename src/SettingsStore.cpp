#include <Arduino.h>
#include "SettingsStore.h"
#include "config.h"

#ifdef SETTINGS_USE_EXTERNAL_FLASH

#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include "flash_config.h"

// External SPI flash backend (e.g. P25Q16H on Seeed XIAO nRF52840).
// https://files.seeedstudio.com/wiki/github_weiruanexample/Flash_P25Q16H-UXH-IR_Datasheet.pdf
class FlashSettingsStore : public SettingsStore {
public:
    FlashSettingsStore() : _flash(&flashTransport) {}

    bool init() override {
        if (!_flash.begin(&_device, 1)) {
            Serial.println("#Flash...KO");
            return false;
        }
        Serial.print("#Flash JEDEC ID: 0x");
        Serial.println(_flash.getJEDECID(), HEX);
        Serial.print("#Flash size: ");
        Serial.print(_flash.size() / 1024);
        Serial.println(" KB");
        return true;
    }

    bool read(void* data, size_t len) override {
        return _flash.readBuffer(SETTINGS_FLASH_ADDR, (uint8_t*)data, len) == (uint32_t)len;
    }

    bool write(const void* data, size_t len) override {
        // Erase the 4 KB sector that holds the settings page first.
        if (!_flash.eraseSector(SETTINGS_FLASH_ADDR / 4096))
            return false;
        return _flash.writeBuffer(SETTINGS_FLASH_ADDR, (const uint8_t*)data, len) == (uint32_t)len;
    }

private:
    static const SPIFlash_Device_t _device;
    Adafruit_SPIFlash _flash;
};

const SPIFlash_Device_t FlashSettingsStore::_device{
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

SettingsStore& getDefaultSettingsStore() {
    static FlashSettingsStore instance;
    return instance;
}

#else // !SETTINGS_USE_EXTERNAL_FLASH

#include <bluefruit.h>

// BLE bond store backend (persists in the SoftDevice flash area).
class BondSettingsStore : public SettingsStore {
public:
    BondSettingsStore() : _store(0) {}

    bool init() override { return true; }

    bool read(void* data, size_t len) override {
        return _store.getData((uint8_t*)data, 0, len);
    }

    bool write(const void* data, size_t len) override {
        _store.putData((const uint8_t*)data, 0, len);
        return true;
    }

private:
    BLEBondStore _store;
};

SettingsStore& getDefaultSettingsStore() {
    static BondSettingsStore instance;
    return instance;
}

#endif // SETTINGS_USE_EXTERNAL_FLASH
