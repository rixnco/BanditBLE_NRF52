#ifndef __SETTINGSSTORE_H__
#define __SETTINGSSTORE_H__

#include <stddef.h>

// Abstraction over the persistence backend (external flash, BLE bond store, ...).
// Mirrors the transport abstraction used for telemetry: the SettingsManager does
// not know *where* bytes are stored, only how to read/write a blob.
class SettingsStore {
public:
    virtual ~SettingsStore() {}

    virtual bool init() = 0;
    virtual bool read(void* data, size_t len) = 0;
    virtual bool write(const void* data, size_t len) = 0;
};

// Returns the backend selected at compile time (see SETTINGS_USE_EXTERNAL_FLASH).
SettingsStore& getDefaultSettingsStore();

#endif // __SETTINGSSTORE_H__
