#ifndef __SETTINGSMANAGER_H__
#define __SETTINGSMANAGER_H__

#include "settings.h"
#include "SettingsStore.h"

// Single source of truth for the device settings.
// Owns the settings_t instance and delegates persistence to an injected
// SettingsStore backend (mirrors the dependency-injection style used for
// telemetry decoders in the AntennaTracker project).
class SettingsManager {
public:
  SettingsManager(SettingsStore& store);

  // Initialization and persistence
  bool init();
  bool load();   // read+validate via store; on failure resets to defaults, returns false
  bool save();   // write current settings via store
  void reset();  // restore compile-time defaults (in RAM only)

  // Typed accessors (idx is 1..6 for gears)
  uint16_t getGear(uint8_t idx) const;
  void     setGear(uint8_t idx, uint16_t value);

  void getImuOffsets(int16_t& x, int16_t& y, int16_t& z) const;
  void setImuOffsets(int16_t x, int16_t y, int16_t z);

private:
  settings_t     m_settings;
  SettingsStore& m_store;
};

#endif
