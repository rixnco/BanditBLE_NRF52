#include "SettingsManager.h"

SettingsManager::SettingsManager(SettingsStore& store)
  : m_settings(settingsDefaults()), m_store(store) {
}

bool SettingsManager::init() {
  return m_store.init();
}

bool SettingsManager::load() {
  settings_t tmp;
  if (!m_store.read(&tmp, sizeof(settings_t)) ||
      tmp.magic != MAGIC || tmp.version != VERSION) {
    reset();
    return false;
  }
  m_settings = tmp;
  return true;
}

bool SettingsManager::save() {
  return m_store.write(&m_settings, sizeof(settings_t));
}

void SettingsManager::reset() {
  m_settings = settingsDefaults();
}

uint16_t SettingsManager::getGear(uint8_t idx) const {
  switch (idx) {
    case 1: return m_settings.gear1;
    case 2: return m_settings.gear2;
    case 3: return m_settings.gear3;
    case 4: return m_settings.gear4;
    case 5: return m_settings.gear5;
    case 6: return m_settings.gear6;
    default: return 0;
  }
}

void SettingsManager::setGear(uint8_t idx, uint16_t value) {
  switch (idx) {
    case 1: m_settings.gear1 = value; break;
    case 2: m_settings.gear2 = value; break;
    case 3: m_settings.gear3 = value; break;
    case 4: m_settings.gear4 = value; break;
    case 5: m_settings.gear5 = value; break;
    case 6: m_settings.gear6 = value; break;
    default: break;
  }
}

void SettingsManager::getImuOffsets(int16_t& x, int16_t& y, int16_t& z) const {
  x = m_settings.imu_x;
  y = m_settings.imu_y;
  z = m_settings.imu_z;
}

void SettingsManager::setImuOffsets(int16_t x, int16_t y, int16_t z) {
  m_settings.imu_x = x;
  m_settings.imu_y = y;
  m_settings.imu_z = z;
}

