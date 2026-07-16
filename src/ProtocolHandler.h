#ifndef __PROTOCOLHANDLER_H__
#define __PROTOCOLHANDLER_H__

#include <Arduino.h>
#include <stdint.h>
#include "SettingsManager.h"

// Live sensor values pulled by the protocol when generating reports.
// Mirrors TelemetryProvider (AntennaTracker): decouples the protocol from
// where the current gear/rpm values actually live.
class SensorProvider {
public:
  virtual ~SensorProvider() {}
  virtual uint8_t  getCurrentGear() = 0;
  virtual uint16_t getCurrentRPM() = 0;
  virtual uint16_t getCurrentGearRaw() = 0;
};

// Actions triggered by the protocol towards the application.
// Mirrors TelemetryListener callbacks: replaces the previous extern globals.
class BanditController {
public:
  virtual ~BanditController() {}
  virtual void onOverride(bool enabled, int rpm, int gear) = 0;
  virtual void onCalibrateIMU() = 0;
};

class ProtocolHandler {
public:
  ProtocolHandler(Print& out,
                  SettingsManager& settings,
                  SensorProvider& sensors,
                  BanditController& controller);

  // Process a complete command line
  void processCommand(const char* buffer);

  void setReportMode(bool useADC, uint32_t period = 0);
  void sendReport();

  // Getters for report mode state
  bool isADCMode() const { return m_reportUseADC; }
  uint32_t getReportPeriod() const { return m_reportPeriod; }

private:
  Print&            m_out;
  SettingsManager&  m_settings;
  SensorProvider&   m_sensors;
  BanditController& m_controller;

  uint32_t m_reportPeriod;
  bool m_reportUseADC;

  void handleParamRequest(const char* buffer);
  void handleQueryRequest(const char* buffer);
  void handleOverrideRequest(const char* buffer);

  bool setParam(int p, const char* ptr);
  void sendError(const char* msg);
  void sendAck();
  void sendParam(int p);
  void sendSettings();
};

#endif
