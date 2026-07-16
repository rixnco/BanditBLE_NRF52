#include <Arduino.h>
#include <bluefruit.h>
#include "protocol.h"
#include "ProtocolHandler.h"
#include "SettingsManager.h"

extern BLEUart nrfuart;

// Create a dual-output stream that writes to Serial and BLE UART
class DualOutputStream : public Print {
public:
  size_t write(uint8_t byte) override {
    Serial.write(byte);
    return nrfuart.write(byte);
  }
};

static DualOutputStream dualOut;
static ProtocolHandler* g_protocol = nullptr;

static char serialBuffer[64];
static uint8_t serialReceived = 0;

static char bleBuffer[64];
static uint8_t bleReceived = 0;

static uint32_t _lastReport = 0;

static bool processCharacter(int c, char* buffer, uint8_t* count);

void initProtocol(SettingsManager& settings, SensorProvider& sensors, BanditController& controller) {
  if (!g_protocol) {
    g_protocol = new ProtocolHandler(dualOut, settings, sensors, controller);
  }
}

void processInput()
{
  if (!g_protocol) return;
  
  uint32_t now = millis();
  uint32_t period = g_protocol->getReportPeriod();

  if (period > 0 && now - _lastReport > period) {
    _lastReport = now;
    g_protocol->sendReport();
  }

  // Process Serial input
  if (Serial.available()) {
    int c = Serial.read();
    if (processCharacter(c, serialBuffer, &serialReceived)) {
      g_protocol->processCommand(serialBuffer);
      serialReceived = 0;
      serialBuffer[0] = 0;
    }
  }

  // Process BLE UART input (if connected)
  if (nrfuart.available()) {
    int c = nrfuart.read();
    if (processCharacter(c, bleBuffer, &bleReceived)) {
      g_protocol->processCommand(bleBuffer);
      bleReceived = 0;
      bleBuffer[0] = 0;
    }
  }
}

// Helper to process a single character into the command buffer
static bool processCharacter(int c, char* buffer, uint8_t* count)
{
  if (c == '\n') {
    buffer[(*count)++] = 0;
    return true;  // command complete
  } else if (*count < 63) {
    // Ignore carriage returns, white spaces and tabs
    if (c != '\r' && c != ' ' && c != '\t') {
      buffer[(*count)++] = c;
    }
  }
  return false;  // incomplete
}
