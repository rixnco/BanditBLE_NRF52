#include <Arduino.h>
#include "ProtocolHandler.h"

ProtocolHandler::ProtocolHandler(Print& out,
                                 SettingsManager& settings,
                                 SensorProvider& sensors,
                                 BanditController& controller)
  : m_out(out), m_settings(settings), m_sensors(sensors), m_controller(controller),
    m_reportPeriod(0), m_reportUseADC(false) {
}

// Get parameter ID from string name
static int getParamID(const char* str, char** endptr) {
  const char* PARAM_NAME[] = {
    "GEAR1", "GEAR2", "GEAR3", "GEAR4", "GEAR5", "GEAR6", "IMU"
  };
  
  for (int t = 0; t < 7; t++) {
    int l = strlen(PARAM_NAME[t]);
    if (strncmp(str, PARAM_NAME[t], l) == 0) {
      if (endptr != NULL) *endptr = (char*)str + l;
      return t;
    }
  }
  if (endptr != NULL) *endptr = (char*)str;
  return -1;
}

void ProtocolHandler::processCommand(const char* buffer) {
  if (buffer[0] == 0) return;

  switch (buffer[0]) {
    case '$':
      handleParamRequest(buffer);
      break;
    case '?':
      handleQueryRequest(buffer);
      break;
    case '!':
      handleOverrideRequest(buffer);
      break;
    default:
      sendError("Unknown command");
  }
}

void ProtocolHandler::handleParamRequest(const char* buffer) {
  const char* ptr = &buffer[1];
  
  switch (*ptr) {
    case '$': {
      ++ptr;
      if (*ptr == 0) {
        sendSettings();
        sendAck();
      } else {
        sendError("Invalid param command");
      }
      break;
    }
    case '<': {
      m_settings.save();
      m_out.println("!SETTINGS,stored");
      sendAck();
      break;
    }
    case '>': {
      if (!m_settings.load()) {
        m_settings.save();
        m_out.println("!SETTINGS,default");
      } else {
        m_out.println("!SETTINGS,restored");
      }
      sendAck();
      break;
    }
    default: {
      char* endptr;
      int p = getParamID(ptr, &endptr);
      
      if (p == -1) {
        // Check for special commands like IMUCLB
        if (strncmp(ptr, "IMUCLB", 6) == 0) {
          ptr += 6;
          if (*ptr == 0) {
            m_out.println("!IMU,calibrating...");
            m_controller.onCalibrateIMU();
            return;
          }
        }
        sendError("Unknown parameter");
        return;
      }
      
      if (*endptr == 0) {
        // Read parameter
        sendParam(p);
      } else if (*endptr == '=') {
        // Write parameter
        ++endptr;
        if (*endptr == 0 || !setParam(p, endptr)) {
          sendError("Invalid parameter value");
          return;
        }
      } else {
        sendError("Invalid parameter syntax");
        return;
      }
      sendAck();
    }
  }
}

void ProtocolHandler::handleQueryRequest(const char* buffer) {
  const char* ptr = &buffer[1];
  bool useADC = false;
  
  if (*ptr == '*') {
    useADC = true;
    ++ptr;
  }
  
  m_reportUseADC = useADC;
  
  if (*ptr == 0) {
    // Single report - will be sent by caller via sendReport()
    sendAck();
  } else {
    // Periodic report
    char* endptr;
    unsigned long period = strtoul(ptr, &endptr, 10);
    if (*endptr != 0) {
      sendError("Invalid period");
      return;
    }
    m_reportPeriod = period;
    sendAck();
  }
}

void ProtocolHandler::handleOverrideRequest(const char* buffer) {
  const char* ptr = &buffer[1];
  
  if (*ptr == '!') {
    ++ptr;
    if (*ptr == 0) {
      m_controller.onOverride(false, 0, 0);
      sendAck();
      return;
    }
  }
  
  // Parse format: "gear,rpm"
  char* endptr;
  unsigned long gear_ul = strtoul(ptr, &endptr, 10);
  
  if (*endptr != ',') {
    sendError("Invalid override format");
    return;
  }
  
  ptr = endptr + 1;
  unsigned long rpm_ul = strtoul(ptr, &endptr, 10);
  
  if (*endptr != 0) {
    sendError("Invalid override value");
    return;
  }
  
  m_controller.onOverride(true, (int)rpm_ul, (int)gear_ul);
  
  // Don't send ACK for override (handled differently)
}

bool ProtocolHandler::setParam(int p, const char* ptr) {
  char* endptr;
  
  switch (p) {
    case 0: case 1: case 2: case 3: case 4: case 5: {
      // GEAR1-6
      unsigned long ul = strtoul(ptr, &endptr, 10);
      if (*endptr != 0) return false;
      m_settings.setGear((uint8_t)(p + 1), (uint16_t)ul);
      break;
    }
    case 6: {
      // IMU - parse "x,y,z"
      long x = strtol(ptr, &endptr, 10);
      if (*endptr != ',') return false;
      
      ptr = endptr + 1;
      long y = strtol(ptr, &endptr, 10);
      if (*endptr != ',') return false;
      
      ptr = endptr + 1;
      long z = strtol(ptr, &endptr, 10);
      if (*endptr != 0) return false;
      
      m_settings.setImuOffsets((int16_t)x, (int16_t)y, (int16_t)z);
      break;
    }
    default:
      return false;
  }
  
  return true;
}

void ProtocolHandler::sendError(const char* msg) {
  m_out.print("!ERROR");
  if (msg != NULL) {
    m_out.print(",");
    m_out.println(msg);
  } else {
    m_out.println("");
  }
}

void ProtocolHandler::sendAck() {
  m_out.println("!OK");
}

void ProtocolHandler::sendParam(int p) {
  const char* PARAM_NAME[] = {
    "GEAR1", "GEAR2", "GEAR3", "GEAR4", "GEAR5", "GEAR6", "IMU"
  };
  
  m_out.print("$");
  m_out.print(PARAM_NAME[p]);
  m_out.print("=");
  
  switch (p) {
    case 0: case 1: case 2: case 3: case 4: case 5:
      m_out.println(m_settings.getGear((uint8_t)(p + 1)));
      break;
    case 6: {
      int16_t x, y, z;
      m_settings.getImuOffsets(x, y, z);
      m_out.print(x);
      m_out.print(",");
      m_out.print(y);
      m_out.print(",");
      m_out.println(z);
      break;
    }
    default:
      m_out.println("???");
  }
}

void ProtocolHandler::sendSettings() {
  for (int i = 0; i < 7; i++) {
    sendParam(i);
  }
}

void ProtocolHandler::setReportMode(bool useADC, uint32_t period) {
  m_reportUseADC = useADC;
  m_reportPeriod = period;
}

void ProtocolHandler::sendReport() {
  uint8_t  gear    = m_sensors.getCurrentGear();
  uint16_t rpm     = m_sensors.getCurrentRPM();
  uint16_t adc_raw = m_sensors.getCurrentGearRaw();

  m_out.print(">");
  if (m_reportUseADC) {
    m_out.print("*");
    m_out.print((int)adc_raw);
  } else {
    m_out.print((int)gear);
  }
  m_out.print(",");
  m_out.println((int)rpm);
}
