#include <Arduino.h>
#include <bluefruit.h>
#include "settings.h"
#include "protocol.h"

// Serial Protocol definition
#define PROTO_QUERY_REQ    '?'
#define PROTO_PARAM_REQ    '$'
#define PROTO_OVERRIDE_REQ '!'

// Output wrapper — sends to both Serial and BLE UART
#define OUTPUT_PRINTLN(x) do { Serial.println(x); if(nrfuart.available()) nrfuart.println(x); } while(0)
#define OUTPUT_PRINT(x)   do { Serial.print(x); if(nrfuart.available()) nrfuart.print(x); } while(0)
#define OUTPUT_WRITE(x)   do { Serial.write(x); if(nrfuart.available()) nrfuart.write(x); } while(0)

extern BLEUart nrfuart;


// Configuration parameters ID
// Used by the protocol handler
#define PARAM_GEAR1           0
#define PARAM_GEAR2           1
#define PARAM_GEAR3           2
#define PARAM_GEAR4           3
#define PARAM_GEAR5           4
#define PARAM_GEAR6           5
#define PARAM_LAST            6


static const char* PARAM_NAME[] = {
  "GEAR1",
  "GEAR2",
  "GEAR3",
  "GEAR4",
  "GEAR5",
  "GEAR6"
};



static char     serialBuffer[64];
static uint8_t  serialReceived=0;

static char     bleBuffer[64];
static uint8_t  bleReceived=0;

static uint32_t _reportPeriod=0;
static uint32_t _lastReport=0;

static int getParamID(const char* str, char** endptr);
static bool processCharacter(int c, char* buffer, uint8_t* count);
static void processCommand(const char* buffer);
static bool processParamRequest();
static void sendError(const char* msg);
static void sendAck();
static void sendParam(int p);
static void sendSettings();
static bool setParam(int p, const char* ptr);
static bool processQueryRequest();
static void sendReport();
static bool processOverrideRequest();


void processInput()
{
  uint32_t now = millis();

  if(_reportPeriod>0 && now-_lastReport>_reportPeriod) 
  {
    _lastReport=now;
    sendReport();
  }

  // Process Serial input
  if(Serial.available()) {
    int c = Serial.read();
    if(processCharacter(c, serialBuffer, &serialReceived)) {
      processCommand(serialBuffer);
      serialReceived = 0;
      serialBuffer[0] = 0;
    }
  }

  // Process BLE UART input (if connected)
  if(nrfuart.available()) {
    int c = nrfuart.read();
    if(processCharacter(c, bleBuffer, &bleReceived)) {
      processCommand(bleBuffer);
      bleReceived = 0;
      bleBuffer[0] = 0;
    }
  }
}

// Helper to process a single character into the command buffer
static bool processCharacter(int c, char* buffer, uint8_t* count)
{
  if(c == '\n') {
    buffer[(*count)++] = 0;
    return true;  // command complete
  } else if(*count < 63) {
    // Ignore carriage returns, white spaces and tabs
    if(c != '\r' && c != ' ' && c != '\t') {
      buffer[(*count)++] = c;
    }
  }
  return false;  // incomplete
}

// Process the complete command in buffer
static void processCommand(const char* buffer)
{
  if(buffer[0] == 0) return;
  
  switch(buffer[0]) {
    case PROTO_PARAM_REQ:
      if(!processParamRequest(buffer)) sendError("Invalid parameter request");
      else sendAck();  
      break;
    case PROTO_QUERY_REQ:
      if(!processQueryRequest(buffer)) sendError("Invalid query request");
      else sendAck();  
      break;
    case PROTO_OVERRIDE_REQ:
      if(!processOverrideRequest(buffer)) sendError("Invalid override request");
      break;
    default:
      sendError("Unknown command");
  }
}



/**
 * Get Parameter ID based on its name.
 */
static int getParamID(const char* str, char** endptr) {
  int l;
  int t;
  for(t=0; t<PARAM_LAST; ++t) {
    l= strlen(PARAM_NAME[t]);
    if(strncmp(str, PARAM_NAME[t], l)==0) {
      if(endptr!=NULL) *endptr=(char*)str+l;    
      break;
    }
  }
  if(t==PARAM_LAST) return -1;
  return t;
}



static bool processParamRequest(const char* buffer) 
{
  const char* ptr= &buffer[1];

  switch(*ptr) {
   case '$':
    ++ptr;
    if(*ptr==0) {
      sendSettings();
      return true;
    } 
    return false;
    break;
   case '<':
    // Store settings.
    writeSettings();
    OUTPUT_PRINTLN("!SETTINGS,stored");
    return true;
    break;
   case '>':
    // restore settings
    bool b;
    if(!(b=readSettings())) {
      resetSettings();
      writeSettings();
    }
    if(b) OUTPUT_PRINTLN("!SETTINGS,restored");
    else OUTPUT_PRINTLN("!SETTINGS,default");
    return true;
    break;
  default:
    int p= getParamID(ptr, (char**)&ptr);
    if(p==-1) return false;
    if(*ptr==0) {
      sendParam(p);
    } else if(*ptr=='=') {
      ++ptr;
      if(*ptr==0 || !setParam(p, ptr)) return false;
    }  
    return true;
  }  
  return false;
}

static void sendError(const char* msg)
{
  OUTPUT_PRINT("!ERROR");
  if(msg!=NULL) {
    OUTPUT_PRINT(",");
    OUTPUT_PRINTLN(msg);
  } else {
    OUTPUT_PRINTLN("");
  }
}

static void sendAck()
{
  OUTPUT_PRINTLN("!OK");
}


static void sendParam(int p) {
  if(p<0 || p>=PARAM_LAST) return;
  OUTPUT_PRINT("$");
  OUTPUT_PRINT(PARAM_NAME[p]);
  OUTPUT_PRINT("=");
  switch(p) {
    case PARAM_GEAR1: OUTPUT_PRINTLN(g_settings.gear1); break;
    case PARAM_GEAR2: OUTPUT_PRINTLN(g_settings.gear2); break;
    case PARAM_GEAR3: OUTPUT_PRINTLN(g_settings.gear3); break;
    case PARAM_GEAR4: OUTPUT_PRINTLN(g_settings.gear4); break;
    case PARAM_GEAR5: OUTPUT_PRINTLN(g_settings.gear5); break;
    case PARAM_GEAR6: OUTPUT_PRINTLN(g_settings.gear6); break;
    default: OUTPUT_PRINTLN("???");
  }
}

static void sendSettings() {
  for(int t=0; t<PARAM_LAST; ++t) {
    sendParam(t);
  }
}


static bool setParam(int p, const char* ptr) {
  unsigned long  ul;
  char* endptr;
  
  switch(p) {
    case PARAM_GEAR1: 
      ul= strtoul(ptr, &endptr, 10);
      if(*endptr!=0) return false;
      noInterrupts();
      g_settings.gear1=ul;
      interrupts();
      break;
    case PARAM_GEAR2: 
      ul= strtoul(ptr, &endptr, 10);
      if(*endptr!=0) return false;
      noInterrupts();
      g_settings.gear2=ul;
      interrupts();
      break;
    case PARAM_GEAR3: 
      ul= strtoul(ptr, &endptr, 10);
      if(*endptr!=0) return false;
      noInterrupts();
      g_settings.gear3=ul;
      interrupts();
      break;
    case PARAM_GEAR4: 
      ul= strtoul(ptr, &endptr, 10);
      if(*endptr!=0) return false;
      noInterrupts();
      g_settings.gear4=ul;
      interrupts();
      break;
    case PARAM_GEAR5: 
      ul= strtoul(ptr, &endptr, 10);
      if(*endptr!=0) return false;
      noInterrupts();
      g_settings.gear5=ul;
      interrupts();
      break;
    case PARAM_GEAR6: 
      ul= strtoul(ptr, &endptr, 10);
      if(*endptr!=0) return false;
      noInterrupts();
      g_settings.gear6=ul;
      interrupts();
      break;
    default:
      return false;
  }
  return true;
}

static bool processQueryRequest(const char* buffer) {
  unsigned long period;
  const char* ptr= &buffer[1];
  char* endptr;
  
  if(*ptr==0) {
    sendReport();
  } else {
    period= strtoul(ptr, &endptr, 10);
    if(*endptr!=0) return false;
    _reportPeriod=period;
  }
  return true;
}

static bool processOverrideRequest(const char* buffer) {
  const char* ptr= &buffer[1];
  char* endptr;
  unsigned long  ul;
  uint16_t rpm;
  uint8_t gear;

  switch(*ptr) {
   case '!':
    ++ptr;
    if(*ptr!=0) return false;
    setOverride(false, 0, 0);
    break;
  default:
    ul= strtoul(ptr, &endptr, 10);
    ptr = endptr;
    if(*ptr++!=',') return false;
    gear = ul;
    ul= strtoul(ptr, &endptr, 10);
    ptr = endptr;
    if(*ptr!=0) return false;
    rpm = ul;

    setOverride(true, rpm, gear);

  }

  return true;
}


extern uint8_t currentGear;
extern uint16_t currentRPM;


static void sendReport() 
{
  int rpm, gear;
  rpm = currentRPM;
  gear = currentGear;
  
  OUTPUT_PRINT(">");
  OUTPUT_PRINT(gear);
  OUTPUT_PRINT(",");
  OUTPUT_PRINTLN(rpm);
}


