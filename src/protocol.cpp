#include <Arduino.h>
#include "settings.h"
#include "protocol.h"

// Serial Protocol definition
#define PROTO_QUERY_REQ    '?'
#define PROTO_PARAM_REQ    '$'
#define PROTO_OVERRIDE_REQ '!'


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



static char     inBuffer[64];
static uint8_t  received=0;
static uint32_t _reportPeriod=0;
static uint32_t _lastReport=0;

static int getParamID(const char* str, char** endptr);

static bool processParamRequest();
static void sendError(const char* msg);
static void sendAck();
static void sendParam(int p);
static void sendSettings();
static bool setParam(int p, char* ptr);

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


  if( !Serial.available() ) return;
   
  // Read Serial input.
  int c;
  c= Serial.read();
  if(c=='\n') {
    inBuffer[received++]=0;
    switch(inBuffer[0]) {
      case PROTO_PARAM_REQ:
        if(!processParamRequest()) sendError("Invalid parameter request");
        else sendAck();  
        break;
      case PROTO_QUERY_REQ:
        if(!processQueryRequest()) sendError("Invalid query request");
        else sendAck();  
        break;
      case PROTO_OVERRIDE_REQ:
        if(!processOverrideRequest()) sendError("Invalid override request");
        // else sendAck();  
        break;
      default:
        sendError("Unknown command");
    }
    // Command processed.
    // Reset buffer
    received=0;
    inBuffer[0]=0;
  } else if(received<63) {
    // Ignore carriage returns, white spaces and tabs
    if(c!='\r' && c!=' ' && c!='\t' ) inBuffer[received++]= c;
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



static bool processParamRequest() 
{
  char* ptr= &inBuffer[1];

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
    // noInterrupts();
    writeSettings();
    // interrupts();
    Serial.println("!SETTINGS,stored");
    return true;
    break;
   case '>':
    // restore settings
    bool b;
    if(!(b=readSettings())) {
      resetSettings();
      writeSettings();
    }
    if(b) Serial.println("!SETTINGS,restored");
    else Serial.println("!SETTINGS,default");
    return true;
    break;
  default:
    int p= getParamID(ptr, &ptr);
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
  Serial.print("!ERROR");
  if(msg!=NULL) {
    Serial.print(","); Serial.println(msg);
  } else {
    Serial.println(); 
  }
}

static void sendAck()
{
  Serial.println("!OK");
}


static void sendParam(int p) {
  if(p<0 || p>=PARAM_LAST) return;
  Serial.print("$");
  Serial.print(PARAM_NAME[p]);
  Serial.print("=");
  switch(p) {
    case PARAM_GEAR1: Serial.println(g_settings.gear1); break;
    case PARAM_GEAR2: Serial.println(g_settings.gear2); break;
    case PARAM_GEAR3: Serial.println(g_settings.gear3); break;
    case PARAM_GEAR4: Serial.println(g_settings.gear4); break;
    case PARAM_GEAR5: Serial.println(g_settings.gear5); break;
    case PARAM_GEAR6: Serial.println(g_settings.gear6); break;
    default: Serial.println("???");
  }
}

static void sendSettings() {
  for(int t=0; t<PARAM_LAST; ++t) {
    sendParam(t);
  }
}


static bool setParam(int p, char* ptr) {
  float f;
  unsigned long  ul;
  
  switch(p) {
    case PARAM_GEAR1: 
      ul= strtoul(ptr, &ptr, 10);
      if(*ptr!=0) return false;
      noInterrupts();
      g_settings.gear1=ul;
      interrupts();
      break;
    case PARAM_GEAR2: 
      ul= strtoul(ptr, &ptr, 10);
      if(*ptr!=0) return false;
      noInterrupts();
      g_settings.gear2=ul;
      interrupts();
      break;
    case PARAM_GEAR3: 
      ul= strtoul(ptr, &ptr, 10);
      if(*ptr!=0) return false;
      noInterrupts();
      g_settings.gear3=ul;
      interrupts();
      break;
    case PARAM_GEAR4: 
      ul= strtoul(ptr, &ptr, 10);
      if(*ptr!=0) return false;
      noInterrupts();
      g_settings.gear4=ul;
      interrupts();
      break;
    case PARAM_GEAR5: 
      ul= strtoul(ptr, &ptr, 10);
      if(*ptr!=0) return false;
      noInterrupts();
      g_settings.gear5=ul;
      interrupts();
      break;
    case PARAM_GEAR6: 
      ul= strtoul(ptr, &ptr, 10);
      if(*ptr!=0) return false;
      noInterrupts();
      g_settings.gear6=ul;
      interrupts();
      break;
    default:
      return false;
  }
  return true;
}

static bool processQueryRequest() {
  unsigned long period;
  char* ptr= &inBuffer[1];
  
  if(*ptr==0) {
    sendReport();
  } else {
    period= strtoul(ptr, &ptr, 10);
    if(*ptr!=0) return false;
    _reportPeriod=period;
  }
  return true;
}

static bool processOverrideRequest() {
  char* ptr= &inBuffer[1];
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
    ul= strtoul(ptr, &ptr, 10);
    if(*ptr++!=',') return false;
    gear = ul;
    ul= strtoul(ptr, &ptr, 10);
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
  
  Serial.print(">"); Serial.print(gear);
  Serial.print(","); Serial.print(rpm); 
  Serial.println();
}


