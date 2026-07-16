#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

class SettingsManager;
class SensorProvider;
class BanditController;

void initProtocol(SettingsManager& settings, SensorProvider& sensors, BanditController& controller);
void processInput();

#endif
