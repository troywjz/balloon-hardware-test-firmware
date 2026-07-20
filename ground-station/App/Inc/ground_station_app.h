#ifndef GROUND_STATION_APP_H
#define GROUND_STATION_APP_H

#include <stdint.h>

void GroundStationApp_EarlySafetyInit(void);
void GroundStationApp_OnUsbData(const uint8_t *data, uint32_t length);
void GroundStationApp_Run(void);

#endif
