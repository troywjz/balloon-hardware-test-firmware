#ifndef FLIGHT_BOARD_TEST_H
#define FLIGHT_BOARD_TEST_H

#include <stdint.h>

void FlightBoardTest_EarlySafetyInit(void);
void FlightBoardTest_OnUsbData(const uint8_t *data, uint32_t length);
void FlightBoardTest_Run(void);

#endif
