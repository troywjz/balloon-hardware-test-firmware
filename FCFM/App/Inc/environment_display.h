#ifndef ENVIRONMENT_DISPLAY_H
#define ENVIRONMENT_DISPLAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
  uint32_t sample_started_ms;
  bool humidity_valid;
  int32_t temperature_x100;
  uint32_t humidity_x100;
  bool ambient_valid;
  uint32_t ambient_pressure_x100;
  bool outer_valid;
  uint32_t outer_pressure_x100;
  bool inner_valid;
  uint32_t inner_pressure_x100;
} FlightBoardEnvironmentSample;

int FlightBoardEnvironment_Format(char *buffer,
                                  size_t size,
                                  const FlightBoardEnvironmentSample *sample,
                                  uint32_t age_ms,
                                  uint32_t sea_level_pressure_pa);

#endif
