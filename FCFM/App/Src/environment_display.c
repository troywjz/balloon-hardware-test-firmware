#include "environment_display.h"

#include <math.h>
#include <stdio.h>

static void FormatHundredths(char *buffer, size_t size, int32_t value, bool valid)
{
  uint32_t magnitude;

  if (!valid)
  {
    (void)snprintf(buffer, size, "NA");
    return;
  }
  magnitude = value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
  (void)snprintf(buffer, size, "%s%lu.%02lu",
                 value < 0 ? "-" : "",
                 (unsigned long)(magnitude / 100U),
                 (unsigned long)(magnitude % 100U));
}

static void FormatPressureKpa(char *buffer, size_t size, int64_t pressure_x100, bool valid)
{
  uint64_t magnitude;
  uint64_t whole_pascals;

  if (!valid)
  {
    (void)snprintf(buffer, size, "NA");
    return;
  }
  magnitude = pressure_x100 < 0 ? (uint64_t)-pressure_x100 : (uint64_t)pressure_x100;
  whole_pascals = (magnitude + 50U) / 100U;
  (void)snprintf(buffer, size, "%s%lu.%03lu",
                 pressure_x100 < 0 && whole_pascals != 0U ? "-" : "",
                 (unsigned long)(whole_pascals / 1000U),
                 (unsigned long)(whole_pascals % 1000U));
}

static void FormatAltitude(char *buffer,
                           size_t size,
                           uint32_t pressure_x100,
                           uint32_t sea_level_pressure_pa,
                           bool valid)
{
  float altitude_m;
  int32_t altitude_x10;
  uint32_t magnitude;

  if (!valid || pressure_x100 == 0U || sea_level_pressure_pa == 0U)
  {
    (void)snprintf(buffer, size, "NA");
    return;
  }

  altitude_m = 44330.0f *
      (1.0f - powf((float)pressure_x100 / (100.0f * (float)sea_level_pressure_pa),
                   0.190294957f));
  altitude_x10 = (int32_t)(altitude_m * 10.0f + (altitude_m < 0.0f ? -0.5f : 0.5f));
  magnitude = altitude_x10 < 0 ? (uint32_t)(-(int64_t)altitude_x10) : (uint32_t)altitude_x10;
  (void)snprintf(buffer, size, "%s%lu.%01lu",
                 altitude_x10 < 0 ? "-" : "",
                 (unsigned long)(magnitude / 10U),
                 (unsigned long)(magnitude % 10U));
}

int FlightBoardEnvironment_Format(char *buffer,
                                  size_t size,
                                  const FlightBoardEnvironmentSample *sample,
                                  uint32_t age_ms,
                                  uint32_t sea_level_pressure_pa)
{
  char temperature[20];
  char humidity[20];
  char altitude[20];
  char ambient[20];
  char outer[20];
  char outer_delta[20];
  char inner[20];
  char inner_delta[20];
  char age[20];

  if (buffer == NULL || sample == NULL || size == 0U)
  {
    return -1;
  }

  FormatHundredths(temperature, sizeof(temperature),
                   sample->temperature_x100, sample->humidity_valid);
  FormatHundredths(humidity, sizeof(humidity),
                   (int32_t)sample->humidity_x100, sample->humidity_valid);
  FormatAltitude(altitude, sizeof(altitude), sample->ambient_pressure_x100,
                 sea_level_pressure_pa, sample->ambient_valid);
  FormatPressureKpa(ambient, sizeof(ambient), sample->ambient_pressure_x100,
                    sample->ambient_valid);
  FormatPressureKpa(outer, sizeof(outer), sample->outer_pressure_x100,
                    sample->outer_valid);
  FormatPressureKpa(outer_delta, sizeof(outer_delta),
                    (int64_t)sample->outer_pressure_x100 - sample->ambient_pressure_x100,
                    sample->outer_valid && sample->ambient_valid);
  FormatPressureKpa(inner, sizeof(inner), sample->inner_pressure_x100,
                    sample->inner_valid);
  FormatPressureKpa(inner_delta, sizeof(inner_delta),
                    (int64_t)sample->inner_pressure_x100 - sample->outer_pressure_x100,
                    sample->inner_valid && sample->outer_valid);
  if (age_ms == UINT32_MAX)
  {
    (void)snprintf(age, sizeof(age), "NA");
  }
  else
  {
    (void)snprintf(age, sizeof(age), "%lu", (unsigned long)age_ms);
  }

  return snprintf(buffer, size,
                  "FC env temperature_c=%s humidity_rh=%s altitude_m=%s "
                  "ambient_kpa=%s outer_kpa=%s outer_minus_ambient_kpa=%s "
                  "inner_kpa=%s inner_minus_outer_kpa=%s age_ms=%s\r\n",
                  temperature, humidity, altitude, ambient, outer, outer_delta,
                  inner, inner_delta, age);
}
