#include "environment_display.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void RequireField(const char *output, const char *field)
{
  if (strstr(output, field) == NULL)
  {
    fprintf(stderr, "missing field: %s\noutput: %s\n", field, output);
    assert(false);
  }
}

int main(void)
{
  FlightBoardEnvironmentSample sample = {
      .humidity_valid = true,
      .temperature_x100 = 2001,
      .humidity_x100 = 4918U,
      .ambient_valid = true,
      .ambient_pressure_x100 = 9734407U,
      .outer_valid = true,
      .outer_pressure_x100 = 9724289U,
      .inner_valid = true,
      .inner_pressure_x100 = 9724500U,
  };
  char output[256];
  int length;

  length = FlightBoardEnvironment_Format(output, sizeof(output), &sample, 150U, 101325U);
  assert(length > 0 && (size_t)length < sizeof(output));
  RequireField(output, "temperature_c=20.01 humidity_rh=49.18");
  RequireField(output, "altitude_m=336.8");
  RequireField(output, "ambient_kpa=97.344 outer_kpa=97.243");
  RequireField(output, "outer_minus_ambient_kpa=-0.101");
  RequireField(output, "inner_kpa=97.245 inner_minus_outer_kpa=0.002");
  RequireField(output, "age_ms=150\r\n");

  sample.outer_valid = false;
  (void)FlightBoardEnvironment_Format(output, sizeof(output), &sample, 100U, 101325U);
  RequireField(output, "outer_kpa=NA outer_minus_ambient_kpa=NA");
  RequireField(output, "inner_kpa=97.245 inner_minus_outer_kpa=NA");
  RequireField(output, "ambient_kpa=97.344");

  sample.outer_valid = true;
  sample.ambient_valid = false;
  sample.humidity_valid = false;
  (void)FlightBoardEnvironment_Format(output, sizeof(output), &sample, UINT32_MAX, 101325U);
  RequireField(output, "temperature_c=NA humidity_rh=NA altitude_m=NA");
  RequireField(output, "ambient_kpa=NA outer_kpa=97.243 outer_minus_ambient_kpa=NA");
  RequireField(output, "inner_minus_outer_kpa=0.002 age_ms=NA");

  sample.ambient_valid = true;
  sample.ambient_pressure_x100 = 10132500U;
  (void)FlightBoardEnvironment_Format(output, sizeof(output), &sample, 0U, 101325U);
  RequireField(output, "altitude_m=0.0");
  sample.ambient_pressure_x100 = 10500000U;
  (void)FlightBoardEnvironment_Format(output, sizeof(output), &sample, 0U, 101325U);
  RequireField(output, "altitude_m=-");
  (void)FlightBoardEnvironment_Format(output, sizeof(output), &sample, 0U, 0U);
  RequireField(output, "altitude_m=NA");

  assert(FlightBoardEnvironment_Format(NULL, sizeof(output), &sample, 0U, 101325U) < 0);
  assert(FlightBoardEnvironment_Format(output, sizeof(output), NULL, 0U, 101325U) < 0);
  puts("environment display tests passed");
  return 0;
}
