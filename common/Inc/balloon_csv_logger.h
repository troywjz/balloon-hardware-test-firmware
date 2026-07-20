#ifndef BALLOON_CSV_LOGGER_H
#define BALLOON_CSV_LOGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ff.h"

#define BALLOON_CSV_LOGGER_BUFFER_SIZE 1024U
#define BALLOON_CSV_LOGGER_PATH_SIZE   24U

typedef struct
{
  FIL file;
  uint8_t buffer[BALLOON_CSV_LOGGER_BUFFER_SIZE];
  size_t buffered_length;
  char path[BALLOON_CSV_LOGGER_PATH_SIZE];
  bool opened;
  FRESULT last_result;
  uint32_t last_sync_ms;
} BalloonCsvLogger;

void BalloonCsvLogger_Reset(BalloonCsvLogger *logger);
bool BalloonCsvLogger_OpenNext(BalloonCsvLogger *logger,
                               const char *drive,
                               const char *prefix,
                               const char *header,
                               uint32_t now_ms);
bool BalloonCsvLogger_OpenAt(BalloonCsvLogger *logger,
                             const char *drive,
                             const char *prefix,
                             unsigned int sequence,
                             const char *header,
                             uint32_t now_ms);
bool BalloonCsvLogger_Write(BalloonCsvLogger *logger,
                            const void *data,
                            size_t length);
bool BalloonCsvLogger_Service(BalloonCsvLogger *logger,
                              uint32_t now_ms,
                              uint32_t sync_period_ms);
bool BalloonCsvLogger_Close(BalloonCsvLogger *logger);

#endif
