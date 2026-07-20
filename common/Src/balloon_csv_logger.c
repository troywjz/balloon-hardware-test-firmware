#include "balloon_csv_logger.h"

#include <stdio.h>
#include <string.h>

static bool BalloonCsvLogger_Flush(BalloonCsvLogger *logger)
{
  UINT bytes_written = 0U;

  if ((logger == NULL) || !logger->opened)
  {
    if (logger != NULL)
    {
      logger->last_result = FR_INVALID_OBJECT;
    }
    return false;
  }

  if (logger->buffered_length == 0U)
  {
    return true;
  }

  logger->last_result = f_write(&logger->file,
                                logger->buffer,
                                (UINT)logger->buffered_length,
                                &bytes_written);
  if ((logger->last_result != FR_OK) ||
      (bytes_written != (UINT)logger->buffered_length))
  {
    if (logger->last_result == FR_OK)
    {
      logger->last_result = FR_DISK_ERR;
    }
    return false;
  }

  logger->buffered_length = 0U;
  return true;
}

static bool BalloonCsvLogger_IsPrefixValid(const char *prefix)
{
  size_t index;

  if ((prefix == NULL) || (strlen(prefix) != 3U))
  {
    return false;
  }

  for (index = 0U; index < 3U; ++index)
  {
    char value = prefix[index];

    if (!(((value >= 'A') && (value <= 'Z')) ||
          ((value >= '0') && (value <= '9')) ||
          (value == '_')))
    {
      return false;
    }
  }
  return true;
}

static bool BalloonCsvLogger_FormatPath(char *output,
                                        size_t output_size,
                                        const char *drive,
                                        const char *prefix,
                                        unsigned int sequence)
{
  size_t drive_length;
  const char *separator;
  int result;

  if ((output == NULL) || (output_size == 0U) || (drive == NULL))
  {
    return false;
  }

  drive_length = strlen(drive);
  separator = ((drive_length > 0U) &&
               (drive[drive_length - 1U] != '/') &&
               (drive[drive_length - 1U] != '\\'))
                  ? "/"
                  : "";
  result = snprintf(output,
                    output_size,
                    "%s%s%s%04u.CSV",
                    drive,
                    separator,
                    prefix,
                    sequence);
  return (result > 0) && ((size_t)result < output_size);
}

void BalloonCsvLogger_Reset(BalloonCsvLogger *logger)
{
  if (logger == NULL)
  {
    return;
  }

  memset(logger, 0, sizeof(*logger));
  logger->last_result = FR_OK;
}

bool BalloonCsvLogger_OpenAt(BalloonCsvLogger *logger,
                             const char *drive,
                             const char *prefix,
                             unsigned int sequence,
                             const char *header,
                             uint32_t now_ms)
{
  if ((logger == NULL) || (drive == NULL) ||
      !BalloonCsvLogger_IsPrefixValid(prefix) || logger->opened ||
      (sequence < 1U) || (sequence > 9999U))
  {
    if (logger != NULL)
    {
      logger->last_result = FR_INVALID_PARAMETER;
    }
    return false;
  }

  logger->path[0] = '\0';
  logger->buffered_length = 0U;
  if (!BalloonCsvLogger_FormatPath(logger->path,
                                   sizeof(logger->path),
                                   drive,
                                   prefix,
                                   sequence))
  {
    logger->last_result = FR_INVALID_NAME;
    return false;
  }

  logger->last_result = f_open(&logger->file,
                                logger->path,
                                FA_CREATE_NEW | FA_WRITE);
  if (logger->last_result != FR_OK)
  {
    return false;
  }

  logger->opened = true;
  logger->last_sync_ms = now_ms;
  if ((header != NULL) && (header[0] != '\0') &&
      (!BalloonCsvLogger_Write(logger, header, strlen(header)) ||
       !BalloonCsvLogger_Service(logger, now_ms, 0U)))
  {
    FRESULT write_result = logger->last_result;

    (void)f_close(&logger->file);
    logger->opened = false;
    logger->buffered_length = 0U;
    logger->last_result = write_result;
    return false;
  }
  return true;
}

bool BalloonCsvLogger_OpenNext(BalloonCsvLogger *logger,
                               const char *drive,
                               const char *prefix,
                               const char *header,
                               uint32_t now_ms)
{
  unsigned int sequence;

  if ((logger == NULL) || (drive == NULL) ||
      !BalloonCsvLogger_IsPrefixValid(prefix) || logger->opened)
  {
    if (logger != NULL)
    {
      logger->last_result = FR_INVALID_PARAMETER;
    }
    return false;
  }

  logger->path[0] = '\0';
  logger->buffered_length = 0U;
  for (sequence = 1U; sequence <= 9999U; ++sequence)
  {
    if (!BalloonCsvLogger_FormatPath(logger->path,
                                     sizeof(logger->path),
                                     drive,
                                     prefix,
                                     sequence))
    {
      logger->last_result = FR_INVALID_NAME;
      return false;
    }

    if (BalloonCsvLogger_OpenAt(logger,
                                drive,
                                prefix,
                                sequence,
                                header,
                                now_ms))
    {
      return true;
    }

    if (logger->last_result != FR_EXIST)
    {
      return false;
    }
  }

  logger->last_result = FR_EXIST;
  return false;
}

bool BalloonCsvLogger_Write(BalloonCsvLogger *logger,
                            const void *data,
                            size_t length)
{
  const uint8_t *bytes = (const uint8_t *)data;

  if ((logger == NULL) || !logger->opened ||
      ((length > 0U) && (data == NULL)))
  {
    if (logger != NULL)
    {
      logger->last_result = (data == NULL) ? FR_INVALID_PARAMETER
                                           : FR_INVALID_OBJECT;
    }
    return false;
  }

  while (length > 0U)
  {
    size_t available = sizeof(logger->buffer) - logger->buffered_length;
    size_t chunk = (length < available) ? length : available;

    memcpy(&logger->buffer[logger->buffered_length], bytes, chunk);
    logger->buffered_length += chunk;
    bytes += chunk;
    length -= chunk;

    if ((logger->buffered_length == sizeof(logger->buffer)) &&
        !BalloonCsvLogger_Flush(logger))
    {
      return false;
    }
  }

  logger->last_result = FR_OK;
  return true;
}

bool BalloonCsvLogger_Service(BalloonCsvLogger *logger,
                              uint32_t now_ms,
                              uint32_t sync_period_ms)
{
  if ((logger == NULL) || !logger->opened)
  {
    if (logger != NULL)
    {
      logger->last_result = FR_INVALID_OBJECT;
    }
    return false;
  }

  if ((uint32_t)(now_ms - logger->last_sync_ms) < sync_period_ms)
  {
    return true;
  }

  if (!BalloonCsvLogger_Flush(logger))
  {
    return false;
  }
  logger->last_result = f_sync(&logger->file);
  if (logger->last_result != FR_OK)
  {
    return false;
  }

  logger->last_sync_ms = now_ms;
  return true;
}

bool BalloonCsvLogger_Close(BalloonCsvLogger *logger)
{
  FRESULT first_result = FR_OK;
  FRESULT result;

  if ((logger == NULL) || !logger->opened)
  {
    if (logger != NULL)
    {
      logger->last_result = FR_INVALID_OBJECT;
    }
    return false;
  }

  if (!BalloonCsvLogger_Flush(logger))
  {
    first_result = logger->last_result;
  }
  if (first_result == FR_OK)
  {
    result = f_sync(&logger->file);
    if (result != FR_OK)
    {
      first_result = result;
    }
  }

  result = f_close(&logger->file);
  if ((first_result == FR_OK) && (result != FR_OK))
  {
    first_result = result;
  }

  logger->opened = false;
  logger->buffered_length = 0U;
  logger->last_result = first_result;
  return first_result == FR_OK;
}
