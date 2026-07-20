/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : fatfs_platform.c
  * @brief          : fatfs_platform source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
*/
/* USER CODE END Header */
#include "fatfs_platform.h"

/* Diagnostic override is volatile-only and returns to disabled after reset. */
static volatile uint8_t sd_force_present = 0U;

uint8_t BSP_PlatformRawDetectLevel(void)
{
    return HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) == GPIO_PIN_SET
               ? 1U
               : 0U;
}

void BSP_PlatformSetForcePresent(uint8_t enabled)
{
    sd_force_present = enabled != 0U ? 1U : 0U;
}

uint8_t BSP_PlatformIsForcePresentEnabled(void)
{
    return sd_force_present;
}

uint8_t	BSP_PlatformIsDetected(void) {
    uint8_t status = SD_PRESENT;
    if (sd_force_present != 0U)
    {
        return SD_PRESENT;
    }
    /* Check SD card detect pin */
    if(BSP_PlatformRawDetectLevel() != 0U)
    {
        status = SD_NOT_PRESENT;
    }
    /* USER CODE BEGIN 1 */
    /* user code can be inserted here */
    /* USER CODE END 1 */
    return status;
}
