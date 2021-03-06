/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"

#include "FreeRTOS.h"
#include "timers.h"

/* USER CODE BEGIN 0 */

static void reload_watchdog(TimerHandle_t tm);

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;
TimerHandle_t watchdog_timer_id;
StaticTimer_t watchdog_timer;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */
	if( (RCC->CSR & RCC_CSR_WWDGRSTF) != 0)
  {
		// TODO: Note the reset
		RCC->CSR = RCC->CSR | RCC_CSR_RMVF;
	}
	
  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  // Aproximately 500ms for watchdog timeout
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095; // Windowed mode disabled
  hiwdg.Init.Reload = 4095; 
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  // Start a thread to pet the watchdog
  
  watchdog_timer_id = xTimerCreateStatic(
    "watchdog_timer",
    pdMS_TO_TICKS(1000),
    pdTRUE,
    0,
    reload_watchdog,
    &watchdog_timer);

  if (NULL != watchdog_timer_id)
  {
    xTimerStart(watchdog_timer_id,
                pdMS_TO_TICKS(0));
  }

  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */
static void reload_watchdog(TimerHandle_t tm)
{
  UNUSED(tm);
	HAL_IWDG_Refresh(&hiwdg);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
