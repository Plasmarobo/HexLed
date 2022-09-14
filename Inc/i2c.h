/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "opt_prototypes.h"
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN Private defines */

#define I2C_SUCCESS (0)
#define I2C_ERROR (1)

typedef void (*address_callback_t)(uint8_t, uint16_t);

/* USER CODE END Private defines */

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

/* USER CODE BEGIN Prototypes */
void i2c1_send(uint16_t address, uint8_t* buffer, uint32_t length, opt_callback_t cb);
void i2c2_send(uint8_t* buffer, uint32_t length, opt_callback_t cb);

void i2c1_receive(uint16_t address, uint8_t* buffer, uint32_t max_length, opt_callback_t cb);
void i2c2_receive(uint8_t* buffer, uint32_t max_length, opt_callback_t cb);

void i2c2_set_address_callback(address_callback_t cb);
void i2c2_set_listen_callback(opt_callback_t cb);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
