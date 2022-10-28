/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    i2c.h
 * @brief   This file contains all the function prototypes for
 *          the i2c.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "opt_prototypes.h"

#include <stdbool.h>
#include <stdint.h>
  /* USER CODE END Includes */

  extern I2C_HandleTypeDef hi2c1;

  extern I2C_HandleTypeDef hi2c2;

  /* USER CODE BEGIN Private defines */

#define I2C_SUCCESS (0)
#define I2C_TIMEOUT (-1)
#define I2C_ERROR (-2)
#define I2C_BUSY (-3)

typedef void (*address_callback_t)(uint8_t, uint16_t);

/* USER CODE END Private defines */

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);

/* USER CODE BEGIN Prototypes */
int i2c1_send(uint16_t address, uint8_t* buffer, uint32_t length, opt_callback_t cb);
int i2c2_send(uint8_t* buffer, uint32_t length, opt_callback_t cb);

int i2c1_receive(uint16_t address, uint8_t* buffer, uint32_t max_length, opt_callback_t cb);
int i2c2_receive(uint8_t* buffer, uint32_t max_length, opt_callback_t cb);

void i2c2_set_address_callback(address_callback_t cb);
void i2c2_set_listen_callback(opt_callback_t cb);

void i2c1_generate_nak(void);
void i2c2_generate_nak(void);

void i2c1_abort(void);
void i2c2_abort(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
