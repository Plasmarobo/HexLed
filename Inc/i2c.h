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

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);

/* USER CODE BEGIN Prototypes */
typedef enum
{
    I2C_REQUEST = 1,
    I2C_SUCCESS = 0,
    I2C_ERROR_UNKNOWN = -1,
    I2C_ERROR_TIMEOUT = -2,
    I2C_ERROR_BUSY = -3,
    I2C_RESET = -4,
}
i2c_status_t;

typedef void (*i2c_callback_t)(int status, uint8_t* rx_data, int rx_length);

// Initializes the I2C driver
void i2c_init(void);
// Sends a command and processes the reply
i2c_status_t i2c_send(uint8_t* tx_buffer, uint16_t tx_size, i2c_callback_t cb);
// Sets callback used to parse incoming commands
i2c_status_t i2c_listen(i2c_callback_t cb);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
