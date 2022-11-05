/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    i2c.c
 * @brief   This file provides code for the configuration
 *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

#include "FreeRTOS.h"

#include "comm_protocol.h"
#include "task.h"
#include "timers.h"

#include "assert.h"
#include "stm32l0xx_hal.h"

static opt_callback_t     i2c1_rx_callback;
static opt_callback_t     i2c1_tx_callback;
static opt_callback_t     i2c2_rx_callback;
static opt_callback_t     i2c2_tx_callback;
static address_callback_t i2c2_address_callback;
static opt_callback_t     i2c2_listen_callback;

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing           = 0x00200C2D;
  hi2c1.Init.OwnAddress1      = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  hi2c1.State                 = HAL_I2C_STATE_RESET;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance              = I2C2;
  hi2c2.Init.Timing           = 0x00200C2D;
  hi2c2.Init.OwnAddress1      = (COMM_PROTOCOL_BASE_ADDRESS << 1);
  hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_ENABLE;
  hi2c2.Init.OwnAddress2      = (COMM_PROTOCOL_ADDRESS_TWO << 1);
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_MASK01; // Mask out the lowest bit (twos bit if you include r/w)
  hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  hi2c2.State = HAL_I2C_STATE_RESET;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLED) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  i2c1_rx_callback      = NULL;
  i2c1_tx_callback      = NULL;
  i2c2_rx_callback      = NULL;
  i2c2_tx_callback      = NULL;
  i2c2_address_callback = NULL;
  i2c2_listen_callback  = NULL;
  /* USER CODE END I2C2_Init 2 */
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PA9     ------> I2C1_SCL
    PA10     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Channel3;
    hdma_i2c1_rx.Init.Request = DMA_REQUEST_6;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Channel2;
    hdma_i2c1_tx.Init.Request = DMA_REQUEST_6;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c1_tx);

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
    /* USER CODE BEGIN I2C1_MspInit 1 */

    /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();

    /* I2C2 DMA Init */
    /* I2C2_RX Init */
    hdma_i2c2_rx.Instance = DMA1_Channel5;
    hdma_i2c2_rx.Init.Request = DMA_REQUEST_7;
    hdma_i2c2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c2_rx);

    /* I2C2_TX Init */
    hdma_i2c2_tx.Instance = DMA1_Channel4;
    hdma_i2c2_tx.Init.Request = DMA_REQUEST_7;
    hdma_i2c2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c2_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c2_tx);

    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(I2C2_IRQn);
    /* USER CODE BEGIN I2C2_MspInit 1 */

    /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PA9     ------> I2C1_SCL
    PA10     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    /* I2C2 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C2_IRQn);
  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef* hi2c, uint8_t direction, uint16_t address_match_code)
{
  if (hi2c == &hi2c2)
  {
    HAL_I2C_DisableListen_IT(hi2c);
    if (NULL != i2c2_address_callback)
    {
      i2c2_address_callback(direction, address_match_code >> 1);
    }
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    if (NULL != i2c1_tx_callback)
    {
      i2c1_tx_callback(I2C_SUCCESS, 0);
      i2c1_tx_callback = NULL;
    }
  }
  else if (hi2c == &hi2c2)
  {
    if (NULL != i2c2_tx_callback)
    {
      i2c2_tx_callback(I2C_SUCCESS, 0);
      i2c2_tx_callback = NULL;
    }
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    if (NULL != i2c1_rx_callback)
    {
      i2c1_rx_callback(I2C_SUCCESS, 0);
      i2c1_rx_callback = NULL;
    }
  }
  else if (hi2c == &hi2c2)
  {
    if (NULL != i2c2_rx_callback)
    {
      i2c2_rx_callback(I2C_SUCCESS, 0);
      i2c2_rx_callback = NULL;
    }
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    if (NULL != i2c1_tx_callback)
    {
      i2c1_tx_callback(I2C_SUCCESS, 0);
      i2c1_tx_callback = NULL;
    }
  }
  else if (hi2c == &hi2c2)
  {
    if (NULL != i2c2_tx_callback)
    {
      i2c2_tx_callback(I2C_SUCCESS, 0);
      i2c2_tx_callback = NULL;
    }
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  if (hi2c == &hi2c1)
  {
    if (NULL != i2c1_rx_callback)
    {
      i2c1_rx_callback(I2C_SUCCESS, 0);
      i2c1_rx_callback = NULL;
    }
  }
  else if (hi2c == &hi2c2)
  {
    if (NULL != i2c2_rx_callback)
    {
      i2c2_rx_callback(I2C_SUCCESS, 0);
      i2c2_rx_callback = NULL;
    }
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
  if (hi2c == &hi2c1)
  {
    if (NULL != i2c1_tx_callback)
    {
      i2c1_tx_callback(hi2c->ErrorCode, 0);
    }
    if (NULL != i2c1_rx_callback)
    {
      i2c1_rx_callback(hi2c->ErrorCode, 0);
    }
  }
  if (hi2c == &hi2c2)
  {
    // IF the master doesn't send as many bytes as we expect,
    // we will throw a tantrum in the form of an AF error.
    // Check if the interface is busy, if it is not, ignore the
    // error and assume the message is in storage
    int i2c_status = hi2c->ErrorCode;
    if ((hi2c->ErrorCode == HAL_I2C_ERROR_AF) && (hi2c->State != HAL_I2C_STATE_BUSY))
    {
      // Override error code: we just have a shorter message than maximum
      i2c_status = I2C_SUCCESS;
    }
    else
    {
      __HAL_I2C_GENERATE_NACK(&hi2c2);
    }
    if (NULL != i2c2_tx_callback)
    {
      i2c2_tx_callback(i2c_status, 0);
    }
    if (NULL != i2c2_rx_callback)
    {
      i2c2_rx_callback(i2c_status, 0);
    }
  }
}

int i2c1_send(uint16_t address, uint8_t* buffer, uint32_t length, opt_callback_t cb)
{
  int result = I2C_SUCCESS;
  if (HAL_OK == HAL_I2C_Master_Transmit_DMA(&hi2c1, address, buffer, length))
  {
    i2c1_tx_callback = cb;
  }
  else
  {
    result = I2C_ERROR;
  }
  return result;
}

int i2c2_send(uint8_t* buffer, uint32_t length, opt_callback_t cb)
{
  int result = I2C_SUCCESS;
  if (HAL_OK == HAL_I2C_Slave_Transmit_DMA(&hi2c2, buffer, length))
  {
    i2c2_tx_callback = cb;
  }
  else
  {
    result = I2C_ERROR;
  }
  return result;
}

int i2c1_receive(uint16_t address, uint8_t* buffer, uint32_t max_length, opt_callback_t cb)
{
  int result = I2C_SUCCESS;
  if (HAL_OK == HAL_I2C_Master_Receive_DMA(&hi2c1, address, buffer, max_length))
  {
    i2c1_rx_callback = cb;
  }
  else
  {
    result = I2C_ERROR;
  }
  return result;
}

int i2c2_receive(uint8_t* buffer, uint32_t max_length, opt_callback_t cb)
{
  int result = I2C_SUCCESS;
  if (HAL_OK == HAL_I2C_Slave_Receive_DMA(&hi2c2, buffer, max_length))
  {
    i2c2_rx_callback = cb;
  }
  else
  {
    result = I2C_ERROR;
  }
  return result;
}

void i2c2_listen(void)
{
  HAL_I2C_EnableListen_IT(&hi2c2);
}

void i2c2_set_address_callback(address_callback_t cb)
{
  i2c2_address_callback = cb;
}

void i2c2_set_listen_callback(opt_callback_t cb)
{
  i2c2_listen_callback = cb;
}

void i2c2_generate_nak(void)
{
  __HAL_I2C_GENERATE_NACK(&hi2c2);
}

void i2c1_abort(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  HAL_I2C_DMAStop(&hi2c1);
  HAL_I2C_DeInit(&hi2c1); // Release pins, reset handle status flag

  __HAL_RCC_I2C2_FORCE_RESET();
  __HAL_RCC_I2C2_RELEASE_RESET();

  GPIO_InitStruct.Pin   = GPIO_PIN_9 | GPIO_PIN_10; // This line is original
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      // GPIO is configured as output
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;          // Strong pull up

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Drive SCL Low
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  // Drive SDA High
  vTaskDelay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  vTaskDelay(20);
  for (uint8_t i = 0; i < 12; ++i)
  {
    // Clock out 12 cycles to clear bus
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    vTaskDelay(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    vTaskDelay(10);
  }
  // Re-enable I2C
  HAL_I2C_Init(&hi2c1);
}

void i2c2_abort(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  HAL_I2C_DMAStop(&hi2c2);
  HAL_I2C_DeInit(&hi2c2); // Release pins, reset handle status flag

  __HAL_RCC_I2C2_FORCE_RESET();
  __HAL_RCC_I2C2_RELEASE_RESET();

  GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11; // This line is original
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;       // GPIO is configured as output
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;           // Strong pull up

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Drive SCL
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  // Drive SDA High
  vTaskDelay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
  // Re-enable I2C
  HAL_I2C_Init(&hi2c1);
}

/* USER CODE END 1 */
