/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
#include "Middlewares/Third_Party/FreeRTOS/Source/include/queue.h"
#include <stdint.h>

#define I2C_BUFFER_SIZE (256)

#define I2C_TASK_STACK_DEPTH (8)
#define I2C_TASK_PRIORITY (6)
#define I2C_QUEUE_DEPTH (2)

#define I2C_TIMEOUT_MS (500)

// States that a channel can eb in
typedef enum
{
    ICS_IDLE = 0, // Waiting for address
    ICS_BUSY,
} i2c_channel_state_t;

typedef struct
{
    int16_t write_length;
    uint8_t write_buffer[I2C_BUFFER_SIZE];
    int16_t read_length;
    uint8_t read_buffer[I2C_BUFFER_SIZE];
    i2c_channel_state_t state;
    TaskHandle_t task_id;
    StackType_t stack[I2C_TASK_STACK_DEPTH];
    StaticTask_t task;
    i2c_callback_t complete_callback;
    QueueHandle_t queue_id;
    uint8_t queue_buffer[I2C_QUEUE_DEPTH * sizeof(i2c_status_t)];
    StaticQueue_t queue;
    uint8_t address;
} i2c_channel_t;

static i2c_channel_t controller;
static i2c_channel_t responder;

static void i2c_controller_callback(I2C_HandleTypeDef *hi2c);
static void i2c_responder_callback(I2C_HandleTypeDef *hi2c);

static void i2c_controller_task_handler(void* userdata);
static void i2c_controller_task_handler(void* userdata);
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
    hi2c1.Init.Timing = 0x00200B2B;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
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
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, i2c_controller_callback);
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, i2c_controller_callback);

    controller.task_id = xTaskCreateStatic(i2c_controller_task_handler,
                    "i2c_controller_task_handler",
                    I2C_TASK_STACK_DEPTH,
                    NULL,
                    I2C_TASK_PRIORITY,
                    &controller.stack,
                    &controller.task);

    controller.queue_id = xQueueCreate(I2C_QUEUE_DEPTH,
                                sizeof(i2c_status_t),
                                controller.queue_buffer,
                                &controller.queue);
    /* USER CODE END I2C1_Init 2 */

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x00200B2B;
    hi2c2.Init.OwnAddress1 = 32;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
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
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_SLAVE_TX_COMPLETE_CB_ID, i2c2_slave_callback);
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_SLAVE_RX_COMPLETE_CB_ID, i2c2_slave_callback);
    HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_LISTEN_COMPLETE_CB_ID, i2c2_address_callback);

    HAL_I2C_EnableListen_IT(&hi2c2);

    responder.task_id = xTaskCreateStatic(responder.task,
                    "i2c_responder_task_handler",
                    I2C_TASK_STACK_DEPTH,
                    NULL,
                    I2C_TASK_PRIORITY,
                    &responder.stack,
                    &responder.task);

    responder.queue_id = xQueueCreate(I2C_QUEUE_DEPTH,
                                sizeof(i2c_status_t),
                                responder.queue_buffer,
                                &responder.queue);
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
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* I2C2 clock enable */
        __HAL_RCC_I2C2_CLK_ENABLE();

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

    if(i2cHandle->Instance == I2C1)
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
int i2c_controller_send(uint8_t* tx_buffer, uint16_t tx_size, i2c_callback_t cb)
{
    if (ICS_BUSY != controller.state)
    {
        memcpy(controller.write_buffer, tx_buffer, tx_size);
        controller.write_length = tx_size;
        controller.complete_callback = cb;
        vTaskNotifyGive(controller.task_id, NULL);
        return I2C_SUCCESS;
    }
    return I2C_ERROR_BUSY;
}

int i2c_responder_listen(i2c_callback_t cb)
{
    if (ICS_BUSY != responder.state)
    {
        responder.complete_callback = cb;
        return I2C_SUCCESS;
    }
    return I2C_ERROR_BUSY;
}

void notifyI2CTask(I2C_HandleTypeDef* hi2c, i2c_status_t status)
{
    BaseType_t high_priority_override = pdFALSE;
    QueueHandle_t queue;
    if (&hi2c1 == hi2c)
    {
        queue = controller.queue_id;
    }
    else if (&hi2c2 == hi2c)
    {
        queue = responder.queue_id;
    }
    xQueueSendFromISR(queue,
                      &status,
                      &high_priority_override);
    if (high_priority_override)
    {
        taskYIELD_FROM_ISR();
    }
}

void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c)
{
    notifyI2CTask(hi2c, I2C_SUCCESS);
}

void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c)
{
    notifyI2CTask(hi2c, I2C_ERROR_UNKNOWN);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    notifyI2CTask(hi2c, I2C_SUCCESS);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
   notifyI2CTask(hi2c, I2C_SUCCESS);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
   notifyI2CTask(hi2c, I2C_SUCCESS);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
   notifyI2CTask(hi2c, I2C_SUCCESS);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    notifyI2CTask(hi2c, I2C_SUCCESS);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
   notifyI2CTask(hi2c, I2C_ERROR_UNKNOWN);
}

static void reset_controller(void)
{
    notifyI2CTask(&hi2c1, I2C_SUCCESS);
}

static void reset_responder(void)
{
    notifyI2CTask(&hi2c2, I2C_SUCCESS);
}

static void controller_task_handler(void* userdata)
{
    // Loop task impl
    i2c_status_t status;
    for(;;)
    {
        // Wait for req to send
        status = xQueueReceive(controller.queue_id,
                                &status,
                                portMAX_DELAY);
        if (I2C_SUCCESS != status)
        {
            reset_i2c1();
            continue;
        }
        // Start TX
        HAL_I2C_Master_Transmit_DMA(&hi2c1,
                                       controller.address << 1,
                                       controller.write_buffer,
                                       controller.write_length);

        // Wait for TX
        status = I2C_ERROR_TIMEOUT;
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKSK(I2C_TIMEOUT_MS));
        if (I2C_SUCCESS != status)
        {
            reset_i2c1();
            continue;
        }
        // Start RX
        HAL_I2C_Master_Receive_DMA(&hi2c1,
                                      controller.address << 1,
                                      controller.read_buffer,
                                      sizeof(controller.read_buffer));

        // Wait for RX
        status = I2C_ERROR_TIMEOUT;
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKSK(I2C_TIMEOUT_MS));
        if (I2C_SUCCESS != status)
        {
            reset_i2c1();
            continue;
        }
        // Process RX
        if (NULL != controller.complete_callback)
        {
            controller.complete_callback(I2C_SUCCESS, sizeof(i2), i2c1_rx_buffer);
        }
    }
}

static void reset_i2c2(void)
{
    xTaskNotifyStateClear(slave_task_id);
}

static void responder_task_handler(void* userdata)
{
    int16_t reply_status;
    i2c_slave_state_t state;
    // Loop task impl
    for(;;)
    {
        state = ISS_IDLE;
        // Wait for address
        i2c2_status = I2C_ERROR_TIMEOUT;
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKSK(I2C_TIMEOUT_MS));
        if (I2C_SUCCESS != i2c2_status)
        {
            reset_i2c2();
            continue;
        }
        state = ISS_READ;
        // Start RX
        HAL_I2C_Slave_Receive_DMA(&hi2c2, i2c2_rx_buffer, sizeof(i2c2_rx_buffer));
        // Wait for RX
        i2c2_status = I2C_ERROR_TIMEOUT;
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKSK(I2C_TIMEOUT_MS));
        if (I2C_SUCCESS != i2c2_status)
        {
            reset_i2c2();
            continue;
        }
        state = ISS_REPLY;
        // Process RX
        if (NULL != slave_cb)
        {
            slave_cb(I2C_SUCCESS, sizeof(i2c2_rx_buffer), i2c2_rx_buffer);
        }
        // Reply/Acknowledge
        HAL_I2C_Slave_Transmit_DMA(&hi2c2, i2c2_tx_buffer, i2c2_write_size);
        i2c2_status = I2C_ERROR_TIMEOUT;
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKSK(I2C_TIMEOUT_MS));
        if (I2C_ERROR == i2c2_status)
        {
            reset_i2c2();
            continue;
        }
    }
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
