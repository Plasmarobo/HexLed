/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>

/* USER CODE BEGIN 0 */
#define UART_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 8)
#define UART_TASK_PRIORITY (4)
// 63 characters + null terminator

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

static StaticQueue_t message_queue_impl;
uint8_t message_queue_buffer[MAX_MESSAGE_CONTENT_LENGTH * MAX_MESSAGE_QUEUE_LENGTH];
static QueueHandle_t message_queue;

static StackType_t stack_buffer[UART_TASK_STACK_SIZE];
static StaticTask_t tcb_buffer;
static TaskHandle_t uart_task;

void uart_task_handler(void* args)
{
  char message[MAX_MESSAGE_CONTENT_LENGTH];
  uint16_t bytes_to_send;
  for(;;)
  {
    xQueueReceive(message_queue,
                  &message,
                  portMAX_DELAY);
    bytes_to_send = strlen(message);
    if (bytes_to_send > MAX_MESSAGE_CONTENT_LENGTH)
    {
      bytes_to_send = MAX_MESSAGE_CONTENT_LENGTH;
    }
    if (HAL_OK != HAL_UART_Transmit_IT(&huart1, (uint8_t*)message, bytes_to_send))
    {
      // An error sending has occured, drop message and return
      // TODO: Notify... someone
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  message_queue = xQueueCreateStatic(MAX_MESSAGE_QUEUE_LENGTH,
                                     MAX_MESSAGE_CONTENT_LENGTH,
                                     message_queue_buffer,
                                     &message_queue_impl);

  uart_task = xTaskCreateStatic(uart_task_handler,
                                "uart_task",
                                UART_TASK_STACK_SIZE,
                                NULL,
                                UART_TASK_PRIORITY,
                                stack_buffer,
                                &tcb_buffer);
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspInit 1 */
    
    /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTaskNotifyFromISR(uart_task, 0, eNoAction, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  /* USER CODE END USART1_IRQn 1 */
}

void send_message(const char* message)
{
  xQueueSendToBack(message_queue,
                   message,
                   pdMS_TO_TICKS(100));
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
