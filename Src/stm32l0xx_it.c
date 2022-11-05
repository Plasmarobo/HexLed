/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm_stack.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef  hdma_spi2_tx;
extern DMA_HandleTypeDef  hdma_spi2_rx;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line 0 and line 1 interrupts.
 */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
 * @brief This function handles EXTI line 2 and line 3 interrupts.
 */
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */

  /* USER CODE END EXTI2_3_IRQn 0 */
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
 * @brief This function handles EXTI line 4 to 15 interrupts.
 */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 4, channel 5, channel 6 and channel 7 interrupts.
  */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 0 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 1 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
 */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */

  /* USER CODE END I2C1_IRQn 0 */
  if (hi2c1.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR))
  {
    HAL_I2C_ER_IRQHandler(&hi2c1);
  }
  else
  {
    HAL_I2C_EV_IRQHandler(&hi2c1);
  }
  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/**
 * @brief This function handles I2C2 global interrupt.
 */
void I2C2_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_IRQn 0 */

  /* USER CODE END I2C2_IRQn 0 */
  if (hi2c2.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR))
  {
    HAL_I2C_ER_IRQHandler(&hi2c2);
  }
  else
  {
    HAL_I2C_EV_IRQHandler(&hi2c2);
  }
  /* USER CODE BEGIN I2C2_IRQn 1 */

  /* USER CODE END I2C2_IRQn 1 */
}

/**
 * @brief This function handles SPI2 global interrupt.
 */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
