
#include "comm_stack.h"

#include "i2c.h"
#include "Drivers/STM32L0xx_HAL_Driver/Inc/stm32l0xx_hal_i2c.h"
#include "Middlewares/Third_Party/FreeRTOS/Source/include/task.h"

#include <stdint.h>

#define I2C_TX_BUFFER_SIZE (256)
#define I2C_RX_BUFFER_SIZE (256)

#define I2C_TASK_STACK_DEPTH (8)
#define I2C_TASK_PRIORITY (6)

#define I2C_TIMEOUT_MS (500)

typedef enum
{
    ISS_IDLE = 0, // Waiting for address
    ISS_READ,
    ISS_REPLY,
} i2c_slave_state_t;

static int16_t i2c1_write_size;
static uint8_t i2c1_tx_buffer[I2C_TX_BUFFER_SIZE];
static int16_t i2c1_read_size;
static uint8_t i2c1_rx_buffer[I2C_RX_BUFFER_SIZE];

static int16_t i2c2_write_size;
static uint8_t i2c2_tx_buffer[I2C_TX_BUFFER_SIZE];
static int16_t i2c2_read_size;
static uint8_t i2c2_rx_buffer[I2C_RX_BUFFER_SIZE];

static void i2c1_master_callback(I2C_HandleTypeDef *hi2c);
static void i2c2_slave_callback(I2C_HandleTypeDef *hi2c);
static void i2c2_address_callback(I2C_HandleTypeDef *hi2c);

static void i2c_master_handler(void* userdata);
static void i2c_slave_handler(void* userdata);

static StackType_t slave_stack[I2C_TASK_STACK_DEPTH];
static StackType_t master_stack[I2C_TASK_STACK_DEPTH];

static StaticTask_t slave_task;
static StaticTask_t master_task;

static TaskHandle_t slave_task_id;
static TaskHandle_t master_task_id;

static int8_t target_address;

static volatile int16_t i2c1_status;
static volatile int16_t i2c2_status;

static i2c_callback_t master_cb;
static i2c_callback_t slave_cb;

void i2c_init(void)
{
    // Init I2C controller
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, i2c1_master_callback);
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, i2c1_master_callback);

    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_SLAVE_TX_COMPLETE_CB_ID, i2c2_slave_callback);
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_SLAVE_RX_COMPLETE_CB_ID, i2c2_slave_callback);
    HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_LISTEN_COMPLETE_CB_ID, i2c2_address_callback);

    HAL_I2C_EnableListen_IT(&hi2c2);

    master_cb = NULL;
    target_address = 0;

    slave_task_id = xTaskCreateStatic(i2c_slave_handler,
                      "i2c_slave_handler",
                      I2C_TASK_STACK_DEPTH,
                      NULL,
                      I2C_TASK_PRIORITY,
                      &slave_stack,
                      &slave_task);

    master_task_id = xTaskCreateStatic(i2c_master_handler,
                      "i2c_master_handler",
                      I2C_TASK_STACK_DEPTH,
                      NULL,
                      I2C_TASK_PRIORITY,
                      &master_stack,
                      &master_task);

}

int i2c_transact(uint8_t* tx_buffer, uint16_t tx_size, i2c_callback_t cb)
{
    memcpy(i2c1_tx_buffer, tx_buffer, tx_size);
    i2c1_write_size = tx_size;
    master_cb = cb;
    vTaskNotifyGive(master_task_id, NULL);
    return I2C_SUCCESS;
}

int i2c_listen(i2c_callback_t cb)
{
    slave_cb = cb;
}

void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c)
{
    if (&hi2c1 == hi2c)
    {
        i2c1_status = I2C_SUCCESS;
        vTaskNotifyGiveFromISR(master_task_id, NULL);
    }
    else if (&hi2c2 == hi2c)
    {
        i2c2_status = I2C_SUCCESS;
        vTaskNotifyGiveFromISR(slave_task_id, NULL);
    }
}

void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c)
{
    if (&hi2c1 == hi2c)
    {
        i2c1_status = I2C_ERROR;
        vTaskNotifyGiveFromISR(master_task_id, NULL);
    }
    else if (&hi2c2 == hi2c)
    {
        i2c2_status = I2C_ERROR;
        vTaskNotifyGiveFromISR(slave_task_id, NULL);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c2_status = I2C_SUCCESS;
    vTaskNotifyGiveFromISR(master_task_id, NULL);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c2_status = I2C_SUCCESS;
    vTaskNotifyGiveFromISR(master_task_id, NULL);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c2_status = I2C_SUCCESS;
    vTaskNotifyGiveFromISR(slave_task_id, NULL);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c2_status = I2C_SUCCESS;
    vTaskNotifyGiveFromISR(slave_task_id, NULL);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    i2c2_status = I2C_SUCCESS;
    vTaskNotifyGiveFromISR(slave_task_id, NULL);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    i2c2_status = I2C_ERROR;
    vTaskNotifyGiveFromISR(slave_task_id, NULL);
}

static void reset_i2c1(void)
{
    //Noop
    xTaskNotifyStateClear(master_task_id);
}

static void master_task_handler(void* userdata)
{
    // Loop task impl
    for(;;)
    {
        // Wait for req to send
        i2c1_status = I2C_ERROR_TIMEOUT;
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKSK(I2C_TIMEOUT_MS));
        if (I2C_SUCCESS != i2c1_status)
        {
            reset_i2c1();
            continue;
        }
        // Start TX
        HAL_I2C_Master_Transmit_DMA(&hi2c1,
                                       target_address << 1,
                                       i2c1_tx_buffer,
                                       i2c1_write_size);

        // Wait for TX
        i2c1_status = I2C_ERROR_TIMEOUT;
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKSK(I2C_TIMEOUT_MS));
        if (I2C_SUCCESS != i2c1_status)
        {
            reset_i2c1();
            continue;
        }
        // Start RX
        HAL_I2C_Master_Receive_DMA(&hi2c1,
                                      target_address << 1,
                                      i2c1_rx_buffer,
                                      sizeof(i2c1_rx_buffer));

        // Wait for RX
        i2c1_status = I2C_ERROR_TIMEOUT;
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKSK(I2C_TIMEOUT_MS));
        if (I2C_SUCCESS != i2c1_status)
        {
            reset_i2c1();
            continue;
        }
        // Process RX
        if (NULL != master_cb)
        {
            master_cb(I2C_SUCCESS, sizeof(i2c1_rx_buffer), i2c1_rx_buffer);
        }
    }
}

static void reset_i2c2(void)
{
    xTaskNotifyStateClear(slave_task_id);
}

static void slave_task_handler(void* userdata)
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
