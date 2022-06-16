
#include "comm_stack.h"

#include "i2c.h"
#include "Drivers/STM32L0xx_HAL_Driver/Inc/stm32l0xx_hal_i2c.h"

#define I2C_TRANSFER_QUEUE_SIZE (32)

typedef struct {
    hal_i2c_transfer_t* current_transfer;
    hal_i2c_controller_t* controller;
    I2C_HandleTypeDef *hi2c;
} i2c_context_t;

static DMA_Data_t DMAData;

static hal_i2c_controller_t i2c1_controller;
static hal_i2c_transfer_t i2c1_transfer_buffer[I2C_TRANSFER_QUEUE_SIZE];
static i2c_context_t i2c1_ctx;

static hal_i2c_controller_t i2c2_controller;
static hal_i2c_transfer_t i2c2_transfer_buffer[I2C_TRANSFER_QUEUE_SIZE];
static i2c_context_t i2c2_ctx;


static void transfer_notify(hal_i2c_controller_t* controller);
static inline void i2c_master_callback_core(i2c_context_t* context);

static void i2c1_master_callback(I2C_HandleTypeDef *hi2c);
static void i2c2_slave_callback(I2C_HandleTypeDef *hi2c);

typedef enum {
    I2C_DEVICE1 = 0,
    I2C_DEVICE2 = 1,
} i2c_device_t;

void i2c_init(void)
{
    // Init I2C controller
    i2c1_ctx.current_transfer = NULL;
    i2c1_ctx.controller = &i2c1_controller;
    i2c1_ctx.hi2c = &hi2c1;

    i2c1_controller->controller_data = (uintptr_t)&i2c1_ctx;
    i2c1_controller->transfer_notify = transfer_notify;

    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, i2c1_master_callback);
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, i2c1_master_callback);

    i2c2_ctx.current_transfer = NULL;
    i2c2_ctx.controller = &i2c2_controller;
    i2c2_ctx.hi2c = &hi2c2;

    i2c2_controller->controller_data = (uintptr_t)&i2c2_ctx;
    i2c2_controller->transfer_notify = transfer_notify;

    HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_LISTEN_COMPLETE_CB_ID, i2c2_slave_callback);
}

int i2c_transact(i2c_command_t* msg, i2c_callback_t cb)
{

}

void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c)
{

}
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{

}

static inline void i2c_master_callback_core(i2c_context_t* context)
{
    bool isr_state;

    hal_i2c_transfer_complete(context->current_transfer);

    isr_state = hal_interrupt_disable();
    hal_i2c_transfer_t* transfer = (hal_i2c_transfer_t*)list_take_first(&context->controller->transfer_list);
    hal_interrupt_enable(isr_state);

    if(transfer != NULL)
    {
        if(transfer->direction == HAL_I2C_READ)
        {
            HAL_I2C_Master_Receive_DMA(context->hi2c,
                                      transfer->device->address << 1,
                                      transfer->buffer,
                                      transfer->length);
        }
        else if(transfer->direction == HAL_I2C_WRITE)
        {
            HAL_I2C_Master_Transmit_DMA(context->hi2c,
                                       transfer->device->address << 1,
                                       transfer->buffer,
                                       transfer->length);
        }
    }

    context->current_transfer = transfer;
}

static inline void i2c_slave_callback_core(i2c_context_t* context)
{
    // Process the data and relpy


}

static void i2c1_master_callback(I2C_HandleTypeDef *hi2c)
{
    UNUSED(hi2c);
    i2c_master_callback_core(&i2c1_ctx);
}

static void i2c2_slave_callback(I2C_HandleTypeDef *hi2c)
{
    UNUSED(hi2c);
    i2c_slave_callback_core(&i2c2_ctx);
}
