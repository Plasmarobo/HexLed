
#ifndef COMM_STACK_H
#define COMM_STACK_H

#include <stdint.h>

#define MAX_MESSAGE_LENGTH (256)

// Input and output channels
typedef enum
{
    CHANNEL_AO,
    CHANNEL_BO,
    CHANNEL_CO,
    CHANNEL_AI,
    CHANNEL_BI,
    CHANNEL_CI,
    CHANNEL_MAX
}
i2c_channel_t;

// A structure containing information needed to send a message
typedef struct
{
    uint8_t address;
    uint8_t message_length;
    uint8_t tx_buffer[MAX_MESSAGE_LENGTH];
    uint8_t rx_buffer[MAX_MESSAGE_LENGTH];
}
i2c_command_t;

// I2C Callback with the result of a transaction
typedef void (*i2c_callback_t)(int status);

// Public API
void i2c_init(void);
int i2c_transact(i2c_command_t* msg, i2c_callback_t cb);

#endif /* COMM_STACK_H */
