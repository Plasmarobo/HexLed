
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

typedef enum
{
    I2C_SUCCESS = 0,
    I2C_ERROR = -1,
    I2C_ERROR_TIMEOUT = -2,
    I2C_ERROR_BUSY = -3,
}
i2c_result_t;

// I2C Callback with the result of a transaction
typedef void (*i2c_callback_t)(int status, int rx_length, uint8_t* data);

// Public API
// Initializes the I2C driver
void i2c_init(void);
// Sends a command and processes the reply
int i2c_command(uint8_t* tx_buffer, uint16_t tx_size, i2c_callback_t cb);
// Writes to the slave reply buffer, should be called from listen callback
int i2c_reply(uint8_t* tx_buffer, uint16_t tx_size);
// Sets callback used to parse incoming commands
int i2c_listen(i2c_callback_t cb);

#endif /* COMM_STACK_H */
