
#include "TCA9544APWR.h"

#include "comm_stack.h"

#define CHANNEL0_SELECT (0x00)
#define CHANNEL1_SELECT (0x01)
#define CHANNEL2_SELECT (0x02)
#define CHANNEL3_SELECT (0x03)
#define CHANNEL_ENABLE (0x04)

// Lower bits are not fixed, but we've tied them to ground in Rev2
#define MUX_ADDRESS (0x70)

typedef enum
{
    MP_IDLE,
    MP_CHANNEL_SELECT,
    MP_SEND,
    MP_LISTEN,
    MP_RESET,
}
multiplexer_state_t;

typedef struct
{
    uint8_t int3 : 1;
    uint8_t int2 : 1;
    uint8_t int1 : 1;
    uint8_t int0 : 1;
    uint8_t reserved : 1;
    uint8_t channel_en : 1;
    uint8_t channel_sel : 2;
}
control_register_t;

static control_register_t control_reg;

// Private functions
int read_control_register(void);
int select_channel(uint8_t subchannel);

// Public Interface
int i2c_multiplexer_init(void)
{

}

int send_to_subchannel(int subchannel, uint16_t tx_len, uint8_t* data, i2c_callback_t reply_cb)
{

    int status = read_control_register();
    if (I2C_SUCCESS != status)
    {

    }
    status = select_channel(subchannel);
    if (I2C_SUCCESS != status)
    {
        return status;
    }

}

int read_control_register(void)
{

}

int select_channel(void)
{

}
