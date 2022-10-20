
#include "TCA9544APWR.h"

#include "comm_protocol.h"
#include "i2c.h"
#include "opt_prototypes.h"

#include <stdint.h>

#define CONTROL_REG_LENGTH (1)

// Lower bits are not fixed, but we've tied them to ground in Rev2
#define MULTIPLEXER_ADDRESS (0xE0)

static uint8_t        selected_channel;
static uint8_t        control_reg;
static uint8_t        cmd;
volatile opt_callback_t cb_cache;

// Private functions
void mp_i2c_callback(int32_t err, uintptr_t userdata)
{
  UNUSED(userdata);
  if (I2C_SUCCESS != err)
  {
    selected_channel = 0;
  }
  if (NULL != cb_cache)
  {
    cb_cache(err, 0);
  }
  cb_cache = NULL;
}

// Public functions

int set_channel(comm_port_t channel, opt_callback_t cb)
{
  if (NULL == cb_cache)
  {
    // Translate channel to enable mask
    cmd = CHANNEL_ENABLE;
    switch (channel)
    {
      case COMM_PORT_A:
        cmd |= CHANNEL0_SELECT;
        break;
      case COMM_PORT_B:
        cmd |= CHANNEL1_SELECT;
        break;
      case COMM_PORT_C:
        cmd |= CHANNEL2_SELECT;
        break;
      default:
        break;
    }
    cb_cache         = cb;
    selected_channel = channel;
    return i2c1_send(MULTIPLEXER_ADDRESS, &cmd, CONTROL_REG_LENGTH, mp_i2c_callback);
  }
  return I2C_BUSY;
}

int clear_channel(opt_callback_t cb)
{
  if (NULL == cb_cache)
  {
    selected_channel = 0;
    cb_cache         = cb;
    return i2c1_send(MULTIPLEXER_ADDRESS, &selected_channel, CONTROL_REG_LENGTH, mp_i2c_callback);
  }
  return I2C_BUSY;
}

uint8_t get_selected_channel(void)
{
  return selected_channel;
}

int update_channel_status(opt_callback_t cb)
{
  if (NULL == cb_cache)
  {
    cb_cache = cb;
    return i2c1_receive(MULTIPLEXER_ADDRESS, &control_reg, CONTROL_REG_LENGTH, mp_i2c_callback);
  }
  return I2C_BUSY;
}

uint8_t get_channel_status(void)
{
  // We only care about channels 0, 1, and 2
  return control_reg & CHANNEL_STATUS_MASK;
}

void reset_tca9544apwr_driver(void)
{
  cb_cache         = NULL;
  selected_channel = 0;
}