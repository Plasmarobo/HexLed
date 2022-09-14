

#ifndef TCA9544APWR_H
#define TCA9544APWR_H

#include "comm_stack.h"

// Public Interface
int i2c_multiplexer_init(void);
int send_to_subchannel(int subchannel, uint16_t tx_len, uint8_t* data, i2c_callback_t reply_cb);

#endif /* TCA9544APWR_H */
