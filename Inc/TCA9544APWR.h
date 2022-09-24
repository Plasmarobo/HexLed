

#ifndef TCA9544APWR_H
#define TCA9544APWR_H

#include "comm_protocol.h"
#include "opt_prototypes.h"

// Public Interface

#define CHANNEL0_SELECT (0x00)
#define CHANNEL1_SELECT (0x01)
#define CHANNEL2_SELECT (0x02)
#define CHANNEL_ENABLE (0x04)

#define CHANNEL0_STATUS (0x10)
#define CHANNEL1_STATUS (0x20)
#define CHANNEL2_STATUS (0x30)
#define CHANNEL_STATUS_MASK (CHANNEL0_STATUS | CHANNEL1_STATUS | CHANNEL2_STATUS)

int     set_channel(comm_port_t port, opt_callback_t cb);
int     clear_channel(opt_callback_t cb);
uint8_t get_selected_channel(void);
int     update_channel_status(opt_callback_t cb);
uint8_t get_channel_status(void);

#endif /* TCA9544APWR_H */
