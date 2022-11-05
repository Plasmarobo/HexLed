
#ifndef COMM_STACK_H
#define COMM_STACK_H

// Handles physical layer message transport for i2c

#include "comm_protocol.h"
#include "opt_prototypes.h"

#include <stdint.h>

#define MAX_MESSAGE_LENGTH (64)

// Public API
void comm_stack_multiplexer_interrupt_handler(void);
void comm_stack_init(void);

void comm_stack_signal_exec_complete(void);
void comm_stack_clear_exec_result(void);
void comm_stack_set_exec_result(protocol_message_t* msg);

void comm_stack_write(comm_port_t port, protocol_message_t* message, opt_callback_t operation_callback);
void comm_stack_read(comm_port_t port, protocol_message_t* message, opt_callback_t operation_callback);

comm_port_t controller_get_port(void);
comm_port_t responder_get_port(void);

#endif /* COMM_STACK_H */
