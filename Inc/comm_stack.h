
#ifndef COMM_STACK_H
#define COMM_STACK_H

// Handles physical layer message transport for i2c

#include "comm_protocol.h"

#include <stdint.h>

#define MAX_MESSAGE_LENGTH (64)

// Public API
void comm_stack_controller_interrupt_handler(void);
void comm_stack_init(void);
/**
 * @brief Runs a command from the comm stack
 *
 * @param command_packet Data from comm stack
 */
void exec_external_command(protocol_message_t command_packet);
/**
 * @brief Tells comm stack to report results if asked
 *
 */
void exec_complete(void);
/**
 * @brief Clear any cached exec results
 *
 */
void clear_exec_results(void);

#endif /* COMM_STACK_H */
