
#ifndef COMM_STACK_H
#define COMM_STACK_H

// Handles physical layer message transport for i2c

#include <stdint.h>

#define MAX_MESSAGE_LENGTH (64)

// Public API
void comm_stack_controller_interrupt_handler(void);
void comm_stack_init(void);

#endif /* COMM_STACK_H */
