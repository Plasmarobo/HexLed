#ifndef NETWORK_STACK_H
#define NETWORK_STACK_H

#include <stdint.h>

// Grid network organized into rows/columns

typedef struct
{
  uint16_t row;
  uint16_t col;
} network_address_t;

typedef enum
{
  COMMAND_NOOP = 0x80,
  COMMAND_READ_ID,
  COMMAND_WRITE_ID,
  COMMAND_READ_LED,
  COMMAND_WRITE_LED,
} network_operation_t;

// Initializes network and starts the task
void network_init(void);
void network_send(network_address_t destination, uint8_t* data, uint16_t length);
// Get our current local address
network_address_t network_get_address(void);
// Returns previous address
network_address_t network_set_address(network_address_t new_address);

#endif /* NETWORK_STACK_H */