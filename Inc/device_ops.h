#ifndef DEVICE_OPS_H
#define DEVICE_OPS_H

#include "comm_protocol.h"

typedef enum
{
  // Basic system operations
  DEVICE_OP_NOOP,
  // TEST: Check if port is aligned. Change appropriate LEDs
  DEVICE_OP_CHECK_PORT,
  DEVICE_OP_SET_ADDRESS,
  DEVICE_OP_GET_ADDRESS,
  DEVICE_OP_SET_PATTERN,
  DEVICE_OP_GET_PATTERN,
  DEVICE_OP_SET_LED,
  DEVICE_OP_GET_LED,
  DEVICE_OP_RESET,
  // Networking operations
  DEVICE_OP_NOTIFY_ADDRESS,
  DEVICE_OP_INVALIDATE_ADDRESS,
  DEVICE_OP_TRACEROUTE,
} device_op_t;

void device_op_execute(protocol_message_t* device_op_data, protocol_message_t* response_buffer);

#endif /* DEVICE_OPS_H */