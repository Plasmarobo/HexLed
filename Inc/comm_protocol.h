#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define COMM_PROTOCOL_MAX_MESSAGE_LENGTH (64)
#define COMM_PROTOCOL_MESSAGE_HEADER_LENGTH (2)

#define COMM_PROTOCOL_HEADER_INDEX (0)
#define COMM_PROTOCOL_LENGTH_INDEX (1)
#define COMM_PROTOCOL_DATA_START (2)

#define COMM_PROTOCOL_BASE_ADDRESS (0x2E)
// The lowest bits of the mask address are all ack'd
// For example if our mask address is 0x80, we will ack 0x80 and 0x82
// Note: remember "odd" addresses are reserved for read-write signals
// In this case, addresses 0x2E-0x33 are our addresses
#define COMM_PROTOCOL_MASK_ADDRESS (0x30)
// Codify addresses here for convenience:
#define COMM_PROTOCOL_ADDRESS_TWO (0x30)
#define COMM_PROTOCOL_ADDRESS_THREE (0x32)
// This address is the first invalid address after the base + mask block
#define COMM_PROTOCOL_MAX_ADDRESS (0x34)

#define INVALID_ID (0xFFFFFFFF)

typedef uint32_t device_id_t;

typedef struct
{
  uint8_t header; // Status or target address
  uint8_t length;
  uint8_t data[COMM_PROTOCOL_MAX_MESSAGE_LENGTH];
} protocol_message_t;

typedef enum
{
  COMM_PORT_A    = 0,
  COMM_PORT_B    = 1,
  COMM_PORT_C    = 2,
  COMM_PORT_MAX  = 3,
  COMM_PORT_NONE = 4,
  COMM_PORT_ALL  = 5,
} comm_port_t;

typedef struct
{
  comm_port_t        port;
  protocol_message_t message;
} message_data_t;

typedef enum
{
  // Response 0x00 - 0x7F
  RESPONSE_SUCCESS = 0x00,
  RESPONSE_ERROR_TIMEOUT,
  RESPONSE_ERROR_REJECTED,
  RESPONSE_ERROR_INVALID_OPERATION,
  RESPONSE_ERROR_INVALID_PARAMETER,
  RESPONSE_ERROR_ID_UNKNOWN,
  RESPONSE_CONFIGURATION_REQUIRED,
  // Command 0x80 - 0xFF
  // SYSTEM
  COMMAND_GET_ID = 0x80,
  COMMAND_SET_ID,
  COMMAND_RESET,
  COMMAND_TIME_SYNC,
  COMMAND_WHOAMI,
  // LED
  COMMAND_SET_LED,
  COMMAND_SET_ALL_LEDS,
  COMMAND_SET_PATTERN,
  // NETWORK
  COMMAND_LIST_CONNECTED,
  COMMAND_TRACEROUTE,
  COMMAND_SEND_TO,
  COMMAND_CLEAR_ROUTING_TABLE,
  COMMAND_SET_LOCATION,
  COMMAND_UPDATE_LOCATION,
  // DATA
  COMMAND_READ_MESSAGE,
  COMMAND_WRITE_MESSAGE,
} header_data_t;

void        set_device_id(device_id_t id);
device_id_t get_device_id(void);

#endif // COMM_PROTOCOL_H