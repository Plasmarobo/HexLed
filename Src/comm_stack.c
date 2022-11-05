#include "comm_stack.h"

#include "FreeRTOS.h"

#include "TCA9544APWR.h"
#include "comm_protocol.h"
#include "device_ops.h"
#include "display.h"
#include "gpio.h"
#include "i2c.h"
#include "opt_prototypes.h"
#include "priorities.h"
#include "queue.h"
#include "serial_output.h"
#include "stm32l0xx_hal_i2c.h"
#include "task.h"
#include "timers.h"

#include <stdint.h>
#include <string.h>

#define COMM_STACK_STARTUP_DELAY_MS (200)
#define COMM_STACK_MULTIPLEXER_TIMEOUT_MS (2000)
#define COMM_STACK_YIELD_TIME_MS (25)
#define COMM_STACK_DEFAULT_TIMEOUT_MS (25000)
#define COMM_STACK_MSG_TIMEOUT_MS (25)
#define COMM_STACK_POLL_INTERVAL (15)
#define COMM_STACK_PEND_TICKS (1)

#define ERROR_RESET_MS (10)

#define RESPONDER_TDMA_PERIOD_MS (100)

#define CONNECTED_INTERRUPTS_MASK (0x7)

#define MESSAGE_QUEUE_MAX_LENGTH (8)

#define COMM_PORT_READY (GPIO_PIN_RESET)
#define COMM_PORT_NOT_READY (GPIO_PIN_SET)

#define CONTROLLER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define RESPONDER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)

#define RESPONDER_ACTION_WRITE (0)
#define RESPONDER_ACTION_READ (1)

typedef struct
{
  GPIO_TypeDef* port;
  uint32_t      pin;
} responder_select_t;

typedef enum
{
  CONTROLLER_IDLE = 0,
  CONTROLLER_READ_MULTIPLEXER,
  CONTROLLER_SET_MULTIPLEXER,
  CONTROLLER_WRITE_HEADER,
  CONTROLLER_WRITE_BODY,
  CONTROLLER_READ_HEADER,
  CONTROLLER_PROCESS_HEADER,
  CONTROLLER_PROCESS_BODY,
  CONTROLLER_RESET_MULTIPLEXER,
} controller_state_t;

typedef enum
{
  COMM_ERROR_NONE = 0,
  COMM_ERROR_DEQUEUE,
  COMM_ERROR_I2C,
  COMM_ERROR_RESPONSE_CODE,
} comm_error_t;

typedef enum
{
  RESPONDER_IDLE = 0,
  RESPONDER_READ_HEADER,
  RESPONDER_READ_BODY,
  RESPONDER_PROCESS_MESSAGE,
  RESPONDER_BUSY,
  RESPONDER_WRITE_HEADER,
  RESPONDER_WRITE_BODY,
  RESPONDER_TRANSACTION_COMPLETE,
  RESPONDER_ERROR,
} responder_state_t;

typedef struct
{
  StaticQueue_t queue_impl;
  uint8_t       queue_buffer[MESSAGE_QUEUE_MAX_LENGTH * sizeof(protocol_operation_t)];
  QueueHandle_t queue;
} port_queue_t;

static protocol_operation_t controller_op_cache;

static port_queue_t controller_message_queue[COMM_PORT_MAX];

static controller_state_t controller_state;
static comm_port_t        controller_port;

static responder_state_t  responder_state;
static comm_port_t          responder_port;
static protocol_message_t responder_message_cache;
static uint8_t              responder_action;

static uint16_t input_channel_addresses[COMM_PORT_MAX] = {
  (COMM_PROTOCOL_BASE_ADDRESS << 1),
  (COMM_PROTOCOL_ADDRESS_TWO << 1),
  (COMM_PROTOCOL_ADDRESS_THREE << 1),
};

static void controller_handler(void* userdata, uint32_t userdata2);
static void controller_handle_error(uint32_t error);
static void responder_handler(void* userdata, uint32_t userdata2);
static void responder_handle_error(uint32_t error);

// Private functions

// Handles i2c interrupts for the controller
void comm_stack_controller_i2c_handler(int32_t err, uintptr_t user)
{
  UNUSED(user);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Advance the state of the controller
  if (I2C_SUCCESS != err)
  {
    controller_handle_error(COMM_ERROR_I2C);
  }
  else
  {
    ++controller_state;
  }
  xTimerPendFunctionCall(controller_handler, NULL, 0, COMM_STACK_PEND_TICKS);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Handles the i2c tx/rx complete
void comm_stack_responder_i2c_handler(int32_t err, uintptr_t user)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  responder_action = RESPONDER_TRANSACTION_COMPLETE;
  // Advance the state of the responder
  if (I2C_SUCCESS != err)
  {
    responder_handle_error(COMM_ERROR_I2C);
  }
  else
  {
    ++responder_state;
  }
  xTimerPendFunctionCall(responder_handler, NULL, 0, COMM_STACK_PEND_TICKS);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

uint8_t responder_address_to_port(uint16_t address)
{
  switch (address)
  {
    case COMM_PROTOCOL_BASE_ADDRESS:
      {
        return COMM_PORT_A;
      }
      break;
    case COMM_PROTOCOL_ADDRESS_TWO:
      {
        return COMM_PORT_B;
      }
      break;
    case COMM_PROTOCOL_ADDRESS_THREE:
      {
        return COMM_PORT_C;
      }
      break;
    default:
      break;
  }
  return COMM_PORT_NONE;
}

static void handle_i2c2_address(uint8_t direction, uint16_t address)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  responder_action                    = direction;
  if (RESPONDER_IDLE != responder_state)
  {
    // We shouldn't get here, super bad state
    responder_state = RESPONDER_ERROR;
    responder_handle_error(COMM_ERROR_I2C);
    // xTimerPendFunctionCall(responder_handler, NULL, I2C_SUCCESS, COMM_STACK_PEND_TICKS);
  }
  else
  {
    responder_port  = responder_address_to_port(address);
    responder_state = RESPONDER_READ_HEADER;
    if (COMM_PORT_NONE != responder_port)
    {
      xTimerPendFunctionCall(responder_handler, NULL, I2C_SUCCESS, COMM_STACK_PEND_TICKS);
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int controller_send_header(void)
{
  return i2c1_send(input_channel_addresses[controller_port],
                   (uint8_t*)&controller_op_cache.message,
                   COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                   comm_stack_controller_i2c_handler);
}

int controller_send_body(void)
{
  return i2c1_send(input_channel_addresses[controller_port],
                   controller_op_cache.message.data,
                   controller_op_cache.message.length,
                   comm_stack_controller_i2c_handler);
}

int controller_update_multiplexer(void)
{
  return update_channel_status(comm_stack_controller_i2c_handler);
}

int controller_set_port(void)
{
  return set_channel(controller_port, comm_stack_controller_i2c_handler);
}

int controller_read_header(void)
{
  return i2c1_receive(input_channel_addresses[controller_port],
                      (uint8_t*)&controller_op_cache.message,
                      COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                      comm_stack_controller_i2c_handler);
}

int controller_read_body(void)
{
  return i2c1_receive(input_channel_addresses[controller_port],
                      controller_op_cache.message.data,
                      controller_op_cache.message.length,
                      comm_stack_controller_i2c_handler);
}

int responder_send_header(void)
{
  return i2c2_send((uint8_t*)&responder_message_cache, COMM_PROTOCOL_MESSAGE_HEADER_LENGTH, comm_stack_controller_i2c_handler);
}

int responder_send_body(void)
{
  return i2c2_send(responder_message_cache.data, responder_message_cache.length, comm_stack_controller_i2c_handler);
}

int responder_read_header(void)
{
  return i2c2_receive((uint8_t*)&responder_message_cache, COMM_PROTOCOL_MESSAGE_HEADER_LENGTH, comm_stack_responder_i2c_handler);
}

int responder_read_body(void)
{
  return i2c2_receive(responder_message_cache.data, responder_message_cache.length, comm_stack_responder_i2c_handler);
}

int responder_reply(header_data_t code)
{
  return i2c2_send((uint8_t*)&responder_message_cache, COMM_PROTOCOL_MESSAGE_HEADER_LENGTH, comm_stack_responder_i2c_handler);
}

void controller_handle_error(uint32_t error_code)
{
  // Log error, reset I2C
  switch (controller_state)
  {
    case CONTROLLER_IDLE:
    case CONTROLLER_READ_MULTIPLEXER:
    case CONTROLLER_SET_MULTIPLEXER:
      // Critical error
      // Reset peripheral
      break;
    case CONTROLLER_WRITE_HEADER:
    case CONTROLLER_WRITE_BODY:
    case CONTROLLER_READ_HEADER:
    case CONTROLLER_PROCESS_HEADER:
    case CONTROLLER_PROCESS_BODY:
      // Timeout or rejection, return to idle
      i2c1_abort();
      controller_state = CONTROLLER_IDLE;
      break;
    case CONTROLLER_RESET_MULTIPLEXER:
      // Critical error
      // Reset peripheral
    default:
      break;
  }
  if (NULL != controller_op_cache.callback)
  {
    controller_op_cache.callback(error_code, 0);
    controller_op_cache.callback = NULL;
  }
}

void controller_handler(void* userdata, uint32_t userdata2)
{
  UNUSED(userdata);
  UNUSED(userdata2);
  switch (controller_state)
  {
    case CONTROLLER_IDLE:
      break;
      // We have been trigger by a falling edge from the multiplexer
    case CONTROLLER_READ_MULTIPLEXER:
      controller_update_multiplexer();
      break;
    case CONTROLLER_SET_MULTIPLEXER:
      if (get_channel_status() == 0)
      {
        // We cannot send anything, and we haven't set the MP yet. Return to idle
        controller_state = CONTROLLER_IDLE;
        break;
      }
      if (uxQueueMessagesWaiting(controller_message_queue[controller_port].queue) == 0)
      {
        // Nothing to dequeue, no reason to interact with the multiplexer
        controller_state = CONTROLLER_IDLE;
        break;
      }
      controller_set_port();
      break;
    case CONTROLLER_WRITE_HEADER:
      if (pdFALSE == xQueueReceive(controller_message_queue[controller_port].queue,
                                   &controller_op_cache,
                                   pdMS_TO_TICKS(COMM_STACK_MSG_TIMEOUT_MS)))
      {
        // Could not dequeue message... set error
        controller_handle_error(COMM_ERROR_DEQUEUE);
        break;
      }
      controller_send_header();
      break;
    case CONTROLLER_WRITE_BODY:
      controller_send_body();
      break;
    case CONTROLLER_READ_HEADER:
      controller_read_header();
      break;
    case CONTROLLER_PROCESS_HEADER:
      if (RESPONSE_BUSY == controller_op_cache.message.header)
      {
        // Poll the target
        controller_state = CONTROLLER_READ_HEADER;
        xTimerPendFunctionCall(responder_handler, NULL, 0, pdMS_TO_TICKS(COMM_STACK_POLL_INTERVAL));
      }
      else if (RESPONSE_SUCCESS == controller_op_cache.message.header)
      {
        if (controller_op_cache.message.length > 0)
        {
          // TODO: send message to higher level stack
          controller_state = CONTROLLER_RESET_MULTIPLEXER;
          xTimerPendFunctionCall(controller_handler, NULL, 0, COMM_STACK_PEND_TICKS);
        }
        else
        {
          controller_read_body();
        }
      }
      else
      {
        controller_handle_error(COMM_ERROR_RESPONSE_CODE);
      }
      break;
    case CONTROLLER_PROCESS_BODY:
      // TODO: send message to higher level stack
      controller_state = CONTROLLER_RESET_MULTIPLEXER;
      xTimerPendFunctionCall(controller_handler, NULL, 0, COMM_STACK_PEND_TICKS);
      break;
    case CONTROLLER_RESET_MULTIPLEXER:
      controller_port = COMM_PORT_NONE;
      controller_set_port();
      if (NULL != controller_op_cache.callback)
      {
        controller_op_cache.callback(I2C_SUCCESS, 0);
        controller_op_cache.callback = NULL;
      }
      controller_state = CONTROLLER_IDLE;
      break;
    default:
      controller_state = CONTROLLER_IDLE;
      break;
  }
}

void responder_handle_error(uint32_t error)
{
  UNUSED(error);
  i2c2_generate_nak();
  i2c2_abort();
  i2c2_listen();
}

void responder_handler(void* userdata, uint32_t userdata2)
{
  UNUSED(userdata);
  UNUSED(userdata2);
  switch (responder_state)
  {
    case RESPONDER_IDLE:
      // TDMA request?
      break;
    case RESPONDER_READ_HEADER:
      if (RESPONDER_ACTION_READ == responder_action)
      {
        // Abort the transaction...
        responder_message_cache.header = RESPONSE_SUCCESS;
        responder_message_cache.length = 0;
        responder_send_header();
        responder_state = RESPONDER_IDLE;
      }
      else if (RESPONDER_ACTION_WRITE == responder_action)
      {
        responder_read_header();
      }
      // Ignore txn complete
      break;
    case RESPONDER_READ_BODY:
      if (RESPONDER_ACTION_READ == responder_action)
      {
        // Abort the transaction...
        responder_message_cache.header = RESPONSE_SUCCESS;
        responder_message_cache.length = 0;
        responder_send_header();
        responder_state = RESPONDER_IDLE;
      }
      else if (RESPONDER_ACTION_WRITE == responder_action)
      {
        responder_read_body();
      }
      // Ignore txn complete
      break;
    case RESPONDER_PROCESS_MESSAGE:
      if (RESPONDER_TRANSACTION_COMPLETE == responder_action)
      {
        // Kick off execution
        responder_state = RESPONDER_BUSY;
        device_op_execute(&responder_message_cache, &responder_message_cache);
      }
      break;
    case RESPONDER_BUSY:
      if (RESPONDER_ACTION_READ == responder_action)
      {
        // ... waiting for a long running task to complete
        responder_message_cache.header = RESPONSE_BUSY;
        responder_message_cache.length = 0;
        responder_send_header();
      }
      else if (RESPONDER_ACTION_WRITE == responder_action)
      {
        i2c2_generate_nak();
        // Can't abort here
      }
      break;
    case RESPONDER_WRITE_HEADER:
      if (RESPONDER_ACTION_READ == responder_action)
      {
        responder_send_header();
      }
      else if (RESPONDER_ACTION_WRITE == responder_action)
      {
        i2c2_generate_nak();
        responder_state = RESPONDER_IDLE;
      }
      break;
    case RESPONDER_WRITE_BODY:
      if (RESPONDER_ACTION_READ == responder_action)
      {
        responder_send_body();
      }
      else
      {
        i2c2_generate_nak();
        responder_state = RESPONDER_IDLE;
      }
      break;
    case RESPONDER_TRANSACTION_COMPLETE:
      responder_state = RESPONDER_IDLE;
      break;
    default:
      break;
  }
  if (RESPONDER_TRANSACTION_COMPLETE == responder_action)
  {
    i2c2_listen();
  }
}

void comm_stack_init(void)
{
  for (uint8_t i = COMM_PORT_A; i < COMM_PORT_MAX; ++i)
  {
    controller_message_queue[i].queue = xQueueCreateStatic(MESSAGE_QUEUE_MAX_LENGTH,
                                                           sizeof(protocol_message_t),
                                                           controller_message_queue[i].queue_buffer,
                                                           &controller_message_queue[i].queue_impl);
  }

  controller_state = CONTROLLER_IDLE;
  responder_port   = COMM_PORT_C;
  controller_port  = COMM_PORT_A;

  i2c2_set_address_callback(handle_i2c2_address);
  i2c2_listen();
}

void comm_stack_set_exec_result(protocol_message_t* msg)
{
  if (NULL != msg)
  {
    responder_message_cache = *msg;
  }
  else
  {
    responder_message_cache.header = RESPONSE_UNKNOWN_ERROR;
    responder_message_cache.length = 0;
  }
}

void comm_stack_signal_exec_complete(void)
{
  responder_state = RESPONDER_WRITE_HEADER;
}

void ccomm_stack_clear_exec_result(void)
{
  responder_state = RESPONDER_IDLE;
}

void comm_stack_write(comm_port_t port, protocol_message_t* message, opt_callback_t operation_callback)
{
  if (port < COMM_PORT_MAX)
  {
    protocol_operation_t op;
    op.message  = *message;
    op.callback = operation_callback;
    xQueueSendToBack(controller_message_queue[port].queue, &op, pdMS_TO_TICKS(COMM_STACK_MSG_TIMEOUT_MS));
    xTimerPendFunctionCall(controller_handler, NULL, 0, COMM_STACK_PEND_TICKS);
  }
}

void comm_stack_read(comm_port_t port, protocol_message_t* message, opt_callback_t operation_callback)
{
  if (port < COMM_PORT_MAX)
  {
    protocol_operation_t op;
    op.message  = *message;
    op.callback = operation_callback;
    xQueueSendToBack(controller_message_queue[port].queue, &op, pdMS_TO_TICKS(COMM_STACK_MSG_TIMEOUT_MS));
    xTimerPendFunctionCall(controller_handler, NULL, 0, COMM_STACK_PEND_TICKS);
  }
}

comm_port_t controller_get_port(void)
{
  return controller_port;
}

comm_port_t responder_get_port(void)
{
  return responder_port;
}