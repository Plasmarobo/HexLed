#include "comm_stack.h"

#include "FreeRTOS.h"

#include "TCA9544APWR.h"
#include "comm_protocol.h"
#include "display.h"
#include "gpio.h"
#include "i2c.h"
#include "priorities.h"
#include "queue.h"
#include "serial_output.h"
#include "stm32l0xx_hal_i2c.h"
#include "task.h"
#include "timers.h"

#include <stdint.h>
#include <string.h>

#define COMM_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)

#define COMM_STACK_STARTUP_DELAY_MS (200)
#define COMM_STACK_MULTIPLEXER_COOLDOWN_MS (200)

#define PEND_TICKS (5)

#define RESPONDER_TDMA_PERIOD_MS (300)
#define RESPONDER_SETTLE_TIME_MS (50)

#define CONTROLLER_MASK_USER_DATA (0x01)
#define CONTROLLER_MASK_INTERRUPT (0x02)
#define CONTROLLER_MASK_UNKNOWN (0x04)

#define CONNECTED_INTERRUPTS_MASK (0x7)

#define MULTIPLEXER_ADDRESS (0xE0)
#define I2C_BUFFER_SIZE (128)
#define I2C_READ (0x01)
#define I2C_WRITE (0x00)

#define NETWORK_MESSAGE_QUEUE_MAX_LENGTH (16)

#define RESPONDER_MAX_READ (256)

#define COMM_PORT_READY (GPIO_PIN_RESET)
#define COMM_PORT_NOT_READY (GPIO_PIN_SET)

typedef enum
{
  RESPONDER_REQUEST_TDMA = 0,
  RESPONDER_REQUEST_READ,
  RESPONDER_REQUEST_WRITE,
  RESPONDER_TRANSACTION_COMPLETE,
  RESPONDER_REQUEST_MAX,
} responder_request_t;

typedef enum
{
  ERROR_NONE = 0,
  ERROR_BUSY,
  ERROR_COMM,
  ERROR_MSG,
  ERROR_INVALID_COMMAND,
  ERROR_INVALID_LENGTH,
  ERROR_END_OF_STREAM,
} error_code_t;

#ifdef DEBUG
const char* comm_port_text[] = {
  "A", "B", "C", "", "", "", "MP",
};

#endif

typedef struct
{
  GPIO_TypeDef* port;
  uint32_t      pin;
} responder_select_t;

typedef enum
{
  CONTROLLER_IDLE = 0,
  CONTROLLER_READ_INTERRUPTS,
  CONTROLLER_SET_CHANNEL,
  CONTROLLER_DETECT_TARGET,
  CONTROLLER_QUERY_TARGET_HEADER,
  CONTROLLER_QUERY_TARGET_DATA,
  CONTROLLER_CLEAR_CHANNEL,
  CONTROLLER_ERROR,
} controller_state_t;

typedef enum
{
  RESPONDER_LISTEN = 0,
  RESPONDER_READ_COMMAND,
  RESPONDER_EXEC_COMMAND,
  RESPONDER_WORKING,
  RESPONDER_WRITE_RESULT,
  RESPONDER_ERROR,
} responder_state_t;

static protocol_message_t controller_message_cache;

static StaticQueue_t network_message_queue_impl;
static uint8_t       network_message_queue_buffer[NETWORK_MESSAGE_QUEUE_MAX_LENGTH * sizeof(network_message_t)];
static QueueHandle_t network_message_queue;

static controller_state_t controller_state;
static uint16_t           target_address;
static comm_port_t        controller_port;

static responder_state_t  responder_state;
static comm_port_t        responder_port;
static uint32_t           responder_request;
static uint8_t            responder_status_buffer;
static protocol_message_t responder_message_cache;

static uint16_t input_channel_addresses[COMM_PORT_MAX] = {
  COMM_PROTOCOL_BASE_ADDRESS,
  COMM_PROTOCOL_MASK_ADDRESS,
  COMM_PROTOCOL_MASK_ADDRESS + 2,
};

static responder_select_t responder_channels[COMM_PORT_MAX] = {
  {
      .port = GPIOA,
      .pin  = GPIO_PIN_3,
  },
  {
      .port = GPIOA,
      .pin  = GPIO_PIN_5,
  },
  {
      .port = GPIOB,
      .pin  = GPIO_PIN_1,
  },
};

static uint8_t input_channel_led[COMM_PORT_MAX] = {
  0,
  2,
  4,
};

static uint8_t output_channel_led[COMM_PORT_MAX] = {
  1,
  3,
  5,
};

static void controller_handler(void* userdata, uint32_t userdata2);
static void responder_handler(void* userdata, uint32_t userdata2);

static void handle_controller_error(int32_t i2c_error, uint8_t response_header, uint32_t port)
{
  serial_printf("Controller error: %d %d %s %s\r\n", i2c_error, response_header, comm_port_text[port]);
}

static void handle_responder_error(int32_t i2c_error, uint8_t command_header)
{
  serial_printf("Responder error: %d %d Responder %s\r\n", i2c_error, command_header);
}

// Handles i2c interrupts from the controller
void comm_stack_controller_i2c_handler(int32_t err, uintptr_t user)
{
  UNUSED(user);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTimerPendFunctionCallFromISR(controller_handler, NULL, (uint32_t)err, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Handles the i2c tx/rx complete
void comm_stack_responder_i2c_handler(int32_t err, uintptr_t user)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTimerPendFunctionCallFromISR(responder_handler, NULL, (uint32_t)err, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Handles the i2c address match
void comm_stack_responder_listen_handler(void)
{
}

// Periodically requests that we signal a new channel active
void responder_timer_handler(TimerHandle_t th)
{
  UNUSED(th);
}

// Notify masters that we are ready to accept messages on a specific channel
void comm_stack_responder_signal_ready(comm_port_t channel, uint8_t value)
{
  // Signals to the masters that this slave is ready for a transaction

  for (comm_port_t i = COMM_PORT_A; i < COMM_PORT_MAX; ++i)
  {
    if (channel == i || COMM_PORT_ALL == channel)
    {
      HAL_GPIO_WritePin(responder_channels[i].port,
                        responder_channels[i].pin,
                        value == GPIO_PIN_RESET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(responder_channels[i].port,
                        responder_channels[i].pin,
                        value == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
  }
}

static void handle_i2c2_address(uint8_t direction, uint16_t address)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  bool       valid_address            = false;
  responder_request                   = I2C_DIRECTION_TRANSMIT ? RESPONDER_REQUEST_READ : RESPONDER_REQUEST_WRITE;
  switch (address)
  {
    case COMM_PROTOCOL_BASE_ADDRESS:
    {
      valid_address     = true;
      responder_port    = COMM_PORT_A;
    }
    break;
    case COMM_PROTOCOL_ADDRESS_TWO:
    {
      valid_address     = true;
      responder_port    = COMM_PORT_B;
    }
    break;
    case COMM_PROTOCOL_ADDRESS_THREE:
    {
      valid_address     = true;
      responder_port    = COMM_PORT_C;
    }
    break;
    default:
      break;
  }
  if (valid_address)
  {
    xTimerPendFunctionCallFromISR(responder_handler, NULL, I2C_SUCCESS, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void controller_handler(void* userdata, uint32_t userdata2)
{
  int32_t i2c_status = (int32_t)userdata2;
  switch (controller_state)
  {
    case CONTROLLER_IDLE:
      target_address                  = 0;
      controller_message_cache.header = 0;
      controller_message_cache.length = 0;
      /*if (controller_port > COMM_PORT_MAX)
      {
        controller_port = COMM_PORT_MIN;
      }*/
      controller_port = COMM_PORT_A;
      i2c_status = update_channel_status(comm_stack_controller_i2c_handler);
      if (I2C_SUCCESS == i2c_status)
      {
        controller_state = CONTROLLER_READ_INTERRUPTS;
      }
      else
      {
        controller_state = CONTROLLER_ERROR;
        xTimerPendFunctionCall(controller_handler, NULL, (uint32_t)i2c_status, PEND_TICKS);
      }
      break;
    case CONTROLLER_READ_INTERRUPTS:
      if (I2C_SUCCESS == i2c_status)
      {
        // TODO: Do something with interrupt states
        i2c_status = set_channel(controller_port, comm_stack_controller_i2c_handler);
        if (I2C_SUCCESS == i2c_status)
        {
          controller_state = CONTROLLER_SET_CHANNEL;
        }
      }

      if (I2C_SUCCESS != i2c_status)
      {
        controller_state = CONTROLLER_ERROR;
        xTimerPendFunctionCall(controller_handler, NULL, (uint32_t)i2c_status, PEND_TICKS);
      }
      break;
    case CONTROLLER_SET_CHANNEL:
      if (I2C_SUCCESS == i2c_status)
      {
        // Test command
        controller_message_cache.header = COMMAND_WHOAMI;
        controller_message_cache.length = 1;
        // Whoami should contain the address we EXPECT address (responder should reply to any/all)
        controller_message_cache.data[0] = input_channel_addresses[controller_port];
        i2c_status                       = i2c1_send(input_channel_addresses[target_address],
                               (uint8_t*)&controller_message_cache,
                               controller_message_cache.length + COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                               comm_stack_controller_i2c_handler);
        if (I2C_SUCCESS == i2c_status)
        {
          controller_state = CONTROLLER_DETECT_TARGET;
        }
      }

      if (I2C_SUCCESS != i2c_status)
      {
        controller_state = CONTROLLER_ERROR;
        xTimerPendFunctionCall(controller_handler, NULL, (uint32_t)i2c_status, PEND_TICKS);
      }
      break;
    case CONTROLLER_DETECT_TARGET:
      if (I2C_SUCCESS == i2c_status)
      {
        i2c_status = i2c1_receive(input_channel_addresses[target_address],
                                  (uint8_t*)&controller_message_cache,
                                  COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                                  comm_stack_controller_i2c_handler);
        // Something ack'd, but we don't know what yet
        display_set_rgb(output_channel_led[controller_port], 255, 0, 0);
        controller_state = CONTROLLER_QUERY_TARGET_HEADER;
      }

      if (I2C_SUCCESS != i2c_status)
      {
        target_address += 1;
        i2c_status = I2C_SUCCESS;
        if (target_address >= COMM_PORT_MAX)
        {
          ++controller_port;
          controller_state = CONTROLLER_IDLE;
        }
        else
        {
          controller_state = CONTROLLER_SET_CHANNEL;
        }
        xTimerPendFunctionCall(controller_handler, NULL, (uint32_t)i2c_status, PEND_TICKS);
      }
      break;
    case CONTROLLER_QUERY_TARGET_HEADER:
      if (I2C_SUCCESS == i2c_status)
      {
        serial_printf("Device at 0x%02x sent response\r\n", input_channel_addresses[target_address]);
        // We have a response, but we don't know if it is correct yet
        display_set_rgb(output_channel_led[controller_port], 255, 153, 51);
        // The response header should indicate success
        if (RESPONSE_SUCCESS != controller_message_cache.header)
        {
          display_set_rgb(output_channel_led[controller_port], 255, 255, 0);
          break;
        }

        if (controller_message_cache.length > 0)
        {
          // Read body of message, [length] bytes
          i2c_status = i2c1_receive(input_channel_addresses[target_address],
                                    (uint8_t*)&controller_message_cache.data,
                                    controller_message_cache.length,
                                    comm_stack_controller_i2c_handler);
          if (I2C_SUCCESS == i2c_status)
          {
            controller_state = CONTROLLER_QUERY_TARGET_DATA;
          }
        }
      }
      if (I2C_SUCCESS != i2c_status)
      {
        controller_state = CONTROLLER_ERROR;
        xTimerPendFunctionCall(controller_handler, NULL, (uint32_t)i2c_status, PEND_TICKS);
      }
      break;
    case CONTROLLER_QUERY_TARGET_DATA:
      if (I2C_SUCCESS == i2c_status)
      {
        // TODO: Dispatch message to internal event system
        // For now, just test alignment: is it the address we asked for?
        if (controller_message_cache.data[0] == input_channel_addresses[controller_port])
        {
          // We are aligned
          display_set_rgb(output_channel_led[controller_port], 0, 255, 0);
        }
        else
        {
          // I/O port mismatch
          display_set_rgb(output_channel_led[controller_port], 204, 0, 204);
        }
        ++controller_port;
        controller_state = CONTROLLER_IDLE;
      }
      else
      {
        controller_state = CONTROLLER_ERROR;
      }
      xTimerPendFunctionCall(controller_handler, NULL, (uint32_t)i2c_status, PEND_TICKS);
      break;
    case CONTROLLER_CLEAR_CHANNEL:
      break;
    case CONTROLLER_ERROR:
      // Report error
      handle_controller_error(i2c_status, controller_message_cache.header, controller_port);
      controller_state = CONTROLLER_IDLE;
      xTimerPendFunctionCall(controller_handler, NULL, (uint32_t)i2c_status, PEND_TICKS);
      break;
    default:  
      controller_state = CONTROLLER_IDLE;
      xTimerPendFunctionCall(controller_handler, NULL, (uint32_t)i2c_status, PEND_TICKS);
      break;
  }
}

void responder_handler(void* userdata, uint32_t userdata2)
{
  int32_t i2c_status = (int32_t)userdata2;
  switch (responder_state)
  {
    case RESPONDER_LISTEN:
      if (RESPONDER_REQUEST_READ == responder_request)
      {
        // We're in an unknown state, reply with our id?
        responder_message_cache.header  = ERROR_NONE;
        responder_message_cache.length  = 1;
        responder_message_cache.data[0] = responder_port;
        responder_state                 = RESPONDER_WRITE_RESULT;
        i2c_status                      = i2c2_send((uint8_t*)&responder_message_cache,
                               COMM_PROTOCOL_MESSAGE_HEADER_LENGTH + responder_message_cache.length,
                               comm_stack_responder_i2c_handler);
      }
      else if (RESPONDER_REQUEST_WRITE == responder_request)
      {
        responder_state = RESPONDER_READ_COMMAND;
        i2c_status      = i2c2_receive((uint8_t*)&responder_message_cache,
                                  COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                                  comm_stack_responder_i2c_handler);
      }

      if (I2C_SUCCESS != i2c_status)
      {
        responder_state = RESPONDER_ERROR;
        xTimerPendFunctionCall(responder_handler, NULL, i2c_status, pdMS_TO_TICKS(0));
      }
      break;
    case RESPONDER_READ_COMMAND:
    {
      if (I2C_SUCCESS == i2c_status)
      {
        // Fetch command data
        responder_state = RESPONDER_EXEC_COMMAND;
        i2c_status      = i2c2_receive((uint8_t*)&responder_message_cache.data,
                                  responder_message_cache.length,
                                  comm_stack_responder_i2c_handler);
      }
      if (I2C_SUCCESS != i2c_status)
      {
        responder_state = RESPONDER_ERROR;
        xTimerPendFunctionCall(responder_handler, NULL, i2c_status, pdMS_TO_TICKS(0));
      }
    }
    break;
    case RESPONDER_EXEC_COMMAND:
    {
      if (I2C_SUCCESS == i2c_status)
      {
        // We have a full message, execute it
        responder_state = RESPONDER_WORKING;
        exec_external_command(responder_message_cache);
      }
    }
    break;
    case RESPONDER_WORKING:
      if (RESPONDER_REQUEST_READ == responder_request)
      {
        responder_status_buffer = ERROR_BUSY;
        i2c2_send(&responder_status_buffer, 1, comm_stack_responder_i2c_handler);
      }
      else if (RESPONDER_REQUEST_WRITE == responder_request)
      {
        // NAK the request? We're busy
        i2c2_receive(NULL, COMM_PROTOCOL_MAX_MESSAGE_LENGTH, comm_stack_responder_i2c_handler);
      }
      break;
    case RESPONDER_WRITE_RESULT:
    {
      if (I2C_SUCCESS == i2c_status)
      {
        responder_state = RESPONDER_LISTEN;
        if (RESPONDER_REQUEST_READ == responder_request)
        {
          i2c_status = i2c2_send((uint8_t*)&responder_message_cache,
                                 COMM_PROTOCOL_MESSAGE_HEADER_LENGTH + responder_message_cache.length,
                                 comm_stack_responder_i2c_handler);
        }
        else if (RESPONDER_REQUEST_WRITE == responder_request)
        {
          // The controller is discarding the result, hand off to the listener
          xTimerPendFunctionCall(responder_handler, NULL, i2c_status, pdMS_TO_TICKS(0));
        }
      }
      if (I2C_SUCCESS != i2c_status)
      {
        responder_state = RESPONDER_ERROR;
        xTimerPendFunctionCall(responder_handler, NULL, i2c_status, pdMS_TO_TICKS(0));
      }
    }
    break;
    case RESPONDER_ERROR:
    {
      handle_responder_error(i2c_status, responder_message_cache.header);
      i2c2_set_address_callback(handle_i2c2_address);
      // responder_state = RESPONDER_LISTEN;
    }
    break;
    default:
      break;
  }
}

/**
 * @brief Handles the GPIO triggered interrupt
 *
 */
void comm_stack_controller_interrupt_handler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTimerPendFunctionCallFromISR(controller_handler, NULL, I2C_SUCCESS, pdMS_TO_TICKS(0));

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void comm_stack_init(void)
{
  network_message_queue = xQueueCreateStatic(NETWORK_MESSAGE_QUEUE_MAX_LENGTH,
                                             sizeof(network_message_t),
                                             network_message_queue_buffer,
                                             &network_message_queue_impl);

  controller_state = CONTROLLER_IDLE;
  responder_state  = RESPONDER_LISTEN;

  // The interrupt logic is falling edge, so drive each master high first
  comm_stack_responder_signal_ready(COMM_PORT_ALL, COMM_PORT_NOT_READY);
  for (uint8_t i = 0; i < COMM_PORT_MAX; ++i)
  {
    display_set_rgb(output_channel_led[i], 0, 0, 128);
    display_set_rgb(input_channel_led[i], 0, 0, 128);
  }
  xTimerPendFunctionCall(controller_handler, NULL, I2C_SUCCESS, pdMS_TO_TICKS(COMM_STACK_STARTUP_DELAY_MS));
  xTimerPendFunctionCall(responder_handler, NULL, I2C_SUCCESS, pdMS_TO_TICKS(0));
}

void exec_external_command(protocol_message_t command_packet)
{
}

void exec_complete(void)
{
  responder_request = RESPONDER_TRANSACTION_COMPLETE;
  responder_state   = RESPONDER_WRITE_RESULT;
}

void clear_exec_result(void)
{
  if (RESPONDER_WRITE_RESULT == responder_state)
  {
    responder_state = RESPONDER_LISTEN;
  }
}