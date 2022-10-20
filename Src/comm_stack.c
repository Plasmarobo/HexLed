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

#define COMM_STACK_STARTUP_DELAY_MS (200)
#define COMM_STACK_MULTIPLEXER_COOLDOWN_MS (200)

#define TARGET_POLL_MS (20)
#define PEND_TICKS (1)
#define ERROR_RESET_MS (10)
#define COMM_PROTOCOL_MIN_EXEC_TIME_MS (5)

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

#define RESPONDER_EVENT_QUEUE_MAX_LENGTH (4)

#define RESPONDER_MAX_READ (256)

#define COMM_PORT_READY (GPIO_PIN_RESET)
#define COMM_PORT_NOT_READY (GPIO_PIN_SET)

#define CONTROLLER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define RESPONDER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)

typedef enum
{
  RESPONDER_REQUEST_WRITE,
  RESPONDER_REQUEST_READ,
  RESPONDER_REQUEST_TXN_COMPLETE,
  RESPONDER_REQUEST_OTHER
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

typedef struct
{
  uint8_t responder_request;
  uint8_t status;
} responder_event_t;

typedef enum
{
  CONTROLLER_IDLE = 0,
  CONTROLLER_READ_INTERRUPTS,
  CONTROLLER_SET_CHANNEL,
  CONTROLLER_SEND_HEADER,
  CONTROLLER_SEND_BODY,
  CONTROLLER_READ_HEADER,
  CONTROLLER_READ_DATA,
  CONTROLLER_CLEAR_CHANNEL,
  CONTROLLER_ERROR,
} controller_state_t;

typedef enum
{
  RESPONDER_LISTEN = 0,
  RESPONDER_READ_HEADER,
  RESPONDER_READ_BODY,
  RESPONDER_EXEC_COMMAND,
  RESPONDER_SEND_HEADER,
  RESPONDER_SEND_BODY,
  RESPONDER_BUSY,
  RESPONDER_ERROR,
} responder_state_t;

static protocol_message_t controller_message_cache;

static StaticQueue_t network_message_queue_impl;
static uint8_t       network_message_queue_buffer[NETWORK_MESSAGE_QUEUE_MAX_LENGTH * sizeof(network_message_t)];
static QueueHandle_t network_message_queue;

static StaticQueue_t responder_event_queue_impl;
static uint8_t       responder_event_queue_buffer[RESPONDER_EVENT_QUEUE_MAX_LENGTH * sizeof(responder_event_t)];
static QueueHandle_t responder_event_queue;

static controller_state_t controller_state;
static uint16_t           target_address;
static comm_port_t        controller_port;

static responder_state_t  responder_state;
static volatile comm_port_t responder_port;
static volatile uint8_t     responder_request_flags;
static protocol_message_t responder_message_cache;
// Used for busy-replies
static uint8_t responder_header_buffer[COMM_PROTOCOL_MESSAGE_HEADER_LENGTH];

static StackType_t  controller_stack_buffer[CONTROLLER_TASK_STACK_SIZE];
static StaticTask_t controller_task_buffer;
static TaskHandle_t controller_task;

static StackType_t  responder_stack_buffer[RESPONDER_TASK_STACK_SIZE];
static StaticTask_t responder_task_buffer;
static TaskHandle_t responder_task;

static uint16_t input_channel_addresses[COMM_PORT_MAX] = {
  (COMM_PROTOCOL_BASE_ADDRESS << 1),
  (COMM_PROTOCOL_ADDRESS_TWO << 1),
  (COMM_PROTOCOL_ADDRESS_THREE << 1),
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

static void controller_handler(void* userdata);
static void responder_handler(void* userdata);

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

  xTaskNotifyFromISR(controller_task, (uint32_t)err, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  i2c1_generate_nak();

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Handles the GPIO triggered interrupt
 *
 */
void comm_stack_controller_interrupt_handler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTaskNotifyFromISR(controller_task, (uint32_t)I2C_SUCCESS, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Handles the i2c tx/rx complete
void comm_stack_responder_i2c_handler(int32_t err, uintptr_t user)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  responder_event_t event;
  event.status            = err;
  event.responder_request = RESPONDER_REQUEST_TXN_COMPLETE;
  // Tell the master we're done
  i2c2_generate_nak();
  xQueueSendToBackFromISR(responder_event_queue, &event, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
      HAL_GPIO_WritePin(responder_channels[i].port, responder_channels[i].pin, value == GPIO_PIN_RESET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(responder_channels[i].port, responder_channels[i].pin, value == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
  }
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
  BaseType_t        xHigherPriorityTaskWoken = pdFALSE;
  responder_event_t event;
  event.responder_request = ((direction == I2C_DIRECTION_TRANSMIT) ? RESPONDER_REQUEST_WRITE : RESPONDER_REQUEST_READ);
  responder_port          = responder_address_to_port(address);
  if (COMM_PORT_NONE != responder_port)
  {
    event.status = I2C_SUCCESS;
    xQueueSendToBackFromISR(responder_event_queue, &event, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void controller_handler(void* userdata)
{
  int32_t i2c_status;
  controller_state = CONTROLLER_IDLE;
  for (;;)
  {
    switch (controller_state)
    {
      case CONTROLLER_IDLE:
        vTaskDelay(pdMS_TO_TICKS(500));
        target_address                  = 0;
        controller_message_cache.header = 0;
        controller_message_cache.length = 0;
        controller_port                 = COMM_PORT_A;
        i2c_status                      = update_channel_status(comm_stack_controller_i2c_handler);
        if (I2C_SUCCESS == i2c_status)
        {
          controller_state = CONTROLLER_READ_INTERRUPTS;
        }
        else
        {
          controller_state = CONTROLLER_ERROR;
          xTaskNotify(controller_task, i2c_status, eSetValueWithOverwrite);
        }
        break;
      case CONTROLLER_READ_INTERRUPTS:
        if (I2C_SUCCESS == i2c_status)
        {
          // TODO: Do something with interrupt states
          if (I2C_SUCCESS == set_channel(controller_port, comm_stack_controller_i2c_handler))
          {
            controller_state = CONTROLLER_SET_CHANNEL;
          }
        }

        if (I2C_SUCCESS != i2c_status)
        {
          controller_state = CONTROLLER_ERROR;
          xTaskNotify(controller_task, i2c_status, eSetValueWithOverwrite);
        }
        break;
      case CONTROLLER_SET_CHANNEL:
        if (I2C_SUCCESS == i2c_status)
        {
          // Test command
          controller_message_cache.header = COMMAND_WHOAMI;
          controller_message_cache.length = 1;
          // Send the port we're connected to
          controller_message_cache.data[0] = controller_port;
          i2c_status                       = i2c1_send(input_channel_addresses[target_address],
                                 (uint8_t*)&controller_message_cache,
                                 COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                                 comm_stack_controller_i2c_handler);
          if (I2C_SUCCESS == i2c_status)
          {
            controller_state = CONTROLLER_SEND_HEADER;
          }
        }

        if (I2C_SUCCESS != i2c_status)
        {
          controller_state = CONTROLLER_ERROR;
          xTaskNotify(controller_task, i2c_status, eSetValueWithOverwrite);
        }
        break;
      case CONTROLLER_SEND_HEADER:
        if (I2C_SUCCESS == i2c_status)
        {
          i2c_status = i2c1_send(input_channel_addresses[target_address],
                                 controller_message_cache.data,
                                 controller_message_cache.length,
                                 comm_stack_controller_i2c_handler);
          if (I2C_SUCCESS == i2c_status)
          {
            controller_state = CONTROLLER_SEND_BODY;
          }
        }
        break;
      case CONTROLLER_SEND_BODY:
        if (I2C_SUCCESS == i2c_status)
        {
          vTaskDelay(pdMS_TO_TICKS(COMM_PROTOCOL_MIN_EXEC_TIME_MS));
          i2c_status = i2c1_receive(input_channel_addresses[target_address],
                                    (uint8_t*)&controller_message_cache,
                                    COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                                    comm_stack_controller_i2c_handler);
          // Something ack'd, but we don't know what yet
          set_pattern_rgb(PATTERN_BREATHE, output_channel_led[controller_port], 255, 0, 0);
          controller_state = CONTROLLER_READ_HEADER;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          set_pattern_rgb(PATTERN_SOLID, output_channel_led[controller_port], 128, 0, 32);
          controller_state = CONTROLLER_ERROR;
          xTaskNotify(controller_task, i2c_status, eSetValueWithOverwrite);
        }
        break;
      case CONTROLLER_READ_HEADER:
        if (I2C_SUCCESS == i2c_status)
        {
          serial_printf("Device at 0x%02x sent response\r\n", input_channel_addresses[target_address]);
          // We have a response, but we don't know if it is correct yet
          set_pattern_rgb(PATTERN_BREATHE, output_channel_led[controller_port], 255, 153, 51);
          // The response header should indicate success
          if (RESPONSE_BUSY == controller_message_cache.header)
          {
            vTaskDelay(pdMS_TO_TICKS(COMM_PROTOCOL_MIN_EXEC_TIME_MS));
            controller_state = CONTROLLER_SEND_BODY;
            xTaskNotify(controller_task, I2C_SUCCESS, eSetValueWithOverwrite);
            break;
          }
          else if (RESPONSE_SUCCESS != controller_message_cache.header)
          {
            serial_printf("Error %02x\r\n", controller_message_cache.header);
            set_pattern_rgb(PATTERN_SOS, output_channel_led[controller_port], 255, 255, 0);
            controller_state = CONTROLLER_IDLE;
            break;
          }
          else if (controller_message_cache.length > 0)
          {
            // Fetch message body
            i2c_status       = i2c1_receive(input_channel_addresses[target_address],
                                      (uint8_t*)&controller_message_cache.data,
                                      controller_message_cache.length,
                                      comm_stack_controller_i2c_handler);
            controller_state = CONTROLLER_READ_DATA;
          }
          else
          {
            controller_state = CONTROLLER_IDLE;
          }
        }
        if (I2C_SUCCESS != i2c_status)
        {
          controller_state = CONTROLLER_ERROR;
          xTaskNotify(controller_task, i2c_status, eSetValueWithOverwrite);
        }
        break;
      case CONTROLLER_READ_DATA:
        if (I2C_SUCCESS == i2c_status)
        {
          // TODO: Dispatch message to internal event system
          // For now, just test alignment: is it the address we asked for?
          if (controller_message_cache.data[0] == controller_port)
          {
            // We are aligned
            set_pattern_rgb(PATTERN_HEARTBEAT, output_channel_led[controller_port], 0, 255, 0);
          }
          else
          {
            // I/O port mismatch
            set_pattern_rgb(PATTERN_BLINK, output_channel_led[controller_port], 204, 0, 204);
          }
          ++controller_port;
          controller_state = CONTROLLER_IDLE;
        }
        break;
      case CONTROLLER_CLEAR_CHANNEL:
        break;
      case CONTROLLER_ERROR:
        // Report error
        handle_controller_error(i2c_status, controller_message_cache.header, controller_port);
        reset_tca9544apwr_driver();
        controller_state = CONTROLLER_IDLE;
        vTaskDelay(pdMS_TO_TICKS(ERROR_RESET_MS));
        xTaskNotify(controller_task, i2c_status, eSetValueWithOverwrite);
        break;
      default:
        controller_state = CONTROLLER_IDLE;
        xTaskNotify(controller_task, i2c_status, eSetValueWithOverwrite);
        break;
    }
    xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, portMAX_DELAY);
  }
}

void responder_handler(void* userdata)
{
  responder_event_t event;
  for (;;)
  {
    BaseType_t item_valid = xQueueReceive(responder_event_queue, &event, portMAX_DELAY);
    if (item_valid)
    {
      switch (responder_state)
      {
        case RESPONDER_LISTEN:
          // Clear status if we're in listen
          event.status = I2C_SUCCESS;
          if (RESPONDER_REQUEST_READ == event.responder_request)
          {
            // We don't have a command, respond with nothing
            responder_header_buffer[COMM_PROTOCOL_HEADER_INDEX] = RESPONSE_SUCCESS;
            responder_header_buffer[COMM_PROTOCOL_LENGTH_INDEX] = 0;
            event.status = i2c2_send(responder_header_buffer, COMM_PROTOCOL_MESSAGE_HEADER_LENGTH, comm_stack_responder_i2c_handler);
            set_pattern_rgb(PATTERN_BREATHE, input_channel_led[responder_port], 0, 255, 0);
          }
          else if (RESPONDER_REQUEST_WRITE == event.responder_request)
          {
            // Read command from controller
            responder_state = RESPONDER_READ_HEADER;
            event.status    = i2c2_receive((uint8_t*)&responder_message_cache,
                                        COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                                        comm_stack_responder_i2c_handler);
          }

          if (I2C_SUCCESS != event.status)
          {
            responder_state         = RESPONDER_ERROR;
            event.responder_request = RESPONDER_REQUEST_OTHER;
            xQueueSendToBack(responder_event_queue, &event, 0);
          }
          break;
        case RESPONDER_READ_HEADER:
          if (responder_message_cache.length > 0)
          {
            if (RESPONDER_REQUEST_WRITE == event.responder_request)
            {
              responder_state = RESPONDER_READ_BODY;
              event.status    = i2c2_receive(responder_message_cache.data,
                                          responder_message_cache.length,
                                          comm_stack_responder_i2c_handler);

              if (I2C_SUCCESS != event.status)
              {
                responder_state         = RESPONDER_ERROR;
                event.responder_request = RESPONDER_REQUEST_OTHER;
                xQueueSendToBack(responder_event_queue, &event, 0);
              }
            }
            else if (RESPONDER_REQUEST_READ == event.responder_request)
            {
              // We are expecting parameters!
              responder_header_buffer[COMM_PROTOCOL_HEADER_INDEX] = RESPONSE_ERROR_INVALID_PARAMETER;
              responder_header_buffer[COMM_PROTOCOL_LENGTH_INDEX] = 0;
              event.status = i2c2_send(responder_header_buffer, COMM_PROTOCOL_MESSAGE_HEADER_LENGTH, comm_stack_responder_i2c_handler);
              set_pattern_rgb(PATTERN_BREATHE, input_channel_led[responder_port], 0, 255, 0);
            }
          }
          else
          {
            exec_external_command(responder_message_cache);
          }
          break;
        case RESPONDER_READ_BODY:
          if ((I2C_SUCCESS == event.status) && (RESPONDER_REQUEST_TXN_COMPLETE == event.responder_request))
          {
            exec_external_command(responder_message_cache);
          }
          break;
        case RESPONDER_EXEC_COMMAND:
          if (RESPONDER_REQUEST_READ == event.responder_request)
          {
            responder_header_buffer[COMM_PROTOCOL_HEADER_INDEX] = RESPONSE_BUSY;
            responder_header_buffer[COMM_PROTOCOL_LENGTH_INDEX] = 0;
            event.status = i2c2_send(responder_header_buffer, COMM_PROTOCOL_MESSAGE_HEADER_LENGTH, comm_stack_responder_i2c_handler);
          }
          else if (RESPONDER_REQUEST_WRITE == event.responder_request)
          {
            // We're not ready to accept new data! Can't abort here
            i2c2_generate_nak();
          }

          if (I2C_SUCCESS != event.status)
          {
            responder_state         = RESPONDER_ERROR;
            event.responder_request = RESPONDER_REQUEST_OTHER;
            event.status            = I2C_SUCCESS;
            xQueueSendToBack(responder_event_queue, &event, 0);
          }
          break;
        case RESPONDER_SEND_HEADER:
          if (RESPONDER_REQUEST_READ == event.responder_request)
          {
            event.status = i2c2_send((uint8_t*)&responder_message_cache,
                                     COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                                     comm_stack_responder_i2c_handler);
          }
          else if (RESPONDER_REQUEST_WRITE == event.responder_request)
          {
            // The controller is discarding the result, replay the event in listen mode
            responder_state = RESPONDER_LISTEN;
            xQueueSendToBack(responder_event_queue, &event, 0);
          }
          else if (RESPONDER_REQUEST_TXN_COMPLETE == event.responder_request)
          {
            if (responder_message_cache.length > 0)
            {
              responder_state = RESPONDER_SEND_BODY;
            }
            else
            {
              responder_state = RESPONDER_LISTEN;
            }
          }
          break;
        case RESPONDER_SEND_BODY:
          if (RESPONDER_REQUEST_READ == event.responder_request)
          {
            event.status = i2c2_send((uint8_t*)&responder_message_cache.data,
                                     responder_message_cache.length,
                                     comm_stack_responder_i2c_handler);
          }
          else if (RESPONDER_REQUEST_WRITE == event.responder_request)
          {
            // The controller is discarding the operation, replay the event in listen mode
            responder_state = RESPONDER_LISTEN;
            xQueueSendToBack(responder_event_queue, &event, 0);
          }
          else if (RESPONDER_REQUEST_TXN_COMPLETE == event.responder_request)
          {
            responder_state = RESPONDER_LISTEN;
          }

          if (I2C_SUCCESS != event.status)
          {
            responder_state         = RESPONDER_ERROR;
            event.responder_request = RESPONDER_REQUEST_OTHER;
            xQueueSendToBack(responder_event_queue, &event, 0);
          }
          break;
        case RESPONDER_ERROR:
          {
            set_pattern_rgb(PATTERN_BLINK, input_channel_led[responder_port], 255, 0, 0);
            handle_responder_error(event.status, responder_message_cache.header);
            i2c2_set_address_callback(handle_i2c2_address);
            responder_state = RESPONDER_LISTEN;
          }
          break;
        default:
          break;
      }
    }
  }
}

void comm_stack_init(void)
{
  network_message_queue = xQueueCreateStatic(NETWORK_MESSAGE_QUEUE_MAX_LENGTH,
                                             sizeof(network_message_t),
                                             network_message_queue_buffer,
                                             &network_message_queue_impl);

  responder_event_queue = xQueueCreateStatic(RESPONDER_EVENT_QUEUE_MAX_LENGTH,
                                             sizeof(responder_event_t),
                                             responder_event_queue_buffer,
                                             &responder_event_queue_impl);

  controller_state = CONTROLLER_IDLE;
  responder_state  = RESPONDER_LISTEN;

  // The interrupt logic is falling edge, so drive each master high first
  comm_stack_responder_signal_ready(COMM_PORT_ALL, COMM_PORT_NOT_READY);
  for (uint8_t i = 0; i < COMM_PORT_MAX; ++i)
  {
    set_pattern_rgb(PATTERN_SOLID, output_channel_led[i], 0, 0, 128);
    set_pattern_rgb(PATTERN_SOLID, input_channel_led[i], 0, 0, 128);
  }

  i2c2_set_address_callback(handle_i2c2_address);

  controller_task = xTaskCreateStatic(controller_handler,
                                      "controller_task",
                                      CONTROLLER_TASK_STACK_SIZE,
                                      NULL,
                                      CONTROLLER_TASK_PRIORITY,
                                      controller_stack_buffer,
                                      &controller_task_buffer);

  responder_task = xTaskCreateStatic(responder_handler,
                                     "responder_task",
                                     RESPONDER_TASK_STACK_SIZE,
                                     NULL,
                                     RESPONDER_TASK_PRIORITY,
                                     responder_stack_buffer,
                                     &responder_task_buffer);
}

void exec_external_command(protocol_message_t command_packet)
{
  responder_state = RESPONDER_EXEC_COMMAND;
  set_pattern_rgb(PATTERN_BLINK, input_channel_led[responder_port], 128, 128, 0);
  switch (command_packet.header)
  {
    case RESPONSE_SUCCESS:
    case RESPONSE_ERROR_TIMEOUT:
    case RESPONSE_ERROR_REJECTED:
    case RESPONSE_ERROR_INVALID_OPERATION:
    case RESPONSE_ERROR_INVALID_PARAMETER:
    case RESPONSE_ERROR_ID_UNKNOWN:
    case RESPONSE_CONFIGURATION_REQUIRED:
      responder_message_cache.header = RESPONSE_ERROR_INVALID_OPERATION;
      responder_message_cache.length = 0;
      break;
      // Command 0x80 - 0xFF
      // SYSTEM
    case COMMAND_GET_ID:
    case COMMAND_SET_ID:
    case COMMAND_RESET:
    case COMMAND_TIME_SYNC:
      responder_message_cache.header = RESPONSE_ERROR_NOT_IMPLEMENTED;
      responder_message_cache.length = 0;
      break;
    case COMMAND_WHOAMI:
      // Check who the controller thinks we are
      if (responder_port == command_packet.data[0])
      {
        set_pattern_rgb(PATTERN_BREATHE, input_channel_led[responder_port], 0, 255, 0);
      }
      else
      {
        set_pattern_rgb(PATTERN_BLINK, input_channel_led[responder_port], 255, 0, 0);
      }
      // Reply with who we actually are
      responder_message_cache.header  = RESPONSE_SUCCESS;
      responder_message_cache.length  = 1;
      responder_message_cache.data[0] = responder_port;
      break;
      // LED
    case COMMAND_SET_LED:
    case COMMAND_SET_ALL_LEDS:
    case COMMAND_SET_PATTERN:
      responder_message_cache.header = RESPONSE_ERROR_NOT_IMPLEMENTED;
      responder_message_cache.length = 0;
      break;
      // NETWORK
    case COMMAND_LIST_CONNECTED:
    case COMMAND_TRACEROUTE:
    case COMMAND_SEND_TO:
    case COMMAND_CLEAR_ROUTING_TABLE:
    case COMMAND_SET_LOCATION:
    case COMMAND_UPDATE_LOCATION:
      responder_message_cache.header = RESPONSE_ERROR_NOT_IMPLEMENTED;
      break;
      // DATA
    case COMMAND_READ_MESSAGE:
    case COMMAND_WRITE_MESSAGE:
      responder_message_cache.header = RESPONSE_ERROR_NOT_IMPLEMENTED;
      responder_message_cache.length = 0;
      break;
    default:
      responder_message_cache.header = RESPONSE_ERROR_INVALID_OPERATION;
      responder_message_cache.length = 0;
      break;
  }
  // For now, everthing calls exec_complete immediately
  // TODO: should be called in a callback when result is ready
  exec_complete();
}

void exec_complete(void)
{
  set_pattern_rgb(PATTERN_SOLID, input_channel_led[responder_port], 0, 128, 0);
  responder_state = RESPONDER_SEND_HEADER;
}

void clear_exec_result(void)
{
  responder_state = RESPONDER_LISTEN;
}