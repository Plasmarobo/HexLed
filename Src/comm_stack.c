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
#define COMM_STACK_YIELD_TIME_MS (25)

#define TARGET_POLL_MS (50)
#define PEND_TICKS (1)
#define ERROR_RESET_MS (10)
#define COMM_PROTOCOL_MIN_EXEC_TIME_MS (5)
#define NETWORK_MESSAGE_WAIT_MS (10)

#define RESPONDER_TDMA_PERIOD_MS (1000)
#define RESPONDER_SETTLE_TIME_MS (50)

#define CONTROLLER_MASK_USER_DATA (0x01)
#define CONTROLLER_MASK_INTERRUPT (0x02)
#define CONTROLLER_MASK_UNKNOWN (0x04)

#define CONNECTED_INTERRUPTS_MASK (0x7)

#define MULTIPLEXER_ADDRESS (0xE0)
#define I2C_BUFFER_SIZE (128)
#define I2C_READ (0x01)
#define I2C_WRITE (0x00)

#define MESSAGE_QUEUE_MAX_LENGTH (8)

#define RESPONDER_EVENT_QUEUE_MAX_LENGTH (4)

#define RESPONDER_MAX_READ (256)

#define COMM_PORT_READY (GPIO_PIN_RESET)
#define COMM_PORT_NOT_READY (GPIO_PIN_SET)

#define CONTROLLER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define RESPONDER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)

enum
{
  RESPONDER_EVENT_WRITE,
  RESPONDER_EVENT_READ,
  RESPONDER_EVENT_TXN_COMPLETE,
  RESPONDER_EVENT_OTHER
};

typedef uint32_t responder_event_t;

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
  CONTROLLER_ERROR,
} controller_state_t;

typedef enum
{
  RESPONDER_LISTEN = 0,
  RESPONDER_READ_HEADER,
  RESPONDER_READ_BODY,
  RESPONDER_EXEC_COMMAND,
  RESPONDER_SEND_RESULT,
  RESPONDER_BUSY,
  RESPONDER_ERROR,
} responder_state_t;

typedef struct
{
  StaticQueue_t queue_impl;
  uint8_t       queue_buffer[MESSAGE_QUEUE_MAX_LENGTH * sizeof(network_message_t)];
  QueueHandle_t queue;
} port_queue_t;

static protocol_message_t controller_message_cache;

port_queue_t controller_message_queue[COMM_PORT_MAX];

static StaticQueue_t responder_event_queue_impl;
static uint8_t       responder_event_queue_buffer[RESPONDER_EVENT_QUEUE_MAX_LENGTH * sizeof(responder_event_t)];
static QueueHandle_t responder_event_queue;

static controller_state_t controller_state;
static comm_port_t        controller_port;

static responder_state_t  responder_state;
static uint32_t             responder_event;
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

static TimerHandle_t tdma_timer;
static StaticTimer_t tdma_buffer;

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

  responder_event = RESPONDER_EVENT_TXN_COMPLETE;
  // Tell the master we're done
  i2c2_generate_nak();
  xTaskNotifyFromISR(responder_task, (uint32_t)err, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

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
      HAL_GPIO_WritePin(responder_channels[i].port,
                        responder_channels[i].pin,
                        (value == GPIO_PIN_RESET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(responder_channels[i].port,
                        responder_channels[i].pin,
                        (value == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  responder_port          = responder_address_to_port(address);
  if (COMM_PORT_NONE != responder_port)
  {
    xTaskNotifyFromISR(responder_task,
                       (direction == I2C_DIRECTION_TRANSMIT) ? RESPONDER_EVENT_WRITE : RESPONDER_EVENT_READ,
                       eSetValueWithOverwrite,
                       &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int controller_send_header(uint16_t address)
{
  int i2c_status = i2c1_send(address,
                             (uint8_t*)&controller_message_cache,
                             COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                             comm_stack_controller_i2c_handler);
  if (I2C_SUCCESS == i2c_status)
  {
    if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
    {
      i2c_status = I2C_TIMEOUT;
    }
  }
  return i2c_status;
}

int controller_send_body(uint16_t address)
{
  int i2c_status = i2c1_send(address, controller_message_cache.data, controller_message_cache.length, comm_stack_controller_i2c_handler);
  if (I2C_SUCCESS == i2c_status)
  {
    if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
    {
      i2c_status = I2C_TIMEOUT;
    }
  }
  return i2c_status;
}

int controller_update_multiplexer(void)
{
  int i2c_status = update_channel_status(comm_stack_controller_i2c_handler);
  if (I2C_SUCCESS == i2c_status)
  {
    if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(COMM_STACK_MULTIPLEXER_COOLDOWN_MS)))
    {
      i2c_status = I2C_TIMEOUT;
    }
  }
  return i2c_status;
}

int controller_set_port(void)
{
  int i2c_status = set_channel(controller_port, comm_stack_controller_i2c_handler);
  if (I2C_SUCCESS == i2c_status)
  {
    if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(COMM_STACK_MULTIPLEXER_COOLDOWN_MS)))
    {
      i2c_status = I2C_TIMEOUT;
    }
  }
  return i2c_status;
}

int controller_read_header(void)
{
  int i2c_status = i2c1_receive(input_channel_addresses[controller_port],
                                (uint8_t*)&controller_message_cache,
                                COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                                comm_stack_controller_i2c_handler);
  if (I2C_SUCCESS == i2c_status)
  {
    if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
    {
      i2c_status = I2C_TIMEOUT;
    }
  }
  return i2c_status;
}

int controller_read_body(void)
{
  if (controller_message_cache.length > 0)
  {
    int i2c_status = i2c1_receive(input_channel_addresses[controller_port],
                                  controller_message_cache.data,
                                  controller_message_cache.length,
                                  comm_stack_controller_i2c_handler);
    if (I2C_SUCCESS == i2c_status)
    {
      if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
      {
        i2c_status = I2C_TIMEOUT;
      }
    }
    return i2c_status;
  }
  return I2C_SUCCESS;
}

int responder_send_header(void)
{
  int i2c_status = i2c2_send((uint8_t*)&responder_message_cache, COMM_PROTOCOL_MESSAGE_HEADER_LENGTH, comm_stack_controller_i2c_handler);
  if (I2C_SUCCESS == i2c_status)
  {
    if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
    {
      i2c_status = I2C_TIMEOUT;
    }
  }
  return i2c_status;
}

int responder_send_body(void)
{
  int i2c_status = i2c2_send(responder_message_cache.data, responder_message_cache.length, comm_stack_controller_i2c_handler);
  if (I2C_SUCCESS == i2c_status)
  {
    if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
    {
      i2c_status = I2C_TIMEOUT;
    }
  }
  return i2c_status;
}

int responder_read_header(void)
{
  int i2c_status = i2c2_receive((uint8_t*)&responder_message_cache,
                                COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                                comm_stack_responder_i2c_handler);
  if (I2C_SUCCESS == i2c_status)
  {
    if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
    {
      i2c_status = I2C_TIMEOUT;
    }
  }
  return i2c_status;
}

int responder_read_body(void)
{
  if (responder_message_cache.length > 0)
  {
    int i2c_status = i2c2_receive(responder_message_cache.data, responder_message_cache.length, comm_stack_responder_i2c_handler);
    if (I2C_SUCCESS == i2c_status)
    {
      if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
      {
        i2c_status = I2C_TIMEOUT;
      }
    }
    return i2c_status;
  }
  else
  {
    return I2C_SUCCESS;
  }
}

int responder_reply(header_data_t code)
{
  responder_header_buffer[COMM_PROTOCOL_HEADER_INDEX] = code;
  responder_header_buffer[COMM_PROTOCOL_LENGTH_INDEX] = 0;
  int i2c_status = i2c2_send(responder_header_buffer, COMM_PROTOCOL_MESSAGE_HEADER_LENGTH, comm_stack_responder_i2c_handler);
  if (I2C_SUCCESS == i2c_status)
  {
    if (pdFALSE == xTaskNotifyWait(0, 0, (uint32_t*)&i2c_status, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
    {
      i2c_status = I2C_TIMEOUT;
    }
  }
  return i2c_status;
}

void controller_handler(void* userdata)
{
  int32_t           i2c_status = I2C_SUCCESS;
  network_message_t envelope;
  controller_state = CONTROLLER_IDLE;
  controller_port  = COMM_PORT_A;
  for (;;)
  {
    switch (controller_state)
    {
      case CONTROLLER_IDLE:
        controller_message_cache.header = 0;
        controller_message_cache.length = 0;
        vTaskDelay(pdMS_TO_TICKS(COMM_STACK_MULTIPLEXER_COOLDOWN_MS));
        i2c_status = controller_update_multiplexer();
        if (I2C_SUCCESS != i2c_status)
        {
          controller_state = CONTROLLER_ERROR;
          break;
        }

        // Do we have the attention of a responder?
        if (get_channel_status() == 0)
        {
          vTaskDelay(pdMS_TO_TICKS(TARGET_POLL_MS));
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(COMM_STACK_YIELD_TIME_MS));
        // Find the next responder to service
        uint8_t test_port = controller_port + 1;
        while (test_port != controller_port)
        {
          if (test_port >= COMM_PORT_MAX)
          {
            test_port = COMM_PORT_A;
          }
          if (is_channel_active(test_port))
          {
            controller_port = test_port;
            break;
          }
          ++test_port;
        }
        // Select the port
        i2c_status = controller_set_port();
        if (I2C_SUCCESS != i2c_status)
        {
          controller_state = CONTROLLER_ERROR;
          break;
        }

        // TEST: Start with an initial WHOAMI
        // Status blank/unconnected
        set_pattern_rgb(PATTERN_SOLID, output_channel_led[controller_port], 0, 0, 0);
        // Test command
        controller_message_cache.header = COMMAND_WHOAMI;
        controller_message_cache.length = 1;
        // Send the ID of our current port
        controller_message_cache.data[0] = controller_port;
        i2c_status                       = controller_send_header(input_channel_addresses[controller_port]);

        if (I2C_SUCCESS != i2c_status)
        {
          controller_state = CONTROLLER_ERROR;
          break;
        }

        // We've got a response, but is it correct?
        set_pattern_rgb(PATTERN_SOLID, output_channel_led[controller_port], 0, 0, 128);

        i2c_status = controller_send_body(input_channel_addresses[controller_port]);
        if (I2C_SUCCESS != i2c_status)
        {
          if (I2C_TIMEOUT == i2c_status)
          {
            controller_state = CONTROLLER_IDLE;
            vTaskDelay(pdMS_TO_TICKS(TARGET_POLL_MS));
          }
          else
          {
            controller_state = CONTROLLER_ERROR;
          }
          break;
        }

        // Expect the responder to be busy working
        set_pattern_rgb(PATTERN_BLINK, output_channel_led[controller_port], 128, 128, 0);
        controller_message_cache.header = RESPONSE_BUSY;
        controller_message_cache.length = 0;

        // Poll the responder
        while (controller_message_cache.header == RESPONSE_BUSY)
        {
          vTaskDelay(pdMS_TO_TICKS(COMM_PROTOCOL_MIN_EXEC_TIME_MS));
          i2c_status = controller_read_header();
          if (I2C_SUCCESS != i2c_status)
          {
            controller_state = CONTROLLER_ERROR;
            break;
          }
        }

        i2c_status = controller_read_body();
        if (I2C_SUCCESS != i2c_status)
        {
          controller_state = CONTROLLER_ERROR;
          break;
        }

        // Process the response
        if (RESPONSE_SUCCESS == controller_message_cache.header)
        {
          // Check the reply port
          if ((controller_message_cache.length > 0) && (controller_message_cache.data[0] == controller_port))
          {
            // Correct port alignment!
            set_pattern_rgb(PATTERN_BREATHE, output_channel_led[controller_port], 0, 255, 0);
          }
          else
          {
            // Malformed response or misaligned port
            set_pattern_rgb(PATTERN_BLINK, output_channel_led[controller_port], 255, 255, 0);
          }
        }
        else
        {
          set_pattern_rgb(PATTERN_SOS, output_channel_led[controller_port], 255, 0, 0);
        }

        // TEST transaction completed, drain network queue
        while (uxQueueMessagesWaiting(controller_message_queue[controller_port].queue) > 0)
        {
          if (pdFALSE ==
              xQueueReceive(controller_message_queue[controller_port].queue, &envelope, pdMS_TO_TICKS(NETWORK_MESSAGE_WAIT_MS)))
          {
            // Handle the network message...
            controller_message_cache = envelope.message;
            i2c_status               = controller_send_header(input_channel_addresses[controller_port]);
            if (I2C_SUCCESS != i2c_status)
            {
              controller_state = CONTROLLER_ERROR;
              break;
            }
            i2c_status = controller_send_body(input_channel_addresses[controller_port]);
            if (I2C_SUCCESS != i2c_status)
            {
              controller_state = CONTROLLER_ERROR;
              break;
            }
            controller_message_cache.header = RESPONSE_BUSY;
            controller_message_cache.length = 0;

            // Poll the responder
            while (controller_message_cache.header == RESPONSE_BUSY)
            {
              vTaskDelay(pdMS_TO_TICKS(COMM_PROTOCOL_MIN_EXEC_TIME_MS));
              i2c_status = controller_read_header();
              if (I2C_SUCCESS != i2c_status)
              {
                controller_state = CONTROLLER_ERROR;
                break;
              }
            }

            i2c_status = controller_read_body();
            if (I2C_SUCCESS != i2c_status)
            {
              controller_state = CONTROLLER_ERROR;
              break;
            }

            // TODO: process the response packet. For now just check success
            if (RESPONSE_SUCCESS != controller_message_cache.header)
            {
              controller_state = CONTROLLER_ERROR;
              break;
            }
          }
        }
        break;
      case CONTROLLER_ERROR:
        // Report error
        set_pattern_rgb(PATTERN_BREATHE, output_channel_led[controller_port], 255, 0, 0);
        handle_controller_error(i2c_status, controller_message_cache.header, controller_port);
        reset_tca9544apwr_driver();
        vTaskDelay(pdMS_TO_TICKS(ERROR_RESET_MS));
        controller_state = CONTROLLER_IDLE;
        break;
      default:
        controller_state = CONTROLLER_IDLE;
        break;
    }
  }
}

void responder_handler(void* userdata)
{
  int i2c_status = I2C_SUCCESS;
  for (;;)
  {
    xTaskNotifyWait(0, 0, &responder_event, portMAX_DELAY);

    switch (responder_state)
    {
      case RESPONDER_LISTEN:
        // Clear status if we're in listen
        i2c_status = I2C_SUCCESS;
        if (RESPONDER_EVENT_READ == responder_event)
        {
          // We don't have a command, respond with nothing
          i2c_status = responder_reply(RESPONSE_SUCCESS);
          set_pattern_rgb(PATTERN_BREATHE, input_channel_led[responder_port], 0, 255, 0);
        }
        else if (RESPONDER_EVENT_WRITE == responder_event)
        {
          // Halt TDMA
          xTimerStop(tdma_timer, 0);
          // Read command from controller
          responder_state = RESPONDER_READ_HEADER;
          i2c_status      = responder_read_header();
          if (I2C_SUCCESS != i2c_status)
          {
            break;
          }
          // Wait for controller request
          if (pdFALSE == xTaskNotifyWait(0, 0, &responder_event, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
          {
            i2c_status = I2C_TIMEOUT;
            break;
          }

          if (RESPONDER_EVENT_WRITE != responder_event)
          {
            // Unexpected read
            i2c_status = I2C_ERROR;
            break;
          }

          i2c_status = responder_read_body();
          if (I2C_SUCCESS != i2c_status)
          {
            break;
          }
          exec_external_command(responder_message_cache);
        }
        break;
      case RESPONDER_EXEC_COMMAND:
        if (RESPONDER_EVENT_READ == responder_event)
        {
          i2c_status = responder_reply(RESPONSE_BUSY);
        }
        else if (RESPONDER_EVENT_WRITE == responder_event)
        {
          // We're not ready to accept new data! Can't abort here
          i2c2_generate_nak();
        }
        break;
      case RESPONDER_SEND_RESULT:
        if (RESPONDER_EVENT_READ == responder_event)
        {
          i2c_status = responder_send_header();
          if (I2C_SUCCESS != i2c_status)
          {
            responder_state = RESPONDER_ERROR;
            break;
          }

          // Wait for controller request
          if (pdFALSE == xTaskNotifyWait(0, 0, &responder_event, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS)))
          {
            responder_state = RESPONDER_ERROR;
            break;
          }

          if (RESPONDER_EVENT_READ != responder_event)
          {
            // Unexpected write
            responder_state = RESPONDER_ERROR;
            break;
          }
          i2c_status = responder_send_body();
        }
        else if (RESPONDER_EVENT_WRITE == responder_event)
        {
          // The controller is discarding the result, replay the event in listen mode
          xQueueSendToBack(responder_event_queue, &responder_event, 0);
        }
        // Restart TDMA
        xTimerStart(tdma_timer, 0);
        responder_state = RESPONDER_LISTEN;
        break;
      case RESPONDER_ERROR:
        {
          set_pattern_rgb(PATTERN_BLINK, input_channel_led[responder_port], 255, 0, 0);
          handle_responder_error(responder_event, responder_message_cache.header);
          i2c2_abort();
          i2c2_generate_nak();
          i2c2_set_address_callback(handle_i2c2_address);
          xTimerStart(tdma_timer, 0);
          responder_state = RESPONDER_LISTEN;
        }
        break;
      default:
        break;
    }

    if (I2C_SUCCESS != i2c_status)
    {
      responder_state = RESPONDER_ERROR;
      responder_event = RESPONDER_EVENT_OTHER;
      xQueueSendToBack(responder_event_queue, &responder_event, 0);
      xTimerStart(tdma_timer, 0);
    }
  }
}

void responder_tdma_handler(TimerHandle_t tm)
{
  if (RESPONDER_LISTEN == responder_state)
  {
    ++responder_port;
    if (responder_port >= COMM_PORT_MAX)
    {
      responder_port = COMM_PORT_A;
    }
    comm_stack_responder_signal_ready(responder_port, GPIO_PIN_SET);
  }
}

void comm_stack_init(void)
{
  for (uint8_t i = COMM_PORT_A; i < COMM_PORT_MAX; ++i)
  {
    controller_message_queue[i].queue = xQueueCreateStatic(MESSAGE_QUEUE_MAX_LENGTH,
                                                           sizeof(network_message_t),
                                                           controller_message_queue[i].queue_buffer,
                                                           &controller_message_queue[i].queue_impl);
  }

  responder_event_queue = xQueueCreateStatic(RESPONDER_EVENT_QUEUE_MAX_LENGTH,
                                             sizeof(responder_event_t),
                                             responder_event_queue_buffer,
                                             &responder_event_queue_impl);

  controller_state = CONTROLLER_IDLE;
  responder_state  = RESPONDER_LISTEN;
  responder_port   = COMM_PORT_C;
  controller_port  = COMM_PORT_A;

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

  tdma_timer = xTimerCreateStatic("tdma_timer", pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS), pdTRUE, (void*)0, responder_tdma_handler, &tdma_buffer);
  xTimerStart(tdma_timer, pdMS_TO_TICKS(COMM_STACK_STARTUP_DELAY_MS));
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
  responder_state = RESPONDER_SEND_RESULT;
}

void clear_exec_result(void)
{
  responder_state = RESPONDER_LISTEN;
}