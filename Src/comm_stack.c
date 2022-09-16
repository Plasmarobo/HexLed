#include "comm_stack.h"

#include "FreeRTOS.h"

#include "comm_protocol.h"
#include "display.h"
#include "gpio.h"
#include "i2c.h"
#include "queue.h"
#include "serial_output.h"
#include "stm32l0xx_hal_i2c.h"
#include "task.h"
#include "timers.h"

#include <stdint.h>
#include <string.h>

#define COMM_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#define COMM_TASK_STACK_PRIORITY (15)

#define COMM_STACK_MESSAGE_PERIOD (125)

#define COMM_STACK_STARTUP_DELAY_MS (100)
#define COMM_STACK_MULTIPLEXER_COOLDOWN_MS (5)

#define CONTROLLER_MASK_USER_DATA (0x01)
#define CONTROLLER_MASK_INTERRUPT (0x02)
#define CONTROLLER_MASK_UNKNOWN (0x04)

#define CONNECTED_INTERRUPTS_MASK (0x7)

#define MULTIPLEXER_ADDRESS (0xE0)
#define I2C_BUFFER_SIZE (128)
#define I2C_READ (0x01)
#define I2C_WRITE (0x00)

#define CHANNEL_MASK(x) (1 << x)

#define I2C_TIMEOUT_MS (100)

#define NETWORK_MESSAGE_QUEUE_MAX_LENGTH (16)

#define RESPONDER_TDMA_PERIOD_MS (300)
#define RESPONDER_SETTLE_TIME_MS (50)

#define RESPONDER_MAX_READ (256)

#define COMM_PORT_READY (GPIO_PIN_RESET)
#define COMM_PORT_NOT_READY (GPIO_PIN_SET)

typedef struct
{
  uint8_t channel : 2;
  uint8_t enable : 1;
  uint8_t reserved : 1;
  uint8_t interrupts : 4;
} control_register_t;

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
  ERROR_TRIGGER,
  ERROR_TIMEOUT,
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

const char* error_text[] = {
  "None", "Trigger", "Timeout", "Comm", "Msg", "InvCmd", "InvLen", "EOS",
};
#endif

typedef struct
{
  GPIO_TypeDef* port;
  uint32_t      pin;
} responder_select_t;

static protocol_message_t controller_message_cache;

static StaticQueue_t network_message_queue_impl;
static uint8_t       network_message_queue_buffer[NETWORK_MESSAGE_QUEUE_MAX_LENGTH * sizeof(network_message_t)];
static QueueHandle_t network_message_queue;

static StackType_t  responder_stack_buffer[COMM_TASK_STACK_SIZE];
static StaticTask_t responder_tcb_buffer;
static TaskHandle_t responder_task;

static StackType_t  controller_stack_buffer[COMM_TASK_STACK_SIZE];
static StaticTask_t controller_tcb_buffer;
static TaskHandle_t controller_task;

static StaticTimer_t responder_timer_buffer;
static TimerHandle_t responder_timer;

static comm_port_t responder_channel;
static uint8_t     responder_read_buffer[RESPONDER_MAX_READ];

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

static void handle_controller_error(uint8_t response, uint8_t status, uint32_t port, uint32_t error_code)
{
  serial_printf("Controller error: %d %d %s %s\r\n", response, status, comm_port_text[port], error_text[error_code]);
}

static void handle_responder_error(uint8_t response, uint8_t status, uint32_t error_code)
{
  serial_printf("Responder error: %d %d Responder %s\r\n", response, status, error_text[error_code]);
}

// Handles i2c interrupts from the controller
void comm_stack_controller_i2c_handler(uint8_t err, uintptr_t user)
{
  UNUSED(user);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTaskNotifyFromISR(controller_task, err, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Handles the i2c tx/rx complete
void comm_stack_responder_i2c_handler(uint8_t err, uintptr_t user)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTaskNotifyFromISR(responder_task, err, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

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
  xTaskNotify(responder_task, RESPONDER_REQUEST_TDMA, eSetValueWithOverwrite);
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
  uint32_t   request_direction        = I2C_DIRECTION_TRANSMIT ? RESPONDER_REQUEST_READ : RESPONDER_REQUEST_WRITE;
  switch (address)
  {
    case COMM_PROTOCOL_BASE_ADDRESS:
    {
      valid_address     = true;
      responder_channel = COMM_PORT_A;
    }
    break;
    case COMM_PROTOCOL_ADDRESS_TWO:
    {
      valid_address     = true;
      responder_channel = COMM_PORT_B;
    }
    break;
    case COMM_PROTOCOL_ADDRESS_THREE:
    {
      valid_address     = true;
      responder_channel = COMM_PORT_C;
    }
    break;
    default:
      break;
  }
  // SHOULD be isr safe...
  serial_printf("i2c Addressed: %d %d\r\n", direction, address);
  if (valid_address)
  {
    xTaskNotifyFromISR(responder_task, request_direction, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void controller_handler(void* arguments)
{
  BaseType_t         task_status;
  uint32_t           i2c_status;
  uint8_t            current_channel = COMM_PORT_MAX;
  control_register_t multiplexer_status;
  network_message_t  network_message;
  vTaskDelay(pdMS_TO_TICKS(COMM_STACK_STARTUP_DELAY_MS));
  for (;;)
  {
    // Deselect Multiplexer channel
    vTaskDelay(pdMS_TO_TICKS(COMM_STACK_MULTIPLEXER_COOLDOWN_MS));
    multiplexer_status.channel = 0;
    multiplexer_status.enable  = 0;
    // Start Multiplexer query
    i2c1_send(MULTIPLEXER_ADDRESS, (uint8_t*)&multiplexer_status, 1, comm_stack_controller_i2c_handler);
    task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (pdFALSE == task_status)
    {
      handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_TIMEOUT);
      continue;
    }
    if (I2C_SUCCESS != i2c_status)
    {
      handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_COMM);
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(COMM_STACK_MULTIPLEXER_COOLDOWN_MS));
    // Start Multiplexer query: read the status register
    i2c1_receive(MULTIPLEXER_ADDRESS, (uint8_t*)&multiplexer_status, sizeof(control_register_t), comm_stack_controller_i2c_handler);
    task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (pdFALSE == task_status)
    {
      handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_TIMEOUT);
      continue;
    }

    if (I2C_SUCCESS != i2c_status)
    {
      handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_COMM);
      continue;
    }

    // Check that we have exclusive control of at least one bus
    // INTs are active low, so look for ANY low bit in the mask
    if ((~multiplexer_status.interrupts & CONNECTED_INTERRUPTS_MASK))
    {
      ++current_channel;
      if (COMM_PORT_MAX <= current_channel)
      {
        current_channel = COMM_PORT_A;
      }
      // Service one responder before switching contexts to give other tasks a chance
      if (~multiplexer_status.interrupts & CHANNEL_MASK(current_channel))
      {
        // Select Multiplexer channel to enable/connect
        multiplexer_status.enable  = 1;
        multiplexer_status.channel = current_channel;

        i2c1_send(MULTIPLEXER_ADDRESS, (uint8_t*)&multiplexer_status, sizeof(control_register_t), comm_stack_controller_i2c_handler);
        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (pdFALSE == task_status)
        {
          handle_controller_error(i2c_status, task_status, current_channel, ERROR_TIMEOUT);
          continue;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          handle_controller_error(i2c_status, task_status, current_channel, ERROR_COMM);
          continue;
        }
        // TEST: Send a whoami to get information about what is connected downstream
        controller_message_cache.header = COMMAND_WHOAMI;
        controller_message_cache.length = 1;
        // Whoami should contain the address we EXPECT address (responder should reply to any/all)
        controller_message_cache.data[0] = input_channel_addresses[current_channel];
        i2c1_send(input_channel_addresses[current_channel],
                  (uint8_t*)&controller_message_cache,
                  controller_message_cache.length + COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                  comm_stack_controller_i2c_handler);
        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (pdFALSE == task_status)
        {
          handle_controller_error(i2c_status, task_status, current_channel, ERROR_TIMEOUT);
          continue;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          handle_controller_error(i2c_status, task_status, current_channel, ERROR_COMM);
          continue;
        }

        // Read the message header (status) from device
        i2c1_receive(input_channel_addresses[current_channel], &controller_message_cache.header, 1, comm_stack_controller_i2c_handler);
        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        // If we have timed out here, it most likely means that input and output ports are mismatched, for example, AI connected to CO
        display_set_rgb(output_channel_led[current_channel], 255, 0, 0);
        if (pdFALSE == task_status)
        {
          handle_controller_error(i2c_status, task_status, current_channel, ERROR_TIMEOUT);
          continue;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          // Unknown error
          handle_controller_error(i2c_status, task_status, current_channel, ERROR_COMM);
          continue;
        }
        // We have a response, but we don't know if it is correct yet
        display_set_rgb(input_channel_led[current_channel], 255, 153, 51);
        // The response header should indicate success
        if (RESPONSE_SUCCESS != controller_message_cache.header)
        {
          handle_controller_error(i2c_status, task_status, current_channel, controller_message_cache.header);
          display_set_rgb(output_channel_led[current_channel], 255, 255, 0);
          continue;
        }

        // Read length info (uint8_t)
        i2c1_receive(input_channel_addresses[current_channel], &controller_message_cache.length, 1, comm_stack_controller_i2c_handler);
        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));

        if (pdFALSE == task_status)
        {
          handle_controller_error(i2c_status, task_status, current_channel, ERROR_TIMEOUT);
          continue;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          handle_controller_error(i2c_status, task_status, current_channel, ERROR_COMM);
          continue;
        }

        // Read body of message, [length] bytes
        i2c1_receive(input_channel_addresses[current_channel],
                     (uint8_t*)&controller_message_cache.data,
                     controller_message_cache.length,
                     comm_stack_controller_i2c_handler);
        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        // TODO: Dispatch message to internal event system
        // For now, just test alignment: is it the address we asked for?
        if (controller_message_cache.data[0] == input_channel_addresses[current_channel])
        {
          // We are aligned
          display_set_rgb(output_channel_led[current_channel], 0, 255, 0);
        }
        else
        {
          // I/O port mismatch
          display_set_rgb(output_channel_led[current_channel], 204, 0, 204);
        }
        // Deselect Multiplexer channel
        multiplexer_status.channel = 0;
        multiplexer_status.enable  = 0;
        i2c1_send(MULTIPLEXER_ADDRESS, (uint8_t*)&multiplexer_status, 1, comm_stack_controller_i2c_handler);
        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (pdFALSE == task_status)
        {
          handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_TIMEOUT);
          continue;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_COMM);
          continue;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(COMM_STACK_MULTIPLEXER_COOLDOWN_MS));
      multiplexer_status.channel = 0;
      multiplexer_status.enable  = 0;
      // Start Multiplexer query
      i2c1_send(MULTIPLEXER_ADDRESS, (uint8_t*)&multiplexer_status, 1, comm_stack_controller_i2c_handler);
      task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
      if (pdFALSE == task_status)
      {
        handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_TIMEOUT);
        continue;
      }
      if (I2C_SUCCESS != i2c_status)
      {
        handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_COMM);
        continue;
      }
      vTaskDelay(pdMS_TO_TICKS(COMM_STACK_MULTIPLEXER_COOLDOWN_MS));
      // ================================== NETWORK MESSAGES ================================== //
      // Dequeue any messages we are waiting to send
      while ((uxQueueMessagesWaiting(network_message_queue) > 0) &&
             (pdTRUE == xQueueReceive(network_message_queue, &network_message, pdMS_TO_TICKS(COMM_STACK_MESSAGE_PERIOD))))
      {
        // Select Multiplexer channel
        multiplexer_status.enable = 1;
        // Determine channel
        multiplexer_status.channel = network_message.target_port;

        i2c1_send(MULTIPLEXER_ADDRESS, (uint8_t*)&multiplexer_status, sizeof(control_register_t), comm_stack_controller_i2c_handler);
        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (pdFALSE == task_status)
        {
          handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_TIMEOUT);
          continue;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_COMM);
          continue;
        }

        // Write data to the responder

        i2c1_send(input_channel_addresses[network_message.target_port],
                  (uint8_t*)&network_message.message,
                  network_message.message.length + COMM_PROTOCOL_MESSAGE_HEADER_LENGTH,
                  comm_stack_controller_i2c_handler);

        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (pdFALSE == task_status)
        {
          handle_controller_error(i2c_status, task_status, network_message.target_port, ERROR_TIMEOUT);
          continue;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          handle_controller_error(i2c_status, task_status, network_message.target_port, ERROR_COMM);
          continue;
        }

        // Read response (ack)
        i2c1_receive(input_channel_addresses[network_message.target_port],
                     (uint8_t*)&controller_message_cache,
                     sizeof(protocol_message_t),
                     comm_stack_controller_i2c_handler);
        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (pdFALSE == task_status)
        {
          handle_controller_error(i2c_status, task_status, network_message.target_port, ERROR_TIMEOUT);
          continue;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          handle_controller_error(i2c_status, task_status, network_message.target_port, ERROR_COMM);
          continue;
        }

        if (RESPONSE_SUCCESS != controller_message_cache.header)
        {
          handle_controller_error(i2c_status, task_status, network_message.target_port, controller_message_cache.header);
          continue;
        }

        // TODO: Handle any response (if required)

        // Deselect Multiplexer channel
        multiplexer_status.channel = 0;
        multiplexer_status.enable  = 0;
        // Start Multiplexer query
        i2c1_send(MULTIPLEXER_ADDRESS, (uint8_t*)&multiplexer_status, 1, comm_stack_controller_i2c_handler);
        task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (pdFALSE == task_status)
        {
          handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_TIMEOUT);
          continue;
        }

        if (I2C_SUCCESS != i2c_status)
        {
          handle_controller_error(i2c_status, task_status, COMM_PORT_MULTIPLEXER, ERROR_COMM);
          continue;
        }
      }
    }
  }
}

void responder_handler(void* arguments)
{
  BaseType_t task_status;
  uint8_t    channel_select = COMM_PORT_A;
  uint32_t   i2c_status;
  uint32_t   request;
  xTimerStart(responder_timer, portMAX_DELAY);
  for (;;)
  {
    // Wait for an incoming request, either from an interrupt or sotfware timer
    taskYIELD();
    task_status = xTaskNotifyWait(pdFALSE, 0, &request, portMAX_DELAY);
    // Check if it's a channel-change request
    if (RESPONDER_REQUEST_TDMA == request)
    {
      // Advance channels
      channel_select++;
      if (channel_select > COMM_PORT_C)
      {
        channel_select = COMM_PORT_A;
      }
      comm_stack_responder_signal_ready(COMM_PORT_ALL, COMM_PORT_NOT_READY);
      vTaskDelay(pdMS_TO_TICKS(RESPONDER_SETTLE_TIME_MS));
      // Falling-edge detection / active-low
      comm_stack_responder_signal_ready(channel_select, COMM_PORT_READY);
      continue;
    }
    else
    {
      // Something has addressed us, lock the channel
      xTimerStop(responder_timer, portMAX_DELAY);
      switch (request)
      {
        case RESPONDER_REQUEST_READ:
          // The controller would like us to give it data
          {
            i2c2_receive(responder_read_buffer, RESPONDER_MAX_READ, comm_stack_responder_i2c_handler);
            task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));

            if (pdFALSE == task_status)
            {
              handle_responder_error(i2c_status, task_status, ERROR_TIMEOUT);
            }
            // Set orange: we have a reply, but is it correct
            display_set_rgb(output_channel_led[responder_channel], 255, 128, 0);
            protocol_message_t incoming_msg;
            incoming_msg.header = responder_read_buffer[COMM_PROTOCOL_HEADER_INDEX];
            incoming_msg.length = responder_read_buffer[COMM_PROTOCOL_LENGTH_INDEX];
            if (incoming_msg.length > 0)
            {
              memcpy(incoming_msg.data, responder_read_buffer + COMM_PROTOCOL_DATA_START, incoming_msg.length);
            }
            if (COMMAND_WHOAMI == incoming_msg.header)
            {
              // For now, just respond with address
              uint8_t expected_address = incoming_msg.data[0];
              if (expected_address == input_channel_addresses[responder_channel])
              {
                display_set_rgb(output_channel_led[responder_channel], 0, 255, 0);
              }
              protocol_message_t outgoing_msg = {
                .header = RESPONSE_SUCCESS,
                .length = 1,
              };
              outgoing_msg.data[0] = input_channel_addresses[responder_channel];
              i2c2_send((uint8_t*)&outgoing_msg,
                        COMM_PROTOCOL_MESSAGE_HEADER_LENGTH + outgoing_msg.length,
                        comm_stack_responder_i2c_handler);
              task_status = xTaskNotifyWait(pdFALSE, 0, &i2c_status, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            }
          }
          break;
        case RESPONDER_REQUEST_WRITE:
          // The controller would like to send data to us
          {
          }
          break;
        case RESPONDER_TRANSACTION_COMPLETE:
          // We've been notified the transaction is finished, clean up
          {
          }
          break;
        default:
          break;
      }
      // We're done servicing the request, unlock the channel
      xTimerStart(responder_timer, pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS));
    }
  }
}

/**
 * @brief Handles the GPIO triggered interrupt
 *
 */
void comm_stack_controller_interrupt_handler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTaskNotifyFromISR(controller_task, CONTROLLER_MASK_INTERRUPT, eSetBits, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void comm_stack_init(void)
{
  network_message_queue = xQueueCreateStatic(NETWORK_MESSAGE_QUEUE_MAX_LENGTH,
                                             sizeof(network_message_t),
                                             network_message_queue_buffer,
                                             &network_message_queue_impl);

  responder_task = xTaskCreateStatic(responder_handler,
                                     "responder_task",
                                     COMM_TASK_STACK_SIZE,
                                     NULL,
                                     COMM_TASK_STACK_PRIORITY + 1,
                                     responder_stack_buffer,
                                     &responder_tcb_buffer);

  controller_task = xTaskCreateStatic(controller_handler,
                                      "controller_task",
                                      COMM_TASK_STACK_SIZE,
                                      NULL,
                                      COMM_TASK_STACK_PRIORITY,
                                      controller_stack_buffer,
                                      &controller_tcb_buffer);

  responder_timer = xTimerCreateStatic("restim",
                                       pdMS_TO_TICKS(RESPONDER_TDMA_PERIOD_MS + RESPONDER_SETTLE_TIME_MS),
                                       pdTRUE,
                                       NULL,
                                       responder_timer_handler,
                                       &responder_timer_buffer);

  i2c2_set_address_callback(handle_i2c2_address);

  // The interrupt logic is falling edge, so drive each master high first
  comm_stack_responder_signal_ready(COMM_PORT_ALL, COMM_PORT_NOT_READY);
  for (uint8_t i = 0; i < COMM_PORT_MAX; ++i)
  {
    display_set_rgb(output_channel_led[i], 0, 0, 128);
    display_set_rgb(input_channel_led[i], 0, 0, 128);
  }
}
