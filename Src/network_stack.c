#include "network_stack.h"

#include "FreeRTOS.h"

#include "comm_protocol.h"
#include "comm_stack.h"
#include "device_ops.h"
#include "display.h"
#include "i2c.h"
#include "priorities.h"
#include "task.h"

#include <stddef.h>
#include <stdint.h>

#define NETWORK_TEST_MS (3000)
#define NETWORK_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)

typedef enum
{
  NETWORK_STARTUP = 0,
  NETWORK_GET_ADDRESS,
  NETWORK_IDLE,
  NETWORK_BUSY,
} network_driver_state_t;

// static network_driver_state_t network_driver_state = NETWORK_STARTUP;

static uint8_t output_channel_led[COMM_PORT_MAX] = {
  1,
  3,
  5,
};

static StackType_t  stack_buffer[NETWORK_TASK_STACK_SIZE];
static StaticTask_t tcb_buffer;
static TaskHandle_t network_task;

void network_operation_callback(int32_t status, uintptr_t userdata);

void network_task_handler(void* userdata)
{
  protocol_message_t msg;
  uint32_t           status;
  uint8_t            port = COMM_PORT_A;
  for (;;)
  {
    // TEST: Request and set some LEDS
    vTaskDelay(pdMS_TO_TICKS(NETWORK_TEST_MS));
    msg.header  = DEVICE_OP_CHECK_PORT;
    msg.length  = 1;
    msg.data[0] = port;
    set_pattern_rgb(PATTERN_SOLID, output_channel_led[port], 0, 0, 0);
    comm_stack_write(port, &msg, network_operation_callback);
    if (pdFALSE == xTaskNotifyWait(0, 0, &status, pdMS_TO_TICKS(NETWORK_TEST_MS)))
    {
      ++port;
      if (port >= COMM_PORT_MAX)
      {
        port = COMM_PORT_A;
      }
      continue;
    }
    if (I2C_SUCCESS == status)
    {
      set_pattern_rgb(PATTERN_BREATHE, output_channel_led[port], 0, 0, 128);
      comm_stack_read(port, &msg, network_operation_callback);
      if (pdFALSE == xTaskNotifyWait(0, 0, &status, pdMS_TO_TICKS(NETWORK_TEST_MS)))
      {
        continue;
      }
      if (I2C_SUCCESS == status)
      {
        if (msg.header == RESPONSE_SUCCESS && msg.length >= 1)
        {
          if (msg.data[0] == port)
          {
            set_pattern_rgb(PATTERN_BREATHE, output_channel_led[port], 0, 128, 0);
          }
          else
          {
            set_pattern_rgb(PATTERN_SOLID, output_channel_led[port], 128, 0, 0);
          }
        }
        else
        {
          set_pattern_rgb(PATTERN_SOS, output_channel_led[port], 128, 0, 0);
        }
      }
    }
    ++port;
    if (port >= COMM_PORT_MAX)
    {
      port = COMM_PORT_A;
    }
  }
}

void network_operation_callback(int32_t status, uintptr_t userdata)
{
  xTaskNotify(network_task, status, eSetValueWithOverwrite);
}

void network_init(void)
{
  network_task = xTaskCreateStatic(network_task_handler,
                                   "network_task",
                                   NETWORK_TASK_STACK_SIZE,
                                   NULL,
                                   NETWORK_TASK_PRIORITY,
                                   stack_buffer,
                                   &tcb_buffer);
}
