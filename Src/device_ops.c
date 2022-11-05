#include "device_ops.h"

#include "comm_protocol.h"
#include "comm_stack.h"
#include "display.h"
#include "network_stack.h"

#include <string.h>

static uint8_t input_channel_led[COMM_PORT_MAX] = {
  0,
  2,
  4,
};

void device_op_execute(protocol_message_t* device_op_data, protocol_message_t* response_buffer)
{
  comm_port_t port = responder_get_port();
  if ((NULL != device_op_data) && (NULL != response_buffer))
  {
    switch (device_op_data->header)
    {
      case DEVICE_OP_CHECK_PORT:
        if (device_op_data->length >= 1)
        {
          if (device_op_data->data[0] == port)
          {
            set_pattern_rgb(PATTERN_BREATHE, input_channel_led[port], 0, 128, 0);
          }
          else
          {
            set_pattern_rgb(PATTERN_BLINK, input_channel_led[port], 128, 0, 0);
          }
          response_buffer->header  = RESPONSE_SUCCESS;
          response_buffer->length  = 1;
          response_buffer->data[0] = port;
        }
        else
        {
          response_buffer->header = RESPONSE_ERROR_INVALID_PARAMETER;
          response_buffer->length = 0;
          set_pattern_rgb(PATTERN_SOS, input_channel_led[port], 128, 0, 0);
        }
        comm_stack_signal_exec_complete();
        break;
      default:
        break;
    }
  }
}