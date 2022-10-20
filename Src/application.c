#include "application.h"

#include "FreeRTOS.h"

#include "comm_stack.h"
#include "display.h"
#include "reset_info.h"
#include "serial_output.h"
#include "timers.h"

void init_application(void)
{
  display_init();
  serial_printf("\n\r\nReset reason: %s\r\n", reset_cause_get_name(get_reset_cause()));
  comm_stack_init();
}

void update_application(void)
{
  vTaskDelay(pdMS_TO_TICKS(100UL));
}