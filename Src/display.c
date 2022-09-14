
#include "display.h"

#include "FreeRTOS.h"

#include "fast_hsv2rgb.h"
#include "leds.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include <stdint.h>
#include <string.h>

//------------------------------------
// Macro
//------------------------------------

#define COLORS_PER_LED (3)
#define LED_COUNT (6)

#define OTHER_INDEX (0)
#define TX_INDEX (1)

#define DISPLAY_MAX_WAIT_MS (100)

//------------------------------------
// Type Definition
//------------------------------------

//------------------------------------
// Variable Declaration
//------------------------------------

#define DISPLAY_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#define DISPLAY_TASK_PRIORITY (6)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static StackType_t stack_buffer[DISPLAY_TASK_STACK_SIZE];
static StaticTask_t tcb_buffer;
static TaskHandle_t display_task;

static SemaphoreHandle_t display_semaphore;
static StaticSemaphore_t display_mutex;

//------------------------------------
// Private Functions
//------------------------------------

void display_update_complete_handler(uint8_t error, uintptr_t userdata)
{
    // Possibly called from ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(display_task, TX_INDEX, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void display_task_handler(void *argument)
{
    uint32_t notification;
    display_clear();
    for(;;)
    {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      xSemaphoreTake(display_semaphore, portMAX_DELAY);
      update_leds(display_update_complete_handler);
      do
      {
        notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      } while (TX_INDEX != notification);
      xSemaphoreGive(display_semaphore);
    }
}

//------------------------------------
// Public Functions
//------------------------------------

void display_init(void)
{
  display_semaphore = xSemaphoreCreateMutexStatic(&display_mutex);

  display_task = xTaskCreateStatic(display_task_handler,
                                   "display_task",
                                   DISPLAY_TASK_STACK_SIZE,
                                   NULL,
                                   DISPLAY_TASK_PRIORITY,
                                   stack_buffer,
                                   &tcb_buffer);
}

void display_set_hsv(uint8_t led, uint8_t h, uint8_t s, uint8_t v)
{
  xSemaphoreTake(display_semaphore, pdMS_TO_TICKS(DISPLAY_MAX_WAIT_MS));
  set_hsv(led, h, s, v);
  xSemaphoreGive(display_semaphore);
  display_request_update();
}

void display_set_rgb(uint8_t led, uint8_t r, uint8_t g, uint8_t b)
{
  xSemaphoreTake(display_semaphore, pdMS_TO_TICKS(DISPLAY_MAX_WAIT_MS));
  set_rgb(led, r, g, b);
  xSemaphoreGive(display_semaphore);
  display_request_update();
}

void display_set_fast_rgb(uint8_t start, uint8_t count, uint8_t* values)
{
  xSemaphoreTake(display_semaphore, pdMS_TO_TICKS(DISPLAY_MAX_WAIT_MS));
  for (uint8_t i = start; i < start + count; ++i)
  {
    set_rgb(i, *values, *(values + 1), *(values + 2));
    values += 3;
  }
  xSemaphoreGive(display_semaphore);
  display_request_update();
}
void display_set_fast_hsv(uint8_t start, uint8_t count, uint8_t* values)
{
  xSemaphoreTake(display_semaphore, pdMS_TO_TICKS(DISPLAY_MAX_WAIT_MS));
  for (uint8_t i = start; i < start + count; ++i)
  {
    set_hsv(i, *values, *(values + 1), *(values + 2));
    values += 3;
  }
  xSemaphoreGive(display_semaphore);
  display_request_update();
}

void display_clear(void)
{
  xSemaphoreTake(display_semaphore, pdMS_TO_TICKS(DISPLAY_MAX_WAIT_MS));
  for (uint8_t led = 0; led < LED_COUNT; ++led)
  {
    set_rgb(led, 0, 0, 0);
  }
  xSemaphoreGive(display_semaphore);
}

void display_request_update(void)
{
  xTaskNotifyGive(display_task);
}