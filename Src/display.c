
#include "display.h"

#include "FreeRTOS.h"

#include "fast_hsv2rgb.h"
#include "leds.h"
#include "priorities.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include <stdint.h>
#include <string.h>

//------------------------------------
// Macro
//------------------------------------

#define COLORS_PER_LED (3)

#define OTHER_INDEX (0)
#define TX_INDEX (1)

#define DISPLAY_MAX_WAIT_MS (10)
#define DISPLAY_UPDATE_MS (5)

#define DISPLAY_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)

#define PATTERN_PERIOD_DEFAULT_MS (1000)

//------------------------------------
// Type Definition
//------------------------------------

typedef struct
{
  pattern_t pattern; // Pattern
  uint32_t substate; // Pattern-defined substate
  TickType_t timer; // Pattern-defined timer in ticks
  TickType_t period; // Pattern-defined period in ticks
  uint8_t led_r;
  uint8_t led_g;
  uint8_t led_b;
} pattern_data_t;

//------------------------------------
// Variable Declaration
//------------------------------------

static StackType_t stack_buffer[DISPLAY_TASK_STACK_SIZE];
static StaticTask_t tcb_buffer;
static TaskHandle_t display_task;

static pattern_data_t patterns[LED_COUNT];

static SemaphoreHandle_t display_semaphore;
static StaticSemaphore_t display_mutex;

//------------------------------------
// Private Functions
//------------------------------------

void pattern_dispatch(uint8_t led_index, pattern_data_t* pattern_ref)
{
  TickType_t time = xTaskGetTickCount();
  if (NULL != pattern_ref)
  {
    TickType_t scale_factor = ((time - pattern_ref->timer) << 8) / pattern_ref->period;
    switch (pattern_ref->pattern)
    {
    case PATTERN_SOLID:
      // Transfer the color data
      display_set_rgb(led_index, pattern_ref->led_r, pattern_ref->led_g, pattern_ref->led_b);
      break;
    case PATTERN_BREATHE:
      {
        switch (pattern_ref->substate)
        {
        case 0:
          {
            // Fade in
            uint8_t r, g, b;
            // Gives a value between 0-255 in periods
            r = (scale_factor * pattern_ref->led_r) >> 8;
            g = (scale_factor * pattern_ref->led_g) >> 8;
            b = (scale_factor * pattern_ref->led_b) >> 8;

            if (scale_factor >= 255)
            {
              //Advance to hold
              pattern_ref->substate = 1;
              pattern_ref->timer = time;
            }
            else
            {
              display_set_rgb(led_index, r, g, b);
            }
            break;
          }
        case 1:
          // Hold for 2 period
          display_set_rgb(led_index, pattern_ref->led_r, pattern_ref->led_g, pattern_ref->led_b);
          if (scale_factor >= 512)
          {
            pattern_ref->substate = 2;
            pattern_ref->timer = time;
          }
          break;
        case 2:
          {
            // Fade out
            uint8_t r, g, b;
            r = (pattern_ref->led_r * (255 - scale_factor)) >> 8;
            g = (pattern_ref->led_g * (255 - scale_factor)) >> 8;
            b = (pattern_ref->led_b * (255 - scale_factor)) >> 8;

            
            if (scale_factor >= 255)
            {
              pattern_ref->substate = 3;
              pattern_ref->timer = time;
            }
            else
            {
              display_set_rgb(led_index, r, g, b);
            }
          }
          break;
        case 3:
          display_set_rgb(led_index, 0, 0, 0);
          if (scale_factor >= 255)
          {
            pattern_ref->substate = 0;
            pattern_ref->timer = time;
          }
          break;
        default:
          pattern_ref->substate = 0;
          pattern_ref->timer = time;
          break;
        }
      }
      break;
    case PATTERN_HEARTBEAT:
      {
        // Calculate the position in the period
        // ON - OFF - ON - OFF - Rest (4)
        if ((0 == pattern_ref->substate) ||
            (2 == pattern_ref->substate)) 
        {
          display_set_rgb(led_index, pattern_ref->led_r, pattern_ref->led_g, pattern_ref->led_b);
        }
        else
        {
          display_set_rgb(led_index, 0, 0, 0);
        }
        
        if (scale_factor >= 32)
        {
          ++pattern_ref->substate;
          pattern_ref->timer = time;
          if (7 <= pattern_ref->substate)
          {
            pattern_ref->substate = 0;
          }
        }
      }
      break;
    case PATTERN_BLINK:
      {
        // ON - OFF
        if ((1 == pattern_ref->substate)) 
        {
          display_set_rgb(led_index, pattern_ref->led_r, pattern_ref->led_g, pattern_ref->led_b);
        }
        else
        {
          display_set_rgb(led_index, 0, 0, 0);
        }

        if (scale_factor >= 64)
        {
          ++pattern_ref->substate;
          pattern_ref->timer = time;
          if (pattern_ref->substate > 1)
          {
            pattern_ref->substate = 0;
          }
        }
      }
      break;
    case PATTERN_SOS:
      {
        // ---___--- Rest ()
        const uint32_t mask = 0b0001010100111011101110010101;
        if ((1 << pattern_ref->substate) & mask)
        {
          display_set_rgb(led_index, pattern_ref->led_r, pattern_ref->led_g, pattern_ref->led_b);
        }
        else
        {
          display_set_rgb(led_index, 0, 0, 0);
        }

        if (scale_factor >= 32)
        {
          ++pattern_ref->substate;
          pattern_ref->timer = time;
          if (pattern_ref->substate > 31)
          {
            pattern_ref->substate = 0;
          }
        }
      }
      break;
    case PATTERN_NONE: // Don't mess with
      break;
    default:
      display_set_rgb(led_index, 0, 0, 0);
      break;
    }
  }
}

void pattern_engine_run(void)
{
  for(uint8_t led = 0; led < LED_COUNT; ++led)
  {
    pattern_dispatch(led, &patterns[led]);
  }
}


void display_update_complete_handler(int32_t error, uintptr_t userdata)
{
    // Possibly called from ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(display_task, TX_INDEX, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void display_task_handler(void *argument)
{
  for (;;)
  {
    xSemaphoreTake(display_semaphore, portMAX_DELAY);
    pattern_engine_run();
    update_leds(display_update_complete_handler);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xSemaphoreGive(display_semaphore);
    vTaskDelay(pdMS_TO_TICKS(DISPLAY_UPDATE_MS));
  }
}

//------------------------------------
// Public Functions
//------------------------------------

void display_init(void)
{
  display_semaphore = xSemaphoreCreateMutexStatic(&display_mutex);
  
  for (uint8_t led = 0; led < LED_COUNT; ++led)
  {
    clear_pattern(led);
  }

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
}

void display_set_rgb(uint8_t led, uint8_t r, uint8_t g, uint8_t b)
{
  xSemaphoreTake(display_semaphore, pdMS_TO_TICKS(DISPLAY_MAX_WAIT_MS));
  set_rgb(led, r, g, b);
  xSemaphoreGive(display_semaphore);
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

void set_pattern_rgb(pattern_t pattern, uint8_t led, uint8_t r, uint8_t g, uint8_t b)
{
  if (led < LED_COUNT)
  {
    patterns[led].pattern = pattern;
    patterns[led].led_r = r;
    patterns[led].led_g = g;
    patterns[led].led_b = b;
    // Only used by breathe for now
    patterns[led].timer = xTaskGetTickCount();
    patterns[led].period = PATTERN_PERIOD_DEFAULT_MS;
  }
}

void clear_pattern(uint8_t led)
{
  if (led < LED_COUNT)
  {
    patterns[led].pattern = PATTERN_NONE;
  }
}