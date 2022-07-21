
#include "display.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "leds.h"
#include "fast_hsv2rgb.h"

#include <stdint.h>
#include <string.h>

//------------------------------------
// Macro
//------------------------------------

#define COLORS_PER_LED (3)
#define LED_COUNT (6)

#define TIMER_INDEX (0)
#define TX_INDEX (1)

//------------------------------------
// Type Definition
//------------------------------------

//------------------------------------
// Variable Declaration
//------------------------------------

#define DISPLAY_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#define DISPLAY_TASK_PRIORITY (12)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static StackType_t stack_buffer[DISPLAY_TASK_STACK_SIZE];
static StaticTask_t tcb_buffer;
static TaskHandle_t display_task;
static TimerHandle_t display_timer_id;
static StaticTimer_t display_timer;

//------------------------------------
// Private Functions
//------------------------------------

void display_update_complete_handler(uint8_t error, uintptr_t userdata)
{
    // Possibly called from ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(display_task, TX_INDEX, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void display_task_handler(void *argument)
{
    uint32_t notification;
    uint32_t led = 0;
    uint16_t hue = 0;
    uint16_t hue_step = 1;
    uint16_t hue_gap = hue_step * LED_COUNT * 32;
    for(;;)
    {
        for(led = 0; led < LED_COUNT; ++led)
        {
            set_rgb(led, 0,0,0);
        }
        notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (TIMER_INDEX == notification)
        {
            for(led = 0; led < LED_COUNT; ++led)
            {
                uint16_t lhue = hue + hue_gap * led;
                if (lhue > HSV_HUE_MAX)
                {
                    lhue -= HSV_HUE_MAX;
                }
                set_hsv(led, lhue, 200, 80);
            }
            hue += hue_step;
            if (hue > HSV_HUE_MAX)
            {
                hue -= HSV_HUE_MAX;
            }
            update_leds(display_update_complete_handler);
            notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            xTimerReset(display_timer_id, pdMS_TO_TICKS(0));
        }
    }
}

void display_timer_handler(TimerHandle_t th)
{
    // Called from task
    xTaskNotify(display_task, TIMER_INDEX, eSetValueWithOverwrite);

    portYIELD();
}

//------------------------------------
// Public Functions
//------------------------------------

void display_init(void)
{
    display_task = xTaskCreateStatic(display_task_handler,
                                        "display_task",
                                        DISPLAY_TASK_STACK_SIZE,
                                        NULL,
                                        DISPLAY_TASK_PRIORITY,
                                        stack_buffer,
                                        &tcb_buffer);
    
    display_timer_id = xTimerCreateStatic(
        "display_timer",
        pdMS_TO_TICKS(5),
        pdFALSE,
        0,
        display_timer_handler,
        &display_timer);

    if (NULL != display_timer_id)
    {
        xTimerStart(display_timer_id,
                    pdMS_TO_TICKS(100));
    }
}
