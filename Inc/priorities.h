#ifndef PRIORITIES_H
#define PRIORITIES_H

// Display should be relatively high priority to service leds
#define DISPLAY_TASK_PRIORITY (16)
// Uart may be lower priority
#define UART_TASK_PRIORITY (4)
// Timer requires high priority to service phy-level comm
#define TIMER_PRIORITY (24)
// Logical network functions can generally be deferred
#define NETWORK_TASK_PRIORITY (9)

#endif // PRIORITIES_H