#include "serial_output.h"

#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifdef DEBUG
static char message_buffer[MAX_MESSAGE_CONTENT_LENGTH + 1];
#endif

int serial_print(const char* str)
{
  int len = 0;
#ifdef DEBUG
  if (NULL != str)
  {
    strncpy(message_buffer, str, MAX_MESSAGE_CONTENT_LENGTH);
    len = strlen(str);
    message_buffer[MAX_MESSAGE_CONTENT_LENGTH] = '\0';
    send_message(message_buffer);
  }
#else
  UNUSED(str);
#endif
  return len;
}

int serial_printf(const char* format, ...)
{
  int len = 0;
#ifdef DEBUG
  if (NULL != format)
  {
    va_list arg;

    va_start(arg, format);
    len = vsnprintf(message_buffer, MAX_MESSAGE_CONTENT_LENGTH, format, arg);
    va_end(arg);
    message_buffer[MAX_MESSAGE_CONTENT_LENGTH] = '\0';
    send_message(message_buffer);
  }
#else
  UNUSED(format);
#endif
  return len;
}
