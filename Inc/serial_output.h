#ifndef SERIAL_OUTPUT_H
#define SERIAL_OUTPUT_H

#include <stdarg.h>

/**
 * @brief Sends characters in debug mode
 *
 * @param str String to send
 * @return int Characters sent/copied
 */
int serial_print(const char* str);
/**
 * @brief Sends a formatted string in debug mode
 *
 * @param format formatstring
 * @param ... varargs
 * @return int Characters send/copied
 */
int serial_printf(const char* format, ...);

#endif // SERIAL_OUTPUT_H