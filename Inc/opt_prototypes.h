#ifndef OPT_PROTOTYPES_H
#define OPT_PROTOTYPES_H

#include <stdint.h>

/**
 * @brief A callback that accepts and error code and address (pointer to userdata)
 * @param status
 * @param userdata
 */
typedef void (*opt_callback_t)(int32_t, uintptr_t);

#endif // OPT_PROTOTYPES_H
