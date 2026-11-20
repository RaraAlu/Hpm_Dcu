#ifndef DCU_PRODUCTION_INIT_H
#define DCU_PRODUCTION_INIT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Called once before the worker starts, after CAN has applied the active Profile. */
bool motor_control_init(void);
bool motor_control_boot_startup(uint64_t boot_id);

#ifdef __cplusplus
}
#endif
#endif
