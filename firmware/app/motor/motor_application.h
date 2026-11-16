#ifndef DCU_MOTOR_APPLICATION_H
#define DCU_MOTOR_APPLICATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize once before scheduler start. All remaining calls belong to the
 * ecat-can task; ISR paths must use the cycle/CAN handoff instead. */
void motor_control_init(void);
void motor_control_process(void);
void motor_control_operational(bool operational);
void motor_control_cycle_fault(void);
void motor_control_output(const void *data);

#ifdef __cplusplus
}
#endif

#endif
