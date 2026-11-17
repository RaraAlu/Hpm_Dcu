#ifndef DCU_ETHERCAT_CAN_TASK_H
#define DCU_ETHERCAT_CAN_TASK_H

#include <stdint.h>

typedef struct {
    uint32_t worker_stack_free_words;
    uint32_t fatal_line;
    const char *fatal_file;
} dcu_rtos_diagnostics_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Worker-task snapshot. Fatal handling halts the CPU; inspect the private
 * instance with a stopped debugger after a fatal error. */
void dcu_rtos_read_diagnostics(dcu_rtos_diagnostics_t *snapshot);
void dcu_rtos_start(void);
void dcu_rtos_assert(const char *file, unsigned long line);

#ifdef __cplusplus
}
#endif
#endif
