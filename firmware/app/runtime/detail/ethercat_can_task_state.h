#ifndef DCU_ETHERCAT_CAN_TASK_STATE_H
#define DCU_ETHERCAT_CAN_TASK_STATE_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "../cfg/ethercat_can_task_config.h"

typedef struct {
    StaticTask_t tcb;
    StackType_t stack[DCU_WORKER_STACK_WORDS] __attribute__((aligned(16)));
    TaskHandle_t handle;
    uint32_t last_stack_check_ms;
} dcu_ethercat_can_task_state_t;

#endif
