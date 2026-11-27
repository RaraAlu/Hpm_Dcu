/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "production_init.h"
#include "ethercat_can_task.h"
#include "runtime/detail/ethercat_can_task_state.h"
#include "realtime_port.h"

static dcu_ethercat_can_task_state_t ecat_can;
_Static_assert(DCU_WORKER_TASK_PRIORITY < configMAX_PRIORITIES, "ecat-can priority exceeds the ready lists");

int main(void)
{
    if (!dcu_production_boot(DCU_PDO_MOTOR_WITH_RAW)) return 0;

    ecat_can.handle = xTaskCreateStatic(ecat_can_task, "ecat-can", DCU_WORKER_STACK_WORDS, NULL,
                                        DCU_WORKER_TASK_PRIORITY, ecat_can.stack, &ecat_can.tcb);
    configASSERT(ecat_can.handle != NULL);
    vTaskStartScheduler();
    dcu_rtos_assert(__FILE__, __LINE__);
    return 0;
}

void dcu_notify_ecat_can_from_isr(void)
{
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(ecat_can.handle, &wake);
    portYIELD_FROM_ISR(wake);
}
