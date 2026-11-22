#ifndef DCU_MOTOR_ETHERCAT_H
#define DCU_MOTOR_ETHERCAT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Worker-owned; ISR paths use the cycle/CAN handoff instead. */
/* Returns false only while a staged PDO still awaits processing. */
bool motor_control_process(void);
void motor_control_operational(bool operational);
void motor_control_discard_profile(void);
void motor_control_cycle_fault(void);
void motor_control_output(const void *data);

uint16_t motor_control_mapping(uint16_t *input_size, uint16_t *output_size);
void motor_control_prepare_input(void);
bool motor_control_neutral_feedback_pending(void);
void motor_control_set_neutral_window(uint32_t period_ns, uint32_t cutoff_us);
void motor_control_neutral_feedback_deadline(void);
/* SSC-owned RAM storage, borrowed once before the worker starts. */
void *dcu_ssc_txpdo_buffer(uint16_t *capacity);

uint8_t MC_AssignWrite(uint16_t index, uint8_t subindex, uint32_t bytes,
                       uint16_t *data, uint8_t complete);
uint8_t MC_OD_Read(uint16_t index, uint8_t subindex, uint32_t bytes,
                   uint16_t *data, uint8_t complete);
uint8_t MC_OD_Write(uint16_t index, uint8_t subindex, uint32_t bytes,
                    uint16_t *data, uint8_t complete);

#ifdef __cplusplus
}
#endif
#endif
