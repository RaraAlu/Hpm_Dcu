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
void motor_control_raw_link_lost(void);
void motor_control_mailbox_stopped(void);
void motor_control_output(const void *data);
bool motor_control_preview_supported(void);
void motor_control_preview(const void *data);
bool motor_control_event_pdo(void);
uint32_t motor_control_event_lease_us(void);

uint16_t motor_control_mapping(uint16_t *input_size, uint16_t *output_size);
bool motor_control_commit_topology(uint32_t generation);
bool motor_control_verify_contract(void);
void motor_control_prepare_input(void);
void motor_control_clear_raw_input(void);
bool motor_control_neutral_feedback_pending(void);
void motor_control_set_cyclic_feedback_window(uint32_t period_us, uint32_t deadline_us);
uint32_t motor_control_cyclic_feedback_required(void);
uint32_t motor_control_cyclic_feedback_missing(void);
void motor_control_fault_cyclic_feedback(uint32_t missing);
void motor_control_set_neutral_window(uint32_t period_ns, uint32_t cutoff_us);
void motor_control_neutral_feedback_deadline(void);
/* SSC-owned RAM storage, borrowed once before the worker starts. */
void *dcu_ssc_txpdo_buffer(uint16_t *capacity);
uint16_t dcu_ssc_station_address(void);

enum { DCU_SDO_2101_PDO_LAYOUT = 0x2101, DCU_SDO_2103_PDO_TOPOLOGY = 0x2103 };
typedef uint8_t dcu_sdo_write_fn(uint16_t index, uint8_t subindex, uint32_t bytes,
                                 uint16_t *data, uint8_t complete);
bool dcu_ssc_bind_sdo_write(uint16_t index, dcu_sdo_write_fn *write);
dcu_sdo_write_fn dcu_pdo_layout_sdo_write;
dcu_sdo_write_fn dcu_pdo_topology_sdo_write;

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
