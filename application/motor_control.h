#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

void motor_control_init(void);
void motor_control_background(void);
void motor_control_process(void);
void motor_control_tick(uint32_t now);
void motor_control_operational(bool operational);
uint16_t motor_control_mapping(uint16_t *input_size, uint16_t *output_size);
void motor_control_input(void *data);
void motor_control_output(const void *data);
void *mc_object(uint16_t index);

uint8_t MC_AssignWrite(uint16_t index, uint8_t subindex, uint32_t bytes,
                       uint16_t *data, uint8_t complete);
uint8_t MC_OD_Read(uint16_t index, uint8_t subindex, uint32_t bytes,
                   uint16_t *data, uint8_t complete);
uint8_t MC_OD_Write(uint16_t index, uint8_t subindex, uint32_t bytes,
                    uint16_t *data, uint8_t complete);

#endif
