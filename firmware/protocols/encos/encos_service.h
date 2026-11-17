#ifndef DCU_ENCOS_SERVICE_H
#define DCU_ENCOS_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "can_frame.h"

typedef struct {
    uint16_t words[4];
    uint8_t group;
} encos_parameter_values_t;

#ifdef __cplusplus
extern "C" {
#endif
bool encos_query(uint8_t code, uint8_t group, dcu_can_frame_t *frame);
/* Values are protocol words, not EtherCAT storage. Failure leaves frame unchanged. */
bool encos_parameter(uint16_t parameter, const encos_parameter_values_t *values,
                     dcu_can_frame_t *frame);
uint8_t encos_parameter_query(uint16_t parameter);
uint16_t encos_be16(const uint8_t *data);
uint32_t encos_be32(const uint8_t *data);
bool encos_float(const uint8_t *data, double scale, int32_t *value);
#ifdef __cplusplus
}
#endif
#endif
