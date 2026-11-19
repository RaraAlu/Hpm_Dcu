#ifndef DCU_DATA_MCAN_ECHO_H
#define DCU_DATA_MCAN_ECHO_H

#include <stdbool.h>
#include <stddef.h>

bool dcu_data_mcan_init(void);
bool dcu_data_mcan_enable(void);
void dcu_data_mcan_disable(void);
bool dcu_data_mcan_stage(const void *data, size_t size);
bool dcu_data_mcan_healthy(void);
void dcu_data_mcan_poll(void);

#endif
