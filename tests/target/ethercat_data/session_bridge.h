#ifndef DCU_DATA_SESSION_BRIDGE_H
#define DCU_DATA_SESSION_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    DCU_SESSION_STATUS_INDEX = 0x2102,
    DCU_SESSION_ACTIVATE_INDEX = 0x2103,
    DCU_SESSION_RESULT_INDEX = 0x2104,
    DCU_SESSION_STATUS_BYTES = 80,
    DCU_SESSION_ACTIVATE_BYTES = 64,
    DCU_SESSION_RESULT_BYTES = 16,
};

bool dcu_data_session_boot(void);
void dcu_data_session_step(bool operational);
void dcu_data_session_pdo(const void *data, size_t size);
void dcu_data_session_can_complete(const void *data, size_t size);
size_t dcu_data_session_read(uint16_t index, void *data, size_t capacity);
bool dcu_data_session_activate(const void *data, size_t size);

#endif
