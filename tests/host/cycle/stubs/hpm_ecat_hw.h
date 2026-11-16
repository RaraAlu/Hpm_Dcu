#ifndef TEST_ECAT_HW_H
#define TEST_ECAT_HW_H
#include <stdint.h>
extern uint32_t test_esc_period, test_esc_activation;
#define HW_EscReadDWord(value, address) \
    ((value) = (address) == 0x9A0U ? test_esc_period : test_esc_activation)
#endif
