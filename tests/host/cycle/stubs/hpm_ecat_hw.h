#ifndef TEST_ECAT_HW_H
#define TEST_ECAT_HW_H
#include <stdint.h>
extern uint32_t test_esc_period, test_esc_activation;
extern uint16_t test_al_event;
#define HW_GetALEventRegister() (test_al_event)
#define ESC_SYSTEMTIME_OFFSET 0x910U
#define HW_EscReadDWord(value, address) \
    ((value) = (address) == 0x9A0U ? test_esc_period : test_esc_activation)
#endif
