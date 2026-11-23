#ifndef TEST_ECAT_HW_H
#define TEST_ECAT_HW_H
#include <stdbool.h>
#include <stdint.h>
bool hw_esc_publish_pdo_before(uint16_t address, const void *src, uint16_t size, uint32_t deadline_us);
extern uint32_t test_esc_period, test_esc_activation;
extern uint16_t test_al_event;
#define HW_GetALEventRegister() (test_al_event)
#define ESC_SYSTEMTIME_OFFSET 0x910U
#define HW_EscReadDWord(value, address) \
    ((value) = (address) == 0x9A0U ? test_esc_period : test_esc_activation)
#endif
