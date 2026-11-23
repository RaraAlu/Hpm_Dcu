#ifndef TEST_CANFD_CLOCK_H
#define TEST_CANFD_CLOCK_H
#include <stdint.h>
enum { clock_ahb0 };
uint32_t clock_get_frequency(unsigned clock);
#endif
