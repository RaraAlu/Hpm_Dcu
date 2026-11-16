#ifndef TEST_CANFD_GPIO_H
#define TEST_CANFD_GPIO_H
#include "board.h"
void gpio_write_pin(GPIO_Type *base, unsigned index, unsigned pin, unsigned level);
#endif
