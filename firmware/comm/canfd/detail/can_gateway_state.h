#ifndef DCU_CAN_GATEWAY_STATE_H
#define DCU_CAN_GATEWAY_STATE_H

#include <stdint.h>
#include "can_frame.h"
#include "hpm_gpio_drv.h"

typedef struct {
    dcu_can_frame_t frame;
    uint32_t timestamp;
} rx_item_t;

typedef struct {
    rx_item_t item[32];
    volatile uint8_t read, write;
} rx_queue_t;

typedef struct {
    GPIO_Type *ctrl;
    uint32_t index;
    uint32_t pin;
    uint8_t on_level;
    uint8_t off_level;
} canfd_led_t;

enum {
    CANFD_LED_TX = 0,
    CANFD_LED_RX,
    CANFD_LED_ERR,
    CANFD_LED_KIND_COUNT,
};

#endif
