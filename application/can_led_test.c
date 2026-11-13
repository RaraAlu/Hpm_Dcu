/*
 * CAN0-CAN3 three-color LED test.
 * Each TX/RX/ERR LED is turned on for 500 ms in sequence.
 */

#include <stdio.h>
#include "can_led_test.h"
#include "board.h"
#include "hpm_gpio_drv.h"

#define CAN_LED_TEST_INTERVAL_MS (500U)

typedef struct {
    GPIO_Type *ctrl;
    uint32_t index;
    uint32_t pin;
    uint8_t on_level;
    uint8_t off_level;
    const char *name;
} can_led_test_item_t;

static const can_led_test_item_t s_can_leds[] = {
    { BOARD_CAN0_TX_LED_CTRL,  BOARD_CAN0_TX_LED_INDEX,  BOARD_CAN0_TX_LED_PIN,  BOARD_CAN0_TX_LED_ON_LEVEL,  BOARD_CAN0_TX_LED_OFF_LEVEL,  "CAN0 TX"  },
    { BOARD_CAN0_RX_LED_CTRL,  BOARD_CAN0_RX_LED_INDEX,  BOARD_CAN0_RX_LED_PIN,  BOARD_CAN0_RX_LED_ON_LEVEL,  BOARD_CAN0_RX_LED_OFF_LEVEL,  "CAN0 RX"  },
    { BOARD_CAN0_ERR_LED_CTRL, BOARD_CAN0_ERR_LED_INDEX, BOARD_CAN0_ERR_LED_PIN, BOARD_CAN0_ERR_LED_ON_LEVEL, BOARD_CAN0_ERR_LED_OFF_LEVEL, "CAN0 ERR" },
    { BOARD_CAN1_TX_LED_CTRL,  BOARD_CAN1_TX_LED_INDEX,  BOARD_CAN1_TX_LED_PIN,  BOARD_CAN1_TX_LED_ON_LEVEL,  BOARD_CAN1_TX_LED_OFF_LEVEL,  "CAN1 TX"  },
    { BOARD_CAN1_RX_LED_CTRL,  BOARD_CAN1_RX_LED_INDEX,  BOARD_CAN1_RX_LED_PIN,  BOARD_CAN1_RX_LED_ON_LEVEL,  BOARD_CAN1_RX_LED_OFF_LEVEL,  "CAN1 RX"  },
    { BOARD_CAN1_ERR_LED_CTRL, BOARD_CAN1_ERR_LED_INDEX, BOARD_CAN1_ERR_LED_PIN, BOARD_CAN1_ERR_LED_ON_LEVEL, BOARD_CAN1_ERR_LED_OFF_LEVEL, "CAN1 ERR" },
    { BOARD_CAN2_TX_LED_CTRL,  BOARD_CAN2_TX_LED_INDEX,  BOARD_CAN2_TX_LED_PIN,  BOARD_CAN2_TX_LED_ON_LEVEL,  BOARD_CAN2_TX_LED_OFF_LEVEL,  "CAN2 TX"  },
    { BOARD_CAN2_RX_LED_CTRL,  BOARD_CAN2_RX_LED_INDEX,  BOARD_CAN2_RX_LED_PIN,  BOARD_CAN2_RX_LED_ON_LEVEL,  BOARD_CAN2_RX_LED_OFF_LEVEL,  "CAN2 RX"  },
    { BOARD_CAN2_ERR_LED_CTRL, BOARD_CAN2_ERR_LED_INDEX, BOARD_CAN2_ERR_LED_PIN, BOARD_CAN2_ERR_LED_ON_LEVEL, BOARD_CAN2_ERR_LED_OFF_LEVEL, "CAN2 ERR" },
    { BOARD_CAN3_TX_LED_CTRL,  BOARD_CAN3_TX_LED_INDEX,  BOARD_CAN3_TX_LED_PIN,  BOARD_CAN3_TX_LED_ON_LEVEL,  BOARD_CAN3_TX_LED_OFF_LEVEL,  "CAN3 TX"  },
    { BOARD_CAN3_RX_LED_CTRL,  BOARD_CAN3_RX_LED_INDEX,  BOARD_CAN3_RX_LED_PIN,  BOARD_CAN3_RX_LED_ON_LEVEL,  BOARD_CAN3_RX_LED_OFF_LEVEL,  "CAN3 RX"  },
    { BOARD_CAN3_ERR_LED_CTRL, BOARD_CAN3_ERR_LED_INDEX, BOARD_CAN3_ERR_LED_PIN, BOARD_CAN3_ERR_LED_ON_LEVEL, BOARD_CAN3_ERR_LED_OFF_LEVEL, "CAN3 ERR" },
};

void can_led_test_run(void)
{
    board_init_can_led_pins();
    printf("CAN LED cycle test: each LED stays on for 500 ms\n");

    while (1) {
        for (uint32_t i = 0; i < ARRAY_SIZE(s_can_leds); i++) {
            printf("%s LED ON\n", s_can_leds[i].name);
            gpio_write_pin(s_can_leds[i].ctrl, s_can_leds[i].index,
                           s_can_leds[i].pin, s_can_leds[i].on_level);
            board_delay_ms(CAN_LED_TEST_INTERVAL_MS);
            gpio_write_pin(s_can_leds[i].ctrl, s_can_leds[i].index,
                           s_can_leds[i].pin, s_can_leds[i].off_level);
        }
    }
}
