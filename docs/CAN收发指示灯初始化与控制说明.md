# CAN 收发指示灯初始化与控制说明

## 1. 文档目的

本文记录参考工程 `kh-ethercat-canfdx4-op/Firmware` 中 4 路 CAN FD 状态指示灯的引脚定义、初始化过程和运行时控制逻辑，供 `HPM_ethercat_canfd_encos_V1.0` 工程配置和移植时使用。

参考实现主要位于：

- `hpm_ethercat_sdk/application/can_app.c`
- `hpm_ethercat_sdk/application/can_app.h`
- `hpm_ethercat_sdk/hpm_sdk_localized_for_hpm5e00evk/boards/hpm5e00evk/board.c`
- `hpm_ethercat_sdk/hpm_sdk_localized_for_hpm5e00evk/boards/hpm5e00evk/board.h`

## 2. LED 引脚和有效电平

原理图为每路 CAN 引出了 TX、RX、ERR 三个 LED 控制信号，可组成三色状态指示。原理图中的完整连接为：

| CAN 通道 | 原理图网络 | HPM 引脚 | GPIO 端口/位 | 参考固件状态 |
|---|---|---|---|---|
| CAN0 | LED_TX0 | PD02 | GPIOD[2] | 已配置、已控制 |
| CAN0 | LED_RX0 | PC16 | GPIOC[16] | 已配置、已控制 |
| CAN0 | LED_ERR0 | PC13 | GPIOC[13] | 未配置、未控制 |
| CAN1 | LED_TX1 | PD07 | GPIOD[7] | 已配置、已控制 |
| CAN1 | LED_RX1 | PD06 | GPIOD[6] | 已配置、已控制 |
| CAN1 | LED_ERR1 | PD03 | GPIOD[3] | 未配置、未控制 |
| CAN2 | LED_TX2 | PB29 | GPIOB[29] | 已配置、已控制 |
| CAN2 | LED_RX2 | PB30 | GPIOB[30] | 已配置、已控制 |
| CAN2 | LED_ERR2 | PB31 | GPIOB[31] | 未配置、未控制 |
| CAN3 | LED_TX3 | PC12 | GPIOC[12] | 已配置、已控制 |
| CAN3 | LED_RX3 | PC11 | GPIOC[11] | 已配置、已控制 |
| CAN3 | LED_ERR3 | PC10 | GPIOC[10] | 未配置、未控制 |

参考固件只定义并控制了表中的 8 个 TX/RX 引脚，全部是低电平点亮：调用 `gpio_write_pin(..., 0)` 点亮，调用 `gpio_write_pin(..., 1)` 熄灭。

原理图虽然定义了 `LED_ERR0～3` 网络，但这 4 个信号没有出现在参考固件的 `BOARD_CANx_ERR_LED_*` 宏、`init_can_led_pins()` 或 CAN 应用逻辑中。因此 ERR 灯当前完全没有被软件使用，其上电状态取决于芯片复位状态及外部电路，不能作为可靠的 CAN 错误指示。

## 3. PinMux 配置

### 3.1 HPM PinMux Tool 中的配置

参考固件在板级 `tool_config.hpmpc` 中建立 `init_can_led_pins` 初始化组，只将以下 TX/RX 引脚配置为 GPIO：

| 引脚 | PinMux 信号 |
|---|---|
| PD02 | `GPIO.A.D[02]` |
| PC16 | `GPIO.A.C[16]` |
| PD07 | `GPIO.A.D[07]` |
| PD06 | `GPIO.A.D[06]` |
| PB29 | `GPIO.A.B[29]` |
| PB30 | `GPIO.A.B[30]` |
| PC12 | `GPIO.A.C[12]` |
| PC11 | `GPIO.A.C[11]` |

生成的 `pinmux.c` 应包含：

```c
void init_can_led_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PD02].FUNC_CTL = IOC_PD02_FUNC_CTL_GPIO_D_02;
    HPM_IOC->PAD[IOC_PAD_PC16].FUNC_CTL = IOC_PC16_FUNC_CTL_GPIO_C_16;
    HPM_IOC->PAD[IOC_PAD_PD07].FUNC_CTL = IOC_PD07_FUNC_CTL_GPIO_D_07;
    HPM_IOC->PAD[IOC_PAD_PD06].FUNC_CTL = IOC_PD06_FUNC_CTL_GPIO_D_06;
    HPM_IOC->PAD[IOC_PAD_PB29].FUNC_CTL = IOC_PB29_FUNC_CTL_GPIO_B_29;
    HPM_IOC->PAD[IOC_PAD_PB30].FUNC_CTL = IOC_PB30_FUNC_CTL_GPIO_B_30;
    HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PC12_FUNC_CTL_GPIO_C_12;
    HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PC11_FUNC_CTL_GPIO_C_11;
}
```

`pinmux.h` 中应有对应声明：

```c
void init_can_led_pins(void);
```

目标工程现已将下面 4 个 ERR 引脚加入该初始化组：

| 原理图网络 | 引脚 | PinMux 信号 |
|---|---|---|
| LED_ERR0 | PC13 | `GPIO.A.C[13]` |
| LED_ERR1 | PD03 | `GPIO.A.D[03]` |
| LED_ERR2 | PB31 | `GPIO.A.B[31]` |
| LED_ERR3 | PC10 | `GPIO.A.C[10]` |

目标工程的 `init_can_led_pins()`、`board.h` 和 `board_init_can_led_pins()` 也已同步增加 ERR 引脚 PinMux、板级宏和上电熄灭初始化；ERR 运行时亮灭接口仍需随 CAN 错误处理逻辑实现。

> 优先使用 HPM PinMux Tool 修改并重新生成 `pinmux.c`、`pinmux.h`。如果必须手工修改 `.hpmpc`，还必须按照 HPM PinMux Tool 0.6.0 的规则重新计算并验证 `clientKey`，否则工具会报告“文件已失效”。

### 3.2 PC16 复用冲突

目标工程当前还把 PC16 用于以下候选功能：

- CAN0_RX_LED：GPIOC[16]
- Brownout 指示：GPIOC[16]
- UART4_TX：PC16
- PPI0_DQ16：PC16

这些初始化函数不一定都会被应用调用，但 PC16 在运行时只能承担一种用途。启用 CAN0_RX_LED 后，不应再调用会重新配置或驱动 PC16 的 `board_init_brownout_indicate_pin()`、`init_uart4_pins()` 或 `init_ppi_pins()`。

当前 `init_brownout_indicate_pin()` 与 CAN LED 都把 PC16 配成 GPIO，但 Brownout 初始化还会把 PC16 输出为低电平，这会直接让 CAN0 RX 灯常亮。因此移植时必须确认 Brownout 功能没有使用 PC16，或者给其中一个功能更换引脚。

### 3.3 ERR 引脚复用冲突

目标工程的通用板级配置还在其他可选初始化函数中使用 ERR 引脚：

| ERR 信号 | 引脚 | 其他候选功能 |
|---|---|---|
| LED_ERR0 | PC13 | PPI0_DQ18、SPI1_MOSI、QEO1_A、MCAN3_STBY |
| LED_ERR1 | PD03 | I2C2_SDA、PWM0_P3、MCAN1_STBY |
| LED_ERR2 | PB31 | ESP Hosted DATA_READY 等 |
| LED_ERR3 | PC10 | PPI0_DQ20、SPI1_SCLK、MCAN2_STBY |

将它们配置为 CAN ERR LED 前，必须根据本板实际应用确认相应初始化函数不会被调用。尤其不能同时把同一引脚作为 MCAN 收发器 STBY 和 ERR LED 输出。

## 4. board.h 中的板级定义

每个 TX/RX/ERR LED 都需要定义 GPIO 控制器、端口、引脚号以及亮灭电平。格式如下：

```c
#define BOARD_CAN0_TX_LED_CTRL      HPM_GPIO0
#define BOARD_CAN0_TX_LED_INDEX     GPIO_DI_GPIOD
#define BOARD_CAN0_TX_LED_PIN       2
#define BOARD_CAN0_TX_LED_OFF_LEVEL 1
#define BOARD_CAN0_TX_LED_ON_LEVEL  0

#define BOARD_CAN0_RX_LED_CTRL      HPM_GPIO0
#define BOARD_CAN0_RX_LED_INDEX     GPIO_DI_GPIOC
#define BOARD_CAN0_RX_LED_PIN       16
#define BOARD_CAN0_RX_LED_OFF_LEVEL 1
#define BOARD_CAN0_RX_LED_ON_LEVEL  0

#define BOARD_CAN0_ERR_LED_CTRL      HPM_GPIO0
#define BOARD_CAN0_ERR_LED_INDEX     GPIO_DI_GPIOC
#define BOARD_CAN0_ERR_LED_PIN       13
#define BOARD_CAN0_ERR_LED_OFF_LEVEL 1
#define BOARD_CAN0_ERR_LED_ON_LEVEL  0
```

CAN1～CAN3 按第 2 节表格使用相同格式定义。ERR 灯低电平有效是根据同组 TX/RX LED 电路作出的移植预期，正式使用前仍应核对完整原理图中的 LED 公共端和限流电阻连接。

建议在 `board.h` 中提供以下板级接口：

```c
void board_init_can_led_pins(void);
void board_can_tx_led_on(uint32_t can_index);
void board_can_tx_led_off(uint32_t can_index);
void board_can_rx_led_on(uint32_t can_index);
void board_can_rx_led_off(uint32_t can_index);
void board_can_err_led_on(uint32_t can_index);
void board_can_err_led_off(uint32_t can_index);
```

## 5. GPIO 初始化过程

应用启动时先调用：

```c
board_init_can_led_pins();
```

该函数执行两个步骤：

1. 调用 `init_can_led_pins()`，把 8 个 PAD 切换为 GPIO 功能。
2. 将 8 个 GPIO 配置为输出，并以熄灭电平 `1` 初始化，避免上电时误亮。

参考代码：

```c
void board_init_can_led_pins(void)
{
    init_can_led_pins();

    gpio_set_pin_output_with_initial(BOARD_CAN0_TX_LED_CTRL, BOARD_CAN0_TX_LED_INDEX,
        BOARD_CAN0_TX_LED_PIN, BOARD_CAN0_TX_LED_OFF_LEVEL);
    gpio_set_pin_output_with_initial(BOARD_CAN0_RX_LED_CTRL, BOARD_CAN0_RX_LED_INDEX,
        BOARD_CAN0_RX_LED_PIN, BOARD_CAN0_RX_LED_OFF_LEVEL);

    /* CAN1、CAN2、CAN3 按相同方式初始化。 */
}
```

目标工程已经存在 `board_init_can_led_pins()` 及 12 个 TX/RX/ERR 输出的初始熄灭配置，并已在 `ecat.c` 启动流程中调用。

## 6. LED 亮灭操作

应用层不直接关心具体 GPIO，而是用 `can_index` 选择 CAN0～CAN3：

```c
void board_can_tx_led_on(uint32_t can_index)
{
    switch (can_index) {
    case 0:
        gpio_write_pin(BOARD_CAN0_TX_LED_CTRL, BOARD_CAN0_TX_LED_INDEX,
            BOARD_CAN0_TX_LED_PIN, BOARD_CAN0_TX_LED_ON_LEVEL);
        break;
    case 1:
        gpio_write_pin(BOARD_CAN1_TX_LED_CTRL, BOARD_CAN1_TX_LED_INDEX,
            BOARD_CAN1_TX_LED_PIN, BOARD_CAN1_TX_LED_ON_LEVEL);
        break;
    case 2:
        gpio_write_pin(BOARD_CAN2_TX_LED_CTRL, BOARD_CAN2_TX_LED_INDEX,
            BOARD_CAN2_TX_LED_PIN, BOARD_CAN2_TX_LED_ON_LEVEL);
        break;
    case 3:
        gpio_write_pin(BOARD_CAN3_TX_LED_CTRL, BOARD_CAN3_TX_LED_INDEX,
            BOARD_CAN3_TX_LED_PIN, BOARD_CAN3_TX_LED_ON_LEVEL);
        break;
    default:
        break;
    }
}
```

`board_can_tx_led_off()`、`board_can_rx_led_on()`、`board_can_rx_led_off()` 使用相同结构，只需替换对应的 TX/RX 宏和 ON/OFF 电平。

目标工程目前只有 LED 引脚初始化函数，还没有这 4 个运行时亮灭接口。

## 7. 应用层状态和定时闪烁

### 7.1 参数

参考工程定义：

```c
#define CAN_LED_BLINK_PERIOD_US       100000U
#define CAN_LED_ACTIVITY_TIMEOUT_MS     100U
```

使用 1 ms 定时回调后，一个完整闪烁周期为 100 ms：

- 前 50 ms 点亮；
- 后 50 ms 熄灭；
- 最后一次收发活动超过 100 ms 后退出活动状态。

### 7.2 每通道状态

```c
typedef struct {
    volatile bool tx_active;
    volatile bool rx_active;
    volatile bool channel_initialized;
    volatile uint32_t tx_last_tick;
    volatile uint32_t rx_last_tick;
} can_led_state_t;

static can_led_state_t s_can_led_state[4];
static volatile uint32_t s_timer_tick;
```

### 7.3 参考工程的显示规则

| 状态 | TX LED | RX LED |
|---|---|---|
| 通道未初始化 | 熄灭 | 常亮 |
| 通道初始化成功、当前无活动 | 常亮 | 熄灭 |
| 发送成功后的约 100 ms | 闪烁 | 不受影响 |
| 接收成功后的约 100 ms | 不受影响 | 闪烁 |

因此参考工程的 TX 灯不是“平时熄灭、发送时点亮”，而是“初始化后常亮、发送时闪烁”。RX 灯在通道未初始化时常亮，这只能作为间接状态提示，并不是明确的错误灯。

如果目标产品希望更直观的行为，可以把空闲状态统一改成熄灭，但这属于产品显示规则调整，不是原参考程序行为。

## 8. 收发事件如何触发 LED

### 8.1 接收成功

各通道轮询 RX FIFO。只有实际取到报文并返回 `status_success` 时，才记录 RX 活动：

```c
if (status == status_success) {
    s_can_led_state[can_index].rx_active = true;
    s_can_led_state[can_index].rx_last_tick = s_timer_tick;
    /* 后续处理接收帧。 */
}
```

没有报文时不改变 LED。持续收到报文会不断刷新 `rx_last_tick`，所以 RX 灯持续闪烁。

### 8.2 发送成功

EtherCAT 输出 PDO 更新后先置发送 pending 标志，主循环发现 pending 后调用 `hpm_can_transmit()`。只有发送函数返回成功时才记录 TX 活动：

```c
status = hpm_can_transmit(can_index, &tx_frame);
if (status == status_success) {
    s_can_led_state[can_index].tx_active = true;
    s_can_led_state[can_index].tx_last_tick = s_timer_tick;
}
```

发送失败不会触发 TX 闪烁。

## 9. CAN 错误灯现状

原理图有每通道独立的 `LED_ERR0～3`。参考固件没有对其进行 PinMux、GPIO 初始化或运行时控制；目标工程现已补齐 PinMux 和 GPIO 上电初始化，但仍没有把以下状态映射到 ERR/TX/RX LED：

- CAN Error Warning；
- CAN Error Passive；
- CAN Bus-Off；
- 接收错误；
- 发送失败。

初始化失败和发送失败目前只通过 `printf()` 输出状态码。EtherCAT RUN/ERR LED 属于 EtherCAT 状态显示，与 CAN 错误无关，不能当作 CAN ERROR LED 使用。

如果目标工程需要 CAN 错误指示，可直接使用原理图预留的 PC13、PD03、PB31、PC10，但必须补齐 PinMux、板级宏和 GPIO 接口，再读取 MCAN 协议状态或错误中断进行控制。建议至少让 Error Warning、Error Passive 或 Bus-Off 使对应 ERR 灯常亮，并在通道恢复正常后熄灭；具体规则应结合产品需求确定。

## 10. 目标工程的定时器冲突

目标工程当前在 `ecat.c` 中使用：

```c
board_timer_create(1000, board_led_toggle);
```

它每 1 秒翻转一次 PA22 系统灯。`board_timer_create()` 内部只有一个静态回调指针，并固定使用 GPTMR3 通道 1。因此不能再简单调用一次：

```c
board_timer_create(1, can_led_timer_callback);
```

第二次调用会覆盖回调并重新配置同一个定时器，导致 PA22 系统灯或 CAN LED 定时逻辑失效。

建议目标工程统一使用一个 1 ms 回调：

```c
static void app_led_timer_callback(void)
{
    static uint32_t sys_led_ms;

    can_led_timer_callback();

    if (++sys_led_ms >= 1000U) {
        sys_led_ms = 0;
        board_led_toggle();
    }
}
```

初始化顺序为：

```c
board_init();
board_init_led_pins();
board_init_can_led_pins();
board_timer_create(1, app_led_timer_callback);
```

也可以为 CAN LED 单独使用另一个硬件定时器，但当前需求没有必要增加第二套定时器。

## 11. 目标工程当前状态和移植清单

当前已经具备：

- `tool_config.hpmpc` 中的 `init_can_led_pins` 已配置 12 个 TX/RX/ERR GPIO；
- `pinmux.c/.h` 已生成 `init_can_led_pins()`；
- `board.h` 已有 12 个 LED 的 GPIO、引脚号、ON/OFF 电平定义；
- `board.c` 已有 `board_init_can_led_pins()`，能把 12 个灯初始化为熄灭；
- `ecat.c` 已在启动时调用 `board_init_can_led_pins()`；
- `.hpmpc` 已通过 PinMux 工具重新签发 `secretKey`，并按修改后的 `content` 重新计算 `clientKey`。

当前仍需实现或接入：

1. 解决并确认 PC16 及 4 个 ERR 引脚的功能归属；
2. 增加 TX/RX/ERR LED 的 `on/off` 板级接口；
3. 增加 4 路 CAN LED 活动及错误状态；
4. 在 CAN 收发成功路径更新时间戳，在错误状态变化路径更新 ERR 灯；
5. 将现有 PA22 系统灯和 CAN LED 合并到同一个 1 ms 定时回调；
6. 上板验证所有 LED 的实际极性与原理图一致。

## 12. 上板验证建议

1. 初始化后逐个调用 12 个 LED 控制通道的 `on()`/`off()`，确认通道、颜色、方向和丝印没有接反。
2. 不接 CAN 总线时，确认空闲显示符合第 7.3 节。
3. 分别向 CAN0～CAN3 接收一帧，确认只有对应 RX 灯闪烁。
4. 分别从 CAN0～CAN3 成功发送一帧，确认只有对应 TX 灯闪烁。
5. 断开终端电阻或制造无 ACK 条件，确认发送失败不会被误认为发送成功，并验证 ERR 灯规则。
6. 连续通信时确认 LED 持续闪烁，停止通信约 100 ms 后恢复空闲状态。
7. 同时观察 PA22，确认合并定时器后仍保持每 1 秒翻转一次。

## 13. `.hpmpc` 无法打开时的重点检查项

本次故障现象为打开 `tool_config.hpmpc` 时弹出“操作失败！错误码：30003”。根因是直接向 JSON 中加入了 4 个 ERR LED 引脚，只重新计算了 `clientKey`，但 `secretKey` 仍对应修改前的 8 引脚配置。客户端文件校验可以通过，但 PinMux 本地服务会把内容与签名不一致的文件判定为 `The file is damaged`。

以后遇到文件打不开，按以下顺序检查：

1. 确认文件是合法 JSON，重点检查 `clientKey`、`secretKey`、`content` 三个顶层字段。
2. 检查 `init_can_led_pins.selectPins` 中新增的 4 个 ERR 引脚及信号是否正确：
   - PC13：`GPIO.A.C[13]`
   - PD03：`GPIO.A.D[03]`
   - PB31：`GPIO.A.B[31]`
   - PC10：`GPIO.A.C[10]`
3. 如果手工改过 `content`，仅重算 `clientKey` 不一定足够。应优先在 HPM PinMux Tool 中重新添加引脚并保存/下载配置，让工具同步更新 `secretKey`、`content.pid` 和 `content.info.updateTime`。
4. 保存后重新计算并核对 `clientKey`。HPM PinMux Tool 0.6.0 的算法是：

   ```text
   lowercase_md5(secretKey + "." + JSON.stringify(content) + ".@hpmicro")
   ```

5. 错误提示可用于快速区分检查方向：
   - “文件已失效”或 `The file is invalid`：优先检查 `clientKey` 是否过期或计算错误。
   - 错误码 `30003` 或 `The file is damaged`：优先检查 `secretKey` 是否仍对应当前 `content`，以及新增引脚是否由 PinMux 工具正式写入。
6. 修复后必须重新打开文件验证；必要时先关闭编辑页并执行 VS Code“重新加载窗口”，排除扩展缓存。

本次最终配置已通过 PinMux 本地导入接口验证，返回 `200 success`。以后修改 `.hpmpc` 时，不要只手工添加引脚并更新 `clientKey`，还要完成 PinMux 工具侧的重新签发。
