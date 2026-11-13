# HPM5E31 EtherCAT 转四路 CAN FD 实时透明透传方案

> 适用目标：HPM5E31 自研板，1 个 EtherCAT 从站，4 路独立 CAN FD；每路挂 8 个电机；每个电机每次收发 1 个 8 字节数据帧。默认 EtherCAT 周期为 1 ms（1 kHz）。  
> 本方案中的“透明”是指网关不解释电机 8 字节数据域，只搬运 CAN ID、帧属性和数据；CAN ID 仍作为透明帧头的一部分保留。

## 1. 结论先行

采用以下固定方案即可落地：

1. EtherCAT 使用 HPM5E31 内置 ESC 和 Distributed Clocks（DC）Sync0，周期 1 ms。
2. MCAN0～MCAN3 分别对应物理 CANFD1～CANFD4，四路同时启动发送；每路内部按槽位 1～8 顺序发送。
3. 每个电机槽位固定为 16 字节：`CAN_ID(4) + Flags(1) + DLC(1) + Seq(2) + Data[8]`。
4. 每路 PDO 增加 16 字节总线头，加 8 个 16 字节槽位，合计 144 字节；四路合计 576 字节。RxPDO 和 TxPDO 各 576 字节。
5. 下行采用 EtherCAT 输出双缓冲快照；上行采用每路 CAN 接收双缓冲环形槽。任何中断都不解析电机协议。
6. SSC Tool 中建立 4 个 RxPDO 和 4 个 TxPDO。XML/ESI 是主站识别文件，xlsx 是同一映射的人读设计源，两者必须按“对象索引、SubIndex、位长、顺序”逐项一致。
7. 1 kHz 下每路每周期若同时存在 8 个命令帧和 8 个反馈帧，1 Mbit/s 仲裁段 + 5 Mbit/s 数据段通常已接近总线极限，必须实测。推荐命令 1 kHz、反馈 500 Hz 交错；若要求反馈也为 1 kHz，应提高 CAN FD 数据相位速率并验证最坏占用率不超过 80%。

## 2. 参考资料核对与必要修正

本方案参考：

- `ENCOS文档/EtherCat教程/EtherCat-CAN转接板说明文档.pdf`
- `ENCOS文档/EtherCat教程/EtherCAT数据传输字典.xlsx`
- `ENCOS文档/EtherCat教程/SSC-IO.xml`
- `EtherCAT到CAN的透明透传实现机制详解.md`
- `自研ethercat转canfd板子接口定义/hpm5e00evk/board.[ch]` 和 `pinmux.[ch]`

ENCOS 参考板的真实映射为：

| 项目 | 参考板定义 |
| --- | --- |
| 物理 CAN 数量 | 2 路 |
| 每路发送槽位 | 3 个 |
| 下行应用对象 / RxPDO | `0x7010` / `0x1601` |
| 上行应用对象 / TxPDO | `0x6000` / `0x1A00` |
| 单向有效大小 | 68 个 PDO Entry，共 688 bit，即 86 byte |
| 发送顺序 | CAN1槽1、CAN2槽4、CAN1槽2、CAN2槽5、CAN1槽3、CAN2槽6 |
| EtherCAT 周期上限 | 文档标称 1000 Hz |

参考总结中的“通道 6 偏移 80～92”不正确。86 字节连续 PDO 的正确偏移是：2 字节公共头，加 6 个 14 字节帧；第 6 帧位于字节 `72～85`。XML 中 SubIndex 从 9 跳到 16，是为了按十六进制分组，并不代表 PDO 中存在填充字节；PDO Entry 仍按 XML 中出现的顺序连续排列。

另外，参考 XML 中 Vendor ID 为 `0x00000009`、Product Code 为 `0x26483052`，属于参考设备，不能直接用于自研产品。自研板必须使用本公司的 Vendor ID、Product Code、Revision Number 和设备名称。

## 3. 板级接口定义

### 3.1 四路 CAN FD 引脚

现有板级文件已经给出四个 MCAN 实例：

| 物理通道 | HPM 外设 | IRQ | TX 引脚 | RX 引脚 | 初始化入口 |
| --- | --- | --- | --- | --- | --- |
| CANFD1 | `HPM_MCAN0` | `IRQn_MCAN0` | `PD00 / MCAN0_TXD` | `PD01 / MCAN0_RXD` | `board_init_can(HPM_MCAN0)` |
| CANFD2 | `HPM_MCAN1` | `IRQn_MCAN1` | `PD05 / MCAN1_TXD` | `PD04 / MCAN1_RXD` | `board_init_can(HPM_MCAN1)` |
| CANFD3 | `HPM_MCAN2` | `IRQn_MCAN2` | `PC08 / MCAN2_TXD` | `PC09 / MCAN2_RXD` | `board_init_can(HPM_MCAN2)` |
| CANFD4 | `HPM_MCAN3` | `IRQn_MCAN3` | `PC15 / MCAN3_TXD` | `PC14 / MCAN3_RXD` | `board_init_can(HPM_MCAN3)` |

`board_init_can_clock()` 已分别把 CAN0～CAN3 外设时钟初始化为 80 MHz。`board_can_transceiver_phy_set()` 和 `init_can_transceiver_phy_pin()` 当前为空，说明现有定义没有软件控制收发器 STB/EN。原理图若存在 STB/EN 引脚，必须补充 GPIO 初始化并在进入 OP 前使能；若收发器硬件常开，应在板级文档中明确。

### 3.2 EtherCAT ESC 引脚

板级文件使用 HPM5E31 内置 `ESC0`，包括：

- `PA09`：ESC 参考时钟；`PA30/PA31`：MDIO/MDC。
- EtherCAT Port 0：`PA16～PA29` 中的 MII RX/TX 信号。
- EtherCAT Port 1：`PF02～PF14` 中的 MII RX/TX 信号。
- `PB26`：两路 EtherCAT PHY 共用复位。
- `PB27/PB28`：Port 0/1 Link 状态。

固件启动时调用 `board_init_ethercat(HPM_ESC)`（实际 ESC 宏以工程 SDK 为准），由板级函数初始化 ESC 时钟、MII 引脚和 PHY 复位。

### 3.3 硬件约束

- 每条 CAN FD 总线只在物理两端各放置 120 Ω 终端；板卡若位于总线中间，板载终端必须可断开。
- 四路必须各自使用支持目标数据相位速率的 CAN FD 收发器。
- CANH/CANL、地参考和供电布线要满足收发器手册；不能把四路 CAN 的 TX/RX 逻辑信号直接并联。
- 上电默认应保持电机不动作。EtherCAT 未进入 OP、输出看门狗超时或 CAN bus-off 时，不继续重复旧控制帧。

## 4. 总体数据链路和实时边界

```text
EtherCAT 主站
    │  1 ms RxPDO：4 × 144 B
    ▼
HPM5E31 ESC / SSC 输出映射
    │  Sync0 时只做快照和缓冲区切换
    ▼
四路 TX 队列 ──► MCAN0 ──► CANFD1 ──► 电机 1～8
             ├─► MCAN1 ──► CANFD2 ──► 电机 1～8
             ├─► MCAN2 ──► CANFD3 ──► 电机 1～8
             └─► MCAN3 ──► CANFD4 ──► 电机 1～8

四路 MCAN RX 中断
    │  仅保存 ID/Flags/DLC/Data，不解析 Data[8]
    ▼
四路 RX 双缓冲 ──► SSC 输入映射 ──► 1 ms TxPDO ──► EtherCAT 主站
```

透明边界如下：

- 网关检查帧头合法性，例如 ID 范围、`DLC=0～8`、FD 帧禁止 RTR、发送掩码和缓冲区容量。
- 网关不解释 `Data[0..7]` 的位置、速度、转矩、温度或配置命令含义。
- 上行默认按到达顺序写入 8 个接收槽，主站依据返回的 CAN ID 自己识别电机。这比固件维护电机 ID 表更透明。

## 5. 固定 PDO 数据结构

### 5.1 单个帧槽，16 字节

```c
typedef struct {
    uint32_t can_id;     /* 标准帧使用低 11 bit；扩展帧使用低 29 bit */
    uint8_t  flags;
    uint8_t  dlc;        /* 本项目电机帧固定为 8 */
    uint16_t seq;        /* 下行由主站填写；上行由从站递增 */
    uint8_t  data[8];    /* 原样透传 */
} canfd_slot_pdo_t;

_Static_assert(sizeof(canfd_slot_pdo_t) == 16, "PDO slot size error");
```

下行 `flags`：

| Bit | 名称 | 含义 |
| --- | --- | --- |
| 0 | `VALID` | 本周期该槽有效；同时还必须满足总线头 `valid_mask` 对应位为 1 |
| 1 | `IDE` | 0=11 位标准 ID，1=29 位扩展 ID |
| 2 | `RTR` | 远程帧；CAN FD 数据帧必须为 0 |
| 3 | `FDF` | 1=CAN FD，0=Classic CAN |
| 4 | `BRS` | 1=数据相位加速；仅 `FDF=1` 时有效 |
| 5 | `ON_CHANGE` | 1=仅当 `seq` 改变时发送；0=每个 EtherCAT 周期发送 |
| 6～7 | Reserved | 写 0 |

上行 `flags`：Bit0=`NEW`，Bit1=`IDE`，Bit2=`RTR`，Bit3=`FDF`，Bit4=`BRS`，Bit5=`ESI`，Bit6～7 保留。

### 5.2 单路总线 PDO，144 字节

```c
typedef struct {
    uint16_t cycle_seq;      /* EtherCAT 周期序号 */
    uint8_t  valid_mask;     /* Bit0～7 对应槽 1～8 */
    uint8_t  mode;           /* 下行：模式；上行：本周期 rx_count */
    uint16_t deadline_us;    /* 下行：本周期 CAN 发送预算，建议 800 */
    uint16_t bus_state;      /* 下行保留；上行记录 error-active/passive/bus-off */
    uint32_t counter0;       /* 下行保留；上行 lost_count */
    uint32_t counter1;       /* 下行保留；上行 bus_off_count/last_error */
    canfd_slot_pdo_t slot[8];
} canfd_bus_pdo_t;

typedef struct {
    canfd_bus_pdo_t bus[4];
} ethercat_canfd_process_image_t;

_Static_assert(sizeof(canfd_bus_pdo_t) == 144, "bus PDO size error");
_Static_assert(sizeof(ethercat_canfd_process_image_t) == 576,
               "process image size error");
```

建议把上下行声明为不同类型或至少使用不同变量，禁止同一内存同时作为 ESC 映射区和 MCAN ISR 写入区。

### 5.3 字节偏移

每路起始偏移为 `bus × 144`，槽位起始偏移为 `bus × 144 + 16 + slot × 16`，其中 `bus=0..3`、`slot=0..7`。

| 总线 | 单向全局偏移 | 槽 1 | 槽 8 |
| --- | ---: | ---: | ---: |
| CANFD1 | 0 | 16～31 | 128～143 |
| CANFD2 | 144 | 160～175 | 272～287 |
| CANFD3 | 288 | 304～319 | 416～431 |
| CANFD4 | 432 | 448～463 | 560～575 |

单槽内部偏移：ID=`+0..3`，Flags=`+4`，DLC=`+5`，Seq=`+6..7`，Data0～7=`+8..15`。EtherCAT/CoE 多字节基本类型按小端解释；CAN 数据域 8 字节保持原顺序，不转换字节序。

## 6. SSC Tool 对象与 PDO 规划

### 6.1 对象编号

| 方向 | CANFD1 | CANFD2 | CANFD3 | CANFD4 |
| --- | --- | --- | --- | --- |
| 输出应用对象（主站→从站） | `0x7000` | `0x7010` | `0x7020` | `0x7030` |
| RxPDO Mapping | `0x1600` | `0x1601` | `0x1602` | `0x1603` |
| 输入应用对象（从站→主站） | `0x6000` | `0x6010` | `0x6020` | `0x6030` |
| TxPDO Mapping | `0x1A00` | `0x1A01` | `0x1A02` | `0x1A03` |

`0x1C12` 的子项依次赋值 `0x1600,0x1601,0x1602,0x1603`；`0x1C13` 的子项依次赋值 `0x1A00,0x1A01,0x1A02,0x1A03`。四个 RxPDO 都分配到 SM2，四个 TxPDO 都分配到 SM3。

如果从参考 `SSC-IO.xml` 工程改造，必须先删除原有演示用的 `0x1A02 AI Inputs`、`0x6020` 模拟量对象及相关映射；本方案会把 `0x1A02/0x6020` 重新用于 CANFD3。原参考输入大小 92 B 是“86 B CAN 输入 + 6 B AI 演示输入”，不能把该 6 B 继续带入本方案。

SyncManager 建议值：

| SM | 起始地址 | 大小 | 用途 |
| --- | --- | ---: | --- |
| SM0 | `0x1000` | 128 B | Mailbox Out |
| SM1 | `0x1080` | 128 B | Mailbox In |
| SM2 | `0x1100` | 576 B (`0x240`) | Outputs / RxPDO |
| SM3 | `0x1800` | 576 B (`0x240`) | Inputs / TxPDO |

PDO SyncManager 使用三缓冲模式，因此 SM2 实际占用 `0x1100～0x17BF`；SM3 从 `0x1800` 开始，避免两个过程数据区重叠。最终地址仍需用 HPM5E31 ESC DPRAM 容量和 SSC Tool 的内存检查确认。

### 6.2 每个应用对象的 SubIndex 规则

每个 `0x70x0/0x60x0` 对象均为 RECORD。总线头使用 `0x01～0x07`；槽位 n 使用十六进制分组 `n0～nB`。SSC Tool 界面如果以十进制显示 SubIndex，`0x10` 会显示为 16，这是正常的。

| SubIndex | 字段 | EtherCAT 类型 | BitLen |
| --- | --- | --- | ---: |
| `0x01` | `CycleSeq` | `UINT` | 16 |
| `0x02` | `ValidMask/RxMask` | `USINT` | 8 |
| `0x03` | `Mode/RxCount` | `USINT` | 8 |
| `0x04` | `DeadlineUs/BusState` | `UINT` | 16 |
| `0x05` | `Reserved` | `UINT` | 16 |
| `0x06` | `Reserved/LostCount` | `UDINT` | 32 |
| `0x07` | `Reserved/BusOffCount` | `UDINT` | 32 |
| `0x10` | `Slot1_ID` | `UDINT` | 32 |
| `0x11` | `Slot1_Flags` | `USINT` | 8 |
| `0x12` | `Slot1_DLC` | `USINT` | 8 |
| `0x13` | `Slot1_Seq` | `UINT` | 16 |
| `0x14～0x1B` | `Slot1_Data0～Data7` | `USINT` | 每项 8 |
| `0x20～0x2B` | Slot2，格式同上 | 同上 | 128 bit/槽 |
| `...` | Slot3～Slot7 | 同上 | 同上 |
| `0x80～0x8B` | Slot8，格式同上 | 同上 | 128 bit/槽 |

注意：对象的 MaxSubIndex 是最高 SubIndex `0x8B`，但该对象实际只有 103 个有效子项（7 个头字段 + 8×12 个槽字段），中间保留空洞不要映射。一个 PDO 的 Entry 顺序必须是：头字段 1～7，然后槽1的 12项，直到槽8的 12项；合计 `16 + 8×16 = 144` 字节。

### 6.3 SSC Tool 操作顺序

1. 新建 HPM5E31 EtherCAT Slave 工程，启用 CoE、PDO、DC Sync0；周期能力设置为 1 ms。
2. 在 Object Dictionary 中创建上述 8 个应用 RECORD 对象，Access 分别为 RxPDO 对象 `RW`、TxPDO 对象 `RO`，并勾选 PDO Mapping。
3. 创建 `0x1600～0x1603`，按顺序映射四个输出对象；创建 `0x1A00～0x1A03`，按顺序映射四个输入对象。
4. 配置 `0x1C12/0x1C13` 和 SM2/SM3，检查每个 PDO 为 144 B、每个方向总计 576 B。
5. 生成 SSC 源码和 ESI XML。不要先手改生成 XML 再期待 SSC 工程自动同步；修改应回到 SSC Tool 工程后重新生成。
6. 在 TwinCAT/SOEM/IGH 中导入 ESI，确认从站进入 OP 后输出 576 B、输入 576 B，PDO 顺序与 C 结构体静态断言一致。

### 6.4 ESI XML 中一条 PDO Entry 的形式

下面只示意 CANFD1 输出槽1；最终 XML 应由 SSC Tool 生成：

```xml
<RxPdo Mandatory="true" Fixed="true" Sm="2">
  <Index>#x1600</Index>
  <Name>CANFD1 Outputs</Name>
  <Entry>
    <Index>#x7000</Index>
    <SubIndex>16</SubIndex><!-- 0x10 -->
    <BitLen>32</BitLen>
    <Name>OUT_CANFD1_SLOT1_ID</Name>
    <DataType>UDINT</DataType>
  </Entry>
  <Entry>
    <Index>#x7000</Index>
    <SubIndex>17</SubIndex><!-- 0x11 -->
    <BitLen>8</BitLen>
    <Name>OUT_CANFD1_SLOT1_FLAGS</Name>
    <DataType>USINT</DataType>
  </Entry>
  <!-- DLC、Seq、Data0..Data7，随后槽2..槽8 -->
</RxPdo>
```

`<SubIndex>` 在参考 ESI 中使用十进制文本，因此 `0x10` 写成 `16`、`0x8B` 写成 `139`。不要把十六进制分组号错误地当成连续十进制号。

## 7. xlsx 传输字典与 XML 对照表

xlsx 建议作为唯一的人读评审表，至少包含以下列：

| 列 | 含义 | 示例 |
| --- | --- | --- |
| Direction | 数据方向 | `Master->Slave` |
| Bus | 物理总线 | `CANFD1` |
| PDO Index | PDO Mapping 对象 | `0x1600` |
| Object Index | 应用对象 | `0x7000` |
| SubIndex Hex | SSC 设计编号 | `0x10` |
| SubIndex XML | XML 十进制值 | `16` |
| Name | 必须与 XML/C 含义一致 | `OUT_CANFD1_SLOT1_ID` |
| EtherCAT Type | ESI 类型 | `UDINT` |
| BitLen | 映射位长 | `32` |
| Byte Offset | 在整个方向过程映像中的偏移 | `16` |
| C Member | C 结构体字段 | `bus[0].slot[0].can_id` |
| Remark | 约束 | `11/29-bit CAN ID` |

CANFD1 下行的前几行应为：

| Direction | Bus | PDO | Object | SubIndex Hex/XML | Name | Type | BitLen | Byte Offset |
| --- | --- | --- | --- | --- | --- | --- | ---: | ---: |
| M→S | CANFD1 | `0x1600` | `0x7000` | `0x01 / 1` | `OUT_CANFD1_CYCLE_SEQ` | `UINT` | 16 | 0 |
| M→S | CANFD1 | `0x1600` | `0x7000` | `0x02 / 2` | `OUT_CANFD1_VALID_MASK` | `USINT` | 8 | 2 |
| M→S | CANFD1 | `0x1600` | `0x7000` | `0x03 / 3` | `OUT_CANFD1_MODE` | `USINT` | 8 | 3 |
| M→S | CANFD1 | `0x1600` | `0x7000` | `0x10 / 16` | `OUT_CANFD1_SLOT1_ID` | `UDINT` | 32 | 16 |
| M→S | CANFD1 | `0x1600` | `0x7000` | `0x11 / 17` | `OUT_CANFD1_SLOT1_FLAGS` | `USINT` | 8 | 20 |
| M→S | CANFD1 | `0x1600` | `0x7000` | `0x12 / 18` | `OUT_CANFD1_SLOT1_DLC` | `USINT` | 8 | 21 |
| M→S | CANFD1 | `0x1600` | `0x7000` | `0x13 / 19` | `OUT_CANFD1_SLOT1_SEQ` | `UINT` | 16 | 22 |
| M→S | CANFD1 | `0x1600` | `0x7000` | `0x14 / 20` | `OUT_CANFD1_SLOT1_DATA0` | `USINT` | 8 | 24 |

其余行用公式化规则展开：

```text
方向基址：下行 0，上行 0
总线全局基址 = (BusNo - 1) × 144
槽位基址 = 总线全局基址 + 16 + (SlotNo - 1) × 16
DataN 偏移 = 槽位基址 + 8 + N
下行对象 = 0x7000 + (BusNo - 1) × 0x10
上行对象 = 0x6000 + (BusNo - 1) × 0x10
槽位 SubIndex 基址 = SlotNo × 0x10
```

生成 XML 前必须执行四项核对：

1. 同一 PDO 的 `BitLen` 合计必须为 `1152 bit = 144 byte`。
2. 四个 PDO 的合计必须为 `4608 bit = 576 byte`。
3. xlsx 每一行的 `Index/SubIndex/BitLen/顺序` 与 XML `<Entry>` 一致。
4. 最后一项 CANFD4 Slot8 Data7 的全局 Byte Offset 必须为 575；过程映像最后一个字节不能越过 576。

## 8. 固件实时实现

### 8.1 初始化

```c
static MCAN_Type *const mcan[4] = {
    HPM_MCAN0, HPM_MCAN1, HPM_MCAN2, HPM_MCAN3
};

void gateway_init(void)
{
    board_init_ethercat(HPM_ESC); /* 宏名以实际 SDK 工程为准 */

    for (unsigned bus = 0; bus < 4; ++bus) {
        board_init_can(mcan[bus]);
        uint32_t can_clk = board_init_can_clock(mcan[bus]);
        mcan_fd_init_one_bus(mcan[bus], can_clk); /* 项目内薄封装 */
        board_can_transceiver_phy_set(mcan[bus], true);
    }

    ssc_init();
    enable_sync0_irq();
}
```

`mcan_fd_init_one_bus()` 只负责调用当前 HPM SDK 的 MCAN 驱动，配置：

- nominal bit rate：建议 1 Mbit/s；data bit rate：先用 5 Mbit/s，硬件允许时评估 8 Mbit/s。
- CAN FD + BRS，8 字节帧；开启 TX FIFO/Queue，深度至少 8。
- RX FIFO 深度至少 16，使用接收中断；验收全部标准/扩展数据帧或按产品需求收窄。
- 开启 error warning、error passive、bus-off 和协议错误中断。

这里不绑定某个 HPM SDK 版本的具体发送函数名，避免驱动 API 升级后文档失效；项目封装层只需提供 `try_enqueue_tx()`、`drain_rx_fifo()`、`get_bus_state()` 三个接口。

### 8.2 Sync0/输出处理

Sync0 高优先级处理中只做三件事：

1. 确认 EtherCAT 处于 OP 且本周期输出有效。
2. 将 SSC 输出过程映像复制到不被主站改写的 `tx_snapshot`，或交换已完成映射的双缓冲指针。
3. 置位四路发送任务；不要在 Sync0 ISR 中轮询等待 CAN 发完。

随后一次性给 MCAN0～MCAN3 各自 TX FIFO 装入最多 8 帧。四条物理总线并行发送，每路按 Slot1→Slot8 排队。使用 `valid_mask` 和 `flags.VALID` 双重判断；`DLC` 大于 8、ID 越界、FD+RTR 或 BRS 未配 FDF 时拒绝该槽并累计诊断计数。

```c
void canfd_dispatch_cycle(const ethercat_canfd_process_image_t *out)
{
    for (unsigned bus = 0; bus < 4; ++bus) {
        const canfd_bus_pdo_t *b = &out->bus[bus];

        for (unsigned slot = 0; slot < 8; ++slot) {
            const canfd_slot_pdo_t *f = &b->slot[slot];
            if ((b->valid_mask & (1u << slot)) == 0 ||
                (f->flags & FLAG_VALID) == 0) {
                continue;
            }
            if (!canfd_header_is_valid(f)) {
                tx_reject_count[bus]++;
                continue;
            }
            if ((f->flags & FLAG_ON_CHANGE) &&
                f->seq == last_tx_seq[bus][slot]) {
                continue;
            }
            if (try_enqueue_tx(mcan[bus], f)) {
                last_tx_seq[bus][slot] = f->seq;
            } else {
                tx_overflow_count[bus]++;
            }
        }
    }
}
```

### 8.3 CAN 接收与 TxPDO

每路 MCAN RX ISR 将硬件 FIFO 一次排空，并按到达顺序放入当前写缓冲的空槽：

```c
void mcan_rx_isr(unsigned bus)
{
    can_hw_frame_t hw;
    while (drain_one_rx(mcan[bus], &hw)) {
        rx_cycle_buffer_t *w = rx_write[bus];
        unsigned n = w->count;
        if (n >= 8) {
            w->lost_count++;
            continue;
        }
        copy_header_and_8_bytes(&w->slot[n], &hw);
        w->slot[n].seq = ++rx_seq[bus];
        w->count = n + 1;
    }
}
```

在下一个 EtherCAT 输入映射点，短暂屏蔽对应 MCAN RX 中断或使用原子指针交换，把 `rx_write` 和 `rx_publish` 对调；然后将 `rx_publish` 复制到 TxPDO。切换后立即清空新的写缓冲的 `count/rx_mask`，但累计错误计数不清零。

这种方式保证：

- ISR 永远不会改写主站正在读取的 TxPDO。
- 一周期最多上报每路 8 帧；第 9 帧开始增加 `lost_count`。
- 主站通过 CAN ID 和 `seq` 判断反馈来源、新旧和丢帧，不依赖固定返回顺序。

若电机反馈量可能超过 8 帧/周期，应扩大内部软件 FIFO，但不建议扩大循环 PDO；由后台按每周期最多 8 帧发布，或通过邮箱协议读取非实时诊断数据。

### 8.4 SSC 回调分工

| SSC 位置 | 只做什么 | 不做什么 |
| --- | --- | --- |
| `APPL_OutputMapping()` | 从 ESC 过程映像顺序复制 576 B 到输出结构 | 不发送 CAN、不解析 Data |
| `APPL_Application()` / Sync0 任务 | 检查状态，交换快照，给四路 TX FIFO 排队 | 不阻塞等发送完成 |
| MCAN RX ISR | 排空 RX FIFO，写入当前接收缓冲 | 不写 TxPDO、不调用 SSC |
| `APPL_InputMapping()` | 发布上一接收窗口的 576 B 快照 | 不访问正在被 ISR 写入的缓冲 |
| 后台任务 | SDO 参数、统计、bus-off 恢复、日志 | 不参与 1 ms 硬实时链路 |

### 8.5 中断优先级

优先级从高到低建议为：ESC Sync0/SM 事件、MCAN0～3 RX、MCAN error/TX complete、系统 tick、日志和 SDO 后台。四个 MCAN RX 使用相同抢占级，ISR 必须短；调试打印只能记录到无锁环形日志，由低优先级任务输出。

共享缓冲交换前后使用 SDK/CPU 提供的 memory barrier。若过程映像或 MCAN Message RAM 位于 Cacheable 区域，必须按 HPM5E31 缓存规则 clean/invalidate；更简单的做法是把这些缓冲放在 non-cacheable RAM。

## 9. 1 kHz 总线预算

EtherCAT 传输 576 B 输出 + 576 B 输入本身不是主要瓶颈，瓶颈是每条 CAN 总线串行发送的帧数。

每路每周期：

- 仅命令：8 帧/ms，即 8000 帧/s。
- 命令 + 每电机一次反馈：16 帧/ms，即 16000 帧/s。

8 字节 CAN FD+BRS 帧在 nominal 1 Mbit/s、data 5 Mbit/s 下，含帧间隔和位填充后通常约为几十微秒；实际值受标准/扩展 ID、位填充、仲裁、应答、重发和电机响应延迟影响。因此必须以示波器/CAN 分析仪测得的“Sync0 到最后一帧结束”作为验收依据，不能只用理论平均值。

推荐预算：

| 阶段 | 预算 |
| --- | ---: |
| Sync0 到四路首帧进入硬件 TX FIFO | ≤ 50 µs |
| 每路 8 个命令帧全部发送完成 | ≤ 500 µs |
| 接收反馈窗口 | 约 500～900 µs |
| 最晚发布/缓冲切换 | 下一个 Sync0 前完成 |
| 单路最坏总线占用率 | ≤ 80% |

推荐运行模式：

- 模式 A（首选）：8 个电机命令均为 1 kHz；电机 1/3/5/7 在偶数周期反馈，2/4/6/8 在奇数周期反馈。每个电机反馈 500 Hz。
- 模式 B：命令和反馈都为 1 kHz；需要更高的数据相位速率、严格的电机响应时隙，并以最坏帧模式测试通过。
- 模式 C：若电机只支持 Classic CAN 1 Mbit/s，则单路 8 命令 + 8 反馈无法可靠满足 1 ms，应降低循环频率、减少反馈或减少单路节点数。

四路“并行”只表示四个 MCAN 控制器同时工作；同一路上的 8 个电机仍共享一条 CAN 总线，帧必然串行，不能用 MCU 主频掩盖总线带宽不足。

## 10. 状态机、看门狗与故障处理

| 条件 | 网关动作 |
| --- | --- |
| EtherCAT 未到 OP | 不发送循环控制帧；TxPDO 报非 OP 状态 |
| 主站输出看门狗超时 | 立即停止重复旧 PDO；是否发送一次安全停机帧由产品安全需求明确配置 |
| `cycle_seq` 连续 N 周期不变 | 视为主站应用冻结；默认 N=3，停止发送并置故障 |
| CAN error-passive | 上报状态，继续接收；限制或暂停新发送由系统策略决定 |
| CAN bus-off | 停止该路发送，清空 TX 队列，计数并按 CAN 标准/产品策略恢复 |
| RX/TX 软件 FIFO 溢出 | 丢弃新帧、增加计数；不得覆盖尚未发布的帧 |
| DLC/Flags/ID 非法 | 拒绝该槽并计数，不尝试“修正”主站数据 |
| EtherCAT 从 OP 退回 SAFEOP/PREOP | 四路停止循环发送并清空未发队列 |

安全停机帧不能由网关猜测电机协议。若需要，使用 SDO 下载每路/每电机的预定义 8 字节安全帧，或由主站在退出 OP 前下发；涉及人身安全时还需要独立 STO/硬件安全链路，不能把普通 EtherCAT/CAN FD 透传当作安全功能。

## 11. 主站使用约定

每个 EtherCAT 周期，主站应：

1. 更新四个 `cycle_seq`。
2. 对需要发送的槽设置 `valid_mask`、`VALID`、CAN ID、`DLC=0～8` 和 `Data[8]`；只有前 DLC 个字节有效。
3. 周期控制帧把 `ON_CHANGE=0`；配置/查询帧把 `ON_CHANGE=1` 并更新槽位 `seq`。
4. 读取 TxPDO 的 `rx_count/rx_mask`，遍历有效槽，通过 CAN ID 交给对应电机协议解析器。
5. 监视 `lost_count`、bus-off 计数、`cycle_seq` 回显和反馈 `seq`。

主站不得假设 TxPDO Slot1 一定对应电机1；默认透明接收模式按到达顺序填槽。若业务必须固定槽位，可增加一个 SDO 配置对象保存 32 个期望 CAN ID，并使用 MCAN acceptance filter/固件 ID 路由，但这是可选的确定性路由功能，不是透明透传的必要条件。

## 12. 联调与验收

### 12.1 分阶段联调

1. **单路单电机**：CANFD1 Slot1 收发 8 字节，核对 ID、Flags、DLC 和字节序。
2. **单路八电机**：验证 Slot1→Slot8 发送顺序、反馈到达顺序和无丢帧。
3. **四路并行**：四路同时各发 8 帧，检查首帧启动偏差和 ISR 占用。
4. **EtherCAT DC**：主站 1 kHz，测量 Sync0 到四路首帧、末帧的抖动。
5. **满反馈压力**：每周期 8 命令 + 8 反馈，记录最坏总线占用；不满足 80% 门限则切换 500 Hz 交错反馈。
6. **故障注入**：拔网线、停主站、断一条 CAN、制造 bus-off、发送非法 DLC、制造 RX 溢出。
7. **长稳测试**：至少 24 h，检查 EtherCAT WKC、AL Status、四路丢帧/重发/bus-off 计数。

### 12.2 必须通过的自动检查

```c
_Static_assert(sizeof(canfd_slot_pdo_t) == 16, "slot");
_Static_assert(offsetof(canfd_slot_pdo_t, data) == 8, "slot data offset");
_Static_assert(sizeof(canfd_bus_pdo_t) == 144, "bus");
_Static_assert(sizeof(ethercat_canfd_process_image_t) == 576, "image");
```

再用一个 PC 侧脚本解析生成的 ESI XML，累计每个 `<RxPdo>/<TxPdo>` 的 `<BitLen>`，断言每个为 1152 bit、每方向四个 PDO 合计 4608 bit；同时把 XML Entry 与 xlsx 的 Index、SubIndex、BitLen 和顺序逐行比较。该检查应纳入生成 ESI 后的版本发布流程。

## 13. 建议的首版参数

| 参数 | 首版值 |
| --- | --- |
| EtherCAT 周期 | 1 ms，DC Sync0 |
| 四路 MCAN 时钟 | 80 MHz（现有板级函数） |
| CAN nominal/data bit rate | 1 Mbit/s / 5 Mbit/s，按收发器和电机能力调整 |
| CAN ID | 优先标准 11 bit；需要时支持扩展 29 bit |
| 帧格式 | CAN FD + BRS，DLC=8；若电机仅 Classic CAN 则清 FDF/BRS |
| 每路 TX FIFO | ≥ 8 |
| 每路硬件 RX FIFO | ≥ 16 |
| 每路每周期发布 RX | 最多 8 帧 |
| 输出看门狗 | 3 个 EtherCAT 周期，即 3 ms |
| CAN 发送截止预算 | 800 µs |
| 反馈策略 | 首版采用奇偶周期交错，单电机 500 Hz |

首版不需要动态 PDO、复杂队列协议或在网关中解析电机 8 字节协议。固定 576 字节过程映像、四路并行 TX FIFO、上行双缓冲已经覆盖当前 4×8 电机的实时透传需求；只有实测证明带宽或槽位不足时再扩展。
