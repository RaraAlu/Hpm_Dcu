# TwinCAT 3 验证 HPM5E31 EtherCAT 转四路 CAN FD 透传网关

> 历史测试步骤：本文八槽、576 B 透传配置不是当前 ENCOS 映射，不能直接用于当前固件。当前文件入口见[目录清单](current/PROJECT_LAYOUT.md)，验证状态见[文档索引](README.md)。

## 1. 本版本实现范围

固件提供 1 个 EtherCAT 从站和 4 路独立 CAN FD，总线对应关系如下：

| EtherCAT PDO | HPM 外设 | 板上接口 | 引脚 |
| --- | --- | --- | --- |
| CANFD1 | MCAN0 | CANFD1 | PD00 TX / PD01 RX |
| CANFD2 | MCAN1 | CANFD2 | PD05 TX / PD04 RX |
| CANFD3 | MCAN2 | CANFD3 | PC08 TX / PC09 RX |
| CANFD4 | MCAN3 | CANFD4 | PC15 TX / PC14 RX |

默认参数为 EtherCAT DC Sync0 1 ms、CAN FD 仲裁段 1 Mbit/s、数据段 5 Mbit/s。每路每周期最多下发 8 帧、上报 8 帧；每帧槽支持 DLC 0～8，未使用的尾部字节保持为 0。网关不解释电机协议。

每路 CAN FD 使用独立的 TX/RX/ERR 指示灯。硬件确认发送完成时 TX 灯至少点亮 1 ms，成功读取一帧时 RX 灯至少点亮 1 ms；MCAN 错误、非法/溢出接收帧、非法发送参数或 TX FIFO 入队失败时 ERR 灯点亮。实现使用 1 ms 定时节拍，不在 MCAN 中断或 EtherCAT 实时循环中延时。连续事件快于约 1 kHz 时 LED 会趋于常亮，不能再凭肉眼区分单帧，这是正常现象。

网关初始化会在启用 MCAN 中断前再次把 12 个活动灯显式写为 OFF。因此板卡刚上电且没有 CAN 数据时，TX/RX/ERR 灯均应熄灭；收到或发出数据后闪烁，数据停止且最后一个至少 1 ms 的脉冲结束后重新熄灭。

TX 灯以 MCAN 硬件“发送完成”事件为准。MCAN 需要同时开启全局 `TC` 中断和 TX FIFO 各元素的 `TXBTIE` 位；只开全局中断会出现分析仪已收到报文、但 TX 绿灯始终不亮。当前固件已使能 8 个 TX FIFO 元素的完成中断。

过程映像每个方向均为 576 字节：

| 方向 | PDO | 对象 | 大小 |
| --- | --- | --- | ---: |
| 主站 → 从站 | 0x1600～0x1603 | 0x7000/0x7010/0x7020/0x7030 | 4 × 144 B |
| 从站 → 主站 | 0x1A00～0x1A03 | 0x6000/0x6010/0x6020/0x6030 | 4 × 144 B |

单路前 16 字节是总线头，后面是 8 个 16 字节槽。槽格式是 `CAN_ID(4) + Flags(1) + DLC(1) + Seq(2) + Data[8]`，全部按 EtherCAT 小端序排列；`Data[8]` 不换序。

## 2. 构建和烧录

在 HPM SDK Environment 命令行进入工程目录后执行：

```powershell
cmake -S . -B build -G Ninja -DBOARD=hpm5e00evk -DHPM_BUILD_TYPE=flash_xip
cmake --build build --parallel 4
```

输出文件为 `build/output/demo.elf`。也可以直接打开：

```text
hpm5e00evk_flash_xip_debug/segger_embedded_studio/ecat_io.emProject
```

烧录后串口应打印 EtherCAT 初始化信息，不能出现 `Init EtherCAT/CAN FD gateway failed`。`CONFIG_CAN_LED_TEST` 必须保持关闭；该测试是阻塞式 LED 产测程序。

烧录后可按以下顺序验证活动灯：

1. PLC 将 `testEnable` 写为 TRUE，CANFD1～4 的 TX 灯应随发送闪烁；1 ms 连续发送时视觉上可能接近常亮。
2. 分析仪分别向四路发送周期帧，对应 RX 灯应闪烁；建议先用 100～500 ms 周期观察单次脉冲。
3. 使用错误波特率、断开终端或发送当前 PDO 不接受的帧进行诊断时，对应 ERR 灯应点亮。完成故障测试后立即恢复正确接线和参数，避免总线长期 Bus-Off。

如修改了 PDO 字典，先重新生成并自检 ESI：

```powershell
node tools/generate_canfd_gateway.js
```

脚本会检查 8 个 PDO、每个 PDO 1152 bit、每个方向 4608 bit，并生成：

```text
SSC/ECAT_CANFD_GATEWAY.xml
SSC/Src/gatewayObjects.h
```

## 3. CAN FD 测试接线

第一次验证不要直接发送未知的电机控制字。优先连接 CAN FD 分析仪、另一块可控节点或确认不会驱动电机的测试 ID。

1. 分析仪配置为 ISO CAN FD，仲裁段 1 Mbit/s，数据段 5 Mbit/s，允许 BRS。
2. CANH、CANL、信号地三线对应连接。
3. 总线只在物理两端各保留一个 120 Ω 终端；断电测量 CANH-CANL 应接近 60 Ω。
4. 先只连接 CANFD1，单路通过后再接其余三路。

## 4. 在 TwinCAT 3 导入 ESI XML

这里的“导入 XML”和后面的“写 EEPROM”是两个不同步骤：XML 先安装到 PC 上的 TwinCAT 设备描述库；`Write EEPROM` 窗口再从该描述库中选择设备，由 TwinCAT 生成并写入从站 SII。不要在 `Write EEPROM` 窗口中查找 XML 文件。

### 4.1 安装正确的 XML

1. 关闭 TwinCAT XAE/Visual Studio，避免设备描述缓存没有刷新。
2. 从工程中找到唯一需要安装的文件：

   ```text
   HPM_ethercat_canfd_encos_V1.0\SSC\ECAT_CANFD_GATEWAY.xml
   ```

3. 将该文件复制到：

   ```text
   C:\TwinCAT\3.1\Config\Io\EtherCAT
   ```

4. 不要复制 `SSC\ECAT_IO.xml` 或 `SSC\ESI\ECAT_IO.xml`。它们是原数字 IO 示例的旧描述，不包含本项目的四路 CAN FD PDO。
5. 如果 TwinCAT 目录中已经存在旧的 `ECAT_IO.xml`，先将其移出该目录。旧文件和新文件使用相同的开发期身份号，同时存在可能造成选型冲突。

复制操作可能需要管理员权限。建议最终确认目标目录中存在：

```text
C:\TwinCAT\3.1\Config\Io\EtherCAT\ECAT_CANFD_GATEWAY.xml
```

### 4.2 让 TwinCAT 重新加载描述

1. 重新打开 TwinCAT XAE。
2. 执行 `TwinCAT > EtherCAT Devices > Reload Device Descriptions`。不同 TwinCAT 版本的菜单位置可能略有差异；找不到该菜单时，关闭并重新打开 XAE 也可以刷新缓存。
3. 在设备选择窗口中展开：

   ```text
   Personal
     └─ HPM5E31 EtherCAT to 4x CAN FD Gateway
   ```

4. 如果没有 `Personal` 或没有上述设备，依次检查：文件名和目录是否正确、XML 是否被误放在子目录、XAE 是否真正重启、旧 `ECAT_IO.xml` 是否仍在描述目录。

当前工程沿用开发期身份：Vendor ID `0x00010000`、Product Code `0x00000001`、Revision `0x00000001`。产品发布前必须同时替换 `SSC/ECAT_CANFD_GATEWAY.xml`、`SSC/Src/ecat_def.h` 和 SSC 工程中的正式公司身份号。

## 5. 扫描从站并烧写 EEPROM/SII

本工程从数字 IO 示例改造而来，板载 Flash EEPROM 中可能保留校验正确、但 PDO 只有 4 字节的旧 SII。仅复制 XML 不会自动修改板上的 SII，首次升级必须执行本节操作。

### 5.1 烧写前检查

1. 烧录最新网关固件并重新上电。
2. 查看串口，必须出现：

   ```text
   EtherCAT IO sample
   EEPROM loading successful, no checksum error.
   ```

3. 串口不能出现 `Init EtherCAT/CAN FD gateway failed`。如果 CAN FD 初始化失败，程序会在进入 SSC `MainLoop()` 前停止，TwinCAT 的 EEPROM 读写请求将无人处理。
4. 将 TwinCAT 切到配置模式：点击 `Restart TwinCAT System in Config Mode`（蓝色齿轮）。
5. 在 `I/O > Devices` 上执行 `Scan`，选择 EtherCAT 网卡，再执行 `Scan for Boxes`。即使旧 SII 使从站暂时显示为 `Box 1` 或 `digital_io`，仍可继续写入新 SII。
6. 在从站 `Online` 页执行 `Request INIT`，确认从站处于 `INIT`。不要在 OP 状态烧写 EEPROM。

### 5.2 执行 Write EEPROM

1. 选中实际扫描到的从站，例如 `Box 1`。
2. 打开：

   ```text
   EtherCAT
     → Advanced Settings
       → ESC Access
         → E²PROM
           → Smart View
   ```

3. 点击 `Write E²PROM...`（部分版本显示为 `Write EEPROM...`）。
4. 在 `Available EEPROM Descriptions` 中展开 `Personal`，选择：

   ```text
   HPM5E31 EtherCAT to 4x CAN FD Gateway
   ```

5. 点击 `OK`，等待约 10 秒。部分 TwinCAT 版本不弹出独立进度条，只在主窗口底部状态栏短暂显示写入或校验状态；此时不要立即断电。

如果列表只有 Beckhoff 设备，说明第 4 节的 XML 尚未被 TwinCAT 加载。`Show Hidden Devices` 只负责显示隐藏条目，不能代替安装 XML。

### 5.3 回读和重新扫描

1. 写入完成后仍停留在 `Smart View`，点击 `Read E²PROM...`。
2. 回读结果至少应满足：

   | 字段 | 期望值 |
   | --- | --- |
   | Vendor ID | `0x00010000` |
   | Product Code | `0x00000001` |
   | Revision No. | `0x00000001` |
   | Mailbox | CoE 有效 |

3. 如果 Vendor/Product/Revision 全为 `0`，不是 PDO 配置问题，而是本次 EEPROM 访问没有被从站固件处理。优先检查串口是否存在 CAN FD 初始化失败、固件是否为最新版本、SSC 主循环是否仍在运行。
4. 确认回读正确后，关闭设置窗口，给板卡完全断电后重新上电。
5. 删除 TwinCAT I/O 树中的旧 Box，重新执行 `Scan for Boxes`。从站应识别为 `HPM5E31 EtherCAT to 4x CAN FD Gateway`。
6. 展开从站，应能看到 Inputs、Outputs 以及对应 PDO。检查：

   - SM2：起始地址 `0x1100`，长度 `0x240`（576 字节）；
   - SM3：起始地址 `0x1800`，长度 `0x240`（576 字节）；
   - 输出 PDO：`0x1600～0x1603`；
   - 输入 PDO：`0x1A00～0x1A03`；
   - 每个 PDO：103 个 Entry、144 字节。

如果从站只能到 PREOP/SAFEOP，先看 `Online > AL Status Code`。`Invalid SM configuration` 或 `Invalid input/output mapping` 通常表示 EEPROM 仍是旧版本、ESI 冲突，或主站缓存未刷新。

## 6. 选择同步模式并配置 1 ms 任务

首次功能验证建议先使用 `SM-Synchron`，确认 EEPROM、PDO、PLC 映射和 CAN FD 收发全部正常后，再切换到 `DC-Synchron` 做实时性验收。这样可以把功能问题和 PC 实时调度问题分开。

### 6.1 建立 1 ms PLC 任务

1. 在 `PLC` 节点上执行 `Add New Item... > Standard PLC Project`，例如命名为 `HPM_eth_test`。
2. 展开 `SYSTEM > Tasks > PlcTask`，将 `Cycle ticks` 设置为 1，确认界面显示 `Cycle Time = 1000 μs`。
3. 在 `Device 2 (EtherCAT) > EtherCAT > Sync Unit Assignment...` 中，将包含网关的 Sync Unit 分配给 `PlcTask`。
4. 如果此前创建过独立 I/O Task，不要让同一个从站同时关联两个任务；本次验证只保留 `PlcTask`。

### 6.2 先使用 SM-Synchron 完成功能验证

1. 打开网关从站的 `DC` 页面或 `Advanced Settings > Distributed Clock`。
2. 将 Operation Mode 选择为 `Synchron`（部分版本显示 `SM-Synchron`），不要启用 SYNC0/SYNC1。
3. 执行顶部菜单 `TwinCAT > Activate Configuration`，同意重启到 Run Mode。
4. 在从站 `Online` 页面确认 `Current State = OP`、`AL Status Code = 0x0000`，并至少稳定运行 1 分钟。

本次现场验证在 `SM-Synchron` 下已经稳定进入 OP，并由 CAN FD 分析仪成功收到 PLC 下发的测试帧。

### 6.3 DC-Synchron 作为后续实时性验收

功能验证通过后，可将 Operation Mode 改为 `DC-Synchron`：Sync Unit Cycle=`1000 μs`、SYNC0=`Sync Unit Cycle ×1`、Shift=`0 μs`、Enable SYNC0 勾选、SYNC1 不勾选。

切换 DC 前先在 `SYSTEM > Tasks > PlcTask > Online` 点击 `Reset` 清零 Exceed Counter，并观察 1～2 分钟。若 Exceed Counter、Frame Missed 或 EtherCAT Lost Frames 持续增加，不应继续连接真实电机。当前测试 PC 在 DC 模式下曾出现 `0x001A Synchronization error`，因此 DC 1 ms 尚未完成稳定性验收；SM-Synchron 的功能验证结果不等于 DC 已通过。

当前固件在进入 OP 前不会发送循环 CAN 帧；从 OP 退回 SAFEOP/PREOP 时会停止新帧并取消待发 TX FIFO。四路 `cycle_seq` 任一路连续 3 个周期不变化，该路也会停止发送，防止重复旧控制帧。

## 7. 创建 PLC 数据类型、GVL 和 PDO 链接

### 7.1 创建 DUT

在 `HPM_eth_test Project > DUTs` 上右键，依次执行 `Add > DUT...`，创建四个独立 DUT。必须使用 1 字节打包的三个过程映像 DUT，否则结构体偏移会与 PDO 不一致。不要把下面四个 `TYPE` 全部粘贴到同一个 DUT 文件中。

#### 7.1.1 DUT：ST_CanFdSlot

新建 DUT 时名称填写 `ST_CanFdSlot`，文件内容为：

```iecst
{attribute 'pack_mode' := '1'}
TYPE ST_CanFdSlot :
STRUCT
    can_id : DWORD;
    flags  : BYTE;
    dlc    : BYTE;
    seq    : UINT;
    data   : ARRAY[0..7] OF BYTE;
END_STRUCT
END_TYPE
```

该类型表示一个 16 字节 CAN FD 帧槽。

#### 7.1.2 DUT：ST_CanFdBus

新建 DUT 时名称填写 `ST_CanFdBus`，文件内容为：

```iecst
{attribute 'pack_mode' := '1'}
TYPE ST_CanFdBus :
STRUCT
    cycle_seq   : UINT;
    valid_mask  : BYTE;
    mode        : BYTE;
    deadline_us : UINT;
    bus_state   : UINT;
    counter0    : DWORD;
    counter1    : DWORD;
    slot        : ARRAY[0..7] OF ST_CanFdSlot;
END_STRUCT
END_TYPE
```

该类型表示一路 CAN FD 的 144 字节过程数据，内部包含 16 字节总线头和 8 个帧槽。创建它之前必须先创建 `ST_CanFdSlot`。

#### 7.1.3 DUT：ST_CanFdImage

新建 DUT 时名称填写 `ST_CanFdImage`，文件内容为：

```iecst
{attribute 'pack_mode' := '1'}
TYPE ST_CanFdImage :
STRUCT
    bus : ARRAY[0..3] OF ST_CanFdBus;
END_STRUCT
END_TYPE
```

该类型表示四路 CAN FD 的完整 576 字节过程映像。创建它之前必须先创建 `ST_CanFdBus`。

#### 7.1.4 DUT：ST_CanFdStats

新建 DUT 时名称填写 `ST_CanFdStats`，文件内容为：

```iecst
TYPE ST_CanFdStats :
STRUCT
    rx_total         : ULINT; // PLC启动后累计接收帧数
    rx_fps           : REAL;  // 最近1秒接收帧率，单位frame/s
    rx_drop_total    : DWORD; // 网关累计接收丢弃数
    rx_drop_rate_pct : REAL;  // 网关接收丢弃率，单位%
    rx_bus_load_pct  : REAL;  // 接收方向估算总线占用率，单位%
    bus_state        : UINT;  // 0正常、1警告、2被动、3 Bus-Off
    bus_off_total    : DWORD; // 累计Bus-Off次数
END_STRUCT
END_TYPE
```

该类型只保存 PLC 计算结果，不参与 EtherCAT PDO 映射，因此不需要 `pack_mode`。`rx_drop_rate_pct` 是网关接收丢弃率，并非 CANH/CANL 物理层的比特误码率；真正的 BER 需要 CAN FD 分析仪或固件额外上报控制器错误计数。

### 7.2 创建 GVL

1. 右键 `HPM_eth_test Project > GVLs`，选择 `Add > Global Variable List...`。
2. 名称填写 `GVL_CANFD`。
3. 写入：

```iecst
VAR_GLOBAL
    gCanOut AT %Q* : ST_CanFdImage;
    gCanIn  AT %I* : ST_CanFdImage;
END_VAR
```

`%Q*` 表示主站发往网关的输出过程映像，`%I*` 表示网关返回主站的输入过程映像，星号表示地址由 TwinCAT 链接时分配。不要手工改成 `%QB0/%IB0`。

执行 `Build`，输出应为 `Build complete -- 0 errors, 0 warnings`。在线检查 `SIZEOF(ST_CanFdSlot)=16`、`SIZEOF(ST_CanFdBus)=144`、`SIZEOF(ST_CanFdImage)=576`。

### 7.3 链接 PDO

TwinCAT 的 Inputs/Outputs 方向以主站为参考：

| EtherCAT PDO | PLC 变量 |
| --- | --- |
| CANFD1 Outputs | `GVL_CANFD.gCanOut.bus[0]` |
| CANFD2 Outputs | `GVL_CANFD.gCanOut.bus[1]` |
| CANFD3 Outputs | `GVL_CANFD.gCanOut.bus[2]` |
| CANFD4 Outputs | `GVL_CANFD.gCanOut.bus[3]` |
| CANFD1 Inputs | `GVL_CANFD.gCanIn.bus[0]` |
| CANFD2 Inputs | `GVL_CANFD.gCanIn.bus[1]` |
| CANFD3 Inputs | `GVL_CANFD.gCanIn.bus[2]` |
| CANFD4 Inputs | `GVL_CANFD.gCanIn.bus[3]` |

首次只链接 CANFD1。选中整个 `CANFD1 Outputs/Inputs` PDO 时可用 `Change Multi Link...` 做连续链接；如果选中的是单个 `IN_CANFD1_CycleSeq`，目标浏览器不会允许选择整个 144 字节的 `bus[0]`，必须展开 `GVL_CANFD.gCanIn > bus > bus[0]`，选择同为 2 字节的 `cycle_seq`。其余字段依次对应，Slot1 对应 `slot[0]`，Slot8 对应 `slot[7]`。

整体链接后抽查首尾：`CycleSeq ↔ cycle_seq`、`Slot1 ID ↔ slot[0].can_id`、`Slot8 Data7 ↔ slot[7].data[7]`。

## 8. 创建 MAIN，验证收发并统计接收数据

### 8.1 创建有效的 MAIN

在 `HPM_eth_test Project > POUs` 下创建 `MAIN`，类型选择 `Program`，语言选择 Structured Text。最终声明区完整内容如下：

```iecst
PROGRAM MAIN
VAR
    testEnable : BOOL := FALSE;

    // 四路测试帧发送周期；在线修改此值即可调速
    sendTimer  : TON;
    sendPeriod : TIME := T#10MS;

    // 四路CAN FD接收统计结果
    aCanStats : ARRAY[0..3] OF ST_CanFdStats;

    // TON按真实时间形成1秒统计窗口，不依赖PLC任务循环次数
    statsTimer : TON;
    busIndex   : INT;

    // 每秒接收数量及接收序号去重状态
    rxWindowCount : ARRAY[0..3] OF UDINT;
    lastRxSeq     : ARRAY[0..3] OF UINT;
    rxSeqValid    : ARRAY[0..3] OF BOOL;
    currentRxSeq  : UINT;
    seqDelta      : UDINT;
    lastSlotIndex : INT;

    // 网关丢弃数的相邻采样值
    lastDropCount : ARRAY[0..3] OF DWORD;
    dropDelta     : DWORD;
    totalDelta    : UDINT;

    // 单帧总线时间估算值，需按分析仪实测结果修正
    rxFrameTimeUs : REAL := 55.0;
END_VAR
```

如果编辑器已自动显示 `PROGRAM MAIN`，只填写 `VAR ... END_VAR`。确认 `PlcTask` 下存在 MAIN 调用；若编译报 `Identifier 'MAIN' not defined`，删除 PlcTask 下失效的 MAIN，再把 `POUs > MAIN` 拖到 `PlcTask`。IEC 语句末尾必须有分号，包括 `END_IF;`。

### 8.2 MAIN 程序体完整代码

以下程序同时完成 CANFD1～4 定时测试发送、四路接收总数/帧率/丢弃率/估算占用率统计。四路分别发送标准 ID `0x123～0x126`，Data0 分别为 `0x11～0x14`。Flags `0x39` 表示 `VALID | FDF | BRS | ON_CHANGE`，从站只在槽 `seq` 改变时发送。

```iecst
// 使用TON形成可调的四路发送周期；默认每10 ms触发一次
sendTimer(IN := testEnable, PT := sendPeriod);

// testEnable为TRUE时维持四路输出有效，但只在定时到达时更新seq
IF testEnable THEN
    FOR busIndex := 0 TO 3 DO
        // EtherCAT周期序号每个PLC扫描都更新，避免从站输出看门狗超时
        GVL_CANFD.gCanOut.bus[busIndex].cycle_seq :=
            GVL_CANFD.gCanOut.bus[busIndex].cycle_seq + 1;
        GVL_CANFD.gCanOut.bus[busIndex].valid_mask := 16#01;
        GVL_CANFD.gCanOut.bus[busIndex].deadline_us := 800;

        // CANFD1～4使用0x123～0x126，便于分析仪区分物理通道
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].can_id :=
            INT_TO_DWORD(16#123 + busIndex);
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].flags := 16#39; // VALID | FDF | BRS | ON_CHANGE
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].dlc := 8;

        IF sendTimer.Q THEN
            // 四路槽序号仅在发送周期到达时变化，各触发一帧
            GVL_CANFD.gCanOut.bus[busIndex].slot[0].seq :=
                GVL_CANFD.gCanOut.bus[busIndex].slot[0].seq + 1;
        END_IF;

        // Data0为0x11～0x14，其余字节相同
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].data[0] :=
            INT_TO_BYTE(16#11 + busIndex);
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].data[1] := 16#22;
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].data[2] := 16#33;
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].data[3] := 16#44;
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].data[4] := 16#55;
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].data[5] := 16#66;
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].data[6] := 16#77;
        GVL_CANFD.gCanOut.bus[busIndex].slot[0].data[7] := 16#88;
    END_FOR;

    IF sendTimer.Q THEN
        // 本扫描完成四路触发后复位TON，下一扫描重新计时
        sendTimer(IN := FALSE);
    END_IF;
ELSE
    sendTimer(IN := FALSE);

    // 停止时同时关闭四路发送有效位
    FOR busIndex := 0 TO 3 DO
        GVL_CANFD.gCanOut.bus[busIndex].valid_mask := 0;
    END_FOR;
END_IF;

// 按最新帧seq统计，避免PLC重复读取同一份PDO时重复累计
FOR busIndex := 0 TO 3 DO
    seqDelta := 0;

    IF GVL_CANFD.gCanIn.bus[busIndex].mode > 0 THEN
        // 最后一个有效槽保存当前PDO窗口中的最新接收序号
        lastSlotIndex :=
            BYTE_TO_INT(GVL_CANFD.gCanIn.bus[busIndex].mode) - 1;
        currentRxSeq :=
            GVL_CANFD.gCanIn.bus[busIndex].slot[lastSlotIndex].seq;

        IF NOT rxSeqValid[busIndex] THEN
            // 第一次看到有效窗口，只统计当前窗口中的帧
            seqDelta :=
                BYTE_TO_UDINT(GVL_CANFD.gCanIn.bus[busIndex].mode);
            rxSeqValid[busIndex] := TRUE;

        ELSIF currentRxSeq = lastRxSeq[busIndex] THEN
            // 同一份PDO被再次读取，不重复统计
            seqDelta := 0;

        ELSIF currentRxSeq > lastRxSeq[busIndex] THEN
            // 正常递增，序号差可以补回PLC未看到的中间PDO窗口
            seqDelta := UINT_TO_UDINT(currentRxSeq)
                      - UINT_TO_UDINT(lastRxSeq[busIndex]);

        ELSIF (lastRxSeq[busIndex] >= 16#FF00)
            AND (currentRxSeq <= 16#00FF) THEN
            // 只有跨越65535边界时才按16位序号回绕计算
            seqDelta := UDINT#65536
                      - UINT_TO_UDINT(lastRxSeq[busIndex])
                      + UINT_TO_UDINT(currentRxSeq);

        ELSE
            // 序号异常变小，按从站重启处理，防止总数突然跳一万多
            seqDelta :=
                BYTE_TO_UDINT(GVL_CANFD.gCanIn.bus[busIndex].mode);
        END_IF;

        // 1ms系统中一次增加超过256帧不合理，过滤重启或旧值残留
        IF seqDelta > UDINT#256 THEN
            seqDelta :=
                BYTE_TO_UDINT(GVL_CANFD.gCanIn.bus[busIndex].mode);
        END_IF;

        lastRxSeq[busIndex] := currentRxSeq;
    END_IF;

    aCanStats[busIndex].rx_total :=
        aCanStats[busIndex].rx_total + UDINT_TO_ULINT(seqDelta);
    rxWindowCount[busIndex] := rxWindowCount[busIndex] + seqDelta;

    // 当前固件counter0为累计接收丢弃数，counter1为Bus-Off累计次数
    aCanStats[busIndex].rx_drop_total :=
        GVL_CANFD.gCanIn.bus[busIndex].counter0;
    aCanStats[busIndex].bus_state :=
        GVL_CANFD.gCanIn.bus[busIndex].bus_state;
    aCanStats[busIndex].bus_off_total :=
        GVL_CANFD.gCanIn.bus[busIndex].counter1;
END_FOR;

// 使用TON按真实时间形成1秒窗口，不用PLC循环次数推算时间
statsTimer(IN := TRUE, PT := T#1S);

IF statsTimer.Q THEN
    FOR busIndex := 0 TO 3 DO
        // 窗口为1秒，窗口帧数即frame/s
        aCanStats[busIndex].rx_fps :=
            UDINT_TO_REAL(rxWindowCount[busIndex]);

        dropDelta := GVL_CANFD.gCanIn.bus[busIndex].counter0
                   - lastDropCount[busIndex];
        lastDropCount[busIndex] :=
            GVL_CANFD.gCanIn.bus[busIndex].counter0;
        totalDelta := rxWindowCount[busIndex]
                    + DWORD_TO_UDINT(dropDelta);

        IF totalDelta > 0 THEN
            // 这是网关接收丢弃率，不是CAN物理层比特误码率
            aCanStats[busIndex].rx_drop_rate_pct :=
                100.0 * DWORD_TO_REAL(dropDelta)
                / UDINT_TO_REAL(totalDelta);
        ELSE
            aCanStats[busIndex].rx_drop_rate_pct := 0.0;
        END_IF;

        // 只估算接收方向：帧数×单帧时间÷1秒×100%
        aCanStats[busIndex].rx_bus_load_pct :=
            UDINT_TO_REAL(rxWindowCount[busIndex])
            * rxFrameTimeUs / 10000.0;

        rxWindowCount[busIndex] := 0;
    END_FOR;

    // TON在本扫描周期复位，下一周期重新开始计时
    statsTimer(IN := FALSE);
END_IF;
```

IEC 语句末尾必须有分号，包括 `END_IF;` 和 `END_FOR;`。

### 8.3 激活并在线启动发送

1. 先按第 7.3 节把 CANFD1～4 的 Outputs 全部链接到 `gCanOut.bus[0]～bus[3]`；需要观察四路返回数据时，也链接对应 Inputs。
2. Build PLC，确认 0 errors。
3. 从顶部菜单执行 `TwinCAT > Activate Configuration`；PLC 节点右键的 `Activate Boot Project...` 不是 I/O 配置激活命令。
4. 执行 `PLC > Login`，同意 Download，然后 `PLC > Start`。
5. 确认 TwinCAT、PLC 均为 Run，从站保持 OP。
6. 不要设置断点。断点会暂停 1 ms PlcTask，造成 Frame Missed，并可能触发从站 `0x001A` 后退 SAFEOP。
7. 在线打开 MAIN，双击 `testEnable` 的准备值，将其改为 TRUE，再执行 `PLC > Write Values`。`testEnable` 初始为 FALSE 时只执行 ELSE、`valid_mask=0`，这是安全设计，不是程序故障。
8. 停止发送时把 `testEnable` 在线写回 FALSE。

在线修改 `sendPeriod` 可以改变测试发送周期，例如：

```iecst
T#10MS  // 默认值，100 frame/s
T#500MS // 2 frame/s，适合肉眼观察TX灯
T#100MS // 10 frame/s
T#20MS  // 50 frame/s
T#5MS   // 200 frame/s
```

CAN FD 分析仪设置为 ISO CAN FD、仲裁段 1 Mbit/s、数据段 5 Mbit/s、BRS 开启。本次现场已在 `SM-Synchron` 下收到 ID `0x123`、DLC 8、数据 `11 22 33 44 55 66 77 88`，证明主站→EtherCAT→HPM5E31→CANFD1 下行链路通过。

四路 CAN FD 是相互独立的物理总线，不能把 CANH/CANL 并在一起。使用四通道分析仪，或逐路移动单通道分析仪验证。预期结果：

- CANFD1 每 `sendPeriod` 收到 ID `0x123`，数据为 `11 22 33 44 55 66 77 88`。
- CANFD2 每 `sendPeriod` 收到 ID `0x124`，数据为 `12 22 33 44 55 66 77 88`。
- CANFD3 每 `sendPeriod` 收到 ID `0x125`，数据为 `13 22 33 44 55 66 77 88`。
- CANFD4 每 `sendPeriod` 收到 ID `0x126`，数据为 `14 22 33 44 55 66 77 88`。
- 标准帧 ID 不超过 `0x7FF`；扩展帧需设置 Flags.IDE，ID 不超过 `0x1FFFFFFF`。
- DLC 大于 8、RTR、`BRS=1/FDF=0` 或保留位非零的输出槽不会发送。
- 当前程序已设置 `ON_CHANGE`（Flags=`0x39`），只有槽 `seq` 改变时才发送；首个 `seq=0` 仍会正常发送一次。

让分析仪或测试节点返回一帧 ID `0x321`、8 字节数据后，应看到：

- `gCanIn.bus[0].mode = 1`；
- `valid_mask.0 = 1`；
- Slot0 Flags.NEW=1，ID/DLC/Data 与总线原帧一致；
- Slot `seq` 随每个接收帧递增；
- `counter0` 为累计接收丢弃数，`counter1` 为累计 bus-off 次数；`bus_state` 0/1/2/3 分别代表 error-active/warning/passive/bus-off。

### 8.4 在监视窗口查看接收统计

PLC 登录并处于 Run 后，从在线 MAIN 声明区展开 `aCanStats > [0]`，右键具体成员选择 `Add Watch`。这种方式会自动绑定正确的 PLC Application，比手工填写实例路径可靠。CANFD1 对应 `[0]`，CANFD2～4 对应 `[1]`～`[3]`。

重点监视：

```text
MAIN.aCanStats[0].rx_total
MAIN.aCanStats[0].rx_fps
MAIN.aCanStats[0].rx_drop_total
MAIN.aCanStats[0].rx_drop_rate_pct
MAIN.aCanStats[0].rx_bus_load_pct
MAIN.aCanStats[0].bus_state
MAIN.aCanStats[0].bus_off_total
MAIN.lastRxSeq[0]
MAIN.seqDelta
```

如果监视窗口显示“未发现应用”，删除该表达式，保持 PLC 在线，再从 MAIN 声明区右键 `Add Watch`。不要把 I/O 映射节点名称 `HPM_eth_test_Instance` 猜作 PLC 符号路径。

接收测试建议先关闭 `testEnable`，然后由分析仪向 CANFD1 周期发送 ID `0x321`、DLC 8、CAN FD+BRS 数据帧：

| 分析仪周期 | 理论帧率 | 1 秒窗口正常显示 |
| ---: | ---: | ---: |
| 20 ms | 50 frame/s | 约 49～51 frame/s |
| 10 ms | 100 frame/s | 约 99～101 frame/s |
| 5 ms | 200 frame/s | 约 199～201 frame/s |

`rx_total` 从 PLC 本次启动后开始累计。`seqDelta` 在新帧到达时通常为 1，PLC 重复读取同一输入 PDO 时为 0；如果 PLC 漏看了中间 PDO，后续序号差会补回遗漏数量。不要通过每次读取 `mode` 直接累加，因为同一输入 PDO 可能被 PLC 读取多次。

`rx_bus_load_pct` 是接收方向估算值：

```text
接收占用率(%) = 1秒接收帧数 × rxFrameTimeUs ÷ 10000
```

默认 `rxFrameTimeUs=55.0` 仅用于 1 Mbit/s 仲裁段、5 Mbit/s 数据段、标准 ID、DLC 8、BRS 的初始估算。它不包含错误帧、重发和本节点发送但未回环接收的帧，应使用 CAN FD 分析仪测得的实际单帧时间校正。

## 9. 满负载与故障验收

按以下顺序扩大测试，不要直接从单帧跳到四路满载：

1. CANFD1 的 Slot1～Slot8 全部有效，确认总线发送顺序与槽顺序一致。
2. 四路同时各发送 8 帧，示波器测量 Sync0 到四路首帧开始时间，目标不超过 50 µs；每路末帧目标不超过 500 µs。
3. 每路返回 8 帧，确认 `mode=8`、`valid_mask=0xFF`；第 9 帧开始 `counter0` 增加，已发布槽不能被覆盖。
4. 停止 PLC 的 `cycle_seq` 更新，3 ms 后分析仪上应停止该路周期帧。
5. 将从站切回 SAFEOP，四路应停止新控制帧。
6. 制造 CAN bus-off，该路 `bus_state=3`、`counter1` 增加且停止发送；首版采用重新初始化/重新上电恢复，不在中断中自动反复恢复。
7. 四路连续运行至少 24 h，记录 EtherCAT WKC、AL Status、`counter0/counter1` 和 CAN 分析仪错误帧。

单路若同时要求 8 条 1 kHz 命令和 8 条 1 kHz 反馈，必须实测最坏总线占用不超过 80%。达不到时，优先让奇数/偶数电机交错反馈，使每台电机反馈为 500 Hz。

## 10. 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 扫描出来仍叫 digital_io | 删除 TwinCAT 旧 ESI、Reload 描述、更新 EEPROM 后重新上电 |
| Write EEPROM 点击 OK 后没有进度 | 确认从站处于 INIT，并检查串口是否出现网关初始化失败；SSC 主循环退出后无法响应 EEPROM 命令 |
| Read EEPROM 后身份号全部为 0 | EEPROM 访问未得到有效响应；先修复固件启动错误，再重新 Write/Read |
| 串口报错 26012 | MCAN Message RAM 越界；必须使用当前已修复的 8 字节 Message RAM 配置 |
| 串口报错 2 | MCAN 参数无效；必须同步清零未分配过滤表对应的默认过滤项 |
| 进入 SAFEOP 报 `0x001D` | SM2 输出区与 SM3 输入区重叠；使用新版 ESI，确认 SM2=`0x1100`、SM3=`0x1800` 后重新写 EEPROM |
| OP 反复退回 SAFEOP，报 `0x001A` | 同步错误；先检查 Frame Missed、任务 Exceed Counter 和断点，再切回 SM-Synchron 分离功能与 DC 实时性问题 |
| `Frame missed 10 times` | 主站周期帧未稳定发送；检查 RT 网卡、任务调度、断点和 PC 实时性能 |
| PLC 只执行 `valid_mask := 0` | `testEnable` 初始值是 FALSE；在线 Write Values 改为 TRUE，不要用断点 |
| 编译报 `Identifier 'MAIN' not defined` | PlcTask 引用了不存在/失效的 MAIN；创建 `PROGRAM MAIN` 并重新拖到 PlcTask |
| SAFEOP 不能进 OP | SM2/SM3 是否均为 576 B，PDO 是否各 144 B，任务是否为 1 ms |
| EtherCAT 有输出但 CAN 无帧 | 从站是否 OP、`cycle_seq` 是否递增、valid_mask 与 VALID 是否同时置位、DLC 是否在 0～8 |
| 只有第一帧 | 是否误设 ON_CHANGE 且没有更新槽 `seq` |
| CAN 能发但收不到 PDO | 分析仪 DLC 是否在 0～8、波特率/BRS/终端是否一致、输入 PDO 是否正确链接 |
| 满载丢反馈 | 查看 `counter0`，测量总线占用，改用 500 Hz 交错反馈 |
| 发送 1000 帧却统计成 1475 帧 | 不要逐 PLC 周期累加 `mode`；同一输入 PDO 会被重复读取，改用最新槽 `seq` 去重 |
| 接收总数突然跳到一万多 | 从站重启后 `seq` 变小被误判为 16 位回绕；仅在 `lastRxSeq>=0xFF00` 且新值 `<=0x00FF` 时承认回绕，并过滤 `seqDelta>256` |
| 发送 1000 帧统计少约 10 帧 | 单纯按新窗口累加 `mode` 会漏掉 PLC 未看到的 PDO；使用合理的 `seq` 差值补回中间窗口，同时观察 `counter0` 是否真的增加 |
| 分析仪 5 ms 发送却只显示约 160 frame/s | 不要用 `statsCycle=1000` 假定一秒；任务实际周期或调用节拍可能不同，使用 `TON(PT:=T#1S)` 形成真实时间窗口 |
| 监视表达式显示“未发现应用” | PLC 在线后从 MAIN 声明区展开变量并右键 `Add Watch`，不要手工猜测 PLC 实例路径 |

## 11. 2026-08-13 联调日志（按发生顺序）

本次没有保存每一步的精确时分秒，以下按实际发生顺序记录，不补写虚构时间。

| 顺序 | 现场现象与证据 | 根因 | 处理和验证 |
| ---: | --- | --- | --- |
| 1 | 已将 `ECAT_CANFD_GATEWAY.xml` 放入 TwinCAT 描述目录，但扫描到的 Box 下没有预期 PDO | PC 上加载了新 ESI，不代表板上旧 SII 已自动更新；从站 Flash EEPROM 仍可能保存数字 IO 示例数据 | 明确区分 ESI 导入和 SII 烧写，准备通过 `Advanced Settings > ESC Access > E²PROM` 更新从站 |
| 2 | `Write E²PROM` 中能在 `Personal` 下选到 `HPM5E31 EtherCAT to 4x CAN FD Gateway`，点击 OK 后没有明显进度；`Read E²PROM` 显示 Vendor ID、Product Code、Revision、Mailbox 和 SM 全为 0 | TwinCAT 已正确加载 XML，但从站没有完成 EEPROM 命令；全 0 不是 PDO 展开问题 | 转向检查板端串口和 SSC 主循环，不再反复导入 XML |
| 3 | 串口输出 `No need to init EEPROM content.`、`EEPROM loading successful, no checksum error.`，随后报 `Init EtherCAT/CAN FD gateway failed: 26012` | 前两行证明 Flash 中存在校验有效的旧 SII；`26012` 对应 `status_mcan_ram_out_of_range`。驱动默认按 64 字节 CAN FD 帧分配 RAM，RX FIFO0 又扩到 16 项，超过每路 2560 字节 Message RAM。初始化失败后 `main()` 直接返回，SSC `MainLoop()` 未运行，所以 TwinCAT EEPROM 请求无人处理 | 根据本协议最大 8 字节负载，将 RX FIFO0/TX FIFO 的数据区改为 8 字节，并关闭未使用的 RX FIFO1、专用 RX Buffer、过滤表 RAM 和 TX Event FIFO；编译通过 |
| 4 | 烧录第一次修正版后，串口错误变为 `Init EtherCAT/CAN FD gateway failed: 2` | `2` 对应 `status_invalid_argument`。虽然过滤表 RAM 已关闭，但 `mcan_get_default_config()` 留下的标准帧和扩展帧过滤项数量仍各为 1，驱动尝试向不存在的过滤表写入默认项，参数校验失败 | 将 `std_id_filter_list.mcan_filter_elem_count` 和 `ext_id_filter_list.mcan_filter_elem_count` 同步清零；非匹配标准帧和扩展帧仍按全局规则进入 RX FIFO0；重新编译通过 |
| 5 | 烧录第二次修正版后不再出现 CAN FD 初始化错误，TwinCAT EEPROM 操作恢复，用户确认问题解决 | MCAN 初始化和 SSC 主循环均正常运行 | 重新执行 Write/Read EEPROM、断电上电和扫描，继续检查 4 路 RxPDO/TxPDO 及 576 字节过程映像 |
| 6 | 从站进入 SAFEOP 时回到 `ERR PREOP`，故障状态下寄存器 `0x0134=0x001D` | `0x001D` 表示 Invalid Output Configuration。原 ESI 使用 SM2=`0x1100`、SM3=`0x1400`，只按单缓冲长度判断不重叠；实际 PDO 采用三缓冲，SM2 占用 `0x1100～0x17BF`，与 SM3 重叠 | 将 SM3 起始地址改为 `0x1800`，并在 ESI 生成器中加入地址自检；重新安装 XML、写 EEPROM、断电重扫后再进入 SAFEOP/OP |
| 7 | 新建 PLC 后首次 Build 只看到 `Project build failed` 汇总信息 | 汇总警告不包含真正语法原因，需要查看 Build Output 或错误列表中的第一条红色错误 | 确认 DUT/GVL 均在同一 PLC Project；随后 Build 输出为 `0 errors, 0 warnings`，TMC 和符号导入成功 |
| 8 | 加入测试程序后报 `Identifier 'MAIN' not defined`、`Program name... expected instead of 'MAIN'` | PlcTask 中存在 MAIN 调用，但 `POUs` 下没有有效的 `PROGRAM MAIN`，或任务仍引用已失效对象 | 在 `POUs` 创建 Program 类型的 MAIN，将其重新拖到 PlcTask，并确保 `END_IF;` 有分号 |
| 9 | 链接 `IN_CANFD1_CycleSeq` 时目标窗口看不到可直接选择的 `gCanIn.bus[0]` | 当前源变量只有 2 字节，整个 `bus[0]` 是 144 字节；Matching Size 会禁止二者直接单项链接 | 展开 `GVL_CANFD.gCanIn > bus > bus[0]`，将该字段链接到 `cycle_seq`；若选中整个 PDO，则使用 Change Multi Link 连续链接 144 字节 |
| 10 | DC-Synchron 下出现 `Frame missed 10 times`，从站反复 `OP → SAFEOP`，AL Code=`0x001A Synchronization error` | 从站检测到 SYNC0 与 SM2 周期事件失配。PLC 任务执行时间约 6.1 μs，负载本身很低，但断点、配置切换或 PC/网卡实时抖动仍会造成周期帧丢失；单次 Exceed Counter=1 需清零后持续观察 | 禁止在 EtherCAT 实时任务中使用断点，任务 Online 页 Reset 后观察 Exceed Counter；为先验证功能，将从站改为 SM-Synchron，DC 1 ms 留作后续实时性专项验收 |
| 11 | PLC 在线时程序一直执行 ELSE，`gCanOut.bus[0].valid_mask=0` | `testEnable` 声明初值为 FALSE，安全分支主动禁止发送 | PLC 保持 Run，不打断点；双击在线变量把准备值改为 TRUE，再执行 Write Values，程序进入发送分支 |
| 12 | SM-Synchron 下在线写入 `testEnable=TRUE` 后，CAN FD 分析仪收到测试帧 | ESI/SII、576 字节过程映像、PLC 输出映射、HPM 网关和 CANFD1 发送链路均工作 | 现场确认 ID=`0x123`、DLC=8、CAN FD+BRS、Data=`11 22 33 44 55 66 77 88`；主站到 CANFD1 的下行透传功能验收通过 |
| 13 | 分析仪每 20 ms 发送 1000 帧，PLC 主站最初显示约 1475 帧 | PLC 每 1 ms 执行一次，但输入 PDO 不一定每次都是新窗口；直接累加 `mode` 会把同一帧重复计算 | 不再逐扫描累加 `mode`，改用当前窗口最后一槽的 `seq` 判断新窗口；相同 `seq` 时增量为 0 |
| 14 | 改为 `seq` 差值后，接收总数曾突然增加一万多 | 从站重启或重新登录后新 `seq` 小于旧值，程序把它无条件当成 16 位回绕，得到很大的差值 | 只允许 `0xFF00→0x00FF` 附近发生正常回绕；其他变小按重启处理，并将单次 `seqDelta>256` 限制为当前 `mode` |
| 15 | 只按“`seq` 改变就累加 `mode`”后，1000 帧仍少约 10 帧 | 该算法能去重，但 PLC 可能完全错过某个瞬时 PDO 窗口，窗口中的帧无法通过 `mode` 恢复 | 对正常递增使用 `currentRxSeq-lastRxSeq`，由后续序号补回中间遗漏窗口；同时检查 `counter0`，区分主站漏看与网关实际丢弃 |
| 16 | 分析仪按 5 ms 周期发送，理论应为 200 frame/s，PLC 一度显示约 160 frame/s | 用 PLC 扫描次数凑 1000 次作为“一秒”，统计窗口依赖任务实际周期和程序调用节拍，时间基准不可靠 | 改用标准 `TON`、`PT=T#1S` 建立真实时间窗口；用户复测确认帧率问题解决 |
| 17 | 增加 CAN 活动灯后，板卡刚上电、没有 CAN 数据时 RX 灯常亮；接收开始后能闪，停止后才灭 | PinMux 生成代码在把 LED 引脚切换为 GPIO 时会短暂写低，而板上 RX/ERR 灯低电平点亮；仅依赖静态初值不能保证应用启动后的最终状态 | 在 `ethercat_canfd_gateway_init()` 启用 MCAN 中断前遍历四路 TX/RX/ERR，清零脉冲计数并按各自 `OFF_LEVEL` 显式熄灭；验证无数据时灭、收到一帧至少亮 1 ms、停止后灭 |
| 18 | TwinCAT 开始发送后红灯亮，最初被误认为 TX 灯；分析仪接入与否会改变亮灯状态 | 现场确认红灯是 ERR，绿灯才是 TX。总线上没有另一个 Active 节点 ACK、分析仪处于 Listen Only、波特率/BRS 不匹配时都会产生 CAN 错误并点亮 ERR；ACK 是其他 CAN 控制器在 ACK 时隙自动确认，不是一条应用回复报文 | 将分析仪设为 Normal/Active、ISO CAN FD、仲裁段 1 Mbit/s、数据段 5 Mbit/s、BRS 开启，并核对终端与公共地；PLC 使用 `sendPeriod` 降至 500 ms 方便分辨活动灯 |
| 19 | 分析仪已经稳定收到 PLC 下发报文，但 TX 绿灯始终不亮；降低发送频率、使用 `ON_CHANGE` 后现象不变 | 报文链路本身正常，问题在 TX 完成中断配置。HPM MCAN 的发送完成有两级使能：全局 `MCAN_INT_TX_COMPLETED` 和每个 TX Buffer/FIFO 元素的 `TXBTIE`。原代码只开全局 TC，而默认 `txbuf_trans_interrupt_mask=0`，因此不会产生 TX 完成中断 | 设置 `config.txbuf_trans_interrupt_mask=(1UL << CANFD_TX_FIFO_DEPTH)-1UL`，使能8个 TX FIFO 元素的完成中断；保留 `MCAN_EVENT_TRANSMIT` 全局中断，发送成功后由硬件事件触发 TX 绿灯至少 1 ms；命令行重新编译通过 |

最终固件修正位于 `application/ethercat_canfd_gateway.c`，ESI 地址修正位于 `tools/generate_canfd_gateway.js`。相关配置旁已添加中文注释，说明为什么 Message RAM 的单项容量保持为 8 字节、关闭过滤表后必须清零默认过滤项，以及三缓冲 SM 地址不能重叠。

### 11.1 DLC 0～8 可变长度支持记录

固件在保持现有 16 字节帧槽、576 字节过程映像及 ESI/XML 不变的前提下，已把收发范围由固定 DLC=8 扩展为 DLC 0～8。下发时仅发送 `data[0..dlc-1]`；上报时也只复制实际有效字节，其余尾部字节清零，避免上一帧数据残留。

验证时把测试槽的 `dlc` 依次改为 `0、1、3、7、8`，每次递增 `seq`，分析仪应看到相同 DLC。反向发送相同 DLC 的报文，TwinCAT 应看到相同 DLC，且 `data[dlc..7]` 为 0。DLC 9～15 仍会被明确拒绝：下发不发送并闪 ERR；接收会丢弃、闪 ERR，并使对应通道 `counter0` 加 1。

截至本日志结束，`SM-Synchron + 1 ms PlcTask` 下 CANFD1 双向透传已验证：PLC 下发帧可由 CAN FD 分析仪接收，分析仪周期发送的帧也可由 PLC 接收并统计总数和帧率。当前 5 ms 周期测试的帧率问题已通过 `TON` 真实时间窗口解决。尚未完成的项目是：其余三路收发、满负载/故障测试、物理层真实误码率测试，以及 `DC-Synchron 1 ms` 在当前 TwinCAT PC 上的长期稳定性验收。

注意：`SSC/Src/eeprom.h` 仍来自原数字 IO SSC 工程。如果执行整片 Flash 擦除，板端 EEPROM 仿真区可能丢失，固件随后可能再次装入旧 SII。量产前应使用 SSC Tool 以网关设备描述重新生成 `eeprom.h`，使固件内置 SII 与 `ECAT_CANFD_GATEWAY.xml` 一致。
