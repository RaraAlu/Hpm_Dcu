# HPM ENCOS 首次控制电机与 TwinCAT 测试说明

本文用于第一次连接一台 ENCOS 电机，通过 TwinCAT 配置 HPM，并让电机以力位混控模式做小角度运动。第一次测试只接 CAN1 的一台电机，确认通信、反馈、停机都正常后再增加电机。

## 1. 测试目标和前提

测试对象约定如下：

| 项目 | 测试值 |
|---|---:|
| EtherCAT 从站 | HPM ENCOS 4CAN 28AXIS IMU |
| CAN通道 | CAN1 |
| 全局轴号 | 0 |
| 电机槽位 | CAN1 Motor1 |
| 电机CAN ID | 1 |
| EtherCAT周期 | 2 ms |
| 命令租约 | 10 ms |
| 控制模式 | Mode 4，力位混控 |

开始前应具备：

- HPM已经烧录新版固件，并写入新版SII。
- TwinCAT已经加载 `SSC/HPM_ENCOS_4CAN_28AXIS_IMU.xml`。
- TwinCAT扫描结果显示ProductCode 2、Revision 2。
- 默认RUN_MIXED下，SM2为784 B，SM3为1072 B。
- 已确认电机CAN ID以及经典CAN/CAN FD通信模式；混控范围将在PREOP中通过SDO事务从电机读取。
- 电机固定可靠，输出端没有负载或人员接触，急停和电源开关可以立即操作。

## 2. 电机ID、通信模式和接线

电机ID在同一路CAN内必须唯一，只允许1～8。不同CAN通道可以重复使用相同ID。第一次测试建议槽位与ID一致：Motor1使用ID1，Motor2使用ID2，以此类推。

当前HPM固件不会主动修改电机ID。若电机不是ID1，先使用ENCOS厂家调试工具修改，或者在HPM的 `0x3000:01 CanId` 中填写电机当前ID。

接线检查：

1. HPM CAN1_H连接电机CAN_H，CAN1_L连接电机CAN_L，并连接通信地。
2. 总线两端各接一个120 Ω终端电阻。断电测量CAN_H与CAN_L，正常约为60 Ω。
3. 第一次只接一台电机，线缆尽量短。
4. 确认电机采用经典CAN 1 Mbps，或者CAN FD+BRS 1/5 Mbps。HPM配置必须完全一致。
5. 先给HPM上电并建立EtherCAT，再给电机动力电源上电。

## 3. 首次配置采用“先读电机、再应用缓存”

电机内部通常已经保存出厂混控范围和Kt。首次控制不手工猜测这些值，也不重新写入电机，流程统一为：

1. 在PREOP中配置CAN通道、帧格式和电机ID。
2. 将控制权切换为SDO。
3. 通过轴参数事务依次查询Kt、Kp/Kd范围、位置/速度/扭矩/电流范围和CAN超时。
4. 每个查询成功后，HPM自动把电机返回的原始值写入对应实际缓存对象。
5. 上位机读回缓存并检查范围是否合理。
6. 根据读取的协议范围设置更小的HPM本地运行安全限值。
7. 执行ApplyGeneration，将候选缓存变为HPM当前运行配置。
8. 将控制权切回PDO，再进入SAFEOP/OP。

直接读取 `0x3001/0x3002` 只会得到HPM当前缓存。上电后必须先执行查询事务，不能把初始0当成电机实际参数。

## 4. 在TwinCAT中安装并扫描从站

1. 将 `SSC/HPM_ENCOS_4CAN_28AXIS_IMU.xml` 复制到：

   ```text
   C:\TwinCAT\3.1\Config\Io\EtherCAT
   ```

2. 启动TwinCAT XAE，执行 `TwinCAT → EtherCAT Devices → Reload Device Descriptions`，或重启Visual Studio/TwinCAT XAE。
3. 新建TwinCAT XAE工程，在I/O下添加EtherCAT Master。
4. 切换到Config Mode，右键EtherCAT Master，执行 `Scan`。
5. 接受找到的新从站，但不要立即进入OP。
6. 选择HPM从站，在EtherCAT页面核对：
   - ProductCode：`0x00000002`
   - Revision：`0x00000002`
   - Output/SM2：784 B
   - Input/SM3：1072 B
7. 在Process Data页面确认当前Assignment是RUN_MIXED：RxPDO包含板头、4个通道头、28轴和SharedRaw；TxPDO还包含IMU。

## 5. PREOP下读取电机参数并配置轴0

将TwinCAT保持在PREOP，打开从站的CoE Online页面。第一次测试建议先手动完成并记录结果；流程确认后再做成上位机自动初始化。

### 5.1 建立最小查询通道

先写入以下对象：

| 对象 | 名称 | 写入值 |
|---|---|---:|
| `0x2010:01` | CAN1 Role | `1` |
| `0x2010:02` | FrameFormat | 经典CAN写`0`，CAN FD+BRS写`1` |
| `0x2010:03` | NominalBitrate | `1000000` |
| `0x2010:04` | DataBitrate | `5000000` |
| `0x2010:05` | ScheduleUs | `2000` |
| `0x2010:06` | RawRateLimit | `0` |
| `0x3000:01` | CanId | `1`，或电机当前实际ID |
| `0x3001:02` | CommunicationMode | 必须与 `0x2010:02` 相同 |
| `0x2003:02` | RequestedOwner | `2`，切换为SDO控制 |

读取 `0x2003:01 Owner`，必须为2。CAN2～CAN4保持Role 0。此时不要设置运动使能，也不要进入OP。

### 5.2 逐项查询电机

轴0的参数事务对象为 `0x3007`：

| 对象 | 名称 | 用法 |
|---|---|---|
| `0x3007:01` | ParameterId | 写入下表参数编号 |
| `0x3007:02` | Operation | 固定写`1`，表示读取 |
| `0x3007:03` | RequestSeq | 最后写；每次查询递增且不能重复 |
| `0x3007:04` | DoneSeq | 等于本次RequestSeq时完成 |
| `0x3007:05` | Result | 0成功、1忙、4超时、7电机返回错误 |
| `0x3007:07` | RawResponseLength | 电机原始回复长度 |
| `0x3007:08` | RawResponse | 电机原始回复，供排查 |

按以下顺序逐项执行；上一项完成且Result为0后才能开始下一项：

| RequestSeq示例 | ParameterId | 从电机读取 | 成功后HPM自动更新的缓存 |
|---:|---:|---|---|
| 1 | 3 | Kt | `0x3001:03 TorqueConstantRaw` |
| 2 | 4 | Kp Min/Max | `0x3002:01/02` |
| 3 | 5 | Kd Min/Max | `0x3002:03/04` |
| 4 | 6 | Position Min/Max | `0x3002:05/06` |
| 5 | 7 | Speed Min/Max | `0x3002:07/08` |
| 6 | 8 | Torque Min/Max | `0x3002:09/0A` |
| 7 | 9 | Current Min/Max | `0x3002:0B/0C` |
| 8 | 10 | CAN Timeout | `0x3001:04 CanTimeoutMs` |

以查询Kt为例：

```text
写 0x3007:01 = 3
写 0x3007:02 = 1
最后写 0x3007:03 = 1
轮询 0x3007:04，直到 DoneSeq = 1
读取 0x3007:05，必须 Result = 0
读取 0x3001:03，得到电机Kt原始值
```

查询Kp时把ParameterId改为4、RequestSeq改为2，后续以此类推。查询成功后固件已经把返回值写进HPM实际候选缓存，上位机不需要再把同一个值写回 `0x3001/0x3002`。

### 5.3 读回并校验HPM缓存

全部查询成功后读回：

| 缓存 | 换算方式 |
|---|---|
| `0x3001:03` | Kt实际值 = Raw/100 Nm/A |
| `0x3001:04` | CAN超时 = Raw ms；必须非0且不大于500 ms |
| `0x3002:01/02` | Kp Min/Max = Raw |
| `0x3002:03/04` | Kd Min/Max = Raw |
| `0x3002:05/06` | Position Min/Max = Raw/100 rad |
| `0x3002:07/08` | Speed Min/Max = Raw/100 rad/s |
| `0x3002:09/0A` | Torque Min/Max = Raw/10 Nm |
| `0x3002:0B/0C` | Current Min/Max = Raw/10 A |

位置、速度、扭矩和电流范围必须是有符号INT16并跨过0。Kp/Kd最小值必须小于最大值。任何查询超时、返回错误或范围异常都应停止流程，不能Apply。

### 5.4 设置HPM本地配置和安全限值

确认缓存正确后写入：

| 对象 | 名称 | 写入说明 |
|---|---|---|
| `0x2001:01` | AxisConfiguredMask | `0x00000001` |
| `0x2001:02` | MaxCommandLeaseMs | `10` |
| `0x2001:03` | FeedbackTimeoutMs | `20` |
| `0x2010:07` | MotorConfiguredMask | `0x01` |
| `0x3000:02` | ModelProfileId | `0x8000` |
| `0x3000:03` | PositionLimitMin | 本地最小位置，单位10^-6 rad |
| `0x3000:04` | PositionLimitMax | 本地最大位置，单位10^-6 rad |
| `0x3000:05` | VelocityLimit | 本地最大速度，单位10^-6 rad/s |
| `0x3000:06` | CurrentLimit | 本地电流上限，单位0.1 A |
| `0x3000:07` | TorqueLimit | 本地扭矩上限，单位mNm |
| `0x3000:08` | StopPolicy | `0` |
| `0x3000:09` | FeedbackPolicy | `1` |
| `0x3000:0A` | PowerQueryPeriodMs | 第一次测试写`0` |

本地安全限值必须位于刚读取的协议范围内，并应小于电机额定值。换算上限参考：Position/Speed查询Raw换成本地单位时乘10000；Current查询Raw与本地0.1 A计数相同；Torque查询Raw换成mNm时乘100。第一次测试应进一步缩小允许范围。

### 5.5 Apply并切回PDO控制

1. 向 `0x2002:01 ApplyGeneration` 写入一个新的非零代数，例如1。
2. 读取 `0x2002:03 Result`，必须为0。
3. 读取 `0x2002:04 AppliedGeneration`，必须等于本次代数。
4. 向 `0x2003:02 RequestedOwner` 写入1。
5. 读取 `0x2003:01 Owner`，确认已经变为1（PDO控制）。
6. 再进入SAFEOP/OP。

若Apply失败，不要进入OP。优先检查查询缓存、量程是否跨0、本地限值是否在读取范围内、CommunicationMode、CAN ID和掩码。

## 6. 在使能前读取当前位置

配置应用成功后进入OP，但所有输出变量先保持0。HPM会对已配置且未使能的电机轮询位置查询。

检查：

- `0x6100:02 OnlineMask` 的bit0应变为1。
- `0x2400:04 RxValidCount` 应持续增加。
- 在CoE Online读取 `0x3005:01 PositionDeg`。
- `0x3005:0B ValidBits` 的bit0应为1。

将位置转换为PLC使用的微弧度：

```text
StartPosition_urad = PositionDeg × π / 180 × 1000000
```

例如查询位置为10°：

```text
10 × π / 180 × 1000000 ≈ 174533
```

把计算结果填入后面的PLC参数 `StartPosition_urad`。首次目标使用当前位置，不能直接使用0。

## 7. 创建TwinCAT PLC测试程序

新建一个Standard PLC Project，将PLC任务周期设置为2 ms。建立以下PLC变量，并在I/O Process Data中逐项链接到HPM变量。

### 7.1 需要链接的下行变量

| PLC变量 | EtherCAT对象 |
|---|---|
| `Out_CycleSeq` | `0x7000:01 Seq` |
| `Out_BoardControl` | `0x7000:02 Ctrl` |
| `Out_ChannelEnableMask` | `0x7000:03 ChEnable` |
| `Out_CommandLeaseMs` | `0x7000:05 LeaseMs` |
| `Out_InterfaceVersion` | `0x7000:07 InterfaceVersion` |
| `Out_AxisEnableMask` | `0x7000:08 AxisEnableMask` |
| `Out_CAN1_Control` | `0x7100:01 ChannelControl` |
| `Out_CAN1_ActiveMask` | `0x7100:02 ActiveMask` |
| `Out_CAN1_ExpectedRole` | `0x7100:03 ExpectedRole` |
| `Out_M0_ControlWord` | `0x7200:01 Ctrl` |
| `Out_M0_Mode` | `0x7200:02 Mode` |
| `Out_M0_TargetPosition` | `0x7200:04 Pos` |
| `Out_M0_TargetVelocity` | `0x7200:05 Vel` |
| `Out_M0_TargetTorque` | `0x7200:06 Torque` |
| `Out_M0_TargetCurrent` | `0x7200:07 Current` |
| `Out_M0_Kp` | `0x7200:08 Kp` |
| `Out_M0_Kd` | `0x7200:09 Kd` |
| `Out_M0_CurrentLimit` | `0x7200:0A CurrentLim` |

Reserved和未使用字段保持0。

### 7.2 建议链接的上行变量

| PLC变量 | EtherCAT对象 |
|---|---|
| `In_BoardStatus` | `0x6000:03 Status` |
| `In_CAN1_Status` | `0x6100:01 ChannelStatus` |
| `In_CAN1_OnlineMask` | `0x6100:02 OnlineMask` |
| `In_M0_StatusWord` | `0x6200:01 Status` |
| `In_M0_ValidFlags` | `0x6200:03 Valid` |
| `In_M0_ActualPosition` | `0x6200:04 Pos` |
| `In_M0_ActualVelocity` | `0x6200:05 Vel` |
| `In_M0_ActualCurrent` | `0x6200:07 Current` |
| `In_M0_MotorFault` | `0x6200:0A MotorFault` |
| `In_M0_BridgeFault` | `0x6200:0B BridgeFault` |
| `In_M0_FeedbackAgeMs` | `0x6200:0D FbAgeMs` |

### 7.3 PLC变量和程序

变量类型必须与对象类型一致。下面的变量需要在TwinCAT中与上一节PDO逐项链接。

```iecst
PROGRAM MAIN
VAR
    (* 操作参数 *)
    StartTest           : BOOL := FALSE;
    StopTest            : BOOL := FALSE;
    StartPosition_urad  : DINT := 0;       (* 填入CoE查询并换算后的当前位置 *)
    MoveDistance_urad   : DINT := 10000;   (* 0.01 rad，小角度测试 *)
    TestKp              : UINT := 100;     (* 表示Kp=1.00，仅在电机允许范围内使用 *)
    TestKd              : UINT := 100;     (* 表示Kd=0.100，仅在电机允许范围内使用 *)

    (* 状态机 *)
    Step                : UINT := 0;
    StopCycles          : UINT := 0;
    TargetPosition      : DINT := 0;

    (* RxPDO：PLC到HPM *)
    Out_CycleSeq            : UDINT;
    Out_BoardControl        : UINT;
    Out_ChannelEnableMask   : USINT;
    Out_CommandLeaseMs      : UINT;
    Out_InterfaceVersion    : UDINT;
    Out_AxisEnableMask      : UDINT;
    Out_CAN1_Control        : UINT;
    Out_CAN1_ActiveMask     : USINT;
    Out_CAN1_ExpectedRole   : USINT;
    Out_M0_ControlWord      : UINT;
    Out_M0_Mode             : USINT;
    Out_M0_TargetPosition   : DINT;
    Out_M0_TargetVelocity   : DINT;
    Out_M0_TargetTorque     : DINT;
    Out_M0_TargetCurrent    : INT;
    Out_M0_Kp               : UINT;
    Out_M0_Kd               : UINT;
    Out_M0_CurrentLimit     : UINT;

    (* TxPDO：HPM到PLC *)
    In_BoardStatus          : UINT;
    In_CAN1_Status          : UINT;
    In_CAN1_OnlineMask      : USINT;
    In_M0_StatusWord        : UINT;
    In_M0_ValidFlags        : USINT;
    In_M0_ActualPosition    : DINT;
    In_M0_ActualVelocity    : DINT;
    In_M0_ActualCurrent     : INT;
    In_M0_MotorFault        : USINT;
    In_M0_BridgeFault       : USINT;
    In_M0_FeedbackAgeMs     : UINT;
END_VAR

(* 每个PLC周期产生新序号，不能停止递增 *)
Out_CycleSeq := Out_CycleSeq + 1;
Out_CommandLeaseMs := 10;
Out_InterfaceVersion := 16#00010000;
Out_CAN1_ExpectedRole := 1;

(* 未使用的混控字段始终为0 *)
Out_M0_TargetVelocity := 0;
Out_M0_TargetTorque := 0;
Out_M0_TargetCurrent := 0;
Out_M0_CurrentLimit := 0;

IF StopTest THEN
    Step := 0;
    StartTest := FALSE;
END_IF

CASE Step OF
    0:  (* 连续发送至少100 ms停止快照，完成HPM会话解锁 *)
        Out_BoardControl := 0;
        Out_ChannelEnableMask := 0;
        Out_AxisEnableMask := 0;
        Out_CAN1_Control := 0;
        Out_CAN1_ActiveMask := 0;
        Out_M0_ControlWord := 0;
        Out_M0_Mode := 0;
        Out_M0_TargetPosition := StartPosition_urad;
        Out_M0_Kp := 0;
        Out_M0_Kd := 0;

        IF StopCycles < 50 THEN
            StopCycles := StopCycles + 1;
        ELSIF StartTest AND ((In_CAN1_OnlineMask AND 1) <> 0) THEN
            TargetPosition := StartPosition_urad;
            Step := 10;
        END_IF

    10: (* 第一次使能，目标仍为当前位置 *)
        Out_BoardControl := 1;
        Out_ChannelEnableMask := 1;
        Out_AxisEnableMask := 1;
        Out_CAN1_Control := 1;
        Out_CAN1_ActiveMask := 1;
        Out_M0_ControlWord := 1;
        Out_M0_Mode := 4;
        Out_M0_TargetPosition := TargetPosition;
        Out_M0_Kp := TestKp;
        Out_M0_Kd := TestKd;

        IF (In_M0_StatusWord AND 16#0008) <> 0 THEN
            Step := 20;
        END_IF

    20: (* 保持当前位置；人工确认稳定后令StartTest产生第二次上升沿 *)
        Out_BoardControl := 1;
        Out_ChannelEnableMask := 1;
        Out_AxisEnableMask := 1;
        Out_CAN1_Control := 1;
        Out_CAN1_ActiveMask := 1;
        Out_M0_ControlWord := 1;
        Out_M0_Mode := 4;
        Out_M0_TargetPosition := TargetPosition;
        Out_M0_Kp := TestKp;
        Out_M0_Kd := TestKd;

        IF NOT StartTest THEN
            Step := 30;
        END_IF

    30: (* 等待再次按下StartTest *)
        Out_BoardControl := 1;
        Out_ChannelEnableMask := 1;
        Out_AxisEnableMask := 1;
        Out_CAN1_Control := 1;
        Out_CAN1_ActiveMask := 1;
        Out_M0_ControlWord := 1;
        Out_M0_Mode := 4;
        Out_M0_TargetPosition := TargetPosition;
        Out_M0_Kp := TestKp;
        Out_M0_Kd := TestKd;

        IF StartTest THEN
            TargetPosition := StartPosition_urad + MoveDistance_urad;
            Step := 40;
        END_IF

    40: (* 保持小角度目标 *)
        Out_BoardControl := 1;
        Out_ChannelEnableMask := 1;
        Out_AxisEnableMask := 1;
        Out_CAN1_Control := 1;
        Out_CAN1_ActiveMask := 1;
        Out_M0_ControlWord := 1;
        Out_M0_Mode := 4;
        Out_M0_TargetPosition := TargetPosition;
        Out_M0_Kp := TestKp;
        Out_M0_Kd := TestKd;
END_CASE
```

如果电机允许的Kp/Kd范围不包含示例中的1.00/0.100，应修改 `TestKp/TestKd`。PLC数值换算是：`Kp变量=实际Kp×100`，`Kd变量=实际Kd×1000`。

## 8. TwinCAT启动电机的实际操作

1. 下载PLC程序，登录PLC并Run。
2. 确认 `Step=0`，`StartTest=FALSE`，`StopTest=FALSE`。
3. 检查 `In_CAN1_OnlineMask AND 1` 不为0。
4. 再次确认 `StartPosition_urad` 是本次上电读取的当前位置，不是上次测试残留值。
5. 将 `StartTest` 置TRUE。程序进入Step 10，电机以当前位置为目标使能力位混控。
6. 观察 `In_M0_StatusWord`：
   - bit0 `0x0001`：Configured
   - bit1 `0x0002`：Online
   - bit2 `0x0004`：Ready
   - bit3 `0x0008`：CommandActive
7. 确认电机没有突然运动、异响或过流，然后把 `StartTest` 置FALSE。
8. 程序进入Step 30后，再把 `StartTest` 置TRUE。目标位置增加0.01 rad，电机应做小角度运动。
9. 观察位置、速度、电流、反馈年龄和故障码。反馈正常时 `FeedbackAgeMs` 应小于20 ms。
10. 测试结束时把 `StopTest` 置TRUE。程序立即撤销板级、通道和轴使能并回到Step 0。

不要通过TwinCAT的“Force一次输出值”直接启动电机。HPM要求CycleSeq持续递增并在10 ms内收到新命令，单次强制写入会触发租约超时。

## 9. 正常现象和故障判断

| 现象 | 检查内容 |
|---|---|
| OnlineMask始终为0 | CAN_H/L、60 Ω终端、CAN ID、经典CAN/CAN FD、1 Mbps/5 Mbps、电机是否上电 |
| ApplyGeneration Result非0 | 电机掩码、量程跨0、限值、ModelProfileId、CommunicationMode、CAN超时 |
| StatusWord只有Configured | 未收到电机查询回复，检查CAN通信 |
| Ready有但CommandActive没有 | 未先发送停止快照、ControlWord没有0→1边沿、板/通道/轴使能不完整 |
| BridgeFaultCode=2 | 目标、Kp/Kd、位置/速度/电流/扭矩限值不合法 |
| BridgeFaultCode=3 | CycleSeq未递增，或PLC任务/链接中断超过CommandLeaseMs |
| BridgeFaultCode=4 | 类型1反馈超过20 ms未收到 |
| BridgeFaultCode=5 | CAN bus-off，检查接线、终端和波特率 |
| BridgeFaultCode=9 | InterfaceVersion不是 `0x00010000` |
| 电机方向相反 | 停机后检查机构坐标定义，不要在线直接修改量程符号 |

电机有运动趋势但方向或刚度不正确时，立即置 `StopTest=TRUE`。在确认方向之前保持小Kp、小Kd、零速度、零前馈扭矩和很小的位置增量。

## 10. 测试记录

首次测试建议记录：

| 项目 | 结果 |
|---|---|
| 电机型号/序列号 | |
| CAN ID及通信模式 | |
| 六组混控量程 | |
| Kt | |
| HPM固件版本 | |
| ESI ProductCode/Revision | |
| ApplyGeneration Result | |
| 初始位置PositionDeg/urad | |
| TestKp/TestKd | |
| 位置增量 | |
| 最大电流 | |
| 最大FeedbackAgeMs | |
| MotorFaultRaw/BridgeFaultCode | |
