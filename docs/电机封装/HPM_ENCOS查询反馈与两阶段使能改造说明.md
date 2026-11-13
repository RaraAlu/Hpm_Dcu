# HPM ENCOS：仅查询反馈与两阶段启动改造说明

> 历史设计交接稿：当前源码已有部分对应实现，不能直接沿用下文“尚未修改”的成文时状态；是否满足全部设计要求及硬件效果仍需验收。当前状态入口见[设计索引](../design/README.md)。

> 2026-09-18 设计交接稿，供 Windows 上的 HPM 工程和 Linux IgH 上位机配套实施。**本文不是已烧录/已验证的固件说明；当前仓库尚未按本文修改控制代码。**

## 1. 结论：是否需要改 SSC？

推荐先做**不改变 PDO 字节布局**的版本：复用单轴 RxPDO `ControlWord` 的空闲 bit3 作为 `StartMotion`，复用 TxPDO `StatusWord` 的空闲 bit12、bit13 分别表示“当前值来自查询”和“已布防/仅查询”。保留单轴 RxPDO 24 B、TxPDO 32 B，以及 `RUN_MIXED` 的 SM2 784 B、SM3 1072 B。这样：

- **不用修改通用 SSC 栈**，不必更改 `SSC/Src/ecatslv.c`、`coeappl.c` 等，也不必重新设计 `0x16xx/0x1Axx` 映射或 SM 地址。业务逻辑应改在 `application/motor_control.c` 等文件。`application/digital_io.c` 已把 `APPL_GenerateMapping()`、`APPL_InputMapping()`、`APPL_OutputMapping()` 接到电机控制模块。
- **测试版可以不改板载 SII/EEPROM 和 XML 的 PDO 条目**，前提是对象索引、子索引、类型、位宽、顺序和字节数均不变。必须同步更新本文件、上位机位定义与兼容性检查；建议提高 `0x2000:01 InterfaceVersion`，使旧版上位机拒绝连接新协议。注意上位机当前也校验 `0x1018:03 Revision` 和 `0x2000:01 InterfaceVersion`。
- **正式发布建议提高 Revision**，让不同语义的固件可识别。此时要同步更新生成源、ESI XML、SII/EEPROM 内置镜像、固件对象字典和上位机期望值；这属于版本发布工作，不是“仅查询”逻辑本身要求 SSC 栈变化。
- 若要新增每字段独立的 PDO 龄期、改变 `0x6200+a` 或 `0x7200+a` 结构大小/顺序，或新增 PDO 映射条目，**必须**同步改生成源、`application/encos_interface.h`、`SSC/Src/gatewayObjects.h`、XML、SII/EEPROM、映射长度与上位机；此时不能沿用旧 784/1072 B 映射。不要只手改 XML。

当前仓库的 `tools/generate_encos.js` 文件在本机检查时不是可读的 JavaScript 文本；Windows 上执行生成命令之前，先确认你持有可运行的原始生成脚本并核对生成结果。`SSC/Src/digital_io.c` 是旧 SSC 示例代码，实际工程使用的业务回调在 `application/digital_io.c`，不要把示例覆盖进来。既有升级流程参考 `readme/电机封装/HPM_ENCOS_SSC与EEPROM升级使用说明.md`。

## 2. 可获得什么反馈，不能获得什么

依据 `material_ref/电机资料/ENCOS+调试技术手册+V1.19EAP.pdf` 的 9.3、10.5 节：问答查询请求可用查询码 1=位置（角度制 float）、2=速度（RPM float）、3=相电流（A float）、4=功率（W float）。成功返回的 CAN 帧**报文类型是 5**，Byte1 是查询码，后续各有 4 字节数据，故每个回复总共 6 字节。请求和回复需要按 CAN 通道、Motor ID、查询码、DLC 及事务状态配对，并检查 Byte0 的电机错误位和浮点值是否有限。

Type-5 的“5”是**回复类型**，不是“查询码 5”；查询码 5 是加速度。类型 5 **没有**电机温度、MOS 温度的查询码。无控制帧的阶段可以显示位置、速度、电流、功率与由电流×Kt估算的力矩，但温度应显示“查询模式不提供/无效”，不能沿用旧 Type-1 温度作为新值。各查询结果采集时刻不同，不能称为一次同步反馈快照。

更重要的是：查询帧不等于电机厂商的物理使能命令。若 ENCOS 协议没有独立的非致动使能指令，本文“使能电机”按钮在查询阶段只能表示 **HPM 已布防、在线监测、尚未发运动控制帧**；不能声称电机已经上力或持位。

## 3. 建议的状态和现有问题

```text
未配置/停机 → 已配置/在线 → 已布防(仅查询) → 启动确认 → 运行(Type-1)
                                      ↘ 停止/超时/故障 → 停机或故障锁存
```

建议协议位定义如下（均为**待实施**定义）：

| 位置 | 含义 | 备注 |
|---|---|---|
| `0x7200+a:01 ControlWord.bit0` | 布防/使能请求 | 当前版本 bit0 会直接使 HPM 发送周期控制帧；新版本需改为仅进入查询状态 |
| `ControlWord.bit1` | 停止 | 保持最高优先级；取消该轴尚未发出的控制帧 |
| `ControlWord.bit2` | 本地故障复位边沿 | 保持现有复位条件 |
| `ControlWord.bit3` | `StartMotion` | 仅 bit0+bit3 且查询反馈/安全条件满足时才可发送 ENCOS 控制帧 |
| `0x6200+a:01 StatusWord.bit3` | `CommandActive` | **只在已实际开始控制帧发送/运行时为 1**；仅查询时为 0 |
| `StatusWord.bit12` | `QuerySource` | 位置/速度/电流来自查询合成快照；运行 Type-1 时清零 |
| `StatusWord.bit13` | `ArmedQueryOnly` | 已布防但尚未发送运动控制帧 |

现有 `motor_control.c::accept_axis()` 在 bit0=1 时先调用 `encos_encode()` 并进入 `active`，调度器随后生成控制帧；`encos_motor.c::encos_encode()` 还以 `(ControlWord & ~7u)` 拒绝 bit3。仅修改上位机按钮或写 bit3 都不能实现该状态机。

现有 HPM 虽会对未运行的电机做内部低频位置查询，`cache_query()` 也能保存查询码 1～4，但只给位置/速度写入诊断缓存，查询电流没有进入实时 TxPDO；`publish()` 的 `FeedbackAgeMs`、`ValidFlags` 与 `FeedbackStale` 依赖 Type-1。当前 `ready()` 的在线窗口也共用 20 ms `FeedbackTimeoutMs`，与多轴分时查询不相容。这些地方必须一起改，不能把旧 `feedback_ms` 简单改成“收到任意回复就刷新”。

## 4. Windows HPM 工程具体改动

| 文件/稳定位置 | 要做的事 |
|---|---|
| `application/motor_control.c` — `axis_t`、`accept_axis()`、`motor_control_tick()` | 将 `armed`（接收 0→1 边沿）与“查询布防/运行”分开；bit0=1、bit3=0 时仅允许查询，不设置 `active`、`control_sent`，不发送位置保持帧。bit3 启动必须有新的有效边沿，防止 OP 恢复后自动重播旧目标。停止、失联、Bus-Off、主站租约过期应清掉启动许可。 |
| `application/motor_control.c` — `start_query()`、`consume_feedback()`、`cache_query()`、`motor_control_process()` | 每通道单个在途查询，按轴轮询并按字段到期时间错峰；成功 Type-5 查询码 1/2/3/4 时更新各字段值和时间戳。普通 SDO 参数查询/维护优先或互斥，内部轮询不能占满服务槽；对超时、错码、错 ID、错 DLC 和错误位分别计数，并有限重试。运行中的 Type-1 控制/反馈优先于低频功率查询。 |
| `application/motor_control.c` — `ready()`、`publish()` | 查询阶段单独计算在线与字段有效性，不再拿 20 ms Type-1 阈值判查询回复。把位置 rad×10^6、速度 rad/s×10^6、电流 A×100、功率 W×1000 放入现有 `0x6200+a` 字段；只有 Kt 和电流均有效时才把电流×Kt估算力矩标有效。Type-1 温度位在仅查询状态清零。显示/发布明确区分 QuerySource 与运行 Type-1。 |
| `application/motor_control.c` — 启动门控和发送调度 | 仅当 HPM 处于 OP、周期命令租约正常、路由有效、CAN 非 Bus-Off、无轴故障、位置和速度查询足够新鲜且目标合法时接受 `StartMotion`；主站另行核对 WKC complete。启动控制帧先采用**刚查询到的位置**作安全起点；待收到新的 Type-1 后，再允许所选模式/用户目标进入发送队列。运行态保持原 Type-1 20 ms 超时保护。所有 CAN 发送继续受通道预算和排队状态约束。 |
| `application/encos_motor.c` — `encos_encode()` | 允许新 bit3 由上层状态机解释；编码器只对实际待发送的运行命令工作，不能以“bit3 已置位”替代上层安全检查。现有 Type-1 解码不应被查询回复覆盖。 |
| `application/encos_interface.h`、`application/encos_storage.inc` | 推荐只增加符号常量、注释与接口版本初值，不改变 `mc_axis_rx_t`/`mc_axis_tx_t` 字节布局；若这些文件由有效生成脚本维护，应改生成源并重新生成，不手改后再被覆盖。 |
| `application/digital_io.c` | 一般无需改回调映射，只核对 OP→SAFEOP 时 `motor_control_operational(false)` 仍立即撤销启动许可和待发控制帧。 |

### 查询频率与门槛的第一版建议

- 每轴位置 10 Hz、速度 10 Hz、电流 5 Hz；功率可从 1～2 Hz 起步，非控制门槛。六轴在同一 CAN 通道时，前三项约 **150 次查询/秒、300 帧请求+回复/秒**（全部正常一问一答时）；七轴为 175 次查询/秒、350 帧/秒。四通道分别调度，不能每个 EtherCAT 1 ms 周期对全部轴逐项查询。
- 查询状态的在线有效窗口先取约 300 ms，并按实际最慢一轮、总线错误与电机处理时间验证。`Ready` 不可再要求 20 ms 内刚好收到一帧查询回复。HPM 内部启动门槛建议位置 ≤100 ms、速度 ≤150 ms、电流 ≤300 ms；新鲜度不足则拒绝启动并继续查询，不自动放宽到旧 PREOP 位置。
- 原 `FeedbackTimeoutMs=20 ms` 仅用于进入运行态并发出首个控制帧之后的 Type-1 保护，不能直接把它全局提高到 300 ms。查询中单笔事务的 100 ms 超时与字段龄期也要区分。
- 正常停止后可以继续低频查询用于诊断；**停止必须先取消待发控制帧**，不能因为查询成功自动重新启动。若电机在仅查询阶段可被外力带动，开始运动前应重新查当前位置并由 HPM 验证速度/目标跳变，禁止直接使用数秒前的快照。

### 在原 32 B TxPDO 内的最小兼容表达

`0x6200+a:04/:05/:07/:0C` 继续承载位置、速度、电流、功率。查询阶段 `ValidFlags` 的 bit0/1/2 独立按各自时间戳判定；bit3（估算力矩）还需 Kt；bit4/5（温度）置 0；bit6（功率）按既有独立 `PowerAgeMs` 管理。`StatusWord.bit4 NewFeedback` 在运行态仍只表示新的 Type-1；查询阶段不冒充 Type-1。若复用 `FeedbackAgeMs`，将其定义为“当前来源的核心字段中**最老**的龄期”，并用 `QuerySource` 区分；未齐或超时用 `0xFFFF`/清有效位表达。运行态恢复原 Type-1 龄期语义。更精细的每字段龄期要新增对象/字段并走完整映射升级流程。

## 5. Linux 上位机配套改动

| 文件/稳定位置 | 要做的事 |
|---|---|
| `software/igh_rt_encos_ctl_HMI/include/hpm_encos_pdo.hpp` | 定义新控制位、状态位和接口版本；`AxisRx`/`AxisTx` 结构在最小方案下保持 24/32 B，`RunMixedRx/Tx` 保持 784/1072 B。 |
| `software/igh_rt_encos_ctl_HMI/include/igh_controller.hpp`、`src/igh_controller.cpp` — `Command`、`make_axis()`、周期输出 | 命令显式包含布防和 `start_motion` 两个状态。仅布防输出 bit0 且其他运动目标为安全中性值；启动请求输出 bit0+bit3。主站仍保持板级/通道级 OP 周期帧及租约，不要以周期 SDO 代替反馈 PDO。 |
| `software/igh_rt_encos_ctl_HMI/src/mainwindow.cpp` — `enable_motor()` | 不再构造/下发当前位置保持的 `Position` 控制命令；点击后等待 `ArmedQueryOnly`、查询源标志、位置/速度/电流有效及无故障。提示文案应叫“已布防/仅查询”，不要宣称硬件已上力。 |
| `software/igh_rt_encos_ctl_HMI/src/mainwindow.cpp` — `start_motor_motion()`、正弦/回零/模式切换、故障恢复 | 先检查查询位置的新鲜度与安全范围；请求启动后等待 `CommandActive` **和启动后新 Type-1**，再下发用户目标。超时或状态掉回仅查询时撤销启动位、停止该轴；正弦和回零也不得绕过这道门。模式切换按“停止→仅查询→安全启动→目标”处理。 |
| `software/igh_rt_encos_ctl_HMI/src/mainwindow.cpp` — 数值与曲线 | 仅查询时显示分时更新的查询值与来源；温度/ MOS 温度显示无效，不画假实时曲线；运行后切回 Type-1 显示。功率继续用独立有效位和龄期。 |

现有 HMI 把 `Command.enabled=true` 直接编码成 `ControlWord=1`，并在 `enable_motor()` 立即下发位置保持命令；`start_motor_motion()` 又要求 `CommandActive` 和 Type-1。这两个路径必须与 HPM 同步改，不能只替换 Windows 固件后继续用旧 HMI。升级时用接口版本严格拒绝旧程序，避免旧 HMI 的 bit0 被新固件误判，或新 HMI 的 bit3 被旧固件拒绝。

## 6. Windows 替换、SSC/SII 与验证顺序

1. 在 Windows 工程备份当前可工作的固件镜像和板卡 SII/Station Alias/Serial。先确认 `application/`、`SSC/Src/`、`tools/` 与本仓库对应版本一致；只移植本方案涉及的源文件，避免用旧 `digital_io.xlsx` 或 `SSC/Src/digital_io.c` 覆盖业务对象。
2. 先实现 HPM 的“bit0 仅查询、bit0+bit3 才运行”与状态发布；本轮设计稿**没有提供可直接替换的源文件**。随后同步修改 Linux HMI、接口版本和说明。仅最小复用位方案时，校验映射字节数和对象列表完全没变，再决定不刷新 SII。
3. 若提高 `0x1018:03 Revision` 或更改 PDO/SDO 对象，先确认生成脚本可正常读取；按 `readme/电机封装/HPM_ENCOS_SSC与EEPROM升级使用说明.md` 重新生成、验证 ESI/SII，烧录/刷新 EEPROM，并更新 HMI 的 Revision 期望。**不要把 XML 名称改动当作固件对象字典或 SII 已同步的证据。**
4. 空载单轴验证：点击使能后 CAN 分析仪只看到查询请求和类型 5 回复，**没有 Type-1 所对应的控制命令**；HPM `CommandActive=0`、`ArmedQueryOnly=1`，位置/速度/电流有效，温度无效。手动转动电机，确认查询位置会更新。
5. 点击启动：先看到当前位安全起步命令，再看到真实 Type-1、新 `CommandActive=1`，最后才下发用户目标。测试断 EtherCAT、拔 CAN、制造超时/Bus-Off、停止后恢复，均不得自动重播旧目标。再从一轴、两轴逐步增至同通道六轴，记录实际查询频率、响应延迟、丢帧和 CAN 错误计数。

本文的状态、位定义、频率和阈值是**实施建议**，不是对当前固件行为或真实六轴硬件结果的宣称。此前 CAN1 Bus-Off 问题仍需独立检查布线、终端、电源、位时序及 MCAN 错误计数；查询频率降低并不能证明物理层已修复。
