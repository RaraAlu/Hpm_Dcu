# 独立 CAN 内部回环测试

日期：2026-11-17。**入口、离线验证及 can0 首轮实板内部回环已完成，12 帧通过；目标已暂停，板上保留独立测试镜像。** 证据见[首轮实板记录](../验证记录/2026-11-17-MCAN0内部回环首轮实板测试记录.md)。本文属于[项目总计划](../设计资料/项目总计划.md)中的控制器内部测试，不是 EtherCAT 到 CAN 的完整往返验收。

## 范围与隔离

无需 can0 接 can1。路径为 `程序 -> can0 发送单元 -> 控制器内部回接 -> RX FIFO0 -> 中断 -> 专用接收队列 -> 程序比较`。不是主程序自行复制发送数据充当接收结果。

使用独立 [main](../../tests/target/can_internal_loopback/main.c) 和[测试实现](../../tests/target/can_internal_loopback/can_internal_loopback.c)，不链接生产应用、CAN transport、motor、SSC、FreeRTOS 或 EEPROM 仿真组件。只复用 SDK/BSP、板级时钟和现有单调时基，不调用 EtherCAT/SII 初始化、CAN 引脚复用或收发器使能函数。不初始化其余三路 CAN；不能据此声明其硬件正常。

`hpm-debug`、`hpm-release` 默认关闭该入口；`hpm-can-loopback` 输出到独立目录，不替换生产构建产物。测试与旧 LED 测试互斥。`scripts/dcu.py` 接受该测试预设，但访问目标仍经过原有确认、镜像冻结、保留区备份和校验门槛。

**这验证的是专用测试路径，不是生产 `dcu_can_send()`、RTOS 调度、Profile 状态机、PDO/SDO 或收发器/端接/线缆。** 当前 YAML 不直接加载到这份固件；测试载荷与其中三组 CAN 字节基准由主机测试独立核对。内部回接使用发送时的同一 ID，不把 `0x600` 改写成对端回复 ID `0x601`。

## 固定首测参数

参数只维护在[测试配置头](../../tests/target/can_internal_loopback/cfg/can_loopback_config.h)，宏名携带单位，数值不带单位。

| 项目 | 当前设置 |
| --- | --- |
| 控制器 | can0，即软件 bus 0；不使用外部接线 |
| 模式 | SDK `mcan_mode_loopback_internal`，经典 CAN，禁止 FD/BRS/RTR/扩展帧 |
| 仲裁速率 / kbps | 1000 |
| CAN ID | `0x600`，收发一致 |
| 数据长度 / 字节 | 8 |
| 测试数据 | 非对称字节、全零、全一，依次循环；来源见[字节向量](../../tools/protocol/cfg/最小回环测试向量.yaml) |
| 总帧数 | 12，单在途请求；覆盖硬件 FIFO 槽与软件环形队列索引回绕 |
| 单帧等待 / ms | 5，必须同时收到 TX 完成事件和匹配 RX 数据 |
| 末尾静默检查 / ms | 5，不允许额外接收或错误事件 |
| FIFO / 软件队列 | 硬件 RX/TX 各 4 槽；软件队列 4 槽、有效容量 3 |

发送前检查 `CCCR.TEST/MON`、`TEST.LBCK`，并确认已离开 INIT 且 FD/BRS 关闭。中断只作有限次 FIFO 搬运和统计，主程序不轮询读取 FIFO 伪装为中断成功。等待同时受硬件时间和有限轮询次数限制；时钟停止也不会无限等下去。

出现初始化错误、模式错误、发送失败、帧不匹配、缺少 TX/RX 中断、超时、队列溢出或控制器错误即失败。收尾屏蔽 can0 中断、取消发送请求并请求 INIT，保留内部回环模式；只有确认停止并复核最终计数后才允许报告 PASS。无论成败，最后进入 `dcu_can_loopback_finished()` 停留，不回到生产业务，也不自动反复测试。

## 离线入口

在工程根目录执行，均不访问板卡：

```bash
python3 -m unittest discover -s tests/host -p test_can_internal_loopback.py -v
python3 scripts/dcu.py build --preset hpm-can-loopback
python3 scripts/dcu.py check-image --preset hpm-can-loopback
```

产物为 `build/hpm-can-loopback/output/dcu.elf`；保留区检查报告位于 `build/hpm-can-loopback/image-check.json`。这两个路径是可变构建产物；实板证据绑定下载事务另存的冻结镜像，不自动适用于后续重建。

## 实板操作流程

首轮已按以下流程执行。以后再次运行仍需重新确认现场和操作范围，不复用历史授权：

1. 重新确认执行器动力及所有业务节点隔离、逻辑供电与 J-Link 接线正常；允许本轮复位及下载测试镜像。无需互连 can0/can1，也不要为了这一步增加 CAN 接线。
2. 审核当前测试 ELF 和保留区检查报告，使用受保护的 `scripts/dcu.py debug-server --preset hpm-can-loopback --skip-build --flash-first` 流程；目标确认按现场明确授权提供，不复用历史确认环境变量。不使用默认 Debug 的 F5 下载入口，以免误选生产镜像。
3. 加载本轮 `artifacts/target/` 下冻结 ELF 的符号，等待 `DCU_DEBUG_READY` 后连接，先在 `main` 检查点确认镜像和停点；不要进行 GDB 目标函数调用。
4. 移除初始化断点，仅在 `dcu_can_loopback_finished` 设置一个硬件断点，再连续运行。不要在每帧等待区单步或暂停，否则时间测量和超时结果失真。
5. 到达结束点后读取私有 `s_report` 并保存日志；核对结果、逐帧内容、计数、错误标记及控制器停止状态。失败先保存现场，不自动修改参数放宽验收。
6. 保持目标暂停并关闭调试服务；测试镜像不会自动恢复为生产镜像。后续恢复生产镜像须另行确认，不能把旧固件直接继续运行当作测试收尾。

### 通过条件

| 检查项 | 必须满足 |
| --- | --- |
| 最终结果 | `s_report.result == DCU_LOOPBACK_PASS`，且 `controller_stopped == true` |
| 完整性 | `frames_checked`、`tx_completed`、`rx_received` 均为 12 |
| 中断 | `irq_count > 0`；RX 和 TX 可能合并为一次 IRQ，不要求 IRQ 数等于帧数 |
| 错误与丢弃 | `error_flags == 0`、`queue_drops == 0` |
| 逐帧结果 | 每项 `matched == true`，ID、DLC、类型和 8 字节内容全部一致 |
| 时间 | 每帧 `elapsed_us` 小于配置截止；仅为软件提交到 ISR 接收交接的间隔，不是物理线速、WCET 或实时抖动验收 |

未完成的首个帧结果可能已留在 `frames[frames_checked]`，用于排查错误 ID/DLC/载荷；不能只看串口最后一个 PASS 字符串。主机故障注入模拟 SDK 寄存器/中断，不证明上述实板条件已经成立。

## 离线验证

以下为入口准备阶段记录；首轮上板前又复跑了 20 项针对性测试、测试镜像构建及保留区检查，全部通过。

- 新增 20 项主机测试通过，涵盖正常往返、TX/RX 中断合并或先后到达、索引/时钟回绕、格式错误、模式失效、初始化/发送失败、缺失中断、超时/停钟、重复/额外帧、软件队列溢出、硬件错误和停止失败；同时核对目标操作确认门槛和独立字节基准。
- 独立测试镜像构建及 Flash 保留区检查通过；生产 Debug/Release 构建回归通过。编译源清单和 ELF 符号检查确认两条入口互相隔离，can0 使用实际 SDK 中断包装入口。
- 使用现有 `/tmp/dcu-rtos-audit-deps` 中的固定 Unicorn 2.1.4 复跑完整 host 测试：276 项通过，无跳过，包含头文件归属、既有生产回归和 RV32 内核/时基执行测试。未安装或升级依赖；不带该 `PYTHONPATH` 时五个 RV32 用例会跳过。
- 37 个 C 测试场景在 ASan/UBSan 下通过；这些是主机替身测试，不是硬件中断或真实 CAN 收发的实测记录。

## 首轮实板结果

2026-11-17，can0 的 12 帧 TX/RX、12 次 IRQ、逐帧字节比对全部通过，错误和丢弃为 0；报告显示停止成功，硬件 INIT 已置位，待发送和 RX FIFO 均为空。软件提交至 ISR 交接间隔为 128 至 158 us，只是本轮观察，不是实时性验收。

64 KiB 保留区运行前后逐字节不变。已保存冻结 ELF、原始报告、YAML 复核结果及调试日志；清除断点并保持目标暂停，关闭调试服务，没有恢复生产镜像。完整证据与未覆盖范围见[首轮实板记录](../验证记录/2026-11-17-MCAN0内部回环首轮实板测试记录.md)。仍未证明物理 CAN、生产 transport 或 EtherCAT/CAN 完整回环。
