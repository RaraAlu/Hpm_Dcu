# 500 Hz 三路 CAN FD（3+5+5）全量闭环 SM2 至 SM3 优化改造方案

本文档基于 `firmware/` 生产可达路径（`hpm-release`）与 `firmware/app/profile/cfg/生产CAN配置.yaml`（Profile rev 11），按 [MCU 固件审查规则](../当前说明/MCU固件审查规则.md) 与 [周期锁存与反馈发布契约](../当前说明/周期锁存与反馈发布契约.md) 编制。

## 1. 目标拓扑与硬实时预算

### 1.1 生产通道与负载分布

当前生产配置启用四路物理 CAN，其中三路 CAN FD（1/5 Mbps，`timestamp_eof: true`，硬件自动重发开启）承载 13 台 MIT 电机，形成 **3 + 5 + 5** 拓扑：

| 物理总线 | 协议模式 | 挂载设备 | 数量 | 关键路径属性 |
| --- | --- | --- | --- | --- |
| Bus 0 (`MCAN0`) | Classic 1 Mbps | `head_1..5` + 旁路手部槽 | 旁路周期槽 | 不参与电机严格因果闭环 |
| Bus 1 (`MCAN1`) | CAN FD 1/5 Mbps | `waist_1..3`（ENCOS，ID 1..3） | 3 台 | 轻载路径 |
| Bus 2 (`MCAN2`) | CAN FD 1/5 Mbps | `left_arm_1..5`（BJCX，ID 1..5） | 5 台 | **关键路径 A** |
| Bus 3 (`MCAN3`) | CAN FD 1/5 Mbps | `right_arm_1..5`（BJCX，ID 1..5） | 5 台 | **关键路径 B（双尾并发撞线）** |

### 1.2 500 Hz 单周期硬截止分析

- **Sync0 周期**：$2000\ \mu\text{s}$（500 Hz）。
- **SM3 严格因果发布截止**：`DCU_CYCLIC_SM3_ADVANCE_US = 400`（对应 RK 主站 $+375\ \mu\text{s}$ 相位契约），即全部 13 台电机必须在 **`Sync0 + `$1600\ \mu\text{s}$` 之前**完成「接收 SM2 $\to$ 编码下发 $\to$ 硬件 TX EOF $\to$ 同路更晚 RX EOF $\to$ 解码写入 `aPdInputData` $\to$ 写完 ESC SM3」。
- **现有实测基线参考**：单轴 500 Hz 实板测得 `publish_max_us` 为 $220\text{--}277\ \mu\text{s}$、`publish_start_delay_max_us` 为 $33\text{--}61\ \mu\text{s}$，且已证实 CAN FD 5 Mbps 存在偶发数据相位 `Bit1Error` 并依赖硬件自动重发恢复（单次重发额外消耗约 $70\text{--}120\ \mu\text{s}$）。
- **结论**：在 3+5+5 下，Bus 2 与 Bus 3 各自承载 5 发 + 5 收 + 4 段 $50\ \mu\text{s}$ 帧间间隔，叠加一次硬件重发和约 $250\ \mu\text{s}$ 的 SM3 刷写后，留给软件唤醒、临界区阻塞与队列抖动的余量极窄，必须清除热路径中的确定性浪费与中断阻塞。

---

## 2. 源码可证问题清单（`hpm-release` 生产可达）

### 问题 1（P0 / MCU-02, MCU-03）：`GatewayApplication::process` 节流门控误拦新周期 SM2 命令

- **位置**：`firmware/app/gateway/gateway_application.cpp:816-823`
- **现状**：

```cpp
uint32_t now_us = dcu_time_now_us();
if (!candidate && cyclic_feedback_missing()) {
    service_cyclic_feedback(dcu_time_now_ms(), now_us);
    return result;
}
if ((uint32_t)(now_us - state_.last_process_us) < policy::process_period_us)
    return result;
```

- **可达路径与后果**：若业务任务在 Sync0 边沿前 $50\ \mu\text{s}$ 内执行过一次无命令唤醒并刷新了 `state_.last_process_us`，当 Sync0 锁存的新 RxPDO 传入（`candidate != nullptr`）时，若距上次执行不足 $50\ \mu\text{s}$，函数直接返回 `consumed = false`。`ethercat_cycle.c:753` 不会建立 `cyclic_` 窗口也不会提交 CAN 批次，任务随即休眠至下一个 $50\ \mu\text{s}$ 周期定时器，造成 **额外 $50\ \mu\text{s}$ 起步延迟**。
- **修正方向**：仅在 `candidate == nullptr` 时应用 `process_period_us` 节流，有新锁存命令时立即受理。

---

### 问题 2（P0 / MCU-01, MCU-02, MCU-05）：等待回包紧自旋引发高频临界区风暴与 MCAN 中断屏蔽

- **位置**：
  - `firmware/app/runtime/ethercat_can_task.c:39-43`
  - `firmware/app/gateway/gateway_application.cpp:777`
  - `firmware/comm/canfd/hpm_mcan_transport.c:190-219`（`read_bus_psr`）、`456-489`（`dcu_can_receive`）
  - `firmware/app/gateway/gateway_publication.cpp:77`
- **可达路径与后果**：
  1. 在 `DCU_CYCLE_WAIT_CAN_FEEDBACK` 期间，`ecat_can_task` 无间隔紧自旋调用 `service_cyclic_feedback()`。
  2. 每次自旋均对三路活跃总线调用 `dcu_can_bus_state(c)` $\to$ `read_bus_psr(c)`。在正常无错态（`!s_first_fault[bus].valid`）下，该函数每次都执行 `taskENTER_CRITICAL()`（屏蔽所有优先级 $\le 3$ 的中断，即 **同时屏蔽四路 MCAN ISR 与 GPTMR2 帧间隔定时器**）并读取读清寄存器 `PSR`。
  3. 紧接着 `dcu_can_receive(c)` 即使在环形队列为空时也会执行 `intc_m_disable_irq(s_irq[c])` / `intc_m_enable_irq(s_irq[c])`。
  4. 在约 $1000\ \mu\text{s}$ 等待窗口内，CPU 高频开关 MCAN 与定时器中断，直接引入 CAN RX/TX_EVT 响应抖动与 Bus 2/3 帧间隔拉长，并在 `prepare_header()` 中再次触发 4 次无谓的 `PSR` 临界区读取。
- **修正方向**：
  1. `s_rx[bus]` 本质为单生产者（MCAN ISR）/ 单消费者（唯一业务任务）环形队列，改用 C11 Acquire/Release 原子读写 `read`/`write`，彻底移除 `dcu_can_receive()` 中的 `intc_m_disable_irq/enable_irq`，空队列零开销快速返回。
  2. 在 `receive_isr()` 处理 `MCAN_EVENT_ERROR` 时维护原子 `s_bus_off_mask`（并在 `dcu_can_bus_state` 同步更新）；热路径 `service_cyclic_feedback()` 仅检查无锁原子 `dcu_can_is_bus_off_fast(c)`，不再在自旋中轮询 `PSR`。

---

### 问题 3（P0 / MCU-02, MCU-03）：`dcu_can_receive` 未校验 `slot->launched` 导致 RX 队头阻塞（Head-of-Line Blocking）

- **位置**：`firmware/comm/canfd/hpm_mcan_transport.c:464-472`、`794-798`；`firmware/app/gateway/gateway_application.cpp:788-806`
- **现状**：
  - `stage_paced_batch()` 一次性把整路 5 个槽的 `slot->feedback = true` 与 `slot->wire_id` 全部写入，但此时仅槽 0 已发射，槽 1..4 尚未发射（`launched == false`）。
  - `dcu_can_receive()` 只要看到队头帧 ID 命中任意 `slot->feedback && slot->wire_id == head.id`，即返回 `false` 拒绝出队。
- **可达路径与后果**：
  - 若总线 RX 队列队头存在槽 4（第 5 台电机）的上一拍迟到帧或残留状态回复，**整条总线的 RX 队列将被队头锁死**，导致槽 0..3 已到达的新回包全部堆积在 `s_rx[bus]` 中无法被及时解码和增量写入 `aPdInputData`，直至第 5 台电机发完并回收 TX 事件后才集中释放，丧失“边收边解码”的流水收益，极端情况下诱发 32 槽软件 RX 队列溢出或 SM3 超期。
  - 另外，`service_cyclic_feedback()` 第 795 行每次仅调用 `collect_tx_events(c, 1)`，若该路同时有 2 个 TX 事件待收割，也会额外延后一次出队匹配。
- **修正方向**：
  1. `dcu_can_receive()` 的前置约束收紧为 `slot->feedback && slot->launched && slot->wire_id == head_id`（仅对**已发射但尚未收割终态**的槽阻塞匹配其回复，未发射槽绝不阻塞队列）。
  2. 在 `submit_cyclic_batches()` 装载新批次前，先排空各参与总线 `s_rx[c]` 中上一拍残留的旧帧（交由常规接收/诊断路径处理或丢弃），避免旧帧落入新周期因果窗口。
  3. `service_cyclic_feedback()` 中在调用 `dcu_can_receive()` 前先收割当前已就绪的 TX 事件（`collect_tx_events(c, CANFD_PACED_SLOTS)`），确保已完成的 TX EOF 立即对 RX 匹配可见。

---

### 问题 4（P1 / MCU-01）：`launch_paced` 屏蔽全局中断，向 Sync0 硬件锁存注入抖动

- **位置**：`firmware/comm/canfd/hpm_mcan_transport.c:529, 555`
- **现状**：`launch_paced()` 使用 `disable_global_irq(CSR_MSTATUS_MIE_MASK)` 保护 `b->next` 检查、`dcu_cyclic_tx_window_current()` 与 `mcan_send_add_request()`。
- **可达路径与后果**：每拍 13 台电机最多执行 13 次 `launch_paced()`（其中 10 次在 MCAN/GPTMR 中断内），每次关闭全局中断都会**连带屏蔽优先级 5 的 `IRQn_ESC_SYNC0` 与优先级 4 的 `IRQn_ESC`**，违背 `cycle_config.h`“仅屏蔽至 `DCU_IRQ_KERNEL_PRIORITY = 3`，保持 Sync0/PDI 不被屏蔽”的快中断隔离契约。
- **修正方向**：
  - 与 `launch_paced()` 共享状态的所有并发者（`cancel_locked`、`dcu_cycle_watchdog_isr`、`dcu_can_submit_batches`、`collect_tx_eofs`、`dcu_can_gap_elapsed_isr`）均处于优先级 $\le 3$（`DCU_IRQ_KERNEL_PRIORITY`）域；`dcu_cyclic_tx_window_current()` 已全量使用原子加载。
  - 将 `launch_paced()` 中的 `disable_global_irq` / `restore_global_irq` 替换为 `portSET_INTERRUPT_MASK_FROM_ISR()` / `portCLEAR_INTERRUPT_MASK_FROM_ISR()`（在任务态调用路径由外层 `taskENTER_CRITICAL()` 已持有内核掩码，内层可直接使用同级阈值或拆分为 `launch_paced_locked()`），**保证 Sync0 与 PDI 全程零屏蔽**。

---

### 问题 5（P1 / MCU-02）：Sync0 缺少即时唤醒机制 + CAN 帧间隔定时器优先级倒挂

- **位置**：
  - `firmware/app/cycle/ethercat_cycle.c:459-476`
  - `firmware/bsp/dcu_hpm5e31/cycle_timers.c:63-73, 83-97`
  - `firmware/cfg/cycle_config.h:21-26`
- **可达路径与后果**：
  1. **Sync0 起步空等 $0\text{--}50\ \mu\text{s}$**：`dcu_cycle_sync0_isr`（优先级 5）不调用 RTOS API（符合 MCU-01），但未触发任何处于内核优先级阈值内的软/硬件唤醒源，导致 `ecat_can_task` 只能被动等待下一个 $50\ \mu\text{s}$ `DCU_WORKER_TIMER_IRQ` 到达。
  2. **帧间隔定时器优先级低于 MCAN**：`DCU_CAN_WAKE_CHANNEL`（驱动三路 $50\ \mu\text{s}$ 帧间间隔）复用 `IRQn_GPTMR2`，其 PLIC 优先级为 `DCU_IRQ_WORKER_PRIORITY = 1`，**低于 `DCU_IRQ_CAN_PRIORITY = 2`**。在 3+5+5 并发收发阶段，任意一路的 MCAN RX/TX_EVT 中断都会抢占并推迟 `dcu_can_gap_elapsed_isr`，直接拉长 Bus 2/3（5 轴关键路径）的帧间间隔。
- **修正方向**：
  1. **Sync0 零延迟级联唤醒（合规 MCU-01）**：在 `dcu_cycle_sync0_isr`（优先级 5）锁存槽位并递增 `handoff.sync0_pending` 后，触发一个处于 `DCU_IRQ_KERNEL_PRIORITY (3)` 的轻量硬件通道（例如独立 GPTMR 单次 $1\ \mu\text{s}$ 比较/重载触发，或片内软件中断/PLIC pending 位），由该优先级 3 ISR 直接调用 `dcu_notify_ecat_can_from_isr()`。将 Sync0 边沿到任务起跑的延迟从 $0\text{--}50\ \mu\text{s}$ 压缩至 $< 3\ \mu\text{s}$，且不破坏快中断零内核调用契约。
  2. **提升 CAN 帧间隔定时器优先级至 `DCU_IRQ_TIMER_PRIORITY (3)`**：将 `DCU_CAN_WAKE_CHANNEL` 移至独立中断源（或独立 GPTMR 实例）并配置优先级为 3（等于 `DCU_IRQ_KERNEL_PRIORITY` 且高于 `DCU_IRQ_CAN_PRIORITY`），避免三路 MCAN RX 突发抢占关键路径的发帧节拍；同时在 `service_gap` 计算 `wait` 时增加 $1\ \mu\text{s}$ 上取整保护或直接以 AHB 时钟换算，减少因 `ATB` 与 `GPTMR` $1\ \mu\text{s}$ 量化差导致的二次重装。

---

### 问题 6（P1 / MCU-02, MCU-04）：3+5+5 下发顺序、失败回滚临界区缺口与 BJCX 预编码/状态查询边界

- **位置**：
  - `firmware/comm/canfd/hpm_mcan_transport.c:802-836`
  - `firmware/app/gateway/gateway_application.cpp:162-171, 724-728`
- **可达路径与后果**：
  1. **轻载先起跑、双重载后起跑**：`dcu_can_submit_batches()` 按 `bus = 0..3` 顺序先向 Bus 1（3 轴）写 Message RAM 并点火，再处理 Bus 2（5 轴）与 Bus 3（5 轴）。虽然差异在十余微秒级，但按 `count` 降序调度符合 Profile 通用性（无需硬编码总线号），可让任意 Profile 下的最重负载总线优先上线。
  2. **失败回滚临界区缺口**：`dcu_can_submit_batches()` 第 828 行退出临界区后，第 829–835 行才再次进临界区清空 `launch_paced()` 失败的 `s_tx[bus]` 与 `s_chain[bus]`；两段临界区之间若触发看门狗或 CAN 中断，会观测到半撤销的 `s_chain[bus]`。
  3. **在途状态轮询阻断整路批次**：`生产CAN配置.yaml` 为 10 台 BJCX 配置了 `status_poll_period_ms: 20`。当主站刚从待机切入非零 MIT 控制首拍时，若某台 BJCX 仍有未结束的状态轮询，第 725 行 `transactions_.busy(c, a)` 会直接把该总线全部 5 轴置 `count = 0` 跳过提交，诱发首拍 `PDO_PIPELINE_CAN_FEEDBACK_MISSING`。
  4. **预编码未覆盖 10 台 BJCX**：`preview_control()` 第 165 行限定 `supports_legacy_queries()`，仅对 Bus 1 的 3 台 ENCOS 生效；未来 BJCX 从全零 MIT 扩展到完整浮点/定点 MIT 控制时，10 台关键路径轴的编码耗时将全部落在 Sync0 之后。
- **修正方向**：
  1. **动态按负载降序点火（保持 Profile 通用原则）**：`dcu_can_submit_batches()` 按各通道 `batches[bus].count` 从大到小排序（3+5+5 自动变为 Bus 2/3 优先，Bus 1 殿后），先写入并启动重载总线槽 0，并在**同一临界区内**直接完成失败总线的状态回滚。
  2. **切入严格周期 MIT 时显式清理未完成状态轮询**：当 `accept_candidate()` 判定本拍存在 `cyclic_.required` 且对应通道上存在内部自动发起的状态轮询（非外部维护服务）时，在提交批次前先显式撤销未发射的轮询槽或按 `poll_can_alias_control_reply` 明确隔离，避免整路 5 轴陪绑拒发。
  3. **开放 BJCX 预览预编码扩展口**：取消 `preview_control()` 中 `supports_legacy_queries()` 的一刀切限制，改为由各驱动显式声明 `supports_preview()`（ENCOS 保持现状，BJCX 仅在确认滚动计数 `counter_` 一致且不跳过限值检查的前提下启用预编码，严格遵守周期契约）。

---

### 问题 7（P2 / MCU-05）：全量缺反馈诊断粒度不足，难以在 3+5+5 下秒级定位故障阶段

- **位置**：`firmware/app/cycle/ethercat_cycle.h:11-27`、`firmware/app/cycle/ethercat_cycle.c:499-500`
- **现状**：触发 `PDO_PIPELINE_CAN_FEEDBACK_MISSING` 时，`cycle_diagnostics` 仅记录 `missing_can_devices`（未匹配的设备掩码）。
- **修正方向**：在 `dcu_cycle_diagnostics_t` 中新增 4 个只读 32 位掩码字段（总增 16 B RAM，不改 PDO 线格式）：
  - `cyclic_required_mask`（本拍要求因果回包的 13 轴掩码）
  - `cyclic_submitted_mask`（本拍成功提交至 CAN 驱动的掩码）
  - `cyclic_tx_eof_mask`（本拍已收到有效硬件 TX EOF 的掩码）
  - `cyclic_matched_mask`（本拍在截止前收到更晚有效 RX EOF 并解码成功的掩码）

配合现有 `s_first_fault[bus]`，在 3+5+5 台架一旦触发缺反馈，停核即可一键区分：**未提交（调度/轮询冲突） / TX 失败（总线或重发超期） / RX 丢失（电机未回或线束异常） / 整路总线故障（如 Bus 2 对应 5 位全缺）**。

---

## 3. 分阶段实施顺序建议

为控制回归风险并符合项目单变量验证规范，建议分 3 个独立提交实施，每次提交均先在 Ubuntu 执行 `python3 scripts/dcu.py build --preset hpm-release` 验证构建与镜像检查：

1. **Batch A（核心去阻塞与正确性修复，零硬件架构改动）**
   - 修复 `gateway_application.cpp:821` 的 `candidate != nullptr` 节流误伤。
   - 修复 `hpm_mcan_transport.c:464-472` 的 `slot->launched` 队头阻塞判断，并在提交批次前排空残留旧 RX、`service_cyclic_feedback()` 先收割就绪 TX 事件。
   - 合并 `dcu_can_submit_batches()` 失败回滚至同一临界区，并按 `count` 降序启动重载总线。
   - 将 `launch_paced()` 改为 `KERNEL_PRIORITY` 掩码，解除对 Sync0/PDI 的全局中断屏蔽。
2. **Batch B（去临界区风暴与低延迟唤醒）**
   - `dcu_can_receive()` 改 SPSC 无锁原子出队；`service_cyclic_feedback()` 与 `prepare_header()` 改用 ISR 同步的无锁总线状态快照，消除热路径 `PSR` 临界区风暴。
   - 为 `dcu_cycle_sync0_isr` 增加优先级 3 级联即时唤醒（压缩起步延迟 $0\text{--}50\ \mu\text{s} \to < 3\ \mu\text{s}$）。
   - 将 `DCU_CAN_WAKE_CHANNEL` 移至优先级 3 中断源，解除与 `IRQn_MCAN`（优先级 2）的优先级倒挂。
3. **Batch C（可观测性增强与 BJCX 扩展准备）**
   - 补齐 `dcu_cycle_diagnostics_t` 的四阶段因果掩码。
   - 清理切入周期 MIT 时的内部状态轮询边界，按驱动能力开放 BJCX `supports_preview()`。

---

## 4. 待台架实测与验收清单（源码不可替代）

以下指标必须按 `AGENTS.md` 在动力隔离条件下完成实板测量，未实测前不宣称满足最坏时延：

| 验收项 | 预设与拓扑 | 必测观测点 | 通过判据 |
| --- | --- | --- | --- |
| **1. 单拍全零 MIT 验收** | `hpm-release`，Profile rev 11（3+5+5 共 13 轴），500 Hz DC Sync0，$+375\ \mu\text{s}$ | 第 $N$ 帧发命令，第 $N+1$ 帧检查全部 13 轴反馈与 `TX 完成计数` 增量 | 13/13 轴均在 $N+1$ 返回本拍新反馈且每轴 TX 完成计数 +1；无旧反馈替代 |
| **2. Sync0 至首帧起跑延迟** | 开启 `CONFIG_CAN_BATCH_SAMPLE` 对照构建 | `sync0_us` 至 Bus 2/3 `batch.submit_us` 差值（Min / Max） | 最大起步延迟较改前（最大 $> 50\ \mu\text{s}$）显著收窄且无 $50\ \mu\text{s}$ 跳阶 |
| **3. 双 5 轴关键路径闭环耗时与余量** | 3+5+5 全负载 60 秒连续运行 | `cyclic_latency.max_ticks`、13 轴最晚 `matched_us - sync0_us`、`publish_max_us`、距 $1600\ \mu\text{s}$ 截止最小实测余量 | `matched_us + publish` 稳定早于 `Sync0 + `$1600\ \mu\text{s}$；需留出至少 1 次硬件重发（$\ge 120\ \mu\text{s}$）余量 |
| **4. 中断与帧间隔确定性** | 3+5+5 并发满载 | `s_feedback_stats[1..3].min_gap_ticks`、`s_feedback_gap_by_slot`、`cycle_timing.sync_interval` | 帧间隔稳定在 $50\ \mu\text{s}$ 附近无大幅毛刺；Sync0 边沿锁存抖动不劣化 |
| **5. 故障注入与精准定位** | 拔断 Bus 2 单台电机线束 / 断掉 Bus 2 整路终端 | `cycle_diagnostics` 四阶段掩码 + `s_first_fault[2]` + AL 状态切换 | 单拍在 $1600\ \mu\text{s}$ 截止准确触发 `PDO_PIPELINE_CAN_FEEDBACK_MISSING`，安全退 `SAFEOP`，四阶段掩码准确指向缺失轴与阶段，不复用旧反馈发布 SM3 |
| **6. 栈水位与资源余量** | `hpm-release` 60 秒运行后 JTAG 读回 | `rtos_diagnostics.worker_stack_free_words`、`.fast_ram` 与 `.ahb_sram` 链接 map 占用 | 无溢出，栈剩余水位满足项目基线要求 |

## 5. 未覆盖范围声明

- 本轮仅完成对 `Hpm_Dcu` 固件源码的静态审查与调用链分析，未在宿主机执行交叉编译（`python3 scripts/dcu.py build --preset hpm-release`），未执行生成链校验，亦未连接目标板测量最坏执行时间、CAN 物理波形与硬件重发频率。
- `third_party/beckhoff_ssc` 与 `third_party/hpm_sdk` 底层寄存器实现默认以现有实板通过版本为基线，未展开逐行审计。