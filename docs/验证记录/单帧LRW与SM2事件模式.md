# 单帧 LRW 与 SM2 事件模式

当前验收基线选用 **DC Sync0 + 1 kHz 单帧 LRW**。RK 的 DriverModule 显式设 `dcu.sync: sync0`；下述 `sm2` 仅是已离线验证的备选模式，本轮不启用，不能作为 DC 实板稳定的证据。

RK 的 IgH 继续用一个 1072 B 的 LRW：下行 SM2 映射 784 B，上行 SM3 映射 1072 B，两者在主站域内重叠，HPM ESC 内的 SM2、SM3 物理缓冲仍独立。WKC 3 只确认该帧在 ESC 上完成读写，不表示 CAN 回包或软件已执行命令。

| 时点 | HPM 的行为 |
| --- | --- |
| 帧 N 经过 ESC | 写入 SM2 的新命令，读出 SM3 已发布的快照；不等本帧命令处理 |
| SM2 事件 | PDI 读完完整命令后提交槽；网关任务下一次运行取槽，不等 Sync0 |
| 帧间发布窗口 | 网关任务处理命令及 CAN 收发，使用已有反馈构建一份完整 SM3；PDI 不重复写同一缓冲，不把未更新的 CAN 数据标成新结果 |
| 帧 N+1 | 继续同帧写 SM2、读 SM3；不要求反馈等于本帧或上一帧命令 |

## 同步选择与保护

- SM2 路径对应 SSC 选择的 0x1C32:1 = 1（输出使用 SM 事件）、0x1C33:1 = 0x22（输入使用 SM2 周期），并要求 ESC 未开启 Sync0/1 脉冲。主站不需要逐项写配置 SDO。ESI 声明可选 SM2 模式；SII 保留原默认模式且字节不变。
- SM2 模式下以 PDI 开始读 SM2 的时刻为发布时间基准，**不是以读完或 CAN 回包时间重新开始计时**。600 µs 构建预算和现有发布提前量保留。没有新的 SM2 事件时，由现有独立周期看门狗按 1 ms 加一轮工人任务定时容差（50 µs）关发送门，不重放旧命令。
- 不改 Profile、PDO 字节、CAN 配置、BSP 或启动日志。原 DC Sync0 模式仍可明确选择，不在同一会话双跑。新的模式没有“距 Sync0 的余量”指标，应改看 SM2 事件到提交/发布的耗时、帧计数和 WKC。

以下是 SM2 对照配置，不用于当前 DC Sync0 验收。对照模式仍允许 RK 以 HPM ESC 为参考时钟，但不开启 Sync0 脉冲。

```yaml
schema_version: 5
connect_device: true
enable_actuator: false
update_frequency: 1000
publish_frequency: 100
actuator_list: []
topics: {}
dcu:
  profile: firmware_idle_lrw
  sync: sm2
```

## 验证边界

离线覆盖 SM2 收齐前不可消费、无 Sync0 能接收、反馈可在无应答时发布、截止超时不写迟到的 SM3，以及丢帧时无需业务任务即可关门。固件与 RK 构建通过不等于实板已进入 OP。首次实板只启用 DriverModule，不接 CAN，先跑 32 拍核对单帧与 WKC 3，再跑 60 秒；这是后续门槛，本次没有烧录或运行板卡。
