# 单拍 PDO 帧到达与中断窗口实测

沿用[槽提交优化复测](单拍PDO槽提交优化复测.md)的 MCU 二进制和 1 kHz 时序：CPU6、FIFO 80、锁内存，Sync0 +255 µs、SM3 +870 µs、发布提前 399 µs、构建预算 600 µs。只运行独立探针，普通 PDO 零使能，不接 CAN，不写 Profile，不切生产 DriverModule。

## 测量口径

- HPM 是唯一从站和 DC 参考时钟，IgH 报告传输补偿为 0 ns。原有 SM2 发送已携带 FRMW 读取参考时钟；探针在 SM3 已发、下一拍还未排队时调用 `ecrt_master_reference_clock_time()`，取回的是 ESC 在**帧首**锁存的低 32 位 DC 时间，不是 RK 收到返回帧的时间，也不是 SM2 写完的时间。[Beckhoff ESC 寄存器说明 §2.15.2.1](https://download.beckhoff.com/download/document/io/ethercat-development-products/ethercat_esc_datasheet_sec2_registers_v3.3.pdf)；[IgH 应用接口](https://docs.etherlab.org/ethercat/1.6/doxygen/group__ApplicationInterface.html)。
- 单独的 32 拍预试发现板上 `ecrt_master_send()` 的返回字节数始终为零，不能证明同帧。正式探针在上拍 SM3 发送后和本拍 SM2 发送后各取 IgH 主口驱动统计快照；计数差为一、SM2 WKC 为一、当拍 FRMW 返回有效才归因。不改收发顺序，不加报文，不使用私有的 IgH 写操作。只读计数结构按本机 CLI 的 DWARF 和运行内核核对；板上 ABI magic 39，结构 248 字节，主口两个 64 位计数的偏移为 32、48。更换内核或 IgH 版本须重新核对。
- 试后只读 ESC `0x0990`、`0x09a0`，逐拍核对实际硬件 Sync0 时间格点。帧首至 Sync0 的间隔在**同一 ESC 的 DC 时钟**上计算，采用有符号模差处理 32 位回绕。MCU 的 PDI 入口至 Sync0 软件锁存另在片内微秒时钟计算，不直接用 Linux 时间戳减 MCU 时间戳。
- 本板 `0x0f82` 读回 `0xd0`，未置 DCSMET 位；`0x09f0/0x09f8/0x09fc` 预检均为零，没有用它们冒充 SM2 中断边沿。两段提前量之差包含帧串行传输、ESC/PDI 响应以及 Sync0 软件打点相对于硬件边沿的偏移，不等同于纯 IRQ 延迟。

## 结果

正式短跑 32/32 通过。60 秒目标第 320 拍停止，前 319 拍全部按时同序号。两轮每拍在 SM2 写阶段均只发一个 844 字节的以太网帧，SM3 仍在单独的读阶段发送；长跑 320 拍全部取得同帧硬件 SOF。长跑首、末各 32 拍的 SOF 至硬件 Sync0 中位间隔从 217.699 降至 198.261 µs，减少约 19.4 µs；同期主站调用发送的中位时刻分别为 +3.364、+3.445 µs（相对计划拍起点）。

最后 127 条同时有完整 MCU 发布轨迹的成功拍，按时间分组，以下均为中位数：

| 序号 | 帧首至硬件 Sync0（µs） | PDI 入口至软件锁存（µs） | ESC 读 784 字节（µs） |
| --- | ---: | ---: | ---: |
| 4969 至 5000 | 204.686 | 136 | 113 |
| 5001 至 5032 | 202.406 | 135 | 113 |
| 5033 至 5064 | 200.580 | 132 | 113 |
| 5065 至 5095 | 198.274 | 130 | 113 |

| 失败边界 | 帧首至硬件 Sync0（µs） | PDI 入口至软件锁存（µs） | 提交路径结束相对软件锁存（µs） |
| --- | ---: | ---: | ---: |
| 5095，回显 5095 | 197.109 | 130 | -4 |
| 5096，回显 5095 | 196.855 | 126 | +5 |

主站在 +981.957 µs 读回旧回显，WKC 1/1，读回时仍为 OP。首个 MCU 故障为 `RX_MISSING`，对应 epoch 328，槽正在提交，无 `TX_LATE`，随后 IgH 记录 0x001A。两拍均为 844 字节，ESC 读取均为 113 µs。

**结论：** 持续收窄的窗口主要在帧到达 ESC 时就已存在，不是 MCU 的 ESC 读取逐渐变慢。单看失败的最后两拍，硬件窗口只少了 0.254 µs，软件窗口少了 4 µs；另外约 3.75 µs 是帧首之后相对软件打点的变化，不能全算到 PDI IRQ 上。本次没有主站 MAC 实际发帧时间，也没有硬件 PDI 中断断言时间，尚不能把长期变化拆成 RK 网卡发帧变化和 DC 参考时钟相位变化。应先核对主站与 DC 的相位跟踪，不据此调整现有保护。

## 复核

证据目录：`artifacts/target/pdo-sof-arrival-20261119/`。正式主站原始数据为 `counter-short.csv`、`counter-long.csv`；`long-rx.bin`、`long-phase.bin`、`long-diag.bin` 为未复位的 MCU 故障现场。核对命令：

```bash
python3 tests/host/ethercat_data/summarize_sm2_arrival.py artifacts/target/pdo-sof-arrival-20261119
```

独立探针通过 `DCU_ONE_CYCLE_SOF_TRACE` 启用本次诊断；本板编译参数为 `DCU_IGH_IOCTL_MAGIC=39`、`DCU_IGH_MASTER_INFO_SIZE=248`、`DCU_IGH_TX_COUNT_OFFSET=32`、`DCU_IGH_TX_BYTES_OFFSET=48`。参数来源在 `counter-abi.txt`、`counter-check.txt`；默认探针不打开统计设备。

此前暂停的固件直接恢复时 CoE 预检未通过，该次没有开始 PDO 测试。复位并核对原诊断镜像、SII 和启动日志后恢复；本轮未烧录，SII 回读相符，原启动编号保留并正常追加至 94，正式两轮 Profile 读回一致。最终主站 Idle、从站退回 PREOP，MCU 暂停保留现场，CPU 亲和设置已恢复。
