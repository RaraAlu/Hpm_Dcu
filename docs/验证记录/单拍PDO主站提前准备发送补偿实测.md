# 单拍 PDO：提前准备、定相位发送实测

2026-11-19。接续[SM3 等待返回复测](单拍PDO主站SM3等待返回复测.md)，在 CPU6 的独立探针上验证软件能否吸收唤醒抖动。**短测 32/32 通过后，长测 60,000/60,000 拍均在 N+1 截止前取得同序号反馈。SM3 发送调用相对原 +870 µs 目标的最大偏差从 18.807 µs 降到 0.549 µs。**

## 补偿方式

仅探针构建增加 `-DDCU_ONE_CYCLE_SM3_PREPARE_US=50`。RK 按 HPM 的 DC 节拍，在 +820 µs 唤醒，接收并处理 SM2 返回、排好 SM3 的 LRD；排队不发帧。同一线程随后短暂忙等，直到原 +870 µs 才调用 `ecrt_master_send()`。若准备过了发送点但还未到 N+1，则立即发送并记录实际偏差；若在发送前已到本拍截止，则放弃该读、终止测试，不挪动下一拍。读取回显仍从 +975 µs 开始；WKC、序号、OP 和截止判定不变。

CSV 中 `sm3_wait_return_ns` 是 +820 µs 等待返回的用户态时间，`sm3_ready_ns` 是排好 SM3 的时间，`read_ns`、`sm3_send_return_ns` 分别在发送调用前后。没有硬件 MAC 时间戳。启动日志的 `spin_us=0` 属于原有整拍唤醒选项；本实验只开启独立的 `sm3_prepare_us=50`。

## 同负载对比

两列均为 60 秒、1 kHz 的独立长测。应用数据相同（SM2 784 B、SM3 1072 B），每拍的 SM2 阶段均为单帧；IgH 管理数据使部分帧从 844 B 增至 858 B，实际总字节数并非逐拍完全相同。

| 指标 | 上轮：+870 µs 才准备 | 本轮：+820 µs 开始准备 |
| --- | ---: | ---: |
| N+1 按时同序号回显 | 60,000 / 60,000 | 60,000 / 60,000 |
| SM3 发送调用晚于目标：中位数 | 8.955 µs | 0.145 µs |
| 同上：P99.9 / 最大 | 12.345 / 18.807 µs | 0.291 / 0.549 µs |
| 提前调用 / 晚超 1 µs 次数 | 0 / 60,000 | 0 / 0 |
| 反馈观察时刻到冻结截止的最小余量 | 3.057 µs | 2.665 µs |
| 相邻有效反馈观察间隔相对 1 ms 的绝对偏差：P99.9 / 最大 | 7.717 / 14.741 µs | 2.482 / 13.561 µs |
| 上述间隔超出 ±1 µs 的次数（各 59,999 个间隔） | 1,165 | 884 |

最晚醒的命令 37608 在 +846.890 µs 才恢复执行（晚 26.890 µs），但 +851.848 µs 已排好读请求，+870.224 µs 调用发送，+981.934 µs 取得对应反馈。整轮无一拍准备超出 +870 µs；准备完成后的最小余量 18.152 µs，排队完成到发送调用的中位时间 41.126 µs，主要是 CPU6 上的主动等待。

这里验证了**发送调用时刻**的补偿，不是物理网口发帧时刻全程 ±1 µs。发送函数本身中位耗时 1.750 µs、最大 2.917 µs。全部计分拍最终为 OP、WKC 1/1；端到端反馈观察节拍仍有超过 ±1 µs 的间隔，最小截止余量也未比上轮增加。两轮是不同时间段的观测，不是对未来最坏情况的保证。

## 台架与证据

- CPU6、FIFO 80、锁内存，启动前核对线程和设备 IRQ 避开 CPU6，结束后恢复临时亲和设置。两次测试共用同一次 MCU 启动（编号 106），参考时钟均为 HPM。
- 读回核对上一轮 MCU 镜像，未烧录；不接 CAN、不写 Profile、不调整 +255 µs Sync0、399 µs 发布提前量、600 µs 构建保护或 PDO 布局，不切生产 DriverModule，也未专项测 Sync0。最终主站 Idle、从站 PREOP，HPM 暂停；SII 终检一致。IgH 在测试退出、释放主站后有 SAFEOP + ERROR 提示，未计作运行中漏拍。
- [32 拍结果](../../artifacts/target/pdo-sm3-prepare50-20261119-SofY6v/short-summary.json)、[60 秒逐拍 CSV](../../artifacts/target/pdo-sm3-prepare50-20261119-SofY6v/hpm-reference-long.csv)、[60 秒验收与分段](../../artifacts/target/pdo-sm3-prepare50-20261119-SofY6v/long-summary.json)、[两轮统计](../../artifacts/target/pdo-sm3-prepare50-20261119-SofY6v/comparison.json)。

相同发送相位的后续耐久结果见[单拍 PDO 三十分钟耐久实测](单拍PDO三十分钟耐久实测.md)；60 秒通过不代表三十分钟零漏拍。

复核命令：

```bash
d=artifacts/target/pdo-sm3-prepare50-20261119-SofY6v
python3 tests/host/ethercat_data/summarize_probe_clock.py "$d" hpm-reference long --sm3-only
python3 "$d/compare.py" artifacts/target/pdo-sm3-wait-20261119-vgAq4b/hpm-reference-long.csv "$d/hpm-reference-long.csv"
```
