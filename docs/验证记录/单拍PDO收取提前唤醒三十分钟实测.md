# 单拍 PDO：提前 50 µs 唤醒收取，30 分钟实测

2026-11-19。复用[已通过 60 秒短测的独立探针](单拍PDO收取提前唤醒对照.md)，二进制 SHA-256 一致。CPU6、FIFO 80、锁内存、1 kHz，RK 跟随 HPM。SM2 在 0 µs 发、SM3 在 +870 µs 发，收取线程以 +925 µs 为唤醒目标，只等本地时钟，+975 µs 起才允许调用 IgH 接收。原有 600 µs 构建保护不变。一次连续采集，没有重跑、接续或剔除异常拍。

## 逐拍结果

| 指标 | 结果 |
| --- | ---: |
| N+1 按时同序号回显 / 实测 | **1,800,000 / 1,800,000** |
| 正确但迟到 / 无有效回显 / 未执行 | **0 / 0 / 0** |
| WKC 1/1、采到 OP、同帧 HPM 参考有效 | 每项均 1,800,000 拍 |
| 最小 N+1 截止余量 | **+16.954 µs**，命令 91927 |
| 首收相对 +975 µs 的偏差：最小 / P99.9 / 最大 | 0.000 / +0.291 / +0.581 µs |
| 完整回环间隔绝对偏差：P50 / P99.9 / 最大 | **0.165 / 1.000 / 2.793 µs** |
| 完整回环间隔超出 ±1 µs | **1,773 / 1,799,999（0.0985%）** |

最小余量按已冻结的 N+1 截止减去 RK 最终核对完序号、WKC、OP 的时刻计算。最紧的命令在 +975.114 µs 开始收取，此后用 7.875 µs 完成确认。全程每拍首轮收齐；最晚一次比 +925 µs 唤醒目标晚 36.671 µs，没有醒过 +975 µs，也没有提前收取。

完整回环间隔使用**相邻两拍有效回显在 RK 上完成确认的时间差**，不是发送间隔、唤醒间隔或 Sync0 周期。本轮所有相邻间隔都可测，没有跳过无效拍或跨拍拼接。604 个间隔长于 1,001 µs，1,169 个短于 999 µs；恰好 ±1 µs 不算超限。最大偏差是命令 279501 至 279502 之间的 **−2.793 µs**。因此 N+1 通过，但完整回环间隔不是全程 ±1 µs。

## 历史故障窗口

以第一拍 RK 单调时钟为零点，29:35 至 29:50 共 15,000 拍，全部按时，最小截止余量 +17.598 µs。29:40 至 29:41 的 1,000 拍也全部按时；最接近上次 29:40.084 停测点的命令 1780587 得到同序号回显，尚余 +19.367 µs。利用 PRE/POST 两次内核日志时间锚点对齐后，计分段内没有 IgH 同步错误、掉线或退 OP 记录，**上次运行中的故障在这一轮没有复现**。

原始日志确有一条 `0x001A`，出现在**最后一拍已确认、主站释放之后**。停核回读也显示：最后计分命令 1800455 已被 HPM 接受，并在 epoch 1800008 发布；首次 `RX_MISSING` 在下一未下发命令的 epoch 1800009。它是本轮停发边界的保护事件，不是计分中漏拍。启动前进入 OP 的过程也单独记录，未将其日志混入计分段。

## 复核与收尾

- [逐拍汇总及两个历史时间窗口](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/endurance-audit.json) · [原始逐拍 CSV](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/hpm-reference-endurance.csv) · [29:40.084 附近逐拍明细](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/incident-context.csv) · [独立复算计数](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/independent-counts.json)。逐拍序号连续，无重复或补拍。1,800,000 个 1 kHz 参考时钟周期是名义 30 分钟；按 RK 单调时钟计算，计分跨度为 1,799.952 秒。
- [探针日志](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/hpm-reference-endurance.log) · [IgH 原始日志](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/kernel-live.log) · [计分结束后日志](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/kernel-after-scoring.log) · [末拍与 HPM 保护对齐](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/exit-correlation.json) · [校验和](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/capture-sha256.txt)。原始 CSV 和日志已核对 RK 与分析机的 SHA-256；[离线复核脚本](../../artifacts/target/pdo-prewake50-30min-20261119-SupjqY/audit_run.py)不访问台架。
- 没有烧录、初始化启动编号日志、接 CAN、改 MCU / Profile / PDO 或切换生产 DriverModule。镜像和保留区先回读核对，原镜像正常启动；本次 epoch 118，Profile 前后只读结果一致。结束时 IgH Idle、从站 PREOP、HPM 已暂停，SII 与备份一致，临时线程亲和性已恢复。
