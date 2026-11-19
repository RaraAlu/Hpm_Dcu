# DriverModule 单帧 LRW 重叠映射实测

2026-11-19，RK3588 / HPM。目标是每 1 ms 一帧：本帧写 SM2，同时读出帧到达前 SM3 已发布的数据。**32 拍通过，60 秒未通过。** 此次只验 EtherCAT 通信，不验 CAN 回环，也不要求回显本拍命令。

| 计分窗口 | 请求拍数 | 实际返回的单帧 LRW | 返回 WKC = 3 | 结果 |
| --- | ---: | ---: | ---: | --- |
| 短测 | 32 | 32 | 32 | 通过 |
| 60 秒 | 60,000 | 361 | 360 | 第 361 拍返回 WKC = 1，停止，不补发 |

## 实际链路

- 只加载 RK 构建目录的 `libpkg1.so` 中的 `DriverModule`，走 `StartMotor / MotorTick / DcuDriver`。未启用 13 轴预设、诊断应用或独立探针；未安装服务。CPU6 独占、FIFO 80、锁内存，RK 跟随 HPM，1 kHz。SM2 和 SM3 不再分开发两帧。
- 只读确认 MCU READY、无已配置电机、RUN_MIXED、当前策略和租期。未配置下载、未改 Profile、PDO、MCU 镜像或 600 µs 保护。每拍从全零输出重建 784 B，仅赋周期序号、租期及接口版本。不能将收到的 1072 B 直接作为下一拍输出。
- 两段 FMMU 均从逻辑地址 `0x00000000` 起：SM2 Output 长 784 B，SM3 Input 长 1072 B。域仅 1072 B，IgH 生成一个 LRW，预期 WKC 为 3（单从站：读贡献 1，写贡献 2）。实际以太网帧为 1132～1146 B，不含 FCS；同帧还包含 DC 或 IgH 管理数据报。
- 逐拍独立解码原生网卡驱动的发送/接收帧记录，配对 LRW 索引，核对逻辑地址、1072 B 长度、发出 WKC 0、返回 WKC；再与物理帧/字节计数、域 WKC、OP 采样交叉核对。IgH 调试输出省略了帧中间部分，数据报头和 WKC 均可读，没有插值，也不宣称这是完整载荷的 PCAP。

## 未通过的那一拍

- 命令序号 1473 对应的帧在总线上返回 WKC 3。HPM 记录 SM2 事件到 Sync0 中断有 126 µs，其中 ESC 读取占 113 µs。ESC 读取结束距 Sync0 仅 4 µs；Sync0 到时槽仍在提交，记录的完成时间晚 **5 µs**。HPM 锁存 `RX_MISSING`，不是 `TX_LATE`。
- 下一帧（计分第 361 拍，序号 1474）仍是一个 1132 B 物理帧，正确返回 RK；LRW 索引 `0xC3`，**WKC 为 1，不是 3**。相邻两帧 FRMW 参考时刻相隔 1000.160 µs。此时主站缓存的 AL 状态仍为 OP，释放附近 IgH 日志随后显示 SAFEOP + ERROR / `0x001A Synchronization error`。不能把 WKC 缺失误报为丢失以太网帧或没有 CAN 应答。
- 以上定位到片内提交与 Sync0 的先后；逐帧内核十六进制日志本身会增加开销，本次不单独归因于记录器、网卡或时钟。60 秒组在进入计分前的预热也记录过一次同步错误，不计入表中；计分开始后没有跳过坏拍、补发或重跑。同步相位和截止规则未改变。

## 证据与收尾

[32 拍逐帧结果](../../artifacts/target/pdo-lrw-cpu6-20261119-Gmntnrq0/32/wire.csv) · [60 秒尝试全部 361 拍](../../artifacts/target/pdo-lrw-cpu6-20261119-Gmntnrq0/60000/wire.csv) · [HPM 故障时间线](../../artifacts/target/pdo-lrw-cpu6-20261119-Gmntnrq0/fault.json) · [原生帧日志](../../artifacts/target/pdo-lrw-cpu6-20261119-Gmntnrq0/60000/native.log.gz) · [复核脚本](../../artifacts/target/pdo-lrw-cpu6-20261119-Gmntnrq0/audit_frames.py) · [实际运行二进制哈希](../../artifacts/target/pdo-lrw-cpu6-20261119-Gmntnrq0/rk-binaries.sha256)。

首次准备时，RT 检查只读 sysfs 和 /boot 文件，未识别 RK 内核，不产生计分帧。核对在运行的内核 UTS 标签及 `/proc/config.gz` 的 `CONFIG_PREEMPT_RT=y` 后修正检测，未关闭 RT 检查。LRW 专项及驱动执行离线测试通过；全量驱动单测中有两项旧 13 轴电流限值期望与该预设不一致（100/70 对 50/35），未修改那些限值，也未在硬件启用。实板结束主站 Idle、从站 PREOP、CPU 亲和性已还原，HPM 已暂停；镜像和 SII 回读一致。启动日志只核对、留档，未擦除或初始化。
