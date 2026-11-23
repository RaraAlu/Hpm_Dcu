# 独立 EtherCAT 到 MCAN 内部回环测试

状态：已完成[首轮实板验收](../验证记录/2026-11-18-EtherCAT到MCAN内部回环首轮实板测试记录.md)，板上保留诊断镜像并已暂停。诊断产品码为 `0x7fff0003`，区别于软件回显镜像 `0x7fff0002`。两者保持同一诊断 PDO 布局，但主站不允许互相替代。

## 数据路径

| 阶段 | 处理与验收 |
| --- | --- |
| PREOP | 只读核对身份、SDO 长度、固定字节和 20 个 PDO 条目的映射；不发 SDO 下载。 |
| RxPDO | 主站写 SM2 的 32 字节。MCU 固定一份完整样本，忙时不覆盖在途数据。 |
| can0 | 仅开控制器时钟和内部回环，不配置 CAN 引脚或收发器。经典标准帧，速率字段 `nominal_bitrate_kbps: 1000`，四帧依次使用 `0x600..0x603`，每帧 8 字节。一次只提交一帧，轮询 TX 完成和 RX FIFO，不阻塞 SSC。 |
| TxPDO | 逐帧核对 ID、帧型、DLC 和字节。收齐四帧且每帧 TX 都完成才从 RX FIFO 的数据重组回显，更新 SM3 的 48 字节及新样本计数。没有从输入内存直接回显的退路。 |
| 退出与故障 | 离开 OP 时丢弃在途样本、停止新发送，不回传旧结果。单帧等待上限 `frame_timeout_ms: 5`。模式、控制器或帧校验出错时使回显无效、禁止后续发送，需要重启诊断镜像。 |

这是控制器内部收发诊断。ID 分片只在内部回环使用，不是最小物理回显 Profile 的请求／回复协议；不验证 CAN 收发器、端接、线缆、外部设备，也不验证一次确认启动或实时性能。

## 离线检查

```bash
python3 scripts/dcu.py build --preset hpm-ethercat-mcan
python3 scripts/dcu.py check-image --preset hpm-ethercat-mcan
```

产物：`build/hpm-ethercat-mcan/output/dcu.elf`。该预设与软件回显、独立扫描及生产镜像分开，不启用生产 CAN transport、电机或 RTOS。

## 实板验收顺序

1. 另行确认电机动力隔离、CAN 不接业务节点、EtherCAT 专用口仅连 HPM 和 J-Link 连接正常。先备份保留区，再受保护下载。启动后避免用断点暂停 SSC。
2. RK3588 先用 `build/cfg/driver_ethercat_mcan_sdo.yaml`，只启用 `DriverModule`，在 PREOP 核对产品码 `0x7fff0003`、对象长度和固定映射。身份或映射不符则不进 OP。
3. 再用 `build/cfg/driver_ethercat_mcan_pdo.yaml`，同样只启用 `DriverModule`。复核 SDO 后按 IgH 实际域偏移测试递增字节、全零、全 `ff`，要求完整 WKC、新样本计数和逐字节回显。记录结果与 MCAN 状态，再确认退回 PREOP、复核保留区。
