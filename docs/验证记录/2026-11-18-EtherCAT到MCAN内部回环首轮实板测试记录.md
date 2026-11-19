# EtherCAT 到 MCAN 内部回环首轮实板测试记录

日期：2026-11-18。**EtherCAT RxPDO → MCAN0 内部收发 → EtherCAT TxPDO 的独立诊断通路通过首轮实板验证。** 电机动力已隔离，未连接 CAN 业务节点；RK3588 专用口仅连接本测试板，只启用 `DriverModule`。

| 步骤 | 实测结果 |
| --- | --- |
| 镜像与保留区 | 复位前先暂停、回读 64 KiB 保留区。烧录并校验冻结版 `hpm-ethercat-mcan` 和保留区，随后无断点启动。 |
| 链路与 SDO | Link UP、单从站、PREOP。SII 与 `0x1018` 均确认诊断产品 `0x7fff0003`，不是软件回显产品 `0x7fff0002`。PREOP 下 36 项 SDO 上传、长度、固定字节和映射全部通过；不走旧的逐项 SDO 配置。 |
| 数据映射 | SM2 `0x1100` 对应 32 字节 RxPDO，SM3 `0x1200` 对应 48 字节 TxPDO。IgH 实际域范围分别为 0–31、32–79，20 个条目位偏移均为 0。 |
| PDO 往返 | OP 且 WKC 完整（值 3）时，`00..1f`、全零、全 `ff` 三组均逐字节匹配，接受的新样本计数依次为 4、7、10。该镜像仅在四帧各自 TX 完成并从 RX FIFO 验证后发布新样本。 |
| MCAN 证据 | 停止后读到 `CCCR=0xa0`（TEST、MON）、`TEST.LBCK=1`、`TXBTO=0xf`（槽 0–3 均有完成位，不是累计帧数）、`RXF0S=0`。重组缓冲为 32 字节 `ff`，诊断状态为 `MCAN_ECHO_READY`，发送门已关闭，无在途槽。 |
| 收尾 | 从站回到 PREOP，无 AL 错误；IgH `Idle / Active: no`，AimRT 已退出。64 KiB 保留区前后逐字节相同。MCU 保留诊断镜像并暂停，调试服务已关闭。 |

两个 AimRT 探针输出成功记录后，由外层限时结束空闲进程，返回码 124 不表示探针失败。实验不覆盖 CAN 收发器、线缆、端接、物理对端的请求/回复，也不覆盖生产 Profile、时延或一次确认启动。

## 原始证据

- [冻结 ELF](../../artifacts/target/20261118T081044.743581Z/image.elf)、[镜像范围](../../artifacts/target/20261118T081044.743581Z/image-check.json)、[保护下载](../../artifacts/target/20261118T081044.743581Z/openocd-session.log)。
- [IgH 预检](../../artifacts/target/20261118T081044.743581Z/rk3588-igh-before.log)、[SDO 只读](../../artifacts/target/20261118T081044.743581Z/driver-ethercat-mcan-sdo.log)、[PDO 往返](../../artifacts/target/20261118T081044.743581Z/driver-ethercat-mcan-pdo.log)、[IgH 收尾](../../artifacts/target/20261118T081044.743581Z/rk3588-igh-after.log)。
- [MCAN 与 AL 寄存器记录](../../artifacts/target/20261118T081044.743581Z/ethercat-mcan-gdb.log)、[复位前保留区](../../artifacts/target/20261118T081044.743581Z/retained-eeprom.bin)、[测试后保留区](../../artifacts/target/20261118T081044.743581Z/retained-eeprom-after-mcan.bin)。
