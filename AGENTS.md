# 调测速记

- **Ubuntu 编译和跑测试。** `python3 scripts/dcu.py test` 跑宿主回归；`python3 scripts/dcu.py build --preset hpm-release-five-axis` 编五轴台架固件。默认生产版用 `hpm-release`。这些操作不用接板。
- **J-Link 烧录，HPM 接电机，RK 跑主站。** HPM 是 EtherCAT 从站，通过 CAN FD 接电机；RK3588 用 IgH 和 `DriverModule` 跑 EtherCAT 周期。
- **烧录、回读或调试前，先确认动力隔离。** 用 `scripts/dcu.py` 的受保护入口，保留 SII 和启动编号日志。`verify` 虽然不写 Flash，也会复位或暂停目标。
- **RK 和 HPM 用同次构建的 PDO 契约。** 从 `build/<preset>/firmware/profile-contract/protocol/ethercat/` 取生成物；在 PREOP 核对 Profile 和双向 PDO 映射，不匹配就不进 OP。
- **先离线，再单拍，后长测。** 台架只发全零 MIT，不把它当停机手段。第 N 帧发命令，第 N+1 帧必须读到每个目标轴的本拍新反馈。另核对每轴 CAN 发送完成计数；`WKC=3`、发出 CAN 帧或收到旧反馈，都不算通过。

细节见[环境准备](docs/当前说明/环境依赖与外部资源准备.md)、[编译调试下载](docs/当前说明/编译调试下载经验总结.md)和[周期契约](docs/当前说明/周期锁存与反馈发布契约.md)。
