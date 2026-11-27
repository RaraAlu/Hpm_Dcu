# 调测

- **项目文档统一存放在根目录 `docs/`。** 根目录的 `README.md` 和 `AGENTS.md` 保留为入口；新增或移动文档时更新[文档索引](docs/文档索引.md)。
- **Ubuntu 只编译，不跑宿主回归。** 生产固件使用 `python3 scripts/dcu.py build --preset hpm-release`。编译不用接板。
- **J-Link 烧录，HPM 接电机，RK 跑主站。** HPM 是 EtherCAT 从站，通过 CAN FD 接电机；RK3588 用 IgH 和 `DriverModule` 跑 EtherCAT 周期。
- **烧录、回读或调试前，先确认动力隔离。** 用 `scripts/dcu.py` 的受保护入口，保留 SII 和启动编号日志。`verify` 虽然不写 Flash，也会复位或暂停目标。
- **RK 和 HPM 用同次构建的 PDO 契约。** 从 `build/<preset>/firmware/profile-contract/protocol/ethercat/` 取生成物；在 PREOP 核对 Profile 和双向 PDO 映射，不匹配就不进 OP。
- **台架收发改动编译后在台架验收。** 先用生产 `DriverModule` 发一拍全零 MIT，不把它当停机手段。第 N 帧发命令，第 N+1 帧必须读到每个目标轴的本拍新反馈，并核对每轴 CAN 发送完成计数；`WKC=3`、发出 CAN 帧或收到旧反馈，都不算通过。单拍失败不自动重试或转长测。

- **审查 `firmware/` 只列 `hpm-release` 生产可达项，按 [MCU 固件审查规则](docs/当前说明/MCU固件审查规则.md)执行。** 区分源码可证的问题和必须上板测量的时序、资源余量；工具未覆盖不算通过。

细节见[环境准备](docs/当前说明/环境依赖与外部资源准备.md)、[编译调试下载](docs/当前说明/编译调试下载经验总结.md)和[周期契约](docs/当前说明/周期锁存与反馈发布契约.md)。
