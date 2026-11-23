# 独立 EtherCAT 扫描测试

日期：2026-11-18。状态：已完成[首轮实板测试](../验证记录/2026-11-18-EtherCAT扫描首轮实板测试记录.md)。无断点复测时 RK3588 专用口 Link UP，主站发现 1 个诊断从站，`DriverModule` 只读身份检查通过。该记录仅对应扫描镜像；后续板卡已切换到[数据回环诊断镜像](独立EtherCAT数据回环测试.md)。

## 目的与隔离

`hpm-ethercat-scan` 只验证 RK3588 GMAC1 到 HPM 的物理链路、ESC 活性和 SII 枚举。它复用板级 EtherCAT 引脚、时钟与 PHY 驱动，轮询处理 SII 读取和 AL 状态请求。未链接 SSC、生产配置/对象字典、CAN 路径、执行器逻辑或 RTOS。INIT/PREOP 以外的状态请求拒绝，不允许进入 SAFEOP/OP。

镜像内置不可写的诊断 SII，产品码为 `0x7fff0001`，与生产产品码不同。SII 不声明邮箱协议；SM2/SM3 只有禁用的零长度描述符，没有 PDO 映射。既有主站只读探针预期能报告 RxPDO/TxPDO 为 0，这不验证真实数据交换。SII 写命令返回错误；启动及运行不读写 Flash 保留区。生产和 CAN 回环预设均默认不启用该入口。

## 执行顺序

| 步骤 | 操作 | 本轮通过条件 |
| --- | --- | --- |
| 1. 离线 | 构建独立预设，核查镜像与编译/链接隔离 | 镜像擦除范围不进入保留区，诊断与生产入口互斥 |
| 2. 下载 | 重新确认执行器动力隔离、J-Link 接线、目标及操作范围；受保护下载加 `--backup-before-reset` 时先 halt 回读备份，再复位并在烧录前后校验 | 先出现 `DCU_RETAINED_BACKUP_COMPLETE`，下载校验后出现 `DCU_DEBUG_READY`；不要使用默认生产 F5 |
| 3. 板端启动 | 使用冻结 ELF，目标自复位连续运行，不在 PHY 启动后设断点；需要时待主站采集完成再暂停查看 `s_scan_status` | `s_scan_status=0`，SII 装载及 PHY 初始化完成；目标连续运行才能及时服务主站 SII 请求 |
| 4. 主站发现 | 在 RK3588 查看 IgH 状态和从站身份，再只启用 `DriverModule` 运行 `driver_ethercat_probe.yaml` | 专用口 Link UP，1 个匹配诊断身份的从站，零 PDO 映射，不请求 OP |
| 5. 收尾 | 保存镜像、日志及保留区前后比较；结束本轮后暂停目标并关闭调试服务 | 保留区字节不变，未自动恢复生产固件 |

离线入口：`python3 scripts/dcu.py build --preset hpm-ethercat-scan`；`python3 scripts/dcu.py check-image --preset hpm-ethercat-scan`。

离线通过 5 项扫描测试，覆盖 SII 校验、诊断身份、无邮箱/无 PDO、读取宽度和越界、AL 状态限制以及目标确认门槛。独立扫描、生产 Debug、CAN 内部回环构建通过。实板证据单独记录，不用离线结果代替 PHY 链路或扫描结果。

若 Link UP 但发现为 0，先分开核对 HPM 启动停点、诊断 SII 和 IgH 帧计数。若仍 Link DOWN，先检查 PHY、专用口和线缆，不进入 PDO/SDO 测试或恢复旧的逐项配置 SDO。
