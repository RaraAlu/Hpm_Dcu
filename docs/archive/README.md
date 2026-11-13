# DCU 历史文件与示例清单

本清单按用途归档，暂不物理移动原文件。旧说明、图片、日志和 SSC 工程保留原路径及原始内容；可读说明仅补充状态提示。对不可解析文件不插入文本，避免破坏原字节和未知封装。

## 可读历史资料

| 原文件 | 身份与阅读限制 |
| --- | --- |
| [工程说明与功能验证](../工程说明与功能验证.md) | 早期移植说明；“尚无 CAN 初始化”不代表当前源码 |
| [CAN 指示灯说明](../CAN收发指示灯初始化与控制说明.md) | 历史引脚、移植及设计资料；源码和时序描述需重新核对 |
| [透传网关流程与指示灯逻辑](../EtherCAT转CANFD网关固件流程与指示灯逻辑.md) | 早期透明网关版本说明，不是当前电机业务总说明 |
| [TwinCAT 透传验证步骤](../TwinCAT3验证EtherCAT转CANFD透传网关.md) | 旧八槽、576 B 过程映像；不能直接作为当前 ENCOS 配置 |
| [Windows 新增文件与 SES 生成步骤](../新增C和H文件后命令行编译及重新生成SES工程.md) | 历史 Windows 开发流程；当前 Ubuntu 入口见根 README |
| [ENCOS 首次控制说明](../电机封装/HPM_ENCOS首次控制电机与TwinCAT测试说明.md) | 历史操作资料，不等于本板当前状态已经验收 |
| [2026-09-17 邮箱压测](../电机封装/2026-09-17_EtherCAT链路与SDO邮箱压测记录.md) | 历史现场观测，根因未闭环，板上实际固件版本当时未核对 |
| [根 README_zh.rst](../../README_zh.rst)、[README_en.rst](../../README_en.rst) | 原 SDK/SSC 示例说明；英文文件另有开发环境补充，当前操作仍以根 README.md 为准 |
| [源码导入记录](../SOURCE_IMPORT.md) | 首次导入时的快照，其中未初始化 Git 的说明属于该时点 |

历史设计方案另见[设计资料索引](../design/README.md)。

## 不是正常可读文本的导入资料

本轮 `file` 将以下文件识别为 `data`，不能按普通 Markdown 审阅。文件名不证明内容或实现状态；下表只提供可读历史材料的查找线索，不宣称两份内容相同。

| 保留文件 | 可读材料或处理方式 |
| --- | --- |
| [多电机调度改版说明](../EtherCAT转CANFD多电机调度改版说明.md) | 暂无已核实的可读副本，保留待恢复 |
| [多电机调度现状与优化建议](../EtherCAT转CANFD多电机调度现状与优化建议.md) | 暂无已核实的可读副本，保留待恢复 |
| [根目录 SSC 升级说明](../HPM_ENCOS_SSC与EEPROM升级使用说明.md) | 同主题可读材料位于[电机封装目录](../电机封装/HPM_ENCOS_SSC与EEPROM升级使用说明.md) |
| [两阶段使能固件实现说明](../HPM_ENCOS两阶段使能固件实现说明.md) | 可参考[历史两阶段设计稿](../电机封装/HPM_ENCOS查询反馈与两阶段使能改造说明.md)，不能据此认定两份等价 |
| [根目录协议升级实施方案](../HPM_ENCOS电机控制协议升级实施方案.md) | 同主题可读材料位于[电机封装目录](../电机封装/HPM_ENCOS电机控制协议升级实施方案.md) |
| [根目录首次控制说明](../HPM_ENCOS首次控制电机与TwinCAT测试说明.md) | 同主题可读材料位于[电机封装目录](../电机封装/HPM_ENCOS首次控制电机与TwinCAT测试说明.md) |

## 旧工具工程和非当前构建入口

| 文件或目录 | 当前身份 | 保留原因 |
| --- | --- | --- |
| [SSC/Src/digital_io.c](../../SSC/Src/digital_io.c) | 旧数字量应用示例，不参与当前默认构建 | 避免与 `application/digital_io.c` 混淆，保留作参考 |
| [SSC/digital_io.xlsx](../../SSC/digital_io.xlsx) | 原数字量对象定义 | 不覆盖当前 ENCOS 对象字典 |
| [SSC/ECAT_IO.esp](../../SSC/ECAT_IO.esp)、[SSC/Config/](../../SSC/Config/) | 原 SSC 工具工程和配置 | 仍可用于检查栈选项，移动前须核对工具引用 |
| [SSC/ECAT_IO.xml](../../SSC/ECAT_IO.xml)、[SSC/ESI/](../../SSC/ESI/) | 原数字量设备描述 | 不是当前 ENCOS 主站导入入口 |
| [SSC/ECAT_CANFD_GATEWAY.xml](../../SSC/ECAT_CANFD_GATEWAY.xml) | 早期透传设备描述 | 与 ENCOS ESI 分开识别 |
| [CMakeLists.txt.localized.bak](../../CMakeLists.txt.localized.bak) | 导入的构建配置备份 | 已由 Git 基线保存，第一轮不删除 |
| [port/hpm_ecat_foe.c](../../port/hpm_ecat_foe.c)、[port/lwip/](../../port/lwip/)、[port/modbus_tcp/](../../port/modbus_tcp/) | 当前默认构建未启用的适配 | 不是已启用功能，也不按“未编译”直接删除 |
| [docs/电机封装/](../电机封装/) 中的 JSON/XML | 历史协议副本 | 不建立第二个维护入口，关系见[生成清单](../current/GENERATED_FILES.md) |

## 图片与原始数据

根 `docs/` 下 PNG 和 [ZCANPRO 日志目录](../ZCANPRO采集的数据/)继续保留原位置。日志是原始采集数据，不等于已确认固件版本的验收报告。后续移动图片或文档前需检查 Markdown、RST、SSC 工具和脚本引用；不为了目录整齐先破坏旧资料。
