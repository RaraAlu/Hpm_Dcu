# DCU 开发工程

独立的 HPM5E31 EtherCAT / ENCOS 固件副本，开发环境为 Ubuntu 24.04 x86_64、CMake、厂商 GNU 工具链、OpenOCD 和 J-Link JTAG。

首次导入建立了可重复构建的基线，当前已按单片机维护职责完成目录迁移：固件位于 `firmware/`，依赖位于 `third_party/`，自研 BSP、生成物与协议交付物分开维护。旧示例、失效工具和过期资料已删除，当前入口见[文档索引](docs/文档索引.md)。工程外的原 `../HPM_ethercat_canfd_encos_V1.0` 目录不在本次清理范围内。

**当前边界：混合语言版本已通过 Debug/Release 编译与离线测试，尚未上板；此前 C 基线已验证 JTAG、下载回读及基础断点/单步/变量/内存读取。扩展 GUI、脱离调试器启动、EtherCAT/CAN 控制业务和实时性仍未验收。**

当前采用 C/C++ 混合固件：电机、产品 PDO/CoE 适配和 IMU 公共模型使用 C++17；入口、SSC 栈及其 C 应用回调、RTOS 编排、硬件适配和生成存储保留 C。混合语言版本已通过离线构建与协议测试，尚未上板，不能沿用此前 C 固件的硬件验收结论。语言归属、首次清理步骤和约束见[混合语言开发规范](docs/当前说明/混合语言开发规范.md)。

已接入电机驱动扩展基础：通用命令/反馈与 ENCOS 实现分开，每轴显式绑定驱动，未知配置拒绝。生产仍只有既有 ENCOS 实现；IMU 仅准备公共样本、有效性和未接入模型，没有具体驱动或采样接入。接入步骤及未完成的抽象见[设备驱动扩展与接入规范](docs/当前说明/设备驱动扩展与接入规范.md)。

当前默认固件已接入 FreeRTOS 和 Sync0 一拍流水线，接收主站配置的 200–1000 Hz，生产运行要求 DC Sync0。先完成从站，主站后续再调整；当前仅完成软件和离线验证，未烧录，1 us 抖动与反馈周期尚未实测。继中断屏蔽上下文和 CAN 竞态修复后，已完成参数事务内存错误修复，以及 PDO/CoE、事务、调度、RTOS 与 BSP 的第一轮职责拆分，见[应用层拆分记录](docs/验证记录/2026-11-16-应用层职责拆分与事务修复记录.md)。后续统一文件命名，并将自有实现中的类型/宏移入头文件；当前 133 项测试、15 项 ASan/UBSan 用例及资源复验见[类型与配置整理记录](docs/验证记录/2026-11-16-类型声明与配置归属整理记录.md)。

## 工程导航

- [文档索引与状态分类](docs/文档索引.md)：区分当前说明、设计资料和历史记录。
- [环境依赖与外部资源准备](docs/当前说明/环境依赖与外部资源准备.md)：系统依赖、锁定工具包、离线资源、VSIX 及新主机恢复流程。
- [编译调试下载经验总结](docs/当前说明/编译调试下载经验总结.md)：日常 CLI/GUI 操作、关键踩坑、保留区保护、排错和环境精简边界。
- [目录层级与维护职责](docs/当前说明/目录层级与维护职责.md)：查找实际构建入口、业务源码、可选适配和本机产物。
- [配置文件归档规范](docs/当前说明/配置文件归档规范.md)：各模块 `cfg/` 的职责、工具固定入口例外、本机私有配置和迁移验证。
- [遗留清理验证记录](docs/验证记录/2026-11-13-遗留清理验证记录.md)：删除范围、活动资源保留、离线验证和后续限制。
- [HPM 开发扩展分类与使用指南](docs/当前说明/开发扩展分类与使用指南.md)：已安装 VS Code 扩展、具体使用方法及与当前 Presets/调试保护的差异。
- [生成文件与来源清单](docs/当前说明/生成文件与来源清单.md)：识别协议生成物、历史副本和待恢复的生成链。
- [混合语言开发规范](docs/当前说明/混合语言开发规范.md)：C/C++ 归属、跨语言接口、实时约束与当前验证范围。
- [设备驱动扩展与接入规范](docs/当前说明/设备驱动扩展与接入规范.md)：公共 Interface、唯一生产驱动、型号注册、参数兼容及后续接入要求。
- [分布式时钟同步与实时系统接入规划](docs/设计资料/分布式时钟同步与实时系统接入规划.md)：200–1000 Hz、Sync0 锁存、一拍反馈与 RTOS；首批从站软件已实施，需求继续补充。
- [同步与实时系统验证记录](docs/验证记录/2026-11-16-同步流水线与实时系统离线验证记录.md)：依赖锁定、截止/故障测试、Debug/Release 资源及未上板边界。
- [实时系统修复与状态归属审查](docs/验证记录/2026-11-16-实时系统修复与状态归属审查.md)：内核上下文和 CAN 竞态修复、常量/静态状态归属、真实内核执行回归和剩余设计成本。
- [应用层职责拆分与事务修复记录](docs/验证记录/2026-11-16-应用层职责拆分与事务修复记录.md)：参数编码失败回归、协议适配/事务/调度私有状态、RTOS 与 BSP 分工及当前验证边界。
- [模块文件命名整理与验证](docs/验证记录/2026-11-16-模块文件命名整理与验证.md)：业务入口、周期编排、任务、定时器及协议范围的命名对照，保留的 SSC 接入名和回归结果。
- [类型声明与配置归属整理记录](docs/验证记录/2026-11-16-类型声明与配置归属整理记录.md)：宏/类型归入头文件、私有状态仍由实现持有、头文件独立编译和双翻译单元检查。
- [设备抽象基础验证记录](docs/验证记录/2026-11-13-设备抽象基础验证记录.md)：驱动替换测试、协议差分、74 个编译单元及资源变化。
- [目录规划与实施状态](docs/设计资料/单片机工程目录重构规划.md)：实际迁移对照、模块归属及尚待解耦的依赖。
- [混合语言目录迁移验证记录](docs/验证记录/2026-11-13-混合语言目录迁移验证记录.md)：源码摘要、70 个默认编译单元、BSP、二进制差异和离线验收范围。
- [目录整理验证记录](docs/验证记录/2026-11-13-目录整理基线.md)：本地 Git 基线、离线检查和首次整理的未改动范围。
- [调试与下载实测记录](docs/验证记录/2026-11-13-调试与下载实测记录.md)：真实目标板、扩展适配器测试、复位/启动暂停修正及保留区回读结果。

## 快速开始

首次搭建或迁移机器，先完成[环境与资源准备](docs/当前说明/环境依赖与外部资源准备.md)。以下命令用于基础依赖与本地工具就绪后的日常开发。

在 VS Code 中打开当前 `Dcu_dev` 目录，而不是上层目录或原固件目录。

入口在 [firmware/app/main.c](firmware/app/main.c)。自研板通过 `firmware/bsp/dcu_hpm5e31/` 接入，不再修改 SDK 内评估板目录。首次从旧布局切换时，先按[清理步骤](docs/当前说明/混合语言开发规范.md)移除旧构建缓存与静态库对象，再使用日常命令。

```bash
cd ~/work/MCU/Hpm/igh_ethercat_v2.2/hpm/Dcu_dev
python3 scripts/dcu.py doctor
python3 scripts/dcu.py build --preset hpm-debug
python3 scripts/dcu.py test
```

Release 构建：

```bash
python3 scripts/dcu.py build --preset hpm-release
```

两种构建分别输出到：

```text
build/hpm-debug/output/dcu.elf
build/hpm-debug/output/dcu.bin
build/hpm-debug/output/dcu.map
build/hpm-debug/image-check.json
build/hpm-release/output/dcu.elf
```

不需要手动设置全局 `HPM_SDK_BASE` 或交叉编译器 PATH。根 `CMakePresets.json` 仅引用 [cmake/cfg/CMakePresets.json](cmake/cfg/CMakePresets.json)，实际构建参数只维护一份。Presets 和脚本使用当前工程内的工具与 SDK，普通构建不会访问 J-Link。

## 固定工具版本

| 工具 | 固定来源或版本 |
| --- | --- |
| HPM SDK | 本地化 1.12.1，保留现有自研板移植 |
| FreeRTOS | SDK 1.12.1 配套 V10.5.1，含两处已记录的优先级上下文补丁；固定提交及 38 文件摘要见 [freertos.lock.json](tools/cfg/freertos.lock.json) |
| CMake | 本机已验证 3.28.3；Presets 要求至少 3.25 |
| GNU GCC / GDB | HPMicro 官方 GCC 13.2.0 / 配套 GDB 13.2 |
| OpenOCD | HPMicro 正式发布标签 `hpm_xpi_v0.3.0` 的 Linux 包 |
| Ninja | Ubuntu 24.04 包 `1.11.1-2` |
| HIDAPI | Ubuntu 包 `0.14.0-1build1`，只解包到本工程内 |

工具在 `.tools/`，安装包摘要、固定下载地址和摘要来源记录在 [toolchain.lock.json](tools/cfg/toolchain.lock.json)。不自动升级。厂商 OpenOCD 程序自身显示 `0.11.0+dev (2025-07-16-11:09)`，这里锁定的是其正式发布标签和附件 SHA-256，不是下载滚动开发分支。

GDB 13.2 支持 RV32 和基本 MI 调试，但该厂商包未启用 Python，因此不配置依赖 Python 的 pretty-printer。GCC 附件没有厂商公布的 SHA-256，锁文件中的 GCC 摘要来自首次官方 HTTPS 下载，不能称为厂商签名认证。

工具丢失或迁移到同架构 Ubuntu 24.04 主机时运行：

```bash
python3 scripts/dcu.py bootstrap
```

该命令校验固定摘要，优先使用 `.tools/cache`；不会安装系统软件、修改 shell 配置、配置 USB 或访问目标。普通 SDK 配置仍使用系统 Python 3.12 和 `PyYAML`；`Jinja2` 用于当前 Presets 已排除的 IAR/SES 工程生成，不是普通构建的强制依赖。系统运行库、离线缓存及扩展安装要求统一见[环境准备文档](docs/当前说明/环境依赖与外部资源准备.md)。

来源：[HPMicro GCC 正式发布](https://github.com/hpmicro/riscv-gnu-toolchain/releases/tag/2023.10.18)、[HPMicro OpenOCD 正式发布](https://github.com/hpmicro/riscv-openocd/releases/tag/hpm_xpi_v0.3.0)。

## VS Code

- `Ctrl+Shift+B`：默认执行 `DCU: Build Debug`。
- 任务菜单提供 Release、离线检查、测试、烧录和校验入口。
- CMake Tools 使用 Presets；C/C++ 索引已从 ROS2/x86 旧配置切换到交叉工具链和编译数据库，其他扩展的 ROS2/Python 辅助设置不作为固件构建来源。
- F5 提供两个独立入口：校验已有 Flash 后调试，或先备份/烧录再调试。
- 两个调试入口都属于**复位后调试**，不是无扰动附加；使用 `stopAtConnect=true` 配合 `launchCompleteCommand=None` 保持连接后暂停，单设后者不够。
- 硬件断点启用，加载符号并连接目标后预设 `main` 断点；手动继续可停到 `main`。目前已验证单个硬件断点，未验收全部断点资源。
- 预设 GDB 断点使用 `unknownBreakpointHandling=stop` 按普通断点暂停。否则 C/C++ 默认可能把 `Hit breakpoint ...` 显示为“出现异常”；这个标签不代表固件崩溃，也不应通过关闭所有异常断点来处理。

调试服务仅监听 `127.0.0.1:3333`，Tcl 和 Telnet 端口关闭。前端必须等到镜像校验、必要烧录和最终暂停全部完成后的 `DCU_DEBUG_READY`，不会把端口开始监听误当作烧录完成。

## USB 权限与板级确认

2026-11-13 本机已枚举 J-Link，USB ID 为 `1366:0101`，序列号 `000050120677`；当前用户读写权限检查已通过。这只证明 USB 可访问，不证明 JTAG 接线或目标芯片已识别。

如果重新安装系统或权限失效，在普通用户的本机终端运行：

```bash
bash scripts/setup-usb.sh
```

向导分三步：安装仅匹配该 VID/PID 的 udev 规则、重载规则、重新插拔并检查权限。sudo 密码只在本机终端输入，不会写入文件；不要用 sudo 启动整个 VS Code。向导不执行 JTAG、复位或烧录命令，重新插拔前仍应隔离电机动力。

[board.local.json](scripts/cfg/board.local.json) 已按用户确认的 Windows 既有默认配置将 `board_verified` 和 `srst_connected` 设为 `true`；Linux 本次 Flash 探测返回 1 MiB 容量、4 KiB 扇区，并完成烧录和回读。用户确认电机动力隔离并授权本次复位、暂停和烧录；这些条件不代表以后每次操作都自动成立。换板、换接线或首次联调前必须重新确认：

1. J-Link 型号、固件、VTref/GND、电源及 TCK/TMS/TDI/TDO 接线。
2. nSRST 确实接到目标复位脚；当前配置为 `srst_only`。
3. 实际 Flash 型号、1 MiB 容量、4 KiB 擦除扇区、引脚组、电压和频率选项。
4. 电机动力已隔离，并有独立急停措施。

只有完成实际核对后，才更新本地配置中的确认状态。不要为了绕过启动检查把两个字段直接改成 true。该文件不应提交到共享版本库，示例配置仍默认未确认。探针报告约 4.8 V 的 VTarget，用户确认接线无问题，但本轮没有独立电压测量，不将其写成电气验收通过。

每次硬件操作还需要单独输入 `RESET_TARGET`，不将确认保存进 CMake 缓存。以下命令均会复位/暂停目标，不能在电机动力未隔离时随意运行：

```bash
python3 scripts/dcu.py verify --confirm-target RESET_TARGET
python3 scripts/dcu.py flash --confirm-target RESET_TARGET
python3 scripts/dcu.py debug-server --confirm-target RESET_TARGET
```

`verify` 只校验映像，不写 Flash，但仍会复位、运行探测算法并暂停目标。已有映像与当前 ELF 不一致时，不允许继续使用错误符号调试。调试期间不要同时重新构建该配置的 ELF。

CMake 也有 `check-image`、`flash`、`verify`、`debug-server` 目标。后三者默认拒绝硬件访问；审核配置后，可通过仅对单次命令有效的 `DCU_TARGET_CONFIRM=RESET_TARGET` 显式授权。普通 `all` 目标不包含它们。

## Flash 和保留数据

当前 Flash XIP 的首个加载地址为 `0x80000400`，入口为 `0x80003000`。默认使用地址感知的 ELF；不要把 BIN 随意写到 `0x80000000`。

必须保留的 EEPROM 仿真区域：`[0x80080000, 0x80090000)`，共 64 KiB。构建后检查非空 ELF 加载段的物理地址，以及按扇区对齐、包含段间空洞的整个擦除跨度。若侵入保留区，构建后的检查失败，硬件入口也再次拒绝。

烧录入口会冻结本次 ELF 副本，再进行地址检查，避免构建过程替换待烧录文件；随后验证探测到的 Flash 容量、备份整个保留区、写入并校验映像、再次校验保留区。不会执行整片擦除、自动恢复备份或自动 `resume`。

每次操作的镜像副本、检查报告、Tcl 事务脚本和保留区备份保存在 `artifacts/target/<UTC时间>/`。命令行烧录/校验另有 OpenOCD 日志；VS Code 调试服务的日志输出在调试控制台。断开 GDB 时显式请求 halt，但这不是物理急停保证。

固件本身已有启动时 SII 升级/迁移逻辑，之后运行可能合法改写 EEPROM；这里的备份和保留区校验描述的是烧录事务，不保证应用运行后所有字节永远不变。无复位附加尚未实现，Flash 探测算法工作区可能覆盖运行现场。

暂停 CPU 后，软件命令租约、反馈超时处理和 PDO 刷新也会停止。不要用断点验证 EtherCAT 实时周期或实际停机时延，也不要认为 CPU 停在断点处电机就一定安全。

## 验证与已知问题

2026-11-13 已验证：

- GCC/GDB/Ninja/OpenOCD 的本机运行，OpenOCD 驱动列表含 `jlink`。
- Debug 与 Release 完整编译链接、ELF/映像生成与保留区检查。
- GDB 能读取新 ELF 的 `main` 地址、源码路径和行号。
- OpenOCD 配置解析，仅执行 `shutdown`，未执行 `init`。
- 71 项离线测试及 CTest 入口，其中包括 35 项保护与工具测试、8 项协议和 C/C++ 互操作测试、4 项目录归属与文档命名测试、8 项配置归档测试、16 项设备抽象测试。
- USB 用户读写权限。
- 实物 TAP `0x1000563d`、单个 RV32 hart、Flash 探测、受保护下载及独立回读；6 个非空 ELF 加载段全部匹配，擦除范围外和 64 KiB EEPROM 均零字节变化。
- C/C++ 两个现有入口的调试适配器，以及 Cortex-Debug 外接受保护 OpenOCD 服务的基础互操作：连接暂停、`main` 硬件断点、单步、变量、16 字节内存读取及断开。

本轮修复了 OpenOCD 缺少 500 ms 复位释放等待、SDK 提前 halt 回调与本板 nSRST 不匹配，以及 C/C++ 连接后自动继续和过早设置断点的问题。最初 C/C++ 测试曾意外运行固件约 30 秒，后续已修复并回读确认 EEPROM 未变化。不能把此前的离线配置检查当作已验证这些启动行为，完整过程见[实测记录](docs/验证记录/2026-11-13-调试与下载实测记录.md)。

未验证：VS Code GUI 点击流程、HPM 插件默认 J-Link GDB Server 配置、SVD/RTOS/SWO 视图、故障注入、脱离调试器启动、EtherCAT/CAN 功能、实时性和电机停止行为。C/C++ 适配器仍提示未识别 RISC-V 的架构元数据并假定 x86_64；实际 GDB/目标为 RV32，基础功能通过，但不能据此保证所有复杂数据展示均兼容。

原 SSC 代码的 GCC 警告仍保留，包括地址比较、位掩码窄化、packed 成员和 Release 邮箱数组边界警告；未通过屏蔽警告或修改协议栈把构建“变绿”。已删除不可解析的旧 JS 生成脚本和 JSON 材料，普通构建继续使用现成生成文件。后续调整协议前必须单独恢复并验收生成链，不能因旧文件已清理就认定可再生成。

`docs/` 中现存文件与子目录统一使用中文名称。`legacy/` 和 `docs/历史资料/` 已删除，仅保留当前说明、设计规划和有效验证记录；旧材料可从 Git 历史追溯，不保留工作区副本。历史验证记录中的旧目录和 Windows 路径只代表当时状态，不作为当前操作入口。
