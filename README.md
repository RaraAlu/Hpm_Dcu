# DCU 开发工程

独立的 HPM5E31 EtherCAT / ENCOS 固件副本，开发环境为 Ubuntu 24.04 x86_64、CMake、厂商 GNU 工具链、OpenOCD 和 J-Link JTAG。

HPM 运行 EtherCAT 从站和 CAN 网关，RK3588 上的 IgH / `DriverModule` 运行生产周期。固件源码在 `firmware/`，第三方栈在 `third_party/`。Ubuntu 只编译，不运行宿主回归；生产验收不使用诊断镜像替代电机往返。

## 从哪里开始

| 任务 | 入口 |
| --- | --- |
| 搭建环境、选择预设、受保护下载 | [环境准备](docs/当前说明/环境依赖与外部资源准备.md) · [编译调试下载](docs/当前说明/编译调试下载经验总结.md) · [上板前提](AGENTS.md) |
| 核对 Profile 和双向 PDO | [启动配置与在线选择](docs/当前说明/按启动配置生成周期布局.md) |
| 理解一拍电机命令或 88 B 透传 | [周期锁存与反馈发布](docs/当前说明/周期锁存与反馈发布契约.md) · [SingleFd 布局与同步](docs/当前说明/透传布局切换.md) |
| 审查固件的中断、实时性、发送授权及掉电行为 | [MCU 固件审查规则](docs/当前说明/MCU固件审查规则.md) |
| 修改驱动、查询或授权 | [设备驱动接入](docs/当前说明/设备驱动扩展与接入规范.md) · [生产启动与重连](docs/当前说明/生产网关一次确认接入.md) |
| 清理 RK 主站驱动及其测试夹具 | [WG3 驱动模块手工清理清单](docs/设计资料/WG3驱动模块手工清理清单.md) |
| 查证据、看下一步 | [文档索引](docs/文档索引.md) · [项目总计划](docs/设计资料/项目总计划.md)（唯一执行计划） |

当前生产配置的接线、运动参数授权和验收边界见[启动配置与在线选择](docs/当前说明/按启动配置生成周期布局.md)。编译通过不代表当前镜像的实机收发已经验收。

## 快速开始

首次搭建或迁移机器，先完成[环境与资源准备](docs/当前说明/环境依赖与外部资源准备.md)。以下命令用于基础依赖与本地工具就绪后的日常开发。

在 VS Code 中打开当前 `Dcu_dev` 目录，而不是上层目录或原固件目录。

入口在 [firmware/app/main.c](firmware/app/main.c)。自研板通过 `firmware/bsp/dcu_hpm5e31/` 接入，不再修改 SDK 内评估板目录。首次从旧布局切换时，先按[清理步骤](docs/当前说明/混合语言开发规范.md)移除旧构建缓存与静态库对象，再使用日常命令。

```bash
cd ~/work/MCU/Hpm/igh_ethercat_v2.2/hpm/Dcu_dev
python3 scripts/dcu.py doctor
python3 scripts/dcu.py build --preset hpm-debug
```

默认生产版（RK 使用同次构建导出的契约）：

```bash
python3 scripts/dcu.py build --preset hpm-release
```

构建产物示例：

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
- 任务菜单提供 Release、工具检查、烧录和校验入口。
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

本机私有 `scripts/cfg/board.local.json` 不随仓库交付，见[未确认的配置模板](scripts/cfg/board.example.json)。历史 Linux Flash 探测返回 1 MiB 容量、4 KiB 扇区，并完成烧录和回读。用户确认电机动力隔离并授权本次复位、暂停和烧录；这些条件不代表以后每次操作都自动成立。换板、换接线或首次联调前必须重新确认：

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

必须同时保留 SII 仿真区 `[0x80080000, 0x80090000)` 和启动编号日志 `[0x80090000, 0x80092000)`；见[保留区布局](docs/当前说明/启动编号日志初始化与烧录保留.md)。构建后检查非空 ELF 加载段的物理地址，以及按扇区对齐、包含段间空洞的整个擦除跨度。若侵入保留区，构建后的检查失败，硬件入口也再次拒绝。

烧录入口会冻结本次 ELF 副本，再进行地址检查，避免构建过程替换待烧录文件；随后验证探测到的 Flash 容量、备份整个保留区、写入并校验映像、再次校验保留区。不会执行整片擦除、自动恢复备份或自动 `resume`。

每次操作的镜像副本、检查报告、Tcl 事务脚本和保留区备份保存在 `artifacts/target/<UTC时间>/`。命令行烧录/校验另有 OpenOCD 日志；VS Code 调试服务的日志输出在调试控制台。断开 GDB 时显式请求 halt，但这不是物理急停保证。

固件本身已有启动时 SII 升级/迁移逻辑，之后运行可能合法改写 EEPROM；这里的备份和保留区校验描述的是烧录事务，不保证应用运行后所有字节永远不变。无复位附加尚未实现，Flash 探测算法工作区可能覆盖运行现场。

暂停 CPU 后，软件命令租约、反馈超时处理和 PDO 刷新也会停止。不要用断点验证 EtherCAT 实时周期或实际停机时延，也不要认为 CPU 停在断点处电机就一定安全。

## 验证与已知问题

以下是 2026-11-13 当时的记录；其中的宿主测试和 CTest 入口不再作为当前构建步骤：

- GCC/GDB/Ninja/OpenOCD 的本机运行，OpenOCD 驱动列表含 `jlink`。
- Debug 与 Release 完整编译链接、ELF/映像生成与保留区检查。
- GDB 能读取新 ELF 的 `main` 地址、源码路径和行号。
- OpenOCD 配置解析，仅执行 `shutdown`，未执行 `init`。
- 71 项离线测试及 CTest 入口，其中包括 35 项保护与工具测试、8 项协议和 C/C++ 互操作测试、4 项目录归属与文档命名测试、8 项配置归档测试、16 项设备抽象测试。
- USB 用户读写权限。
- 实物 TAP `0x1000563d`、单个 RV32 hart、Flash 探测、受保护下载及独立回读；6 个非空 ELF 加载段全部匹配，擦除范围外和 64 KiB EEPROM 均零字节变化。
- C/C++ 两个现有入口的调试适配器，以及 Cortex-Debug 外接受保护 OpenOCD 服务的基础互操作：连接暂停、`main` 硬件断点、单步、变量、16 字节内存读取及断开。

本轮修复了 OpenOCD 缺少 500 ms 复位释放等待、SDK 提前 halt 回调与本板 nSRST 不匹配，以及 C/C++ 连接后自动继续和过早设置断点的问题。最初 C/C++ 测试曾意外运行固件约 30 秒，后续已修复并回读确认 EEPROM 未变化。不能把此前的离线配置检查当作已验证这些启动行为，完整过程见[实测记录](docs/验证记录/2026-11-13-调试与下载实测记录.md)。

该轮调试工具测试未验证：VS Code GUI 点击流程、HPM 插件默认 J-Link GDB Server 配置、SVD/RTOS/SWO 视图、故障注入、脱离调试器启动、EtherCAT/CAN 功能、实时性和电机停止行为。C/C++ 适配器仍提示未识别 RISC-V 的架构元数据并假定 x86_64；实际 GDB/目标为 RV32，基础功能通过，但不能据此保证所有复杂数据展示均兼容。

原 SSC 的 GCC 警告按其验证记录保留，不通过屏蔽警告宣称通过。旧脚本已删除；目前的[协议生成来源及隔离再生成校验](docs/当前说明/生成文件与来源清单.md)与当时状态不同。

`docs/` 中现存文件与子目录统一使用中文名称。`legacy/` 和 `docs/历史资料/` 已删除，仅保留当前说明、设计规划和有效验证记录；旧材料可从 Git 历史追溯，不保留工作区副本。历史验证记录中的旧目录和 Windows 路径只代表当时状态，不作为当前操作入口。
