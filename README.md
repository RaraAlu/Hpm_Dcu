# DCU 开发工程

独立的 HPM5E31 EtherCAT / ENCOS 固件副本，开发环境为 Ubuntu 24.04 x86_64、CMake、厂商 GNU 工具链、OpenOCD 和 J-Link JTAG。

首次导入建立了可重复构建的基线。本轮在此基础上建立本地 Git 基线、目录职责清单和文档分类，不移动业务源码或修改控制逻辑。原 `../HPM_ethercat_canfd_encos_V1.0` 目录保持不动。导入范围见 [源码导入记录](docs/SOURCE_IMPORT.md)。

**当前边界：Debug/Release 编译、工具运行、源码符号读取和离线保护测试已验证；实际 JTAG 连接、烧录、断点和硬件运行尚未验收。**

## 工程导航

- [文档索引与状态分类](docs/README.md)：区分当前说明、设计资料和历史记录。
- [目录层级与维护职责](docs/current/PROJECT_LAYOUT.md)：查找实际构建入口、业务源码、可选适配和本机产物。
- [生成文件与来源清单](docs/current/GENERATED_FILES.md)：识别协议生成物、历史副本和待恢复的生成链。
- [目录整理验证记录](docs/validation/2026-11-13-directory-baseline.md)：本地 Git 基线、离线检查和本轮未改动范围。

## 快速开始

在 VS Code 中打开当前 `Dcu_dev` 目录，而不是上层目录或原固件目录。

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

不需要手动设置全局 `HPM_SDK_BASE` 或交叉编译器 PATH。Presets 和脚本使用当前工程内的工具与 SDK，普通构建不会访问 J-Link。

## 固定工具版本

| 工具 | 固定来源或版本 |
| --- | --- |
| HPM SDK | 本地化 1.12.1，保留现有自研板移植 |
| CMake | 本机已验证 3.28.3；Presets 要求至少 3.25 |
| GNU GCC / GDB | HPMicro 官方 GCC 13.2.0 / 配套 GDB 13.2 |
| OpenOCD | HPMicro 正式发布标签 `hpm_xpi_v0.3.0` 的 Linux 包 |
| Ninja | Ubuntu 24.04 包 `1.11.1-2` |
| HIDAPI | Ubuntu 包 `0.14.0-1build1`，只解包到本工程内 |

工具在 `.tools/`，安装包摘要、固定下载地址和摘要来源记录在 [toolchain.lock.json](tools/toolchain.lock.json)。不自动升级。厂商 OpenOCD 程序自身显示 `0.11.0+dev (2025-07-16-11:09)`，这里锁定的是其正式发布标签和附件 SHA-256，不是下载滚动开发分支。

GDB 13.2 支持 RV32 和基本 MI 调试，但该厂商包未启用 Python，因此不配置依赖 Python 的 pretty-printer。GCC 附件没有厂商公布的 SHA-256，锁文件中的 GCC 摘要来自首次官方 HTTPS 下载，不能称为厂商签名认证。

工具丢失或迁移到同架构 Ubuntu 24.04 主机时运行：

```bash
python3 scripts/dcu.py bootstrap
```

该命令校验固定摘要，优先使用 `.tools/cache`；不会安装系统软件、修改 shell 配置、配置 USB 或访问目标。SDK 仍使用系统 Python 3.12 及 `PyYAML`、`Jinja2`，本机已验证可导入。

来源：[HPMicro GCC 正式发布](https://github.com/hpmicro/riscv-gnu-toolchain/releases/tag/2023.10.18)、[HPMicro OpenOCD 正式发布](https://github.com/hpmicro/riscv-openocd/releases/tag/hpm_xpi_v0.3.0)。

## VS Code

- `Ctrl+Shift+B`：默认执行 `DCU: Build Debug`。
- 任务菜单提供 Release、离线检查、测试、烧录和校验入口。
- CMake Tools 使用 Presets；C/C++ 扩展使用本次构建的编译数据库，不再使用原目录的 ROS2/x86 配置。
- F5 提供两个独立入口：校验已有 Flash 后调试，或先备份/烧录再调试。
- 两个调试入口都属于**复位后调试**，不是无扰动附加；不自动继续运行。
- 硬件断点启用，`main` 断点预设；实际断点资源数仍需连接目标后确认。

调试服务仅监听 `127.0.0.1:3333`，Tcl 和 Telnet 端口关闭。前端必须等到镜像校验、必要烧录和最终暂停全部完成后的 `DCU_DEBUG_READY`，不会把端口开始监听误当作烧录完成。

## USB 权限与板级确认

2026-11-13 本机已枚举 J-Link，USB ID 为 `1366:0101`，序列号 `000050120677`；当前用户读写权限检查已通过。这只证明 USB 可访问，不证明 JTAG 接线或目标芯片已识别。

如果重新安装系统或权限失效，在普通用户的本机终端运行：

```bash
bash scripts/setup-usb.sh
```

向导分三步：安装仅匹配该 VID/PID 的 udev 规则、重载规则、重新插拔并检查权限。sudo 密码只在本机终端输入，不会写入文件；不要用 sudo 启动整个 VS Code。向导不执行 JTAG、复位或烧录命令，重新插拔前仍应隔离电机动力。

[board.local.json](config/board.local.json) 已生成，但保持 `board_verified=false` 和 `srst_connected=false`。其中 1 MiB、4 KiB 扇区、Flash 选项是现有 SDK 的候选参数，**不是实物探测结果**。首次硬件联调前必须确认：

1. J-Link 型号、固件、VTref/GND、电源及 TCK/TMS/TDI/TDO 接线。
2. nSRST 确实接到目标复位脚；当前配置为 `srst_only`。
3. 实际 Flash 型号、1 MiB 容量、4 KiB 擦除扇区、引脚组、电压和频率选项。
4. 电机动力已隔离，并有独立急停措施。

只有完成实际核对后，才更新本地配置中的确认状态。不要为了绕过启动检查把两个字段直接改成 true。该文件不应提交到共享版本库。

每次硬件操作还需要单独输入 `RESET_TARGET`，不将确认保存进 CMake 缓存。以下命令均会复位/暂停目标，目前因板级未确认而被阻止：

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
- 33 项离线保护与工具测试，以及 CTest 入口。
- USB 用户读写权限。

未验证：实际 TAP/CPU/Flash 识别、烧录与校验、断点/单步/复位恢复、脱离调试器启动、EtherCAT/CAN 功能、实时性和电机停止行为。

原 SSC 代码的 GCC 警告仍保留，包括地址比较、位掩码窄化、packed 成员和 Release 邮箱数组边界警告；未通过屏蔽警告或修改协议栈把构建“变绿”。既有 JS 生成脚本/部分 JSON 不是可解析文本，普通构建使用现成生成文件，不会执行这些脚本。后续业务整理时需单独恢复并验收生成链。

`docs/` 中导入的旧说明和测试记录按[历史材料清单](docs/archive/README.md)识别，原路径暂时保留。旧文档中的 `readme/`、`doc/` 和 Windows 路径是历史上下文，不代表当前目录布局；历史记录也不代表当前板上固件的状态。
