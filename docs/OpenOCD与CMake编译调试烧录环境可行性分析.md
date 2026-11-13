# OpenOCD 与 CMake 编译、调试、烧录环境可行性分析

> 环境选型历史分析：下文保留方案评估时的状态。当前已落地的开发入口见根[README](../README.md)，本轮离线验证见[目录整理记录](validation/2026-11-13-directory-baseline.md)；硬件仍未验收。

核实日期：2026-11-13。目标环境：用户已确认的 Ubuntu + J-Link，目标为本工程 HPM5E31 固件。

本文是方案分析，不是环境部署完成报告。本轮阅读本地文件、核对官方资料，并在新建的 `/tmp` 目录解压经官方摘要校验的两个 OpenOCD 程序，仅尝试查询适配器列表；两者均在动态加载阶段因缺少共享库失败，没有进入目标初始化。没有安装工具或依赖，没有修改源码、构建或编辑器配置，没有访问调试硬件、复位、暂停或烧录目标。

用户补充要求：使用稳定的环境。选型以固定正式版本、官方摘要、可复现配置和本板验收为准，不自动追新，也不把版本较旧当作已经证明更稳定。

## 1. 结论与前提修正

**结论：Linux 原生 CMake + HPMicro GNU 工具链 + HPMicro OpenOCD + RISC-V GDB 路线具有明确的官方支持基础，值得实施。当前只能确认方案可行，不能把它表述为“本工程已经在 Ubuntu 编译、烧录、调试通过”。**

需要区分三个问题：

1. **编译**由 CMake 和交叉工具链完成，OpenOCD 不是编译器。
2. **烧录**需要 OpenOCD 的 Flash 驱动、正确的板级参数和可用的 JTAG 链路。
3. **调试**由 RISC-V GDB 通过 OpenOCD 远程协议完成；编辑器只作为前端，不应成为命令行链路的必要条件。

HPM SDK 官方说明 CMake 构建支持 Linux，并提供 GNU GCC 工具链入口、Linux 依赖和 OpenOCD/GDB 工作流程。因此不需要把 Windows 的 SDK GUI 或 SES 当成 Ubuntu 下的必需品。[HPM SDK Getting Started](https://hpm-sdk.readthedocs.io/en/latest/get_started.html)、[SDK CMake User Guide](https://hpm-sdk.readthedocs.io/en/latest/cmake_user_guide.html)

不能笼统地说“标准 OpenOCD 不支持 HPM Flash”：当前上游手册已经列出 `hpm_xpi`。但也不能反过来推断“Ubuntu 仓库随便装一个 OpenOCD 就够了”：上游 `v0.12.0` 的 Flash 驱动表没有 `hpm_xpi`，而新手册不代表旧二进制具备相同能力。应检查具体版本、编译进二进制的驱动及 HPM5E00 算法支持。[上游 hpm_xpi 手册](https://openocd.org/doc/html/Flash-Commands.html)、[上游 v0.12.0 驱动表](https://github.com/openocd-org/openocd/blob/v0.12.0/src/flash/nor/drivers.c)

## 2. 推荐架构

```text
工程源码 + 本地化 HPM SDK
    -> CMake + Ninja
    -> HPMicro RISC-V GNU GCC / binutils
    -> 本次构建的 ELF + map + bin

RISC-V GDB / 编辑器调试前端
    -> 本机 OpenOCD GDB 服务
    -> OpenOCD jlink 适配器驱动
    -> J-Link 的 JTAG 接口
    -> HPM5E31
```

建议先打通命令行构建、地址检查、烧录校验和 GDB，再接入现有编辑器。OpenOCD 能提供 GDB 远程服务；GDB 必须支持目标 CPU，不能默认使用 Ubuntu 原生 x86 GDB。[OpenOCD GDB 工作方式](https://openocd.org/doc/html/GDB-and-OpenOCD.html)

这里的 J-Link 是硬件探针，GDB Server 是 OpenOCD，不是同时再启动一套 SEGGER J-Link GDB Server。两套服务不要同时争用同一个 USB 探针。后者是本方案的工程约束，不是已完成的硬件测试结论。

## 3. 工具版本候选

以下是本次核实过的候选，不宣称“必须最新”，也不把版本较新等同于本板已经验证。

| 组件 | 建议起点 | 已确认范围 |
| --- | --- | --- |
| HPM SDK | 保留工程内的本地化 SDK 1.12.1 | 版本来自本地 `VERSION`；避免同时进行 SDK 升级和开发环境迁移 |
| CMake / Ninja / Python | 保留现有 CMake 3.28.3、Python 3.12.3；补齐 Ninja | 不为追新升级现有可用工具；官方 Getting Started 给出的最低版本为 CMake 3.13、Python 3.8 |
| GNU GCC | HPMicro `2023.10.18` 标签的 GCC 13.2.0 newlib multilib Linux 包 | 发布记录实际日期为 2024-04-01，不是标签名称中的日期；支持 B 扩展的 multilib，官方构建平台为 Ubuntu 20.04 x86_64 |
| OpenOCD | 稳定优先，先固定官方 `hpm_xpi_v0.3.0` Linux 包作为候选基线 | `v0.3.0` 于 2025-06-30 发布，已明确增加 HPM5E00 支持且静态检查可见 J-Link 驱动；仍须解决宿主依赖并完成实板验收，不因为版本较旧就宣称更稳定。`v0.5.0` 仅作为本次对比对象，不是默认升级方向 |
| GDB | 支持 RISC-V 的交叉 GDB，优先与所选 HPMicro 工具链配套 | 包内 GDB、宿主依赖和版本信息须在部署阶段检查，不能仅凭 GCC 能运行就判定 GDB 也能运行 |

版本和宿主平台依据：[SDK 依赖说明](https://hpm-sdk.readthedocs.io/en/latest/get_started.html)、[GCC 官方发布页](https://github.com/hpmicro/riscv-gnu-toolchain/releases/tag/2023.10.18)、[GCC 官方附件清单](https://github.com/hpmicro/riscv-gnu-toolchain/releases/expanded_assets/2023.10.18)、[OpenOCD v0.5.0 发布页](https://github.com/hpmicro/riscv-openocd/releases/tag/hpm_xpi_v0.5.0)、[OpenOCD v0.3.0 支持说明](https://github.com/hpmicro/riscv-openocd/releases/tag/hpm_xpi_v0.3.0)。发布年份额外通过 GitHub 官方 API 的 `published_at` 核对：[OpenOCD 发布 API](https://api.github.com/repos/hpmicro/riscv-openocd/releases)、[GCC 发布 API](https://api.github.com/repos/hpmicro/riscv-gnu-toolchain/releases)。

已核实的 Linux 附件名称：

- GCC：`rv32imac_zicsr_zifencei_multilib_b_ext-linux.tar.gz`。
- OpenOCD v0.3.0：`openocd-linux.tar.gz`，发布说明记录其 Linux 包于 2025-07-16 用 Ubuntu 20.04 重新构建；官方附件 SHA-256 为 `bd951e31d7a114d43366a77521e6c8b036657b1df3019072d8fe49d12aa05bbb`。[v0.3.0 官方发布说明](https://github.com/hpmicro/riscv-openocd/releases/tag/hpm_xpi_v0.3.0)、[官方发布 API](https://api.github.com/repos/hpmicro/riscv-openocd/releases)
- OpenOCD v0.5.0：`openocd-linux-x86_64.tar.gz`，官方附件 SHA-256 为 `c1ab35677acfbd7663d7a88eb1631ff76e3d96e0d85a43d8601acd6dc6bc8f4a`。[OpenOCD 官方附件清单](https://github.com/hpmicro/riscv-openocd/releases/expanded_assets/hpm_xpi_v0.5.0)

OpenOCD v0.5.0 的 Linux 发布工作流使用 Debian 12 容器并设置静态链接选项。这是发布构建方式的证据，不是对所有 Ubuntu 版本、CPU 架构和 USB 权限的运行保证；部署时仍须核验主机架构、可执行文件和适配器驱动。[该标签的官方发布工作流](https://github.com/hpmicro/riscv-openocd/blob/hpm_xpi_v0.5.0/.github/workflows/release.yml)

### 3.1 预编译包的 J-Link 支持不能想当然

本次先对上述两个官方附件做了流式静态检查：核对下载数据的 SHA-256 与官方摘要一致，把压缩包内二进制送入 `strings`，不以字符串出现代替运行验证。随后补做了下述不初始化硬件的程序启动检查。

| 官方附件中的程序 | 静态检查结果 |
| --- | --- |
| v0.3.0 `bin/openocd` | 包含 `jlink_adapter_driver`、`jaylink_init`、`src/jtag/drivers/jlink.c` 和 `hpm_xpi_probe` |
| v0.5.0 `usr/local/bin/openocd` | 包含 `ftdi_adapter_driver` 和 `hpm_xpi_probe`，未发现 `jlink_adapter_driver`、`jaylink_init` 或 J-Link 驱动源码路径；存在的 `jlink serial` 兼容 Tcl 文本不能证明驱动已编入 |

检查对象：[官方 v0.3.0 Linux 附件](https://github.com/hpmicro/riscv-openocd/releases/download/hpm_xpi_v0.3.0/openocd-linux.tar.gz)、[官方 v0.5.0 Linux 附件](https://github.com/hpmicro/riscv-openocd/releases/download/hpm_xpi_v0.5.0/openocd-linux-x86_64.tar.gz)。上表是本次静态检查事实，不是成功查询适配器列表或连接 J-Link 的结果。

**无硬件启动检查结果：两版当前均不能直接运行。**重新下载到 `/tmp/hpm-openocd-audit-XdHz3H` 的附件再次核验摘要一致：

| 版本标签 | 下载文件 SHA-256 |
| --- | --- |
| `hpm_xpi_v0.3.0` | `bd951e31d7a114d43366a77521e6c8b036657b1df3019072d8fe49d12aa05bbb` |
| `hpm_xpi_v0.5.0` | `c1ab35677acfbd7663d7a88eb1631ff76e3d96e0d85a43d8601acd6dc6bc8f4a` |

在工作目录 `/tmp` 分别执行以下命令，没有传入任何 `-f` 配置脚本，也没有执行 `init`；`shutdown` 用于在查询后退出配置阶段，且外围设定 5 秒超时：

```sh
timeout --signal=TERM 5s /tmp/hpm-openocd-audit-XdHz3H/v0.3.0/bin/openocd -c 'adapter list' -c 'shutdown'
timeout --signal=TERM 5s /tmp/hpm-openocd-audit-XdHz3H/v0.5.0/usr/local/bin/openocd -c 'adapter list' -c 'shutdown'
```

两条命令都立即返回 `127`，没有输出运行时版本或适配器列表；动态加载器的原始诊断分别为：

```text
/tmp/hpm-openocd-audit-XdHz3H/v0.3.0/bin/openocd: error while loading shared libraries: libhidapi-hidraw.so.0: cannot open shared object file: No such file or directory
/tmp/hpm-openocd-audit-XdHz3H/v0.5.0/usr/local/bin/openocd: error while loading shared libraries: libhidapi-hidraw.so.0: cannot open shared object file: No such file or directory
```

因此，版本标签和静态符号已核实，但**运行时版本/驱动列表仍未核实**。缺少的是宿主共享库，不是已经证明的 USB 接线故障。本轮没有安装依赖、补库重试或尝试实际探针连接；安装阶段应先解决宿主依赖，再重做同类无初始化检查。

源码证据与前述静态符号结果一致：v0.5.0 的 Linux 发布步骤没有安装 libjaylink 开发包，也没有启用内部 libjaylink；`configure.ac` 默认不启用内部库，并把 J-Link 可用性取决于内部库或系统 `libjaylink >= 0.2`。因此有充分理由认为该版 Linux 预编译包漏编了 J-Link，不能把“官方有 Linux 包”直接等同于“该包满足 Ubuntu + J-Link”。两版因缺库启动失败本身不能用于判断驱动是否存在。[发布工作流](https://github.com/hpmicro/riscv-openocd/blob/hpm_xpi_v0.5.0/.github/workflows/release.yml#L32-L62)、[configure.ac 默认项及依赖检查](https://github.com/hpmicro/riscv-openocd/blob/hpm_xpi_v0.5.0/configure.ac#L384-L387)、[J-Link 构建判定](https://github.com/hpmicro/riscv-openocd/blob/hpm_xpi_v0.5.0/configure.ac#L612-L645)

实施时以 v0.3.0 为首选候选，先解决其宿主依赖，再核验适配器列表、本地脚本兼容性和实板行为；通过后冻结版本、摘要和配置，不自动升级。仅在基线验收失败且证据指向 OpenOCD 版本缺陷时，才另行评估替代版本或源码构建。作为备用路径，v0.5.0 源码若要启用 J-Link，必须固定标签与子模块，显式指定 `--enable-jlink`，并提供系统 libjaylink 或该标签支持的 `--enable-internal-libjaylink`；内部库选项已标记弃用。这不是本轮要执行的部署步骤。[该标签内部库构建检查](https://github.com/hpmicro/riscv-openocd/blob/hpm_xpi_v0.5.0/configure.ac#L678-L685)、[弃用提示](https://github.com/hpmicro/riscv-openocd/blob/hpm_xpi_v0.5.0/configure.ac#L779-L782)

## 4. 工具链兼容性不是“都是 RISC-V 就行”

**已验证的本地事实：**旧生成目录的 [build.ninja](../hpm5e00evk_flash_xip_debug/build.ninja) 第 55、86 行使用 `-mabi=ilp32 -march=rv32imac_zicsr_zifencei`。这是旧构建快照，不是新 Linux 构建的验收结果。本地 [toolchain.cmake](../hpm_sdk_localized_for_hpm5e00evk/cmake/toolchain.cmake) 第 184-202 行设置默认 ABI/ISA；[HPM5E31 CMakeLists.txt](../hpm_sdk_localized_for_hpm5e00evk/soc/HPM5E00/HPM5E31/CMakeLists.txt) 第 49-53 行还会在条件满足时追加 `zba_zbb_zbc_zbs`。

**含义：**换用支持 ISA 20191213 的 GCC 后，重新配置出的 ISA 可能比旧 Ninja 文件多出 B 子扩展。不要为追求与旧文件字面一致而任意删减 SDK 参数，也不要把旧 Windows 生成目录直接复用到 Linux。

GNU 文档明确区分 `-march` 的指令集和 `-mabi` 的调用约定；例如允许浮点指令不等于采用浮点寄存器传参 ABI。`ilp32`、`ilp32f`、`ilp32d` 不能随意互换。[GNU GCC RISC-V 选项](https://gcc.gnu.org/onlinedocs/gcc/RISC-V-Options.html)

建议以官方 bare-metal newlib multilib 包为起点，检查实际编译命令、库选择、`nano.specs` 和 ELF 属性，再考虑替换工具链。不能直接以 `riscv64-linux-gnu-*` 等 Linux 用户态工具链替代；某个 `riscv64-unknown-elf-*` 工具链理论上可能有所需 RV32 multilib，但前缀相似或 GCC 版本相同本身不构成兼容证据。若工程实际使用 Andes 自定义指令或 DSP 库，另须核对相应工具链，不能从标准 RISC-V ISA 支持推断兼容。[HPMicro GCC multilib 配置](https://github.com/hpmicro/riscv-gnu-toolchain/releases/tag/2023.10.18)、[SDK 工具链说明](https://hpm-sdk.readthedocs.io/en/latest/cmake_user_guide.html)

## 5. 本地 OpenOCD 脚本的可复用部分与限制

本地已经有入口 [hpm5e00_all_in_one.cfg](../hpm_sdk_localized_for_hpm5e00evk/boards/openocd/hpm5e00_all_in_one.cfg)：第 21-35 行按探针、SoC、板级顺序加载配置，第 23-24 行默认却是 `ft2232`。用户使用 J-Link，因此后续部署须显式选择 `jlink`，不能沿用默认探针。

[soc/hpm5e00.cfg](../hpm_sdk_localized_for_hpm5e00evk/boards/openocd/soc/hpm5e00.cfg) 第 9-15 行定义 TAP、RISC-V target 和工作 RAM；第 19-20 行进一步执行 `[find soc/hpm_common.cfg]`、`[find soc/hpm_common_csr.cfg]`。所以仅把入口文件路径写成绝对路径还不够，必须把本地 SDK 的 `boards/openocd` 纳入搜索路径，同时保留所选 OpenOCD 安装包自己的标准脚本目录，因为探针脚本还要查找 `interface/jlink.cfg`。OpenOCD 支持 `-s` 和 `OPENOCD_SCRIPTS`，且先找到的同名文件优先，部署时应记录实际解析路径以免混用不同版本脚本。[OpenOCD 脚本搜索顺序](https://openocd.org/doc/html/Running.html)

[boards/hpm5e00evk.cfg](../hpm_sdk_localized_for_hpm5e00evk/boards/openocd/boards/hpm5e00evk.cfg) 第 51、60 行声明 `hpm_xpi` bank，初始大小为 `0x02000000`，分别映射到 `0x80000000` 和 `0xB0000000`，并使用 `opt0=0x5`、`opt1=0x1000`。但 [hpm5e00evk.yaml](../hpm_sdk_localized_for_hpm5e00evk/boards/hpm5e00evk/hpm5e00evk.yaml) 第 9-11 行给链接脚本的板载 Flash 大小是 `1M`。

**不能把这直接解释为“实物一定有 32 MiB Flash”。**官方 fork v0.5.0 的 `hpm_xpi_probe()` 从目标端算法获取 Flash 信息，再用 `flash_info.total_sz_in_bytes` 重写 `bank->size`；配置中的声明值可能被探测结果覆盖。但该行为同样不能证明自研板参数正确，实物容量、引脚组、电压、Flash 型号和频率仍须核实。[固定版本 hpm_xpi.c 第 113-136 行](https://github.com/hpmicro/riscv-openocd/blob/hpm_xpi_v0.5.0/src/flash/nor/hpm_xpi.c#L113-L136)

## 6. J-Link、XIP 与安全边界

### 6.1 J-Link 链路

OpenOCD 官方 `jlink` 驱动支持 J-Link USB 适配器的 JTAG/SWD，并提醒部分硬件与固件组合存在兼容性差异；因此“有 J-Link”还不足以替代型号、硬件版本、固件和接线确认。Linux 还需要适当的 USB/udev 权限，不应把 `sudo` 运行所有调试进程当成默认解决方案。[OpenOCD J-Link 驱动](https://openocd.org/doc/html/Debug-Adapter-Configuration.html)、[Linux 适配器权限](https://openocd.org/doc/html/Running.html)

本地 [probes/jlink.cfg](../hpm_sdk_localized_for_hpm5e00evk/boards/openocd/probes/jlink.cfg) 第 5、10-11 行默认 10 MHz、JTAG、`srst_only`。这不是对自研板接线和信号完整性的验证。待确认项包括 J-Link 型号与固件、VTref/GND、TCK/TMS/TDI/TDO、nSRST 是否真正接到目标、目标供电和 JTAG 引脚复用。首次验证应从保守频率开始，不把默认 10 MHz 当作已验收值。

### 6.2 XIP 断点

正确声明 Flash bank 后，OpenOCD 能把 Flash 内存映射传给 GDB，GDB 据此优先在只读区域使用硬件断点；GDB 的 `hbreak` 可以显式请求硬件断点。硬件触发资源有限，实际可用数量应从目标/OpenOCD 枚举确认，本文不猜测 HPM5E31 的断点数量。[OpenOCD Flash 内存映射与 GDB](https://openocd.org/doc/html/GDB-and-OpenOCD.html)、[GNU GDB 硬件断点](https://sourceware.org/gdb/current/onlinedocs/gdb.html/Set-Breaks.html)

工程判断：Flash XIP 调试应先确认 `info mem` 与实际 ELF 地址吻合，再验证少量硬件断点、单步和复位后行为；不要把对 RAM 写指令的软件断点方案直接套到外部 NOR Flash，也不能假设换成 J-Link 硬件就自动获得某种商业调试器的无限 Flash 断点能力。

### 6.3 连接也可能改变目标状态

本地 [boards/hpm5e00evk.cfg](../hpm_sdk_localized_for_hpm5e00evk/boards/openocd/boards/hpm5e00evk.cfg) 第 74-75 行的 `gdb-attach` 直接执行 `reset halt`。因此当前配置中的“attach”不是无扰动附加，而是会复位并暂停目标。第 70-71 行还设置了复位初始化动作；Flash probe 在官方驱动内也会把算法放入工作 RAM 并运行，不能把“只读探测”理解为目标状态完全不变。[hpm_xpi_probe 实现](https://github.com/hpmicro/riscv-openocd/blob/hpm_xpi_v0.5.0/src/flash/nor/hpm_xpi.c#L35-L136)

本地 [soc/hpm5e00.cfg](../hpm_sdk_localized_for_hpm5e00evk/boards/openocd/soc/hpm5e00.cfg) 第 15 行把 `0x00000000` 起的 `0x20000` 字节配置为工作区，并设置 `work-area-backup 0`；[flash_xip.ld](../hpm_sdk_localized_for_hpm5e00evk/soc/HPM5E00/HPM5E31/toolchains/gcc/flash_xip.ld) 第 14、49-59、187-199 行把同一 ILM 用于固件的 `.vectors` 和 `.fast`。因此即使去掉 `reset halt`，Flash 自动探测/算法仍可能覆盖运行现场。无复位附加必须独立设计和验收；在确认自动探测、内存映射请求和工作区保护前，不允许隐式 `load` 或 probe。可以评估专门的仅目标附加配置，但本文不承诺已实现无扰动附加。

本地 [probes/jlink.cfg](../hpm_sdk_localized_for_hpm5e00evk/boards/openocd/probes/jlink.cfg) 第 4 行使用 `bindto 0.0.0.0`，会把服务暴露到所有 IPv4 接口。未来应在加载探针脚本后由工程包装配置覆盖为 `127.0.0.1`，不改 SDK 原文件，也不把调试接口直接开放到不可信网络。[OpenOCD bindto 语义](https://openocd.org/doc/html/General-Commands.html)

### 6.4 电机与 EtherCAT 调试

这是安全风险推断，不是本轮硬件测量结果：暂停 CPU 后，CPU 上的软件停机逻辑、命令超时处理和 PDO 刷新不能继续正常执行；ESC、DMA、定时器和电机驱动器却不一定同步停止。不能假定断点会让整机进入安全状态。

首次连接、复位、暂停和烧录，应先断开电机动力或通过独立硬件禁止输出，明确恢复和急停手段。断点可用于验证程序流程，不可用于证明 EtherCAT 周期、通信超时停机、电机实际停止时延或实时抖动达标；这些必须在不暂停 CPU 的条件下另行验收。

## 7. 烧录数据完整性

建议默认使用本次构建的地址感知 ELF，而不是手填一个猜测的 BIN 基地址。BIN 不携带原始地址；只有查清生成规则、首个加载地址和可能的头部/填充后，才能为其选择烧录偏移。旧生成产物只可作为对照，不可替代新构建检查。

即使 ELF 的加载段没有直接覆盖 EEPROM/参数区，也不能单凭这一点证明烧录安全。OpenOCD 文档明确说明 `flash write_image erase` 可能擦除所在扇区的未写部分，甚至擦除映像段之间的空洞。后续实施必须检查映像范围、实际擦除扇区和保留区是否相交，事先备份，禁止以整片擦除作为常规下载步骤。[OpenOCD 镜像擦写警告](https://openocd.org/doc/html/Flash-Commands.html)

## 8. 分阶段验收建议

| 阶段 | 内容 | 通过条件与边界 |
| --- | --- | --- |
| A. 离线环境 | 固定版本、核验摘要、主机兼容性、RISC-V 工具和脚本路径 | 不连接目标；记录版本及 `jlink` 适配器驱动可用性 |
| B. 离线构建 | 新建独立 Linux build 目录，保留旧 Windows 生成目录，编译并检查 ELF/map | 编译链接成功；ISA/ABI、Flash LMA、启动头和保留区检查通过 |
| C. 隔离动力的连接 | 连接 J-Link、确认供电/链路/复位路径，必要时探测 Flash | 用户明确允许影响目标状态；识别到正确 TAP/CPU/容量，不能只看到端口监听就算成功 |
| D. 隔离动力的烧录调试 | 先备份保留数据，受控擦写、校验、复位启动、少量硬件断点 | 烧录校验、脱离调试器启动、断点和恢复行为均有证据 |
| E. 业务验收 | EtherCAT 状态转换、PDO/CAN-FD、看门狗、急停和实时性 | 不依赖断点；独立验证硬件输出与物理停止行为 |

阶段 A/B 的通过不能代替 C/D，C/D 的通过也不能代替 E。本轮未执行上述部署和硬件验收。

## 9. 尚待确认

- 宿主已核实为 Ubuntu 24.04.4 LTS、x86_64；两版 OpenOCD 已确认缺少 `libhidapi-hidraw.so.0`，补齐该依赖后是否还缺其他库仍需检查。
- J-Link 的具体型号、硬件/固件版本、序列号、JTAG 与 nSRST 接线。
- 自研板实际 Flash 型号、容量、电压、引脚组、时钟与板级启动配置。
- 新 Linux 构建的完整依赖、链接产物、启动地址及 EEPROM/参数区擦除保护。
- 用户期望的是复位后调试，还是必须保留运行现场的附加调试；当前脚本仅能确认前者的动作意图。

总体建议：先保持 SDK 和固件逻辑不变，只建立可复现的命令行构建与受控调试链路；把工具链迁移、板级 Flash 校正和业务功能修改拆开验收，避免一次变更多个变量而难以定位问题。

## 10. 本机与工程审计

### 10.1 当前基础环境

以下为本轮本机只读命令的结果，不是从旧说明文档推断的安装状态。

| 检查 | 结果 | 对方案的影响 |
| --- | --- | --- |
| `/etc/os-release`、`uname -a` | Ubuntu 24.04.4 LTS，x86_64 | 可以评估官方 Linux x86_64 工具，不需要 Windows 虚拟机 |
| `cmake --version` | 3.28.3 | 已有可用的 CMake；无需为迁移先升级 CMake |
| `python3 --version`、依赖导入 | Python 3.12.3、PyYAML 6.0.1、Jinja2 3.1.2 | 本地 SDK 列出的两项 Python 依赖已经可导入 |
| `code --list-extensions` | 已有 `ms-vscode.cmake-tools`、`ms-vscode.cpptools` | 可以复用现有 VS Code，主要缺工作区配置 |
| `command -v` | 当前 PATH 未找到 OpenOCD、Ninja、所检查的 RISC-V GCC/GDB；已有 Make 和宿主 GDB | 需要准备目标工具；没有在 PATH 中找到不等于整台机器从未安装 |
| `lsusb` | 本次未看到可识别的 J-Link，看到 CP210x 串口桥 | 只能确认串口类设备枚举，不能把它当作 JTAG 链路成功的证据 |

额外执行 `gdb --nx --batch -ex 'set architecture riscv:rv32' -ex 'show architecture'`，当前宿主 GDB 返回 `Undefined item: "riscv:rv32"`，架构仍为 `i386`。因此当前 `/usr/bin/gdb` 不能直接承担该 RV32 调试链路；需要核验并固定支持目标的交叉 GDB 或多架构 GDB。这是未加载固件、未连接服务器的本地架构查询。

本地依赖来源为 [scripts/requirements.txt](../hpm_sdk_localized_for_hpm5e00evk/scripts/requirements.txt)。现有 [.vscode/c_cpp_properties.json](../.vscode/c_cpp_properties.json) 仍指向 ROS2 和 `/usr/bin/gcc`，不能用于准确解释本工程的目标宏、头文件和 ISA。应改用 SDK 已启用的 `compile_commands.json` 或 CMake Tools 配置提供器，而不是继续手填 ROS2 includePath。[本地 hpm-sdk-config.cmake](../hpm_sdk_localized_for_hpm5e00evk/cmake/hpm-sdk-config.cmake)

### 10.2 构建前需要解决什么

- 根 [CMakeLists.txt](../CMakeLists.txt) 第 17-25 行已经固定本地化 SDK 和 `hpm5e00evk` 板名。这里的板级代码已被项目定制，不能重新下载同名官方 EVK 代码直接替换。
- SDK 的 [application.cmake](../hpm_sdk_localized_for_hpm5e00evk/cmake/application.cmake) 第 19-21 行要求 `GNURISCV_TOOLCHAIN_PATH`；先前独立临时目录的配置检查在此失败，尚未进入完整固件编译。
- 当前工程缺少被 [ecat.c](../ecat.c) 第 17 行、[motor_control.c](../application/motor_control.c) 第 1 行等文件引用的 `motor_control.h`。完整构建前必须恢复或按实际接口补齐它；这属于工程完整性问题，不是 OpenOCD 支持问题。
- `tools/generate_encos.js`、`tools/verify_encos.js` 在本机不是可解析的 JavaScript，`SSC/encos_interface.json` 也不是可解析 JSON。这影响协议再生成和配套校验。但现有生成头文件、存储和对象字典已经存在，当前 CMake 不调用这些 JS；不能把恢复全部生成链误当成每次普通固件编译的必经步骤。
- [旧 CMakeCache.txt](../hpm5e00evk_flash_xip_debug/CMakeCache.txt) 和 [build.ninja](../hpm5e00evk_flash_xip_debug/build.ninja) 保留 Windows 盘符、`.exe` 和 `cmd.exe` 命令。新建独立 Linux 构建目录，保留旧目录作为对照，不在其上直接构建或批量替换路径。

建议继续使用 SDK 现有工具链、启动和链接规则，首个配置保持 `HPM_BUILD_TYPE=flash_xip`、`CMAKE_BUILD_TYPE=debug`。不另造一套通用 RISC-V toolchain 文件覆盖 SDK，也不同时切换 RAM 启动模式。调试版和最终优化版分别验证，不把优化后的变量不可见或单步跳行直接判成调试器故障。[本地 SDK 构建类型与调试选项](../hpm_sdk_localized_for_hpm5e00evk/CMakeLists.txt)

### 10.3 本工程地址与保留区

本轮执行 `readelf -W -l hpm5e00evk_flash_xip_debug/output/demo.elf`，旧 ELF 的首个非空加载段物理地址为 `0x80000400`，入口为 `0x80003000`，Flash 加载数据末端为 `0x80057100`。`demo.bin` 开头是 NOR 配置，和 [flash_xip.ld](../hpm_sdk_localized_for_hpm5e00evk/soc/HPM5E00/HPM5E31/toolchains/gcc/flash_xip.ld) 第 22-24 行定义的配置、启动头、应用布局一致。**这些是旧产物的地址证据，不证明旧产物与当前所有源码一致。**

因此不能假设该原始 BIN 应从 `0x80000000` 写入。默认烧录本次生成的 ELF；若另用 BIN，必须对每次产物确认起始 LMA、生成工具是否添加填充，再选择地址，不复用未经确认的偏移。

EEPROM 保留区由 [board.h](../hpm_sdk_localized_for_hpm5e00evk/boards/hpm5e00evk/board.h) 第 643 行的偏移 `0x80000` 与 [ecat_config.h](../ecat_config.h) 第 18 行的长度 `0x10000` 给出。结合 Flash 基址 `0x80000000`，物理保留范围为：

```text
起始地址：0x80080000
结束地址：0x80090000（不包含）
长度：    64 KiB
```

这里需要保留整个 64 KiB Flash 仿真存储区，不只是 16 KiB SII 逻辑镜像。当前 [flash_xip.ld](../hpm_sdk_localized_for_hpm5e00evk/soc/HPM5E00/HPM5E31/toolchains/gcc/flash_xip.ld) 的末尾断言只检查整块 Flash 容量，没有专门防止代码或加载数据侵入该保留区。环境集成时应增加构建后检查：根据 ELF 非空加载段的物理加载地址检查范围，并核对烧录工具实际擦除扇区及段间空洞；新固件增大时应直接阻止不安全的烧录。

另需区分三套参数：[board.c](../hpm_sdk_localized_for_hpm5e00evk/boards/hpm5e00evk/board.c) 第 81 行的启动配置和 OpenOCD 板级脚本都使用 `opt0=0x5`；[board.h](../hpm_sdk_localized_for_hpm5e00evk/boards/hpm5e00evk/board.h) 第 218 行供运行期 NOR API 使用的是 `0x6`。不同阶段不要求数值机械相等，不能据此断言旧产物过时或直接统一参数；应按实际 Flash、引脚和各阶段频率分别核对。

## 11. 一体化环境的落地范围

建议把命令行作为唯一执行入口，再让 VS Code 调用相同动作，避免编辑器和终端各维护一套编译/烧录命令。

| 拟新增或调整的配置 | 用途与边界 |
| --- | --- |
| `CMakePresets.json` | 共享 SDK 构建类型、生成器、独立构建目录和常用构建配置 |
| `CMakeUserPresets.json` | 每台机器的工具安装路径；不作为共享工程配置提交 |
| 工程级 OpenOCD 包装配置 | 显式使用 J-Link、本地 SDK 搜索路径、保守 JTAG 频率和本机回环监听；不直接改 SDK 原脚本 |
| CMake 命名目标或薄脚本 | 分离 `build`、`flash`、`verify`、`debug-server`，执行前检查版本、镜像和保留区；普通构建绝不自动烧录 |
| `.vscode/launch.json`、必要任务配置 | 使用 `cppdbg` 和支持 RISC-V 的 GDB 连接本机 OpenOCD；显式管理符号文件和硬件断点 |
| 工作区 IntelliSense 配置 | 使用同一 Linux 构建生成的编译数据库，替换当前 ROS2/x86 假设 |

CMake 官方区分可共享的项目 Presets 和不应提交的个人 Presets；本机 CMake 3.28.3 已能支持这一方式。[CMake 3.28 Presets](https://cmake.org/cmake/help/v3.28/manual/cmake-presets.7.html)

VS Code 的现有 C/C++ 扩展支持 `miDebuggerPath`、`miDebuggerServerAddress`、调试服务启动配置和 `hardwareBreakpoints`。因此可以复用 `cppdbg`，没有必要为了“一体化”先更换 IDE；具体断点上限必须按目标实际报告配置，不复制文档示例中的数量。[VS Code 官方调试配置](https://code.visualstudio.com/docs/cpp/launch-json-reference)

首阶段交付应以“复位后调试”为范围：先完成受控烧录和校验，再复位、暂停并调试对应 ELF；不要默认自动恢复电机控制。附加已有固件、不重刷、不复位的模式另设入口并单独验收，其工作区和自动探测风险见 6.3 节。所谓一键操作是减少重复配置，不是把擦除、复位、暂停或电机安全确认隐藏起来。
