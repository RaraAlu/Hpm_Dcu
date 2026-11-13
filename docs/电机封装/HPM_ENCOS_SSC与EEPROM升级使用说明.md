# HPM ENCOS SSC 与 EEPROM 升级使用说明

> 历史生成与升级流程：当前 JS 生成/校验工具无法解析，本文命令不能直接视为可运行入口。先检查[生成文件与来源清单](../current/GENERATED_FILES.md)，不要覆盖现有生成物或板上数据。

## 1. 本次升级结果

该版本把一块 HPM EtherCAT 从原来的数字量示例升级为 4 路 CAN、每路 7 个电机、共 28 轴的 ENCOS 控制网关，并保留 IMU 与 CAN 原始帧通道。

- EtherCAT 标识：ProductCode `0x00000002`，Revision `0x00000002`。
- 默认 PDO：`RUN_MIXED`，SM2 为 784 字节，SM3 为 1072 字节。
- 其他 PDO：`RUN_BASE` 为 736/1024 字节，`GATEWAY_FULL` 为 544/544 字节。
- SII EEPROM 映像：16 KiB。
- 默认 ESC 地址：SM2 `0x1100`，SM3 `0x1A30`，默认 PDO 三缓冲区结束地址 `0x26C0`（最后有效地址 `0x26BF`）。
- 生成的 ESI：`SSC/HPM_ENCOS_4CAN_28AXIS_IMU.xml`。
- 生成的 SII：`SSC/HPM_ENCOS_4CAN_28AXIS_IMU.bin`。
- 固件内置 SII：`SSC/Src/eeprom.h`。
- 对象字典：`SSC/Src/gatewayObjects.h`。
- PDO 字节偏移表：`SSC/encos_pdo_offsets.md`。

对象和 PDO 只维护一份源数据，即 `tools/generate_encos.js`。不要分别手改 XML、`gatewayObjects.h`、接口结构体和偏移表，否则这些文件会失去一致性。

## 2. 重新生成 EtherCAT 配置

电脑需安装 Beckhoff EtherCAT Slave Stack Code Tool。本机验证版本为 SSC Tool 5.13，默认目录为：

```text
C:\Program Files (x86)\Beckhoff\EtherCAT Slave Stack Code Tool
```

在项目目录运行：

```powershell
node tools/generate_encos.js
powershell -ExecutionPolicy Bypass -File tools/generate_sii.ps1
node tools/verify_encos.js
```

三步分别完成：

1. 生成 C 接口、SSC 对象字典、ESI XML 和 PDO 偏移表。
2. 调用 SSC 随附的 `ET9300Utilities.dll`，将 ESI 转为 16 KiB SII，并更新固件的 `eeprom.h`。
3. 检查对象数量、映射数量、三种 PDO 长度、SM 地址、ProductCode、Revision、SII 长度和 ENCOS 手册命令样例。

如 SSC 安装在其他目录：

```powershell
powershell -ExecutionPolicy Bypass -File tools/generate_sii.ps1 -SscToolDir 'D:\Tools\EtherCAT Slave Stack Code Tool'
```

## 3. SSC Tool 图形界面的使用方法

`SSC/ECAT_IO.esp` 是原 SSC 工程，主要用于查看和维护 SSC 栈选项，例如 PDI、邮箱、CoE、DC、EEPROM 仿真和 Explicit Device ID。新的 28 轴业务对象由生成器维护，不再以旧的 `digital_io.xlsx` 为准。

操作顺序：

1. 启动 SSC Tool，打开 `SSC/ECAT_IO.esp`。
2. 检查应用使用 CoE、PDO Assign、EEPROM Emulation、Explicit Device ID；Complete Access 保持关闭。
3. 若只调整业务对象、PDO 或电机参数，不需要在 SSC Tool 内重新建对象，修改 `tools/generate_encos.js` 后执行上一节的三个命令。
4. 若修改 SSC 栈选项并重新生成 `SSC/Src`，先生成到临时目录并比较差异。SSC Tool 生成会覆盖栈文件；合并完成后再次运行 `generate_encos.js`，恢复并校验 `gatewayObjects.h` 等业务文件。
5. 构建 HPM 固件，确认链接成功后再烧录。

不要把旧版 `SSC/digital_io.xlsx` 重新导入并覆盖当前对象字典。它只描述原数字量示例，不包含 4 CAN、28 轴、IMU、原始帧和 16 KiB SII。

## 4. 将新配置写入 HPM EtherCAT EEPROM

当前发布的板载SII保留完整TxPDO/RxPDO Category，因此IgH可以从EEPROM读取全部PDO索引、子索引、位宽和类型。为满足IgH要求，生成SII时不写入可读名称，并把仅用于板载SII的设备类型名缩短为10字节`HPM_28AXIS`；正式ESI仍保留完整设备显示名和全部PDO名称。当前SII终止标记位于word 3658，有效内容包含终止标记共7318字节；PDO地址、过程数据布局及控制逻辑不变。

### 更小的精简SII方案（推荐用于固定主站工程）

若后续需要更大EEPROM余量，可以在生成SII的临时XML中删除`RxPdo`、`TxPdo`和`PdoMapping`描述，只保存身份、General、FMMU、SyncManager和DC类别。该方案终止标记位于word 146，但不改变正式ESI和固件CoE对象字典中的77个PDO映射。

- TwinCAT需要安装配套ESI，通过ESI获得完整PDO定义。
- IgH需要在应用代码中显式配置PDO，或者读取`0x1C12/0x1C13`及对应`0x16xx/0x1Axx`映射对象。
- 只依赖板载SII、既不加载ESI也不读取CoE的工具无法枚举PDO。
- 该精简方案适合PDO固定且主站配置受控的正式工程；需要从EEPROM自描述全部PDO时，应使用当前7318字节的完整PDO版本。

本项目的 EtherCAT EEPROM 是 HPM 外部 NOR Flash 上的软件仿真区。新固件使用 64 个 256 字节页保存 16 KiB SII，不要求板上另接一颗 16 KiB 串行 EEPROM。

### 方法 A：烧录新固件后自动升级（推荐）

1. 先记录每块板当前的 Station Alias 和 Serial Number。
2. 烧录本版本固件，并保持首次上电过程供电稳定。
3. 固件首次启动时检测旧的 2 KiB、按字保存的 EEPROM 数据。
4. 若旧数据有效，固件先保存一份 2 KiB 回退副本，再迁移为 16 KiB 分页映像。
5. 迁移时保留 Station Alias（SII 字节偏移 8）和 Serial Number（偏移 28），更新 ProductCode/Revision，并重新计算配置区校验字节。
6. ProductCode 所在的第 0 页最后提交，减少掉电后出现“新标识、旧内容”的机会。
7. 断电重启，从站重新进入 INIT/PREOP 后用 TwinCAT 扫描并核对身份与 PDO。

固件启动时在没有当前有效映像、ProductCode 不同、固件内 Revision 更新或 SII 主体布局不同时替换 SII。主体比较跳过板级 Alias/Serial 区域，因此从旧的 1136/1456 字节 PDO 布局升级到新的 784/1072 字节布局时也会自动更新，同时保留板卡身份。日常启动内容一致时不会反复擦写 Flash。`ecat_flash_eeprom_format()` 保持为空，避免误清除每块板的 Alias 和 Serial。

### 方法 B：用 TwinCAT 在线写入

1. 把 `SSC/HPM_ENCOS_4CAN_28AXIS_IMU.xml` 复制到 TwinCAT 的 EtherCAT ESI 目录，通常是：

   ```text
   C:\TwinCAT\3.1\Config\Io\EtherCAT
   ```

2. 重启 TwinCAT XAE，或执行 Reload Device Descriptions。
3. 将系统切换到 Config Mode，扫描 EtherCAT 设备。
4. 打开从站的 EtherCAT 页面，在 Advanced Settings / ESC Access / E2PROM 中先执行 Read 并保存旧映像。
5. 执行 Write EEPROM/Update EEPROM，选择本设备描述。界面名称会随 TwinCAT 版本略有不同。若使用 Hex Editor 整片写入 `SSC/HPM_ENCOS_4CAN_28AXIS_IMU.bin`，需在写入后恢复该板的 Alias 和 Serial；通用映像中的这两个板级值为 0。Alias 改动还必须由工具更新 SII 配置校验字节；现场工具不能可靠恢复时应使用方法 A。
6. 等待写入结束后断电重启从站，再次扫描。不要在写入过程中断电、复位或拔掉 EtherCAT 线。

在线写入会经过 SSC EEPROM 回调更新 RAM 页；SSC 的 Store 请求会调用 `ecat_flash_eeprom_flush()`，把脏页写入 NOR Flash。

## 5. 写入后的检查项目

在 TwinCAT 中至少核对：

- Vendor ID 保持 HPM 原值，ProductCode 为 `2`，Revision 为 `2`。
- EEPROM 大小为 16384 字节。
- 每块板的 Alias 和 Serial Number 仍为原值且互不重复。
- 默认 SM2：起始 `0x1100`、长度 784；SM3：起始 `0x1A30`、长度 1072。
- PDO 中有 Board、4 个 CAN Channel、28 个 Motor、IMU 和 SharedRaw 对象。
- 从 INIT 进入 PREOP、SAFEOP、OP 均正常，无 Invalid SM configuration 或 Invalid PDO mapping。
- 显式设备 ID 不为 0。运行时顺序为对象 `0x2004:02` 配置值、SII Serial 低 16 位、板上拨码值加 1。

若 TwinCAT 仍显示旧设备，先删除旧扫描结果、重新加载 ESI，再重新扫描；不要直接沿用旧从站节点的缓存 PDO。

## 6. 电机参数和运行限制

- 只有 `ModelProfileId = 0x8000` 的显式自定义量程配置会通过运行配置检查；未知电机量程不会自动猜测。
- `StopPolicy` 当前必须为 0，停止时撤销待发帧并停止周期命令。
- ENCOS 手册中“加速度参数”的应答长度与示例有冲突，因此 ParameterId 1 暂时拒绝写入。
- Kt 标定模式的配置值与查询值在手册中存在编号冲突，使用前需由电机厂家确认。
- MotorMaintenance (`B+8`) 目前仅支持 `Opcode=3`、`Argument0/1=0` 的真实电机当前位置置零：
  需 PREOP、SDO 控制权、同 CAN 通道无活动轴；HPM 向该轴发送 `0x7FF` 置零帧，核对电机回包，
  并在发送后保持该通道至少 500 ms 不再发 CAN 帧。其他维护操作和 SaveToken 仍返回 Unsupported；
  普通电机参数暂不承诺掉电保存。这不影响 EtherCAT SII 的 16 KiB EEPROM 保存。

## 7. 主站首次配置电机

SII 只保存 EtherCAT 身份和 PDO 描述，不保存具体电机的量程。主站每次上电应在 PREOP 中按以下顺序下发：

1. 写 `0x2001:01` 的 `AxisConfiguredMask`。
2. 写 `0x2010`～`0x2013` 的 4 路 CAN 参数和 `MotorConfiguredMask`。
3. 对轴 `a=0..31`，写 `0x3000 + 0x10*a` 的路由/限值、`0x3001 + 0x10*a` 的基础参数、`0x3002 + 0x10*a` 的自定义量程。当前 `ModelProfileId` 使用 `0x8000`，`StopPolicy` 使用 0。
4. 最后给 `0x2002:01 ApplyGeneration` 写入一个新的非零代数，读 `0x2002:03 Result`，应为 0；再读 `0x2002:04 AppliedGeneration` 确认一致。
5. 进入 SAFEOP/OP 后，PDO 的 `Version` 固定写 `0x00010000`，先发送停止帧完成会话解锁，再发送周期命令。默认 `LeaseMs` 最大 10 ms，主站中断命令后固件会按租约停止轴。

`0x2002:02 SaveToken` 当前不执行电机配置持久化，因此这些 SDO 参数需要由主站启动流程重复下发。

## 8. 发布前硬件验证

当前自动验证覆盖代码编译、ESI/SII 生成、PDO 布局和手册已知编码向量。正式发布前仍需在目标板完成：

1. 旧 2 KiB EEPROM 到 16 KiB 的首次上电迁移及掉电恢复测试。
2. TwinCAT 在线整片写入、读回比对和重启测试。
3. 4 路 CAN 各接 8 个真实电机，在 SAFEOP/OP 下检查周期、丢帧、超时和急停。
4. 核对具体电机型号的量程、反馈类型、加速度应答和 Kt 标定编号。
