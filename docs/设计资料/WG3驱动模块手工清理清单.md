## Context｜WG3 手工删除工作单

目标：在 `test_rp_3588:/home/zkbot/rob_dev/wg3_deploy` 仅保留 **WG3 YAML 动态 PDO（含电机观测、头部旁带）** 和 **按通道掩码并行收发的事件 CAN 透传**。本单交给用户手工实施，不是批量删除命令；未改远端文件，未编译、跑测或操作硬件。

基线：2026-11-27 13:06 UTC，`master`，HEAD `285269a`，含未提交及未跟踪文件。`S/` = `src/module/driver_module/`（64 文件，7,729 行）；`T/` = `test/module/driver_module/`（37 文件，6,386 行）。行号指本次工作树，只用于定位，请按函数或测试名称删除。先保全当前工作树，尤其是未跟踪的契约、设备及测试文件；不要以 HEAD 或 `git clean` 代替备份。

**数字口径：A 类 14 个文件 2,410 行 + B 类 3 个文件 307 行 + C 类 9 个函数 333 行 = 3,050 行已量化候选。** 计注释和空行。D、E 是关联项，不把混合文件整文件行数加进删除量。净差额要在修改和迁移测试之后才知道。

### A｜可以整文件退役：14 个。每组同时处理右列引用

|批次|逐文件清单（括号内为物理行数）|同批必须处理|
|---|---|---|
|A1 独立探针，779 行|`S/dcu/transport/igh_probe.cc` (57)；`S/dcu/transport/igh_data_probe.cc` (273)；`T/dcu/transport/igh_probe_test.cc` (123)；`T/dcu/transport/igh_data_probe_test.cc` (326)|移除 `EthercatProbeStage/Config`、`ethercat_probe`、`ProbeEthercat/ProbeEthercatData` 声明、初始化/启动分支和无 DCU 桩。删 `T/dcu/transport/CMakeLists.txt:43–51` 两个目标。保留生产的 PREOP 校验。|
|A2 空载 LRW 录制，441 行|`S/dcu/transport/lrw_transport.cc` (224)；`S/dcu/transport/lrw_timing.h` (117)；`T/dcu/transport/lrw_timing_test.cc` (100)|删 `T/CMakeLists.txt:52–54` 测试目标；移除 `OpenOverlap/ExchangeOverlap/CloseOverlap`、录制文件/采样行、旧解析和设备生命周期分支；`igh_transport.h` 仍引用该头和类型。保留正常 `ProfileClock`/DC 锁相、`cycle_schedule.h`。|
|A3 非 Profile 旧电机 SDO 初始化，947 行|`S/dcu/transport/hpm_setup.cc` (271)；`S/dcu/transport/hpm_setup.hpp` (157)；`T/dcu/transport/hpm_sdo_test.cc` (418)；`T/dcu/transport/hpm_setup_test.cc` (101)|去掉 `IghTransport::Open` 的 `ConfigureBoard()` 和旧头引用；先迁移 `igh_transport_test.cc` 的旧头、假函数和整帧断言。删 `T/dcu/transport/CMakeLists.txt:1,11–20`。**不是删整个 SDO 通道**。|
|A4 X2 双电机 ROS 集成运行器，243 行|`T/ros2_integration_test.py` (180)；`T/fake_runtime.py` (48)；`T/module/fake_test_pkg.cc` (15)|删 `T/CMakeLists.txt:28–35` 的 `driver_fake_test_pkg`，同步处理 B 类 runtime YAML 对旧包的引用。**不要连删测试硬件替身 `fake_devices.cc/.h`。**|

`S/dcu/transport/CMakeLists.txt:28`：在各批次移除 `igh_probe.cc`、`igh_data_probe.cc`、`lrw_transport.cc`、`hpm_setup.cc`，保留 `igh_transport.cc` 和 `single_fd_transport.cc`。保留 `DRIVER_MODULE_ENABLE_DCU=OFF` 的拒绝实现，不要删整个 `else()`。

### B｜先迁移消费者，再整文件删除：3 个，307 行

- [ ] `T/fixtures/schema5_fixture.cc` (231)：它替代了生产的 `LoadDriverConfig` 和 `ValidatePublicChannels`，不只是替代硬件。让保留测试链接 `driver_module::configuration` 的正式实现后，再从 `T/fixtures/CMakeLists.txt:1` 去掉这个源文件。仅在确实不再使用时清对应定义和链接依赖。
- [ ] `T/fixtures/driver_fake.yaml` (23)：`driver_module_test.cc:51`、`driver_execution_test.cc:82` 用它构造老式可运动的双轴配置。为仍保留的测试换成最小合法 WG3 输入，明确 `can_passthrough` 是 true 还是 false。
- [ ] `T/fixtures/runtime.yaml` (53)：旧 fake 包、关节指令/loopback 端点；`driver_profile_test.cc:10`、旧 Python 启动器等依赖它。若执行器测试仍需启动 AimRT，就保留最小测试 runtime 或由测试构造，不再要求旧关节通路。

### C｜可精确定位的局部删除：9 个函数，333 行

|文件|候选函数（行号）|小计|
|---|---|---:|
|`S/dcu/session/hpm_config.cc`|`ScalarOr` 27–34；`ProfileIndex` 36–47；`LoadProfileTiming` 50–70；`ParseConfig` 100–290。**保留 `ReadSchedule` 72–98 及其 `Scalar`、`Require`**；同步改 `hpm_config.h` 声明和旧测试。|232|
|`S/module/driver_cycle.cc`|`ValidateCommand` 62–72；`NativeCommand` 135–142；`StepDamping` 159–196；`JointState` 270–277。先拆 `Step`/测试对它们的调用。|65|
|`S/module/driver_module.cc`|`JointCmdCallback` 124–159。连同旧 `JointCommand` 订阅者移除，**保留 `FaceControl` 订阅与有效期检查**。|36|

### D｜生产混合文件：只清指定分支，禁止按文件名整删

|文件（相对 S/）|删除或收缩|不能一起删除|
|---|---|---|
|`configuration/driver_config.h`、`driver_config.cc`、`config_check.cc`|探针字段、旧关节/网关/阻尼参数及只为其打印的端点；以实际剩余调用裁剪。|正式 `LoadDriverConfig`、`SingleFrameDebug`、禁动校验、离线检查器的 **两个** 模式、外设命中和调度。`command_timeout` 仍用于头部命令。|
|`dcu/session/hpm_config.h`|旧解析函数声明、`overlap_*`、录制参数、`command_window_cycles`、`wkc_report_only` 的可选策略，以及只为旧可控轴语法服务的字段。|`LinkConfig`、`ProfileTiming`、观测路由、动态选择掩码。**反馈轴掩码不等同于命令授权掩码**；不要用前者替代后者。|
|`dcu/transport/igh_transport.cc/.h`、`igh_unavailable.cc`|A1/A2 的接口和状态；A3 的非 Profile 固定位置注册、固定映像整帧 `memcpy` 和旧配置入口。`igh_unavailable.cc:9–11,17–18` 是旧声明对应的桩。|`ReadMotorProfile`、`register_selected`、`ProfileImage::Encode/Decode`、`Receive/Send/ExchangeStop`、异常释放、无 DCU 的正常报错桩。|
|`dcu/dcu_driver.cc/.h`|旧 `overlap_idle` 特判、`WindowSent/WindowDone/WindowMatched` 计分、旧受控轴 `Enable/Write/WriteWithFallback/CommitCommands` 和专用准备缓冲区；相关调用和禁动测试同步处理。|`Start/Receive/Send`、`AttachSideband`、反馈可用性、锁相、只读电机状态、`RequestStop/StopCycle/Stop`。若移除 `Write`，把原测试的拒绝写入断言转成逐轴输出为零，不是取消禁动保证。|
|`dcu/session/profile_window.hpp`、`profile_schedule.hpp`|拆掉台架运动发令窗口的时长、计分、部分窗口日志；`profile_schedule.hpp` 中与之专用的统计仅在无读者后清。|`ProfileWindow::Inspect` 现在还能拒绝零窗口里的意外 `AxisStartMotion`；先保留或等效替代该拒绝和测试。保留在用的 `ProfileClock`、相位、准备时序与故障判断。|
|`dcu/session/hpm_control.cc/.hpp`|`EncodeHybrid`、`MakeMotionEntry`、`CheckMotionContinuation`、`MotionHandshake`、纯阻尼及旧可控轴准入分支；解除与在用诊断之间的引用后删除。去掉 report-only 特例。|`StartupHandshake`、`LinkHealth` 严格处理、Stop/复位确认。不要按函数前后行一刀切。|
|`dcu/session/hpm_session.cc/.h`|旧 `Prepare/Commit`、primary/fallback/entry 的可控轴槽、活动运动晋级和 report-only 缓存回退。|`Observe/Advance/BuildOutputs` 的主循环、采样序号、启动握手、Stop、头部旁带许可、故障锁存和只读观测。|
|`dcu/session/hpm_feedback.hpp`、`hpm_session_snapshot.cc`|非 Profile 的 `FeedbackTimestampFilter`/旧关节反馈读出及其专用统计；仅为旧运动状态存在的诊断字段和无调用的包装接口。|`PdoPublication`、原始发布时间/采样参考、重复帧不续期、`DecodeMotorStates`、`FormatFaultSummary` 以及故障快照。|
|`dcu/session/hpm_diagnostics.cc/.hpp`、`hardware/driver_types.h`|旧运动阶段标签、仅由可控轴消费的限位/命令数据，删前查剩余引用。|真实故障字段、时间、电机观测类型 `MotorState`、电流换算、仍被使用的 CAN 计数与诊断。即使结构字段物理上是零，也不能据此删除线上的 PDO 字段。|
|`module/driver_module.cc/.h`、`driver_cycle.cc`|C 类 5 个运动函数及其调用；旧 gateway、command policy、关节命令队列、`ApplyDirection`/`FillFeedback`/`Enable` 等仅有旧调用者的辅助件；`Configure` 中旧批次预分配；`Step` 中仅对旧关节命令的 watchdog。|AimRT 初始化、两条互斥路径、真实采集/发布、头部有效期与顺序、异常关停、故障捕获。不是删掉 `Step` 就算完成。|
|`module/driver_execution.cc`|`Start` 探针分支、动态 PDO 的旧关节 `driver_feedback_thread` 提交、`DriverTick` 里的 overlap 录制、旧关节队列锁与有界运动窗口收尾；清对应退出等待。|PDO 实时线程、非阻塞发布、运行/停止时序；CAN 模式的 `RunSingleFd` 和 `RunSingleFdPublisher` 是另一条实际通路。|
|`module/driver_publication.cc`、`module/publication_snapshot.h`、`module/driver_diagnostics.cc`|去无端点的关节 `feedback_frames_` 队列、`RunFeedbackPublisher`、旧阻尼/关节遥测；可去无消费者的 `PublicationSnapshot::sequence` 和 `publication_sequence_`。|独立 `observation_frames_`、`RunPublisher`、`PublishMotorStates`、CAN 发布线程、故障快照和最终错误报告；不要把软件发布序号与设备的 `publish_seq` 混为一谈。|
|`cfg/wg3.yaml`|只在代码不再请求旧反馈执行器后，移除 `driver_feedback_thread` 一项。|完整部署 YAML、自定义设备清单、头部和电机端点、事件 CAN 路由、`pdo/sdo` 调度、正常工作/发布执行器。|
|`configuration/CMakeLists.txt`、`module/CMakeLists.txt`、`dcu/session/CMakeLists.txt`、`hardware/CMakeLists.txt`|旧网关/消息/契约等依赖必须等最后一个包含或调用消失后，才从对应目标移除。尤其检查旧解析移除后 session 是否仍用 Profile 编译定义。|不能因为旧关节通路退出就删除公共库或公共消息包；按剩余头文件依赖构建验证。|

### E｜测试目录：混合用例的删除边界

下列行号是**测试定位区间，不是可执行的整段删除命令**。迁移 = 新输入或替身先能证明同一条生产性质，再移除旧断言。

|文件（相对 T/）|可随旧功能退出|保留或先迁移|
|---|---|---|
|`module/driver_module_test.cc`|116–200：旧阻尼、关节映射和关节命令时钟；229–241：原生极性往返；269–328：旧运动指令及阻尼；371–381：外置旧 DCU 配置；413–484、498–539：旧策略/限位/遥测参数。|15–21 生命周期；99–114 故障抓取后再诊断；201–228 一次停止、启动失败、禁动；242–268 保留收发失败和停止语义；329–339 电机反馈顺序；541–546 有符号电流。23–38、88–98、340–370、382–412、486–496 属混合：把离线不开硬件、拒绝非法清单/运动、陈旧反馈断言改测正式 WG3，不保留旧 YAML 正例。|
|`module/driver_execution_test.cc`|旧 `driver_fake.yaml` 装配、只测试旧关节命令锁/loopback 的细节、290–296 的旧 `connect_device=false` 配置。|36–73 周期与被中断的等待；153–211 改为生产 PDO/头部或事件 CAN 的入口竞争与慢消费者；213–288 绝对等待失败、实时条件失败、断链、停止失败、执行器约束；298–310 关闭后不再访问设备。|
|`module/driver_profile_test.cc`|8–17 旧 `/driver_test/joint_cmds` 配置；19–77 X2 13 可控轴清单；80–116 X2 极性运动闭环。|120–125 的“无 DCU 支持应离线拒绝且不打开设备”迁到正式 WG3 配置；之后若无用例才整删该文件。|
|`dcu/session/dcu_config_test.cc`|旧 `dcu_network`、axis defaults、型号限位、极性、固定多板和 X2 十三轴构造正例。|迁移 60–89 的目标故障边界、101–114 的只构造不开硬件、265–334 中非法路由、物理路由不依赖声明顺序、未知型号拒绝。先比对 `wg3_config_test.py` 已有等价覆盖；不要重复抄用例。|
|`dcu/session/dcu_control_test.cc`|42–509 中旧可控轴 query/增益/运动进入/继续及阻尼握手；833–870 的 WKC report-only 正例。|511–565 严格 WKC/链路/OP；636–831 启动、Stop、回执、复位边界。把仅依赖“已授权运动轴”的布置改成 WG3 板/通道，不删防误动、丢帧及故障拒绝断言。|
|`dcu/session/dcu_feedback_test.cc`|17–124 中非 Profile 量化时间过滤器和 gateway 专用部分；236–366 旧 filter 的缓存回退。|126–191、215–228 的首次/重复发布、回绕、倒序、时钟回退及采样参考；134–144 去 report-only 专用断言仍保留“重复不续期”；193–213 拆出周期采样参考，不保留旧网关前提。|
|`dcu/session/hpm_session_test.cc`|142–161、182–244、305–317 的旧发令准备/提交/运动晋级；263–292 的 report-only 专有行为。|121–140、163–180、246–261、294–303、319–374 中的启动、重复帧、严格 WKC、故障、Stop、采样时刻和反馈解码，改用 WG3 观测轴；只退役旧运动场景，不移除仍需的故障覆盖。|
|`dcu/session/dcu_diagnostics_test.cc`|专属 X2 轴名、运动相位/极性故障输出，例如 116–180、337–343 中对应断言。|30–114 的通用故障来源、真实观测和摘要；182–335 中仍在解码的计数有效性、回绕和不完整帧；对依赖旧运动 phase 的期望作局部调整。|
|`dcu/transport/igh_transport_test.cc`|43–77 的两块固定映像、`sizeof` 整帧比较、假 `ConfigureBoard`，不能代表动态 PDO。|把用例换成稀疏选择的实际映像；79–147 保留失败释放、DC 配置错误、WKC/收发错误、停止路径。对应 `T/dcu/transport/CMakeLists.txt:32–41` 也要让被测目标使用生产契约分支。|
|`fixtures/fake_devices.cc/.h`、`module/driver_module_test_peer.h`|旧可控轴运动回显、关节/阻尼/命令锁访问、已退役接口的桩。|受控的硬件边界替换、故障注入、停止计数、线程归属、关闭顺序。先让替身给出 WG3 需要的锁相/有效反馈，不能为了让假设备 ready 而放宽生产门槛。|
|`module/allocation_probe.cc/.h`|目前用于旧阻尼/网关分配测试；若不迁移实时无分配断言、且删除后无测试调用，才可删这两个文件及 `T/CMakeLists.txt:38` 的源列表项。|如仍要求确保 WG3 周期无分配，应复用于 WG3 有效工作负载，不因名字带 `probe` 误作 EtherCAT 探针删除。|
|`dcu/transport/wg3_selection_test.cc`、`devices/head/head_only_driver_test.cc`、`wg3_config_test.py`|不整删；把固定部署清单形状的假设限制在明确的参考配置用例。|前两者使用明确关闭透传的测试输入，先断言 `.dcu` 有值再取值。保留 PREOP 映射拒绝、无运动授权、头部有效期、多路 CAN 与设备模式互斥和正式加载器覆盖。|
|`CMakeLists.txt`、`fixtures/CMakeLists.txt`、`dcu/session/CMakeLists.txt`、`dcu/transport/CMakeLists.txt`|删上列退出的源、目标、add_test、旧夹具路径和已无读者的宏。`dcu/CMakeLists.txt` 仍需装配 session 和 transport，不整删。|保留当前 WG3 用例的构建入口，以及迁移后的驱动生命周期和传输失败测试。|

### 两个目录的其余文件：默认保留，不作为删除候选

- 生产：`S/CMakeLists.txt`、`S/dcu/CMakeLists.txt` 仍装配模块；`S/cfg/rk/production-contract/` 下 5 个契约头（含 `dcu_can_event.hpp`），`S/dcu/transport/{profile_binding.hpp,profile_image.hpp,igh_cycle.hpp}`，`S/dcu/session/{hpm_encos_pdo.hpp,pdo_cycle.h,sideband_cycle.hpp,single_fd.hpp}`。主机侧 wire 结构、动态 PDO 索引和契约不要手改。
- 生产：`S/devices/head/{face_command.hpp,face_pose.hpp}`、`S/devices/motor/{motor_device.cc,motor_device.h}`、`S/module/{cycle_schedule.h,driver_single_fd.cc}`、`S/dcu/transport/{single_fd_transport.cc,single_fd_transport.h,single_fd_unavailable.cc}`、`S/hardware/fault.h`。`single_fd` 是当前多路 CAN 的在用代码名，不表示旧单通道功能。
- 测试：`T/dcu/transport/{single_fd_test.cc,igh_cycle_test.cc,igh_fake_result.h}`、`T/devices/head/face_pdo_test.cc` 以及 E 类列出的保留用例。`igh_fake_result.h` 不是独立探针，切勿随 A1 连删。
- 范围外：不要删 `test/fixtures/x2/driver_module/driver_dcu_hpm.yaml`（仿真 `profile_test.py:16` 仍消费），不要删 `src/common/robot_gateway` 或 `CanFdFrame.msg`。`SwitchCanChannel.srv` 与其生成目标在两目录外，确认仓外 API 兼容性后另议。公共故障日志测试所依赖的已删 X2 配置应更换夹具，不应删除日志测试或恢复旧部署。已删除及 `build/` 生成物不列为待删源码。

### 手工执行及每批验收

1. **先保全未提交/未跟踪的工作，再固定离线输入。** 准备 PDO 模式、稀疏设备清单、事件 CAN 模式各一份最小输入。先证明保留用例在原基线上能表达：正式配置加载、不打开硬件、禁动、事件模式互斥。基线原有失败单列。
2. **按 A1 → A2 → A3 分批。** 每批同步清源、声明、分支、替身和 CMake；仓内搜索本批退役符号的剩余实现/调用，区别真正调用与否定断言。每批从仓库根的受支持构建入口编译 `driver_config_check`、`driver_module` 和受影响的离线测试。不要从 `driver_module` 子目录单独配置。
3. **A4、B、C、D/E 联动迁移。** 先让需要的用例能链接正式 `LoadDriverConfig`，再移除 Schema 5 符号与旧配置。剥离关节/阻尼/录制/旧反馈线程后，核对所有 future、通知、Close 和等待配对，避免出现关闭挂起。
4. **离线验收（RK 项目的离线目标）**：显式构建 `wg3_selection_test`、`head_only_driver_test`、`face_pdo_test`、`igh_cycle_test`、`driver_module_test`、`driver_execution_test` 及迁移后的 `igh_transport_test`；`wg3_config_test` 是 CTest 调用 Python 与 `driver_config_check`，`SingleFd.*` 位于 `driver_module_test`，不是各有同名独立可执行文件。保留或新建的用例要明确覆盖：稀疏 SM2/SM3、拒绝错误映射、透明帧多通道身份和长度、空心跳不重发、头部到期不再发送、无运动输出、严格 WKC、断链、重复反馈不续期、故障后停止和释放。检查重链实际包目标 `x2_deploy_pkg1`（先确认目标名仍在），避免仅静态库成功。不要运行生产启动脚本作为离线测试。
5. **实机验收是另一关。** 编译/假 IgH 测试通过不等于台架通过；涉及周期收发的修改需按项目台架规程、动力隔离和单独授权验证，不能自动开主站、发运动命令或重试。以上步骤均为待执行清单，本轮没有实施。
