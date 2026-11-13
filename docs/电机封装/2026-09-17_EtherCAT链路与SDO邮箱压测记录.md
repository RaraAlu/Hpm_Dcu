# EtherCAT 链路与 HPM SDO 邮箱压测记录（2026-09-17）

## 范围与现场

本记录仅保存 2026-09-17 的现场只读诊断及不触发电机运动的 SDO 压测结果，供后续继续定位；本次未修改主站配置、HPM 固件或上位机代码。压测由用户授权的系统弹窗完成，未在聊天或文件中记录密码。

- 主站：IgH EtherCAT 1.6.9，PREEMPT_RT 内核 `6.18.49-igh-rt3`，`CONFIG_HZ=1000`；EtherCAT 网卡 `enP3p49s0`，RTL8125B、`r8169` 驱动、`ec_generic` 模块，协商速率 100 Mb/s 全双工。
- 从站：一块 HPM 28 轴板，扫描时处于 PREOP，ProductCode `0x00000002`、Revision `0x00000002`；邮箱 RX `0x1000/128` 字节、TX `0x1080/128` 字节。CAN1 接有 6 个电机。
- 板上**实际烧录固件版本未核对**；以下固件时序分析仅基于仓库源码，不证明板上运行的就是该版本。

## 复现与测量

| 检查 | 实测结果 | 边界 |
|---|---|---|
| 上位机多轴连接 | `SDO读失败0x3057:04 ret=-5 abort=0x00000000`。内核对应日志为 `Received unknown response while uploading SDO 0x3057:04`，数据 `00 30 60 57 30 03 08 00 00 00` | `0x60` 是先前对同一对象 `:03` 写操作的 CoE download ACK；读 `:04` 时出现旧 ACK。`abort=0` 表示没有取得明确的 CoE Abort，不应解释成电机参数拒绝。 |
| CLI 串行读取 | 读取身份与六轴 DoneSeq，210/210 成功 | 证明低强度访问可成功，不证明时序无问题。 |
| CLI 同值写/读 | 对各轴 `0x3007+0x10*a:01` 写回原 ParameterId，然后读 `:04`，180/180 成功 | 同值写不提交电机参数，不会触发运动。 |
| CLI 真实参数查询 | 六轴各进行 ParameterId=3 的 Kt 查询，共 30/30 成功 | 仅查询，不下发运动命令。 |
| 与上位机相同的 IgH ecrt API，同进程立即写后读 | 六轴各 200 轮，共 1200 对；1179 成功、21 失败，失败分布于六轴，约 1.75% | 写入的是原有 ParameterId `0x000A`，不提交新参数；失败为 `-EIO`、`abort=0`。 |
| 同进程纯读取 | 1200 次中 1199 成功、1 次失败 | 写后读比纯读更易复现。 |
| 同进程写后等待 20 ms 再读 | 1200 对中 1179 成功、21 失败 | 固定 20 ms 等待未解决问题。 |

高强度测试期间内核同时出现 `Received unknown response while uploading SDO`、`Received upload response for wrong SDO` 及 CoE 上传/下载 `No response`。旧写 ACK 落入新读事务是**已观测现象**；究竟由 IgH/通用网卡路径中的迟到报文、ESC 邮箱重发/排队，还是两者共同造成，目前没有收发时间戳或 ESC 邮箱跟踪，不能单独归因于 HPM 固件。

## 链路时序对照

上位机未运行且 IgH master 为 Idle 时，`ethercat master` 连续约 10 秒两次采样：主网卡 Tx/Rx 分别从 `964436/964435` 增至 `968647/968646`，各增加 4211 帧；Common `Lost frames` 保持 421 不变。`ethercat crc` 的 HPM Port 0 CRC、PHY、FWD 均为 0，第二次采样未增加。

另一次独立的 12 秒窗口，NIC Tx/Rx 分别从 `15988509/15988013` 增至 `15993754/15993258`，均增加 5245 帧；`tx_errors=2`、`rx_errors=76`、`align_errors=76` 均未增长。同一窗口内核仍多次成对记录 `datagram TIMED OUT` 与 `UNMATCHED`。这支持“报文到达 IgH 时已超过其匹配期限”的判断，不支持把该窗口的告警直接解释成新增 CRC 错误或永久丢帧。历史 NIC 对齐错误和当晚更早的两次链路 DOWN/UP 仍需单独排查，不能据此排除线缆、接头或供电问题。

主站源码 `ethercat/master/globals.h` 的名义 `EC_IO_TIMEOUT` 为 500 µs；本机构建的 `ethercat/config.h` 未启用 `EC_HAVE_CYCLES`，实际使用 jiffies 判定，且内核 HZ=1000，超时阈值至少按 1 个节拍量化，不能把上述告警直接称为“精确超过 500 µs”。IgH `ethercat/master/master.c` 会在超时后将该 datagram 出队，迟到响应因此可能被报为 `UNMATCHED`。

## 固件源码待验证点

- `hpm/HPM_ethercat_canfd_encos_V1.0/ecat.c` 的主循环执行 `MainLoop()`；SSC `SSC/Src/ecatappl.c::MainLoop()` 在 PREOP/自由运行条件下先执行 `ECAT_Application()`，随后 `ECAT_Main()`；`SSC/Src/ecatslv.c::ECAT_Main()` 内执行 `MBX_Main()`。
- `application/digital_io.c::APPL_Application()` 调用 `application/motor_control.c::motor_control_process()`。后者从入口到返回保持全局中断关闭，遍历四路 CAN，每路最多处理 32 个接收帧并进行调度。因此在相应运行条件下可能延后邮箱服务；**尚未实测执行时长，也未核对板上固件**。
- EtherCAT 帧经过 ESC 的处理是硬件 on-the-fly 路径，不能仅凭上述 CPU 关中断段解释 Idle 状态的 datagram 超时。邮箱 PDI 处理延迟与主站/NIC 收帧延迟应分开验证。

## 下次继续时的建议顺序

1. 保留这次配置作基线，换一块专用 EtherCAT 网卡，优先选择有 IgH 原生驱动的型号；重复 Idle 收发计数、`TIMED OUT/UNMATCHED` 统计及六轴同进程 SDO 压测，判断 `ec_generic`/RTL8125B 路径的影响。不要仅凭一次成功连接认定修复。
2. 对照板上固件构建版本；在 HPM 侧记录邮箱写事件、CoE 请求处理、应答写入及 `motor_control_process()` 全局关中断的最长耗时，核对失败时是否真的产生重复或迟到的 `0x60` ACK。
3. 若换网卡后仍复现，再用主站收发时间戳与 ESC 邮箱日志配对事务、索引、子索引及邮箱计数器；同时复查先前链路掉线时的线缆、接头和供电。

截至本记录写入时，**根因未闭环**；不宜只靠增加 SDO 固定等待时间或把所有失败归因于 CAN 电机超时。
