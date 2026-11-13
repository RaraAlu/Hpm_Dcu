# 源码导入基线

日期：2026-11-13。

来源：同级 `../HPM_ethercat_canfd_encos_V1.0/`。用户要求先复制为独立工程，再进行后续梳理整理优化。

通过文件复制建立独立副本，没有将源码或 SDK 链接回原目录。没有修改原目录。

排除项：原 `.vscode/`、`.codegraph/`、Windows 生成目录 `hpm5e00evk_flash_xip_debug/`、Python 缓存。旧 `.vscode` 数据库约 1 GiB 且其配置为 ROS2，不属于本工程的新开发环境。

本轮相对业务源码的必要修补只有：按现有函数定义补齐缺失的 `application/motor_control.h`。没有改动电机控制实现、PDO、对象字典、SSC 协议栈、SDK 驱动、板级引脚或时序参数。

根 `CMakeLists.txt` 新增 `cmake/devtools.cmake`，接入构建后镜像检查、测试及显式硬件目标。其余新增项为 Presets、锁定工具、开发脚本、离线测试、VS Code 配置和开发说明。

核对结果：`application/` 相比来源只新增上述头文件；`SSC/` 内容相同；SDK 已有文件的内容与来源相同。当前目录尚未初始化 Git 或创建提交，避免把环境搭建与版本管理决策混在一起。

离线编译成功不等于板上烧录或硬件验收。后续优化应以本目录为工作副本，分离协议/调度改动、工具升级和硬件验收。
