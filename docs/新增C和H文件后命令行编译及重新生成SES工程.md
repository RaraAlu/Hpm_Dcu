# 新增 C/H 文件后命令行编译及重新生成 SES 工程

## 1. 适用工程

本文适用于：

```text
E:\BOT_Procedure\BOT_ethercat_canfd_HPM_V1.0\HPM_ethercat_canfd_encos_V1.0
```

HPM SDK 环境目录为：

```text
F:\HPMicro_develop\SDK\sdk_env_v1.12.1
```

如果 SDK 安装位置不同，需要修改下文的 `$sdkEnv`。

## 2. 新增 C/H 文件后修改 CMakeLists.txt

例如在 `application` 目录新增：

```text
application/example.c
application/example.h
```

在工程根目录的 `CMakeLists.txt` 中登记包含路径和文件：

```cmake
sdk_inc(application)
sdk_src(application/example.c application/example.h)
```

- `.c` 必须登记，否则不会参与编译，也不会被写入自动生成的 SES 工程。
- `.h` 不产生目标文件，但登记后会显示在 SES 工程树中，便于索引和跳转。
- `sdk_inc(application)` 使其他目录可以直接使用 `#include "example.h"`。
- 整个固件只能保留一个 `main()`。普通功能文件只提供函数，由主入口调用。

如果文件只在某个配置下使用，可以写成：

```cmake
option(CONFIG_EXAMPLE "Build example" OFF)

sdk_inc(application)
if(CONFIG_EXAMPLE)
sdk_compile_definitions(-DCONFIG_EXAMPLE=1)
sdk_src(application/example.c application/example.h)
endif()
```

文件登记完成后，保留工程末尾的：

```cmake
sdk_app_src(ecat.c)
generate_ide_projects()
```

`generate_ide_projects()` 负责生成 IAR 和 SEGGER Embedded Studio 工程。

## 3. PowerShell 环境设置

打开 PowerShell，执行：

```powershell
$sdkEnv = 'F:\HPMicro_develop\SDK\sdk_env_v1.12.1'
$projectDir = 'E:\BOT_Procedure\BOT_ethercat_canfd_HPM_V1.0\HPM_ethercat_canfd_encos_V1.0'
$buildDir = 'hpm5e00evk_flash_xip_debug'

$env:GNURISCV_TOOLCHAIN_PATH = "$sdkEnv\toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win"
$env:PATH = "$sdkEnv\tools\cmake\bin;$env:GNURISCV_TOOLCHAIN_PATH\bin;$sdkEnv\tools\python3;$sdkEnv\tools\Python3\Scripts;$sdkEnv\tools\ninja;$sdkEnv\tools\openocd;$env:PATH"

Set-Location $projectDir
```

这些环境变量只对当前 PowerShell 窗口有效，重新打开窗口后需要再次设置。

## 4. 重新配置并生成 SES 工程

以 CAN LED 测试模式为例：

```powershell
& "$sdkEnv\tools\cmake\bin\cmake.exe" `
    -S . `
    -B $buildDir `
    -G Ninja `
    -DHPM_BUILD_TYPE=flash_xip `
    -DCONFIG_CAN_LED_TEST=ON `
    -DCMAKE_MAKE_PROGRAM="$sdkEnv\tools\ninja\ninja.exe"
```

执行成功后，SES 工程位于：

```text
hpm5e00evk_flash_xip_debug\segger_embedded_studio\ecat_io.emProject
```

不要长期手工修改自动生成的 `ecat_io.emProject`，下一次执行 CMake 时手工修改会被覆盖。源文件、包含路径和宏定义应写在 `CMakeLists.txt` 中。

## 5. 命令行编译

生成工程后执行：

```powershell
& "$sdkEnv\tools\cmake\bin\cmake.exe" --build $buildDir
```

成功后主要输出文件为：

```text
hpm5e00evk_flash_xip_debug\output\demo.elf
hpm5e00evk_flash_xip_debug\output\demo.bin
```

如果只是修改已有 `.c/.h` 内容，没有新增文件、删除文件或修改 CMake 配置，通常只需要执行上述 `--build` 命令。

如果修改了 `CMakeLists.txt`，`--build` 一般会自动重新运行 CMake；为了明确刷新 SES 工程，建议先执行第 4 节的配置命令，再执行编译命令。

## 6. CAN LED 测试开关

生成并编译 LED 测试配置：

```powershell
& "$sdkEnv\tools\cmake\bin\cmake.exe" -S . -B $buildDir -G Ninja `
    -DHPM_BUILD_TYPE=flash_xip `
    -DCONFIG_CAN_LED_TEST=ON `
    -DCMAKE_MAKE_PROGRAM="$sdkEnv\tools\ninja\ninja.exe"

& "$sdkEnv\tools\cmake\bin\cmake.exe" --build $buildDir
```

恢复 EtherCAT 正常程序：

```powershell
& "$sdkEnv\tools\cmake\bin\cmake.exe" -S . -B $buildDir -G Ninja `
    -DHPM_BUILD_TYPE=flash_xip `
    -DCONFIG_CAN_LED_TEST=OFF `
    -DCMAKE_MAKE_PROGRAM="$sdkEnv\tools\ninja\ninja.exe"

& "$sdkEnv\tools\cmake\bin\cmake.exe" --build $buildDir
```

由于 CMake 选项会保存在 `CMakeCache.txt` 中，切换测试和正式程序时应明确传入 `ON` 或 `OFF`。

## 7. 本次 CAN LED 文件配置

本次新增文件：

```text
application/can_led_test.c
application/can_led_test.h
```

对应的 `CMakeLists.txt` 配置为：

```cmake
option(CONFIG_CAN_LED_TEST "Build the CAN three-color LED cycle test" ON)

sdk_inc(application)
if(CONFIG_CAN_LED_TEST)
sdk_compile_definitions(-DCONFIG_CAN_LED_TEST=1)
sdk_src(application/can_led_test.c application/can_led_test.h)
endif()

sdk_app_src(ecat.c)
generate_ide_projects()
```

`ecat.c` 是唯一主入口，通过 `can_led_test.h` 中的声明调用 `can_led_test_run()`，因此不存在两个 `main()`。

## 8. 常见错误

### 8.1 `GNURISCV_TOOLCHAIN_PATH is not set yet`

说明没有执行第 3 节的环境设置，或者工具链目录填写错误。

### 8.2 新文件没有出现在 SES 工程中

依次检查：

1. 文件是否已写入 `sdk_src(...)`；
2. 是否重新执行第 4 节的 CMake 配置命令；
3. 打开的是否是当前构建目录下最新生成的 `ecat_io.emProject`；
4. 条件编译选项是否为 `ON`。

### 8.3 头文件找不到

将头文件所在目录加入：

```cmake
sdk_inc(application)
```

然后重新生成并编译。

### 8.4 出现多个 `main` 定义

只允许 `ecat.c` 使用 `sdk_app_src(...)` 并定义 `main()`。其他测试文件应定义普通函数，例如：

```c
void example_run(void);
```

再由 `ecat.c` 根据配置宏调用。

### 8.5 CMake 无法写入构建目录

先关闭正在占用工程文件的 SES、VS Code 或其他构建进程，再重新执行配置命令。仍然失败时检查构建目录的只读属性和当前用户写权限。
