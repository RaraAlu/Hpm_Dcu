# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# If you do not want to use flash to simulate eeprom. It requests to regenerate
# the SSC code, please refer to the README
option(CONFIG_CAN_INTERNAL_LOOPBACK_TEST "Build the isolated MCAN0 internal loopback image" OFF)
option(CONFIG_ETHERCAT_SCAN_TEST "Build the isolated EtherCAT discovery image" OFF)
option(CONFIG_SYNC0_HW_CAPTURE_TEST "Measure ESC Sync0 with a spare GPTMR channel" OFF)
option(CONFIG_ETHERCAT_DATA_TEST "Build the isolated EtherCAT data echo image" OFF)
option(CONFIG_ETHERCAT_MCAN_ECHO "Route diagnostic PDOs through internal MCAN0" OFF)
option(CONFIG_GATEWAY_STATIC_PROFILE "Build the offline static profile gateway" OFF)
option(CONFIG_GATEWAY_SESSION_TEST "Build the offline static profile session gate" OFF)
option(CONFIG_CAN_LED_TEST "Build the blocking CAN three-color LED cycle test" OFF)
if(CONFIG_ETHERCAT_MCAN_ECHO AND NOT CONFIG_ETHERCAT_DATA_TEST)
    message(FATAL_ERROR "The internal MCAN echo is only supported by the EtherCAT data test.")
endif()
if(CONFIG_GATEWAY_STATIC_PROFILE AND NOT CONFIG_ETHERCAT_MCAN_ECHO)
    message(FATAL_ERROR "The static profile is only supported by the isolated MCAN echo.")
endif()
if(CONFIG_GATEWAY_SESSION_TEST AND NOT CONFIG_GATEWAY_STATIC_PROFILE)
    message(FATAL_ERROR "The gateway session test requires the isolated static profile.")
endif()
if(CONFIG_ETHERCAT_DATA_TEST AND (CONFIG_ETHERCAT_SCAN_TEST OR CONFIG_CAN_INTERNAL_LOOPBACK_TEST OR CONFIG_CAN_LED_TEST))
    message(FATAL_ERROR "EtherCAT data echo cannot be combined with other test images.")
endif()
if(CONFIG_ETHERCAT_SCAN_TEST AND (CONFIG_CAN_INTERNAL_LOOPBACK_TEST OR CONFIG_CAN_LED_TEST))
    message(FATAL_ERROR "EtherCAT discovery and CAN test images are mutually exclusive.")
endif()
if(CONFIG_CAN_INTERNAL_LOOPBACK_TEST AND CONFIG_CAN_LED_TEST)
    message(FATAL_ERROR "CAN internal loopback and LED test images are mutually exclusive.")
endif()
if(CONFIG_CAN_INTERNAL_LOOPBACK_TEST OR CONFIG_ETHERCAT_SCAN_TEST OR CONFIG_ETHERCAT_DATA_TEST)
    set(CONFIG_EEPROM_EMULATION 0)
    set(CONFIG_FREERTOS 0)
else()
    set(CONFIG_EEPROM_EMULATION 1) # using eeprom emulation compenent
    set(CONFIG_FREERTOS 1)
    set(CONFIG_FREERTOS_HEAP custom)
endif()

if("${HPM_BUILD_TYPE}" STREQUAL "")
    SET(HPM_BUILD_TYPE flash_xip)
endif()


set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/third_party/hpm_sdk")
set(DCU_BOARD "dcu_hpm5e31")
if(BOARD AND NOT "${BOARD}" STREQUAL "${DCU_BOARD}")
    message(FATAL_ERROR "This firmware supports only ${DCU_BOARD}.")
endif()
set(BOARD ${DCU_BOARD})
set(BOARD_SEARCH_PATH "${CMAKE_CURRENT_SOURCE_DIR}/firmware/bsp")


find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(dcu LANGUAGES C CXX ASM)

sdk_compile_definitions(-DUSE_SYSCALL_INTERRUPT_PRIORITY=1)
sdk_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>"
                    "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>")
set_property(TARGET ${APP_ELF_NAME} PROPERTY LINKER_LANGUAGE CXX)

if(CONFIG_CAN_INTERNAL_LOOPBACK_TEST)
    add_subdirectory(tests/target/can_internal_loopback)
elseif(CONFIG_ETHERCAT_SCAN_TEST)
    add_subdirectory(tests/target/ethercat_scan)
elseif(CONFIG_ETHERCAT_DATA_TEST)
    add_subdirectory(third_party/beckhoff_ssc)
    add_subdirectory(tests/target/ethercat_data)
else()
    add_subdirectory(third_party/beckhoff_ssc)
    add_subdirectory(firmware)
    if(CONFIG_CAN_LED_TEST)
        add_subdirectory(tests/target/can_led)
    endif()
endif()
generate_ide_projects()
include(cmake/devtools.cmake)
