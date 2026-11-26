# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# If you do not want to use flash to simulate eeprom. It requests to regenerate
# the SSC code, please refer to the README
# Do not silently turn an old diagnostic build directory into a production image.
if(CONFIG_CAN_INTERNAL_LOOPBACK_TEST OR CONFIG_ETHERCAT_SCAN_TEST OR CONFIG_ETHERCAT_DATA_TEST OR
   CONFIG_ETHERCAT_MCAN_ECHO OR CONFIG_CAN_LED_TEST OR CONFIG_GATEWAY_STATIC_PROFILE OR
   CONFIG_GATEWAY_SESSION_TEST OR CONFIG_SYNC0_HW_CAPTURE_TEST OR CONFIG_CAN3_TDCO_AB_TEST)
    message(FATAL_ERROR "Test fixtures are retired; select a production preset in a fresh build directory.")
endif()
set(CONFIG_EEPROM_EMULATION 1)
set(CONFIG_FREERTOS 1)
set(CONFIG_FREERTOS_HEAP custom)

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

add_subdirectory(third_party/beckhoff_ssc)
add_subdirectory(firmware)
generate_ide_projects()
include(cmake/devtools.cmake)
