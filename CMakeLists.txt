# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

# If you do not want to use flash to simulate eeprom. It requests to regenerate
# the SSC code, please refer to the README
set(CONFIG_EEPROM_EMULATION 1) # using eeprom emulation compenent

if("${HPM_BUILD_TYPE}" STREQUAL "")
    SET(HPM_BUILD_TYPE flash_xip)
endif()


# the following lines before "### END ###" are added during project localization
### START ###
set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/hpm_sdk_localized_for_hpm5e00evk")
set(LOCALIZED_BOARD "hpm5e00evk")
if(BOARD)
  if(NOT ${BOARD} MATCHES ${LOCALIZED_BOARD})
    message(FATAL_ERROR "ABORT:\n hpm sdk has been localized for ${LOCALIZED_BOARD} already.")
  endif()
endif()
set(BOARD ${LOCALIZED_BOARD})
### END ###


find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(ecat_io)

option(CONFIG_CAN_LED_TEST "Build the blocking CAN three-color LED cycle test" OFF)

sdk_inc(SSC/Src)
sdk_src(SSC/Src/coeappl.c)
sdk_src(SSC/Src/ecatappl.c)
sdk_src(SSC/Src/ecatcoe.c)
sdk_src(SSC/Src/ecatslv.c)
sdk_src(SSC/Src/mailbox.c)
sdk_src(SSC/Src/objdef.c)
sdk_src(SSC/Src/sdoserv.c)

sdk_inc(port)
sdk_src(port/hpm_ecat_hw.c)
sdk_src(port/hpm_ecat_phy.c)
if(CONFIG_EEPROM_EMULATION)
sdk_src(port/hpm_ecat_e2p_emulation.c)
endif()

sdk_inc(application)
sdk_src(application/digital_io.c)
# EtherCAT PDO 与四路 MCAN 之间的实时透明透传层
sdk_src(application/ethercat_canfd_gateway.c)
sdk_src(application/encos_motor.c application/motor_control.c)
sdk_link_libraries(m)
if(CONFIG_CAN_LED_TEST)
#sdk_compile_definitions(-DCONFIG_CAN_LED_TEST=0)
sdk_src(application/can_led_test.c application/can_led_test.h)
endif()

sdk_inc(.)
sdk_app_src(ecat.c)
generate_ide_projects()
include(cmake/devtools.cmake)
