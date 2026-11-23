sdk_inc(cycle runtime gateway profile boot_identity)
set(_tools "${PROJECT_SOURCE_DIR}/tools/protocol")
set(DCU_CAN_PROFILE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/profile/cfg/生产CAN配置.yaml"
    CACHE FILEPATH "Production CAN Profile YAML")
set(_can_profile_header "${CMAKE_CURRENT_BINARY_DIR}/dcu_can_profile_generated.h")
set(_contract_files
    firmware/generated/encos/encos_interface.h firmware/generated/encos/encos_storage.inc
    firmware/generated/ethercat/digital_ioObjects.h firmware/generated/ethercat/gatewayObjects.h
    firmware/generated/ethercat/eeprom.h protocol/ethercat/dcu_profile_pdo.hpp protocol/ethercat/dcu_igh_layout.hpp
    protocol/ethercat/周期布局.json
    protocol/ethercat/HPM_ENCOS_4CAN_28AXIS_IMU.xml protocol/ethercat/HPM_ENCOS_4CAN_28AXIS_IMU.bin)
list(TRANSFORM _contract_files PREPEND "${DCU_PROTOCOL_ROOT}/")
add_custom_command(OUTPUT "${_can_profile_header}" ${_contract_files}
    COMMAND "${PYTHON_EXECUTABLE}" "${_tools}/generate_can_profile.py"
        --profile "${DCU_CAN_PROFILE_FILE}" --output "${_can_profile_header}"
    COMMAND "${PYTHON_EXECUTABLE}" "${_tools}/generate.py"
        --profile "${DCU_CAN_PROFILE_FILE}" --output-root "${DCU_PROTOCOL_ROOT}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${PROJECT_SOURCE_DIR}/protocol/ethercat/dcu_igh_layout.hpp"
        "${DCU_PROTOCOL_ROOT}/protocol/ethercat/dcu_igh_layout.hpp"
    DEPENDS "${DCU_CAN_PROFILE_FILE}" "${_tools}/generate_can_profile.py"
        "${PROJECT_SOURCE_DIR}/protocol/ethercat/dcu_igh_layout.hpp"
        "${_tools}/check_gateway_contract.py" "${_tools}/generate.py"
        "${_tools}/cfg/接口定义.json" "${_tools}/cfg/设备模板.xml"
    COMMENT "Generating CAN configuration and the HPM/RK cyclic contract from one Profile"
    VERBATIM)
add_custom_target(dcu_can_profile_header DEPENDS "${_can_profile_header}" ${_contract_files})
add_dependencies(app dcu_can_profile_header)
add_dependencies(hpm_sdk_lib dcu_can_profile_header)
sdk_inc("${CMAKE_CURRENT_BINARY_DIR}")
sdk_src(
    production_init.cpp
    gateway/gateway_application.cpp
    gateway/gateway_publication.cpp
    gateway/can/raw_layout.cpp
    gateway/can/can_bus_scheduler.cpp
    gateway/can/can_transactions.cpp
    gateway/can/raw_can_gateway.cpp
    profile/production_profile.cpp
    gateway/startup/production_startup.cpp
    gateway/policy/profile_update.cpp
    gateway/startup/gateway_readiness.c
    boot_identity/gateway_identity.c
    boot_identity/port/gateway_id_flash.c
)
sdk_src(cycle/pdo_pipeline.c cycle/ethercat_cycle.c)
option(CONFIG_PDO_PHASE_TRACE "Trace PDO phase boundaries by command sequence" OFF)
if(CONFIG_PDO_PHASE_TRACE)
    sdk_compile_definitions(-DCONFIG_PDO_PHASE_TRACE=1)
endif()
option(CONFIG_CAN2_FIRST_PDO_TRACE "Capture the first four-axis CAN2 production MIT PDO" OFF)
if(CONFIG_CAN2_FIRST_PDO_TRACE)
    sdk_compile_definitions(-DCONFIG_CAN2_FIRST_PDO_TRACE=1)
endif()
if(CONFIG_SYNC0_HW_CAPTURE_TEST)
    sdk_compile_definitions(-DCONFIG_SYNC0_HW_CAPTURE_TEST=1)
    sdk_src(cycle/sync0_hw_capture.c)
endif()
sdk_src(runtime/ethercat_can_task.c)
include(runtime/hpm_freertos_context.cmake)
sdk_app_src(main.c)
