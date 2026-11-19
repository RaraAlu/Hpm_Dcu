set(_ecat_port "${PROJECT_SOURCE_DIR}/firmware/comm/ethercat/port")
sdk_inc(. ../ethercat_bench "${_ecat_port}" "${PROJECT_SOURCE_DIR}/firmware/cfg"
    "${PROJECT_SOURCE_DIR}/firmware/ports"
    "${PROJECT_SOURCE_DIR}/third_party/hpm_sdk/components/eeprom_emulation")
sdk_src("${_ecat_port}/hpm_ecat_phy.c" ../ethercat_bench/bench_esc.c
    data_contract.c ssc_app.c ssc_bench_hw.c)
sdk_app_src(main.c)
sdk_ld_options("-Wl,--wrap=OBJ_Write")
if(CONFIG_ETHERCAT_MCAN_ECHO)
    sdk_compile_definitions(-DDCU_DATA_BACKEND_MCAN=1)
    sdk_src(mcan_echo.c)
endif()
if(CONFIG_GATEWAY_STATIC_PROFILE)
    set(_gateway "${PROJECT_SOURCE_DIR}/firmware/app/gateway")
    set(_tools "${PROJECT_SOURCE_DIR}/tools/protocol")
    set(DCU_STATIC_PROFILE_FILE "${_gateway}/cfg/单路内部回环配置.yaml" CACHE FILEPATH "Static gateway Profile YAML")
    set(_generated "${CMAKE_CURRENT_BINARY_DIR}/dcu_static_profile_generated.h")
    set(_profile_args)
    if(CONFIG_GATEWAY_SESSION_TEST)
        list(APPEND _profile_args --require-session)
        sdk_compile_definitions(-DDCU_DATA_SESSION_GATE=1)
        set(_nor "${PROJECT_SOURCE_DIR}/third_party/hpm_sdk/components/eeprom_emulation/port")
        sdk_inc("${_nor}")
        sdk_src("${_gateway}/gateway_readiness.c" "${_gateway}/gateway_task.c"
            "${_gateway}/gateway_identity.c" session_bridge.c "${_gateway}/port/gateway_id_flash.c" "${_nor}/hpm_nor_flash.c")
    endif()
    add_custom_command(OUTPUT "${_generated}"
        COMMAND "${PYTHON_EXECUTABLE}" "${_tools}/generate_static_profile.py"
            --profile "${DCU_STATIC_PROFILE_FILE}" --output "${_generated}" ${_profile_args}
        DEPENDS "${DCU_STATIC_PROFILE_FILE}" "${_tools}/generate_static_profile.py"
            "${_tools}/check_gateway_contract.py"
        COMMENT "Generating the isolated static gateway Profile"
        VERBATIM)
    add_custom_target(dcu_static_profile_header DEPENDS "${_generated}")
    add_dependencies(app dcu_static_profile_header)
    add_dependencies(hpm_sdk_lib dcu_static_profile_header)
    sdk_compile_definitions(-DDCU_DATA_STATIC_PROFILE=1)
    sdk_inc("${_gateway}" "${CMAKE_CURRENT_BINARY_DIR}")
    sdk_src("${_gateway}/static_profile.c")
endif()
