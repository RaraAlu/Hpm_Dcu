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
