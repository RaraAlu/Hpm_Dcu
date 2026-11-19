set(_ecat_port "${PROJECT_SOURCE_DIR}/firmware/comm/ethercat/port")
sdk_inc(. ../ethercat_bench "${_ecat_port}" "${PROJECT_SOURCE_DIR}/firmware/cfg" "${PROJECT_SOURCE_DIR}/firmware/ports")
sdk_src("${_ecat_port}/hpm_ecat_phy.c" ../ethercat_bench/bench_esc.c scan_contract.c)
sdk_app_src(main.c)
