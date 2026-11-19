sdk_inc(. ${PROJECT_SOURCE_DIR}/firmware/cfg ${PROJECT_SOURCE_DIR}/firmware/ports)
sdk_src(can_internal_loopback.c)
sdk_app_src(main.c)
