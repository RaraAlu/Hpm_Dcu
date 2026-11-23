set(DCU_PROTOCOL_ROOT "${CMAKE_CURRENT_BINARY_DIR}/profile-contract")
sdk_inc(cfg ports "${DCU_PROTOCOL_ROOT}/firmware/generated/encos" "${DCU_PROTOCOL_ROOT}/firmware/generated/ethercat"
    "${DCU_PROTOCOL_ROOT}/protocol/ethercat")

add_subdirectory(app)
add_subdirectory(comm)
add_subdirectory(protocols/encos)
add_subdirectory(protocols/bjcx)
add_subdirectory(devices)
