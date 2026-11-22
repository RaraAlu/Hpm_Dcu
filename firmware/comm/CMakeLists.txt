sdk_inc(canfd ethercat ethercat/port)
sdk_compile_definitions(-DDCU_ECAT_CYCLE_PORT=1)
option(CONFIG_CAN3_TDCO_AB_TEST "Use TDCO=13 on CAN3 for the production endurance comparison" OFF)
if(CONFIG_CAN3_TDCO_AB_TEST)
    sdk_compile_definitions(-DCONFIG_CAN3_TDCO_AB_TEST=1)
endif()
sdk_src(
    canfd/hpm_mcan_transport.c
    canfd/can_tx_gate.c
    ethercat/ethercat_callbacks.c
    ethercat/encos_storage.c
    ethercat/motor_dictionary.cpp
    ethercat/motor_coe.cpp
    ethercat/motor_adapter.cpp
    ethercat/motor_pdo.cpp
    ethercat/motor_services.cpp
    ethercat/port/hpm_ecat_hw.c
    ethercat/port/hpm_ecat_phy.c
)
if(CONFIG_EEPROM_EMULATION)
    sdk_src(ethercat/port/hpm_ecat_e2p_emulation.c)
endif()
