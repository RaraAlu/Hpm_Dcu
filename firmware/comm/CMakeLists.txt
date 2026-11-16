sdk_inc(canfd ethercat ethercat/port)
sdk_src(
    canfd/ethercat_canfd_gateway.c
    canfd/can_tx_gate.c
    ethercat/ethercat_callbacks.c
    ethercat/encos_storage.c
    ethercat/motor_dictionary.cpp
    ethercat/motor_coe.cpp
    ethercat/motor_pdo.cpp
    ethercat/port/hpm_ecat_hw.c
    ethercat/port/hpm_ecat_phy.c
)
if(CONFIG_EEPROM_EMULATION)
    sdk_src(ethercat/port/hpm_ecat_e2p_emulation.c)
endif()
