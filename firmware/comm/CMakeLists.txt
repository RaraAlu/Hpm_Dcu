sdk_inc(canfd ethercat ethercat/port)
sdk_src(
    canfd/ethercat_canfd_gateway.c
    ethercat/digital_io.c
    ethercat/encos_storage.c
    ethercat/port/hpm_ecat_hw.c
    ethercat/port/hpm_ecat_phy.c
)
if(CONFIG_EEPROM_EMULATION)
    sdk_src(ethercat/port/hpm_ecat_e2p_emulation.c)
endif()
