#include "bench_esc.h"
#include "hpm_ecat_phy.h"
#include "hpm_interrupt.h"

bool dcu_bench_sii_prepare_read(dcu_bench_sii_reader_t reader, uint32_t word, unsigned size)
{
    uint64_t data;
    if (!reader(word, size, &data)) return false;
    esc_write_eeprom_data(HPM_ESC, data);
    return true;
}

bool dcu_bench_sii_prepare_reload(dcu_bench_sii_reader_t reader)
{
    uint64_t config;
    uint64_t alias;
    if (!reader(0, 4U, &config) || !reader(4, 4U, &alias)) return false;
    uint8_t flags = (uint8_t)(config >> 8);
    uint32_t data = (uint16_t)alias;
    data |= (uint32_t)ESC_ESC_CFG_ELDAP_GET(flags) << 16;
    data |= (uint32_t)ESC_ESC_CFG_ELP0_GET(flags) << 17;
    data |= (uint32_t)ESC_ESC_CFG_ELP1_GET(flags) << 18;
    data |= (uint32_t)ESC_ESC_CFG_ELP2_GET(flags) << 19;
    data |= (uint32_t)ESC_ESC_CFG_ELP3_GET(flags) << 20;
    esc_write_eeprom_data(HPM_ESC, data);
    return true;
}

bool dcu_bench_sii_service(dcu_bench_sii_reader_t reader)
{
    uint16_t status = HPM_ESC->EEPROM_CTRL_STAT;
    if (!(status & ESC_EEPROM_CTRL_STAT_EE_EMU_MASK) ||
        !(status & ESC_EEPROM_CTRL_STAT_BUSY_MASK)) return false;

    uint8_t command = ESC_EEPROM_CTRL_STAT_CMD_GET(status);
    bool ok = false;
    if (command == esc_eeprom_read_cmd) {
        unsigned size = status & ESC_EEPROM_CTRL_STAT_NUM_RD_BYTE_MASK ? 8U : 4U;
        ok = dcu_bench_sii_prepare_read(reader, esc_get_eeprom_word_address(HPM_ESC), size);
    } else if (command == esc_eeprom_reload_cmd) {
        ok = dcu_bench_sii_prepare_reload(reader);
    } else if (command == esc_eeprom_idle_cmd) {
        ok = true;
    }
    /* WRITE and unsupported commands are rejected; there is no backing store. */
    esc_eeprom_emulation_ack(HPM_ESC, command, !ok, false);
    return ok && command == esc_eeprom_reload_cmd;
}

hpm_stat_t dcu_bench_esc_start(dcu_bench_sii_reader_t reader, unsigned timeout_ms)
{
    board_init_ethercat(HPM_ESC);
    intc_m_disable_irq(IRQn_ESC);
    intc_m_disable_irq(IRQn_ESC_SYNC0);
    intc_m_disable_irq(IRQn_ESC_SYNC1);
    intc_m_disable_irq(IRQn_ESC_RESET);
    esc_disable_irq(HPM_ESC, esc_irq_mask_all);
    esc_eeprom_clock_config_t config = {
        .core_clock_en = true,
        .phy_refclk_en = true,
        .eeprom_emulation = true,
        .eeprom_size_over_16kbit = false,
    };
    esc_config_eeprom_and_clock(HPM_ESC, &config);
    HPM_ESC->PDI_AL_EVT_MSK = 0U;
    bool reloaded = false;
    for (unsigned ms = 0; ms < timeout_ms; ++ms) {
        reloaded |= dcu_bench_sii_service(reader);
        if (reloaded && (HPM_ESC->ESC_DL_STAT & ESC_ESC_DL_STAT_EPLC_MASK)) break;
        board_delay_ms(1);
    }
    if (!reloaded || !(HPM_ESC->ESC_DL_STAT & ESC_ESC_DL_STAT_EPLC_MASK)) return status_timeout;

    /* Initialize AL before releasing the PHYs, so the master sees INIT. */
    HPM_ESC->AL_STAT_CODE = 0;
    HPM_ESC->AL_STAT = 1U;
    ecat_phy_reset();
    esc_set_phy_offset(HPM_ESC, BOARD_ECAT_PHY_ADDR_OFFSET);
    hpm_stat_t status = ecat_phy_config(HPM_ESC);
    if (status != status_success) return status;
    esc_config_ctrl_signal_function(HPM_ESC, BOARD_ECAT_NMII_LINK0_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link0, BOARD_ECAT_PORT0_LINK_INVERT);
#if BOARD_ECAT_SUPPORT_PORT1
    esc_config_ctrl_signal_function(HPM_ESC, BOARD_ECAT_NMII_LINK1_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link1, BOARD_ECAT_PORT1_LINK_INVERT);
#endif
#if BOARD_ECAT_SUPPORT_PORT2
    esc_config_ctrl_signal_function(HPM_ESC, BOARD_ECAT_NMII_LINK2_CTRL_INDEX,
                                    esc_ctrl_signal_func_alt_nmii_link2, BOARD_ECAT_PORT2_LINK_INVERT);
#endif
    esc_config_nmii_link_source(HPM_ESC, true, BOARD_ECAT_SUPPORT_PORT1, BOARD_ECAT_SUPPORT_PORT2);
    return status_success;
}
