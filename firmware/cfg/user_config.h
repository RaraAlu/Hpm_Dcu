/*
 * Copyright (c) 2024,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/* this file for eeprom emulation component */

#ifndef _USER_CONFIG_H
#define _USER_CONFIG_H

#include "ecat_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define E2P_DEBUG_LEVEL        E2P_DEBUG_LEVEL_WARN /* if use E2P_DEBUG_LEVEL_INFO, excessive printf */
#define E2P_MAX_VAR_CNT        (1100) /* 1024 legacy words + 64 SII pages + config/backup */
#define E2P_FLUSH_BUF_SIZE     (4096)

#ifdef __cplusplus
}
#endif

#endif
