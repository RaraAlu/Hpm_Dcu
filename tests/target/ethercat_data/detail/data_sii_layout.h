#ifndef DCU_DATA_SII_LAYOUT_H
#define DCU_DATA_SII_LAYOUT_H

#include "../cfg/data_config.h"

#define DCU_DATA_SM(addr, size, ctrl, type) DCU_DATA_LE16(addr), DCU_DATA_LE16(size), ctrl, 0, 1, type
#define DCU_DATA_SII_ENTRY(index, sub) DCU_DATA_LE16(index), sub, 0, 7, 32, 0, 0
#define DCU_DATA_SII_RX(sub) DCU_DATA_SII_ENTRY(DCU_DATA_RX_OBJECT, sub),
#define DCU_DATA_SII_TX(sub) DCU_DATA_SII_ENTRY(DCU_DATA_TX_OBJECT, sub),
#define DCU_DATA_PDO_HEADER(category, mapping, count, sm) \
    DCU_DATA_LE16(category), DCU_DATA_LE16(4U * ((count) + 1U)), \
    DCU_DATA_LE16(mapping), count, sm, 0, 0, 0, 0

#endif
