#ifndef DCU_GATEWAY_IDENTITY_FORMAT_H
#define DCU_GATEWAY_IDENTITY_FORMAT_H

#define DCU_ID_MAGIC UINT32_C(0x31444947)
enum { DCU_ID_SLOTS = DCU_ID_SECTOR_BYTES / DCU_ID_PAGE_BYTES };

typedef struct {
    uint32_t epoch;
    unsigned active_sector;
    unsigned next_slot[DCU_ID_SECTORS];
    bool found;
} dcu_gateway_id_scan_t;

#endif
