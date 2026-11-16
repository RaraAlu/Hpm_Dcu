#ifndef TEST_ECATSLV_H
#define TEST_ECATSLV_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint16_t u16SyncType; } test_sync_parameters_t;
extern test_sync_parameters_t sSyncManOutPar, sSyncManInPar;
extern bool bDcSyncActive;
extern uint16_t nPdOutputSize;
#define SYNCTYPE_DCSYNC0 2U
#define STATE_SAFEOP 4U
#define ALSTATUSCODE_NOERROR 0U
#define ALSTATUSCODE_SYNCERROR 0x1AU
#define ALSTATUSCODE_SYNCHRONNOTSUPPORTED 0x28U
#define ALSTATUSCODE_DCINVALIDSYNCCFG 0x30U
#define ALSTATUSCODE_DCSYNC0CYCLETIME 0x36U
#define ESC_DC_SYNC_UNIT_ACTIVE_MASK 0x100U
#define ESC_DC_SYNC0_ACTIVE_MASK 0x200U
#define ESC_DC_SYNC1_ACTIVE_MASK 0x400U
#define ESC_DC_UNIT_CONTROL_OFFSET 0x980U
#define ESC_DC_SYNC0_CYCLETIME_OFFSET 0x9A0U
void ECAT_StateChange(uint8_t state, uint16_t error);
#endif
