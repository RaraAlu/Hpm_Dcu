#ifndef TEST_ECATSLV_H
#define TEST_ECATSLV_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint16_t u16SyncType; } test_sync_parameters_t;
extern test_sync_parameters_t sSyncManOutPar, sSyncManInPar;
extern bool bDcSyncActive;
extern bool bEcatInputUpdateRunning;
extern uint16_t nPdOutputSize;
extern uint16_t LatchInputSync0Value, LatchInputSync0Counter;
extern uint8_t nAlStatus;
extern uint8_t u8LocalErrorState;
extern uint16_t u16LocalErrorCode;
extern bool bEcatWaitForAlControlRes;
#define SYNCTYPE_DCSYNC0 2U
#define SYNCTYPE_SM_SYNCHRON 1U
#define SYNCTYPE_SM2_SYNCHRON 0x22U
#define STATE_SAFEOP 4U
#define STATE_INIT 1U
#define STATE_PREOP 2U
#define STATE_MASK 0x0fU
#define ALSTATUSCODE_NOERROR 0U
#define ALSTATUSCODE_UNSPECIFIEDERROR 1U
#define ALSTATUSCODE_SYNCERROR 0x1AU
#define ALSTATUSCODE_SMWATCHDOG 0x1BU
#define ALSTATUSCODE_SYNCHRONNOTSUPPORTED 0x28U
#define ALSTATUSCODE_DCINVALIDSYNCCFG 0x30U
#define ALSTATUSCODE_DCSYNC0CYCLETIME 0x36U
#define ESC_DC_SYNC_UNIT_ACTIVE_MASK 0x100U
#define ESC_DC_SYNC0_ACTIVE_MASK 0x200U
#define ESC_DC_SYNC1_ACTIVE_MASK 0x400U
#define ESC_DC_SYNC_UNIT_AUTO_ACTIVE_MASK 0x800U
#define ESC_DC_UNIT_CONTROL_OFFSET 0x980U
#define ESC_DC_SYNC0_CYCLETIME_OFFSET 0x9A0U
void ECAT_StateChange(uint8_t state, uint16_t error);
#endif
