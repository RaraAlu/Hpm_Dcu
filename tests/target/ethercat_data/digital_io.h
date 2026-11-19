#ifndef DCU_DATA_SSC_APPLICATION_H
#define DCU_DATA_SSC_APPLICATION_H

#include "ecat_def.h"
#include "objdef.h"

extern TOBJECT ApplicationObjDic[];
void APPL_AckErrorInd(UINT16 transition);
UINT16 APPL_StartMailboxHandler(void);
UINT16 APPL_StopMailboxHandler(void);
UINT16 APPL_StartInputHandler(UINT16 *int_mask);
UINT16 APPL_StopInputHandler(void);
UINT16 APPL_StartOutputHandler(void);
UINT16 APPL_StopOutputHandler(void);
UINT16 APPL_GenerateMapping(UINT16 *input_size, UINT16 *output_size);
void APPL_OutputMapping(UINT16 *data);
void APPL_InputMapping(UINT16 *data);
void APPL_Application(void);
UINT16 APPL_GetDeviceID(void);

#endif
