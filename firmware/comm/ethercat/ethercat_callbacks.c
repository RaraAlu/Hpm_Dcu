/*
* This source file is part of the EtherCAT Slave Stack Code licensed by Beckhoff Automation GmbH & Co KG, 33415 Verl, Germany.
* The corresponding license agreement applies. This hint shall not be removed.
*/

/**
\addtogroup digital_io digital_io
@{
*/

/**
\file ethercat_callbacks.c
\brief EtherCAT application callbacks for the DCU motor gateway.
 Created with SSC Tool application parser 1.6.4.0
\version 0.0.0.1
*/


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecat_def.h"

#include "applInterface.h"

#include "detail/ethercat_dictionary_owner.h"

#include "ecat_config.h"
#include "hpm_gpio_drv.h"
#include "motor_ethercat.h"
#include "ethercat_cycle.h"
#include "encos_interface.h"
#include "hpm_ecat_e2p_emulation.h"

/* Fixed SSC process image; the ESC destination is still selected by SM3. */
extern UINT16 aPdInputData[MAX_PD_INPUT_SIZE >> 1];

void *dcu_ssc_txpdo_buffer(uint16_t *capacity)
{
    *capacity = sizeof(aPdInputData);
    return aPdInputData;
}


UINT32 APPL_GetDipSw(void);
void APPL_SetLed(UINT32 value);
/*--------------------------------------------------------------------------------------
------
------    local types and defines
------
--------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/


/**
 \brief    The function is called when an error state was acknowledged by the master
*/

void    APPL_AckErrorInd(UINT16 stateTrans)
{
    if (stateTrans == SAFEOP_2_SAFEOP)
        dcu_cycle_ack_safeop_error();
}


/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
             all general settings were checked to start the mailbox handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case this function will be called cyclically
            until a value unequal NOERROR_INWORK is returned

*/

UINT16 APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
             to stop the mailbox handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*/

UINT16 APPL_StopMailboxHandler(void)
{
    motor_control_discard_profile();
    return ALSTATUSCODE_NOERROR;
}

/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                       register (0x204) when this function is succeeded. The event mask can be adapted
                       in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
           all general settings were checked to start the input handler. This function
           informs the application about the state transition, the application can refuse
           the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete
           the transition by calling ECAT_StateChange.
*/

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    (void)pIntMask;
    motor_control_discard_profile();
    return dcu_cycle_configure();
}

/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*/

UINT16 APPL_StopInputHandler(void)
{
    dcu_cycle_stop_inputs();
    return ALSTATUSCODE_NOERROR;
}

/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete
           the transition by calling ECAT_StateChange.
*/

UINT16 APPL_StartOutputHandler(void)
{
    UINT16 result = dcu_cycle_start_outputs();
    if (result != ALSTATUSCODE_NOERROR)
        return result;
    /* 只有 EtherCAT 进入 OP 后才允许向电机总线发送周期帧。 */
    motor_control_operational(true);
    return ALSTATUSCODE_NOERROR;
}

/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*/

UINT16 APPL_StopOutputHandler(void)
{
    dcu_cycle_stop_outputs();
    /* OP→SAFEOP 时立即阻止旧 PDO 继续驱动电机。 */
    motor_control_operational(false);
    return ALSTATUSCODE_NOERROR;
}

/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*/

UINT16 APPL_GenerateMapping(UINT16 *pInputSize, UINT16 *pOutputSize)
{
    /* 按已应用的产品 PDO 档位生成大小；运行会话内禁止切换档位。 */
    return motor_control_mapping(pInputSize,pOutputSize);
}

/**
\param      pData  pointer to input process data

\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*/
void APPL_InputMapping(UINT16 *pData)
{
    /* The cycle task fills aPdInputData before publishing it to SM3. */
    (void)pData;
}

/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*/
void APPL_OutputMapping(UINT16 *pData)
{
    /* 只保存 EtherCAT 输出快照，CAN 发送放到应用周期执行。 */
    dcu_cycle_stage(pData);
}

/**
\brief    This function will called from the synchronisation ISR
            or from the mainloop if no synchronisation is supported
*/
void APPL_Application(void)
{
    /* 周期任务负责 CAN 调度与 SSC 工作；Sync0 只交接周期事件。 */
}

#if EXPLICIT_DEVICE_ID

/**
 \return    The Explicit Device ID of the EtherCAT slave

 \brief     Read the Explicit Device ID (from an external ID switch)
*/
UINT16 APPL_GetDeviceID(void)
{
    UINT16 configured = ((mc_board_identity_t *)mc_object(0x2004))->ExplicitDeviceId;
    UINT16 serial = (UINT16)ecat_flash_eeprom_serial();
    return configured != 0U ? configured : (serial != 0U ? serial : (UINT16)(APPL_GetDipSw() + 1U));
}
#endif

void APPL_SetLed(UINT32 value)
{
#if BOARD_ECAT_SUPPORT_DIGITAL_IO
    UINT8 led0 = ((value & 1) ?  BOARD_ECAT_OUT_ON_LEVEL : !BOARD_ECAT_OUT_ON_LEVEL);
    UINT8 led1 = ((value & 2) ?  BOARD_ECAT_OUT_ON_LEVEL : !BOARD_ECAT_OUT_ON_LEVEL);

    gpio_write_pin(BOARD_ECAT_OUT1_GPIO, BOARD_ECAT_OUT1_GPIO_PORT_INDEX, BOARD_ECAT_OUT1_GPIO_PIN_INDEX, led0);
    gpio_write_pin(BOARD_ECAT_OUT2_GPIO, BOARD_ECAT_OUT2_GPIO_PORT_INDEX, BOARD_ECAT_OUT2_GPIO_PIN_INDEX, led1);
#else
    (void) value;
#endif
}

UINT32 APPL_GetDipSw(void)
{
#if BOARD_ECAT_SUPPORT_DIGITAL_IO
    UINT8 pin0_level = gpio_read_pin(BOARD_ECAT_IN1_GPIO, BOARD_ECAT_IN1_GPIO_PORT_INDEX, BOARD_ECAT_IN1_GPIO_PIN_INDEX);
    UINT8 pin1_level = gpio_read_pin(BOARD_ECAT_IN2_GPIO, BOARD_ECAT_IN2_GPIO_PORT_INDEX, BOARD_ECAT_IN2_GPIO_PIN_INDEX);

    UINT32 val = pin0_level << 0 | pin1_level << 1;

    return val;
#else
    return 0;
#endif
}

#if USE_DEFAULT_MAIN

/**

 \brief    This is the main function

*/
#if _PIC24 && EL9800_HW
int main(void)
#elif _WIN32
int main(int argc, char *argv[])
#else
void main(void)
#endif
{
    /* initialize the Hardware and the EtherCAT Slave Controller */
#if FC1100_HW
#if _WIN32
    u16FcInstance = 0;
    if (argc > 1) {
        u16FcInstance = atoi(argv[1]);
    }
#endif
    if (HW_Init()) {
        HW_Release();
        return;
    }
#else
    HW_Init();
#endif
    MainInit();

    bRunApplication = TRUE;
    do {
        MainLoop();
    } while (bRunApplication == TRUE);

    HW_Release();
#if _PIC24
    return 0;
#endif
}
#endif /* #if USE_DEFAULT_MAIN */
/** @} */
