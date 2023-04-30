/*==================================================================================================
*   Project              : BMS SDK AUTOSAR 4.4
*   Platform             : CORTEXM
*   Peripheral           : 
*   Dependencies         : Phy_665a
*
*   Autosar Version      : 4.4.0
*   Autosar Revision     : ASR_REL_4_4_REV_0000
*   Autosar Conf.Variant :
*   SW Version           : 1.0.0
*   Build Version        : S32K3_BMS_SDK_1_0_0_D2304_ASR_REL_4_4_REV_0000_20230413
*
*   (c) Copyright 2020 - 2023 NXP Semiconductors
*   All Rights Reserved.
*
*   NXP Confidential. This software is owned or controlled by NXP and may only be
*   used strictly in accordance with the applicable license terms. By expressly
*   accepting such terms or by downloading, installing, activating and/or otherwise
*   using the software, you are agreeing that you have read, and that you agree to
*   comply with and are bound by, such license terms. If you do not agree to be
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
==================================================================================================*/

#ifdef __cplusplus
extern "C"{
#endif


/*==================================================================================================
 *                                        INCLUDE FILES
 * 1) system and project includes
 * 2) needed interfaces from external units
 * 3) internal and external interfaces from this unit
==================================================================================================*/

/* Including necessary configuration files. */
#include "Mcu.h"
#include "Spi.h"
#include "Platform.h"
#include "Port.h"
#include "Dio.h"
#include "CDD_Bms_SpiIf.h"
#include "CDD_Bms_common_Types.h"
#include "CDD_Bms_Common.h"
#include "CDD_Bcc_772c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>

#ifndef LVBMS_H
#define LVBMS_H

/*==================================================================================================
 *                                          CONSTANTS
 ==================================================================================================*/

/*==================================================================================================
 *                                      DEFINES AND MACROS
 ==================================================================================================*/
#define SPI_BCC_CHAIN_ADDR		1U
#define SPI_BCC_DEVICE_ADDR		1U
/*==================================================================================================
 *                                            ENUMS
 ==================================================================================================*/
typedef enum
{
    NO_ERROR = 0U,
    ERROR_PHY_TD_SEND,
    ERROR_TD_FILL,
    ERROR_ENUMERATE,
    ERROR_CONFIGURE
} LVBMS_Error;
/*==================================================================================================
 *                                STRUCTURES AND OTHER TYPEDEFS
 ==================================================================================================*/
typedef struct
{
    float Current;
    float StackVoltage;
    float CT1Voltage;
    float CT2Voltage;
    float CT3Voltage;
    float CT4Voltage;
    float CT5Voltage;
    float CT6Voltage;
    float PackTemperature;
} LVBMS_Measurements;
/*==================================================================================================
 *                                GLOBAL VARIABLE DECLARATIONS
 ==================================================================================================*/

/*==================================================================================================
 *                                    FUNCTION PROTOTYPES
 ==================================================================================================*/ 
void LVBMS_Init(void);
LVBMS_Error LVBMS_SystemConfig(void);
Std_ReturnType LVBMS_StartMeasurements(void);
Std_ReturnType LVBMS_ReadMeasurements(LVBMS_Measurements* Measurements);
void TD_Wait(const Bms_TDType* MessageTD);

/*LVBMS tasks*/
void LVBMS_MeasureTask(void* parameters);
void LVBMS_DashboardTask(void* parameters);
void LVBMS_FaultHandlerTask(void* parameters);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* LVBMS_H */
