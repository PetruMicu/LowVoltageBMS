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

#include "LVBMS.h"


/*==================================================================================================
 *                                          CONSTANTS
 ==================================================================================================*/

/*==================================================================================================
 *                                      DEFINES AND MACROS
 ==================================================================================================*/
 /*Queue is created to hold a maximum of 10 measurement cycles*/
#define QUEUE_LENGTH    10U

/*Stack sizes in 32bit words for each LVBMS tasks*/
#define MEASUREMENTS_STACK_SIZE    256
#define DASHBOARD_STACK_SIZE       256
#define FAULTHANDLER_STACK_SIZE    256
/*==================================================================================================
 *                                            ENUMS
 ==================================================================================================*/

/*==================================================================================================
 *                                STRUCTURES AND OTHER TYPEDEFS
 ==================================================================================================*/

/*==================================================================================================
 *                                GLOBAL VARIABLE DECLARATIONS
 ==================================================================================================*/
/* Queue used to send and receive measurements from battery cell controller*/
QueueHandle_t xMeasurementsQueue;
/*==================================================================================================
 *                                    FUNCTION PROTOTYPES
 ==================================================================================================*/ 
 /*LVBMS tasks*/
void LVBMS_MeasureTask(void* parameters);
void LVBMS_DashboardTask(void* parameters);
void LVBMS_FaultHandlerTask(void* parameters);
/*==================================================================================================
 *                                    GLOBAL FUNCTIONS
 ==================================================================================================*/

int main(void)
{
	/*Create task handlers for LVBMS tasks*/
	TaskHandle_t xMeasureHandler;
	TaskHandle_t xDashboardHandler;
	TaskHandle_t xFaultHandler;

    /*Initialize all drivers used by LVBMS*/
    LVBMS_Init();

    /*Create measurement queue for synchronisation between measurement and dashboard tasks*/
    xMeasurementsQueue = xQueueCreate(QUEUE_LENGTH,sizeof(LVBMS_Measurements));

    /*Create LVBMS tasks*/
    xTaskCreate(LVBMS_MeasureTask,
                "Measurements Task",
                MEASUREMENTS_STACK_SIZE,
                NULL_PTR,
                1U,
                &xMeasureHandler);
    xTaskCreate(LVBMS_DashboardTask,
                "Dashboard Task",
                DASHBOARD_STACK_SIZE,
                NULL_PTR,
                2U,
                &xMeasureHandler);
    // xTaskCreate(LVBMS_FaultHandlerTask,
    //             "Fault Handler Task",
    //             FAULTHANDLER_STACK_SIZE,
    //             NULL_PTR,
    //             0U,
    //             &xMeasureHandler);
    
    /*Start FreeRTOS Scheduler*/
    vTaskStartScheduler();
}
#ifdef __cplusplus
}
#endif

/** @} */
