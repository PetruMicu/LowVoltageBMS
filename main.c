/*==================================================================================================
*   Project              : LVBMS
==================================================================================================*/

#ifdef __cplusplus
extern "C"{
#endif


/*==================================================================================================
 *                                        INCLUDE FILES
==================================================================================================*/

#include "LVBMS.h"


/*==================================================================================================
 *                                          CONSTANTS
 ==================================================================================================*/

/*==================================================================================================
 *                                      DEFINES AND MACROS
 ==================================================================================================*/
/*Stack sizes in 32bit words for each LVBMS tasks*/
#define MEASUREMENTS_STACK_SIZE     256
#define CONTROL_STACK_SIZE          256
#define FAULTHANDLER_STACK_SIZE     256
/*==================================================================================================
 *                                            ENUMS
 ==================================================================================================*/

/*==================================================================================================
 *                                STRUCTURES AND OTHER TYPEDEFS
 ==================================================================================================*/

/*==================================================================================================
 *                                GLOBAL VARIABLE DECLARATIONS
 ==================================================================================================*/
/* Semaphore used to send and receive fault status from battery cell controller*/
SemaphoreHandle_t xFaultSemaphore;
/*Mutex used for device communication to ensure exclusive access to Bms_TD_Send*/
SemaphoreHandle_t xCommMutex;

/*Task handlers for LVBMS tasks*/
TaskHandle_t xMeasureHandler;
TaskHandle_t xControlHandler;
TaskHandle_t xFaultHandler;
TaskHandle_t xFaultLED_Handler;
/*==================================================================================================
 *                                    FUNCTION PROTOTYPES
 ==================================================================================================*/ 
 /*LVBMS tasks*/
void LVBMS_MeasureTask(void* parameters);
void LVBMS_ControlTask(void* parameters);
void LVBMS_FaultHandlerTask(void* parameters);
void LVBMS_FaultLEDHandlerTask(void* parameters);
/*==================================================================================================
 *                                    GLOBAL FUNCTIONS
 ==================================================================================================*/

int main(void)
{
	/*Initialize all drivers used by LVBMS*/
	LVBMS_Init();

    /*Create semaphore for fault signaling*/
    xFaultSemaphore = xSemaphoreCreateBinary();
    /*Create mutex used for device communication*/
    xCommMutex = xSemaphoreCreateMutex();
    
    /*Create LVBMS tasks*/
    xTaskCreate(LVBMS_MeasureTask,
                "Measurements Task",
                MEASUREMENTS_STACK_SIZE,
                NULL_PTR,
                1U,
                &xMeasureHandler);
    xTaskCreate(LVBMS_ControlTask,
                "Control Task",
                CONTROL_STACK_SIZE,
                NULL_PTR,
                1U,
                &xControlHandler);
     xTaskCreate(LVBMS_FaultHandlerTask,
                 "Fault Handler Task",
                 FAULTHANDLER_STACK_SIZE,
                 NULL_PTR,
                 1U,
                 &xFaultHandler);
     xTaskCreate(LVBMS_FaultLEDHandlerTask,
				 "FaultLED Task",
				 128U,
				 NULL_PTR,
				 1U,
				 &xFaultLED_Handler);

     vTaskSuspend(xFaultLED_Handler);

    /*Start FreeRTOS Scheduler*/
    vTaskStartScheduler();
}
#ifdef __cplusplus
}
#endif

/** @} */
