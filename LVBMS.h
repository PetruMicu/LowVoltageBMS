/*==================================================================================================
*   Project              : LVBMS
==================================================================================================*/

#ifdef __cplusplus
extern "C"{
#endif


/*==================================================================================================
 *                                        INCLUDE FILES
==================================================================================================*/

/* Including necessary configuration files. */
#include "Mcu.h"
#include "Spi.h"
#include "Platform.h"
#include "Port.h"
#include "Dio.h"
#include "Icu.h"
#include "Uart.h"
#include "CDD_Bms_SpiIf.h"
#include "CDD_Bms_common_Types.h"
#include "CDD_Bms_Common.h"
#include "CDD_Bcc_772c.h"

/* Including FreeRTOS*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Including FreeMASTER Driver */
#include "freemaster.h"
#include "freemaster_cfg.h"
#include "freemaster_s32k3xx_lpuart.h"

#ifndef LVBMS_H
#define LVBMS_H

/*==================================================================================================
 *                                          CONSTANTS
 ==================================================================================================*/

/*==================================================================================================
 *                                      DEFINES AND MACROS
 ==================================================================================================*/
#define SPI_BCC_CHAIN_ADDR		        1U        /*Chain allocated for Battery Cell Controller*/
#define SPI_BCC_DEVICE_ADDR		        1U        /*Device address allocated for Battery Cell Controller*/

#define ACTIVE_CELL_NUMBER              6U        /*Number of cell used by the LVBMS (max: 6)*/
#define VMIN                            15        /*Minimum allowed stack voltage*/
#define VMAX                            24        /*Maximum recomended stack voltage*/
#define IMIN                            100       /*Current value when fully charged*/
#define RSHUNT					        10000U    /* Shunt resistor value in uOhm*/
#define VCT_ANX_RES                     152.58789 /*Resolution used for voltage measurements in uV/LSB*/
#define VPRW_RES                        2.44148   /*Resolution used for stack voltage measurements in mV/LSB*/
#define C_RATED                         15000U    /*Maximal capacity of the battery pack in mAh*/
#define MAX_VOLTAGE_DIFF                0.8       /*Maximal allowed cell voltage difference*/
#define CHARGE_OPERATING_EFFICIENCY     1.0
#define DISCHARGE_OPERATING_EFFICIENCY  1.0

#define CELL_VOLTAGE_VALUE(x)        ((x & 0x7fff) * VCT_ANX_RES / 1000000U)
#define STACK_VOLTAGE_VALUE(x)       ((x & 0x7fff) * VPRW_RES / 1000U)
#define DELAY(x)                	 (x / portTICK_PERIOD_MS)

/*Delay in ms*/
#define MEASUREMENTS_DELAY      DELAY(5U)
#define FAULT_HANDLER_DELAY     DELAY(100U)
#define LED_FAULT_DELAY         DELAY(300U)
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

typedef enum
{
	NO_FAULT = 0U,
	FAULT
}LVBMS_FaultStatus;

typedef enum
{
	IDLE = 0U,
	CHARGING,
    DISCHARGING
}LVBMS_State;

typedef enum
{
    CELL_OK = 0U,
    CELL_OV,
    CELL_UV,
}LVBMS_CellFaultStatus;
/*==================================================================================================
 *                                STRUCTURES AND OTHER TYPEDEFS
 ==================================================================================================*/
typedef struct
{
    LVBMS_CellFaultStatus CTState[ACTIVE_CELL_NUMBER];
    uint8 CTBalEnabled[ACTIVE_CELL_NUMBER];
}LVBMS_CellState;

typedef struct
{
    float Current;
    float StackVoltage;
    float CTVoltage[ACTIVE_CELL_NUMBER];
    float PackTemperature;
    float DeltaQ;
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
LVBMS_FaultStatus LVBMS_CheckFault(void);
Std_ReturnType LVBMS_ClearFault(void);
void LVBMS_CellBalanceControl(void);

void FaultEvent(void);

/*LVBMS tasks*/
void LVBMS_MeasureTask(void* parameters);
void LVBMS_ControlTask(void* parameters);
void LVBMS_FaultHandlerTask(void* parameters);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* LVBMS_H */
