/*==================================================================================================
*   Project              : LVBMS
==================================================================================================*/

#ifdef __cplusplus
extern "C"{
#endif


/*==================================================================================================
 *                                        INCLUDE FILES
==================================================================================================*/
#include <math.h>
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
#define VMIN                            17        /*Minimum allowed stack voltage*/
#define VMAX                            25        /*Maximum recomended stack voltage*/
#define MAX_CHARGE_CURRENT              15000
#define MAX_DISCHARGE_CURRENT           -15000

#define RSHUNT					        10000U    /* Shunt resistor value in uOhm*/
#define VCT_ANX_RES                     152.58789 /*Resolution used for voltage measurements in uV/LSB*/
#define VPRW_RES                        2.44148   /*Resolution used for stack voltage measurements in mV/LSB*/
#define C_RATED                         15000U    /*Maximal capacity of the battery pack in mAh*/
#define MAX_VOLTAGE_DIFF                0.8       /*Maximal allowed cell voltage difference*/

#define CELL_VOLTAGE_VALUE(x)           ((x & 0x7fff) * VCT_ANX_RES / 1000000U)
#define STACK_VOLTAGE_VALUE(x)          ((x & 0x7fff) * VPRW_RES / 1000U)

#define VCT_UV_RES                      19.53125 /*Cell undervoltage threshold resolution in mv/LSB*/
#define MC33772C_ALL_CT_UV_TH_VAL       ((uint16)(2.85 * 1000U / VCT_UV_RES) & 0x7fff)

/*Temperature calculation*/
#define RNTC                            10000U    /*NTC Resistor value at 25 *C*/
#define RVOLTAGE_DEVIDER                6800U     /*Value of the resistor used in the voltage divider with the NTC*/
#define BETA                            3380U     /*Beta Value specified in datasheet*/
#define CONVERT_KELVIN_TO_CELSIUS(x)    (x - 273.15)
#define CONVERT_CELCIUS_TO_KELVIN(x)    (x + 273.15)            

/*Delay in ms*/
#define DELAY(x)                	    (x / portTICK_PERIOD_MS)
#define MEASUREMENTS_DELAY              DELAY(5U)
#define FAULT_HANDLER_DELAY             DELAY(100U)
#define LED_FAULT_DELAY                 DELAY(300U)
/*==================================================================================================
 *                                            ENUMS
 ==================================================================================================*/
 /*Used for debugging*/
typedef enum
{
    NO_ERROR = 0U,
    ERROR_PHY_TD_SEND,
    ERROR_TD_FILL,
    ERROR_ENUMERATE,
    ERROR_CONFIGURE,
    ERROR_INITIAL_ESTIMATION
} LVBMS_ErrorType;

typedef enum
{
	NO_FAULT = 0U,
	FAULT
}LVBMS_FaultStatusType;

typedef enum
{
	IDLE = 0U,
	CHARGING,
    DISCHARGING
}LVBMS_StateType;

typedef enum
{
    CELL_OK = 0U,
    CELL_OV,
    CELL_UV,
}LVBMS_CellFaultStatusType;
/*==================================================================================================
 *                                STRUCTURES AND OTHER TYPEDEFS
 ==================================================================================================*/
typedef struct
{
    LVBMS_CellFaultStatusType CTState[ACTIVE_CELL_NUMBER];
    uint8 CTBalEnabled[ACTIVE_CELL_NUMBER];
}LVBMS_CellStateType;

typedef struct
{
    float Current;
    float StackVoltage;
    float CTVoltage[ACTIVE_CELL_NUMBER];
    double PackTemperature;
    float DeltaQ;
} LVBMS_MeasurementsType;

/*==================================================================================================
 *                                GLOBAL VARIABLE DECLARATIONS
 ==================================================================================================*/

/*==================================================================================================
 *                                    FUNCTION PROTOTYPES
 ==================================================================================================*/ 
void LVBMS_Init(void);
LVBMS_ErrorType LVBMS_SystemConfig(void);
Std_ReturnType LVBMS_StartMeasurements(void);
Std_ReturnType LVBMS_ReadMeasurements(LVBMS_MeasurementsType* Measurements);
LVBMS_FaultStatusType LVBMS_CheckFault(void);
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
