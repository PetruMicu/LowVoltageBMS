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
extern QueueHandle_t xMeasurementsQueue;
/*Counter used to keep track of battery cell controller's responses*/
uint8 BccMsgCounter = 0u;

/*==================================================================================================
 *                                    FUNCTION PROTOTYPES
 ==================================================================================================*/ 
static void LVBMS_ExtractData(Phy_TDType* TD, uint16* ptrData);
static void printData(LVBMS_Measurements* Measurements);
/*==================================================================================================
 *                                    STATIC FUNCTIONS
 ==================================================================================================*/
static void LVBMS_ExtractData(Phy_TDType* TD, uint16* ptrData)
{
    uint8 idx;
    /*Iterate through all response messages and extract data field*/
    for (idx = 0u; idx < TD->Response.MsgNum; idx++)
    {
        /*Copy data field into ptrData*/
        *(uint16*)(ptrData + idx) = *(uint16*)(TD->Response.Data + idx*3U);
    }
}

static void printData(LVBMS_Measurements* Measurements)
{
    printf("Current: %.2f\n", (double)Measurements->Current);
    printf("Stack Voltage: %.2f\n", (double)Measurements->StackVoltage);
    printf("Cell 1 Voltage: %.2f\n", (double)Measurements->CT1Voltage);
    printf("Cell 2 Voltage: %.2f\n", (double)Measurements->CT2Voltage);
    printf("Cell 3 Voltage: %.2f\n", (double)Measurements->CT3Voltage);
    printf("Cell 4 Voltage: %.2f\n", (double)Measurements->CT4Voltage);
    printf("Cell 5 Voltage: %.2f\n", (double)Measurements->CT5Voltage);
    printf("Cell 6 Voltage: %.2f\n", (double)Measurements->CT6Voltage);
    printf("--------------------------------------------------------\n");

}
/*==================================================================================================
 *                                    GLOBAL FUNCTIONS
 ==================================================================================================*/
void TD_Wait(const Bms_TDType* MessageTD)
{
    volatile Phy_TStatusInfoType *p_TDStatus = (volatile Phy_TStatusInfoType *)(uint32)&MessageTD->PhyTD->Status;
    while((*p_TDStatus == PHY_TS_PENDING));
}

void LVBMS_Init(void)
{
    /*Initialize Mcal Drivers*/
    Mcu_Init(NULL_PTR);
    Mcu_InitClock(McuClockSettingConfig_0);
    while(MCU_PLL_LOCKED != Mcu_GetPllStatus())
    {

    }
    Mcu_DistributePllClock();
    Mcu_SetMode(McuModeSettingConf_0);
    Platform_Init(NULL_PTR);
    Port_Init(NULL_PTR);
    Spi_Init(NULL_PTR);
    Spi_SetHWUnitAsyncMode(0u, SPI_INTERRUPT_MODE);
    Gpt_Init(NULL_PTR);
    OsIf_Init(NULL_PTR);
    /*Initialize BMS SPI Driver*/
    Bms_SpiIf_Init();
    /*Initialize Bcc_772c Driver*/
    Bcc_772c_Init(NULL_PTR);
}

LVBMS_Error LVBMS_SystemConfig(void)
{
    Phy_ErrorStatusType ErrorStatus;
	Std_ReturnType Status = (Std_ReturnType)E_OK;

    /*Wake-up the device and assign an address*/

    /*Clear the TD before appending messages*/
    Bms_TD_Clear(&LVBMS_TD);

    /*Append a wake-up message for chain/device */
    Status |= Bcc_772c_SPIWakeupDevice(SPI_BCC_CHAIN_ADDR, &LVBMS_TD);
    /*Append Enumeration message for the device*/
    Status |= Bcc_772c_Enumerate(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD);
    /*Append NOP to receive auto-read from enumerate request*/
    Status |= Bcc_772c_COM_InsertNop(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD);
    
    if (Status == (Std_ReturnType)E_NOT_OK)
    {
        /*Error while packaging the request frames*/
        return ERROR_TD_FILL;
    }

    /* Call Bcc_772c_Send */
    Bms_TD_Send(&LVBMS_TD, &ErrorStatus);

    /* Wait for TD status to be updated by the BMS SPI driver */
    TD_Wait(&LVBMS_TD);

    /*Check if device has been the right address*/
    if (LVBMS_TD.PhyTD->Status == PHY_TS_FINISHED)
    {
        Status = (SPI_BCC_DEVICE_ADDR == MC33772C_INIT_CID(LVBMS_TD.PhyTD->Response.Data[9U])) ? (Std_ReturnType)E_OK : (Std_ReturnType)E_NOT_OK;
        if (Status == (Std_ReturnType)E_NOT_OK)
        {
            /*Device was not enumerated, no point in going further*/
            return ERROR_ENUMERATE;
        }
    }
    else
    {
        /*Transmission failed, no point in going further*/
        return ERROR_PHY_TD_SEND;
    }
        
    /*Clear the TD before appending messages*/
	Bms_TD_Clear(&LVBMS_TD);

    /*Configure the device*/
    Status |= Bcc_772c_SYS_Configure(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, 0, &LVBMS_TD);
    Status |= Bcc_772c_MSR_Configure(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, 0, &LVBMS_TD);
    Status |= Bms_TD_Send(&LVBMS_TD, &ErrorStatus);
    
    /* Wait for TD status to be updated by the BMS SPI driver */
    TD_Wait(&LVBMS_TD);

    if (Status == (Std_ReturnType)E_NOT_OK)
    {
        return ERROR_CONFIGURE;
    }

    return NO_ERROR;
}

Std_ReturnType LVBMS_StartMeasurements(void)
{
    Std_ReturnType Status = (Std_ReturnType)E_OK;
    Phy_ErrorStatusType ErrorStatus;
    /* Clear TD */
	Status = Bms_TD_Clear(&LVBMS_TD);

    /* Start measurements */
    Status |= Bcc_772c_MSR_StartConversion(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, MC33772C_ADC_CFG_AVG_2_SAMPLES_ENUM_VAL, &LVBMS_TD);
	
    /* Add 2 ms delay to wait for the device to finish current measurement */
	Status |= Bms_TD_InsertPhyEvent(&LVBMS_TD, PHY_TIMER, 2000U);
	Status |= Bcc_772c_COM_InsertNop(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD);

    /* Call Bcc_772c_Send */
	Status |= Bms_TD_Send(&LVBMS_TD, &ErrorStatus);
    
    TD_Wait(&LVBMS_TD);
    /* Clear TD */
    Status |= Bms_TD_Clear(&LVBMS_TD);

    return Status;
}

Std_ReturnType LVBMS_ReadMeasurements(LVBMS_Measurements* Measurements)
{
    Std_ReturnType Status = (Std_ReturnType)E_OK;
    Phy_ErrorStatusType ErrorStatus;
    /*Array to store raw MEAS_xxxx register values*/
    uint16 measurementData[20U] = {0U};
    uint32 NTC_Resistance;

    /*Clear the TD before appending messages*/
    Bms_TD_Clear(&LVBMS_TD);

    /*Pack multiple read requests to retrieve all measurements*/
    Status |= Bcc_772c_MSR_GetData(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, BCC_772C_MSR_TYPE_ALL, &LVBMS_TD);

    /*Send the TD*/
    Status |= Bms_TD_Send(&LVBMS_TD, &ErrorStatus);

    /*Wait for transmission*/
    TD_Wait(&LVBMS_TD);

    if (ErrorStatus == PHY_NO_ERROR)
    {
        /*Extract data fields into measurementData*/
        LVBMS_ExtractData(LVBMS_TD.PhyTD, measurementData);

        /*TO DO - Remove Current or simulate it somehow*/
        Measurements->Current = (float)((measurementData[1U] & 0x7fff)  << 4U | (measurementData[2U] & 0x000F));
        Measurements->StackVoltage = (float)((measurementData[3U] & 0x7fff) * 5.0 / 32768.0);
        Measurements->CT1Voltage = (float)((measurementData[4U] & 0x7fff) * 5.0 / 32768.0);
        Measurements->CT2Voltage = (float)((measurementData[5U] & 0x7fff) * 5.0 / 32768.0);
        Measurements->CT3Voltage = (float)((measurementData[6U] & 0x7fff) * 5.0 / 32768.0);
        Measurements->CT4Voltage = (float)((measurementData[7U] & 0x7fff) * 5.0 / 32768.0);
        Measurements->CT5Voltage = (float)((measurementData[8U] & 0x7fff) * 5.0 / 32768.0);
        Measurements->CT6Voltage = (float)((measurementData[9U] & 0x7fff) * 5.0 / 32768.0);
        NTC_Resistance = (float)((measurementData[15U] & 0x7fff) * 5.0 / 32768.0);
        Measurements->PackTemperature = 0;
    }
    
    return Status;
}

void LVBMS_MeasureTask(void* parameters)
{
    LVBMS_Error ErrorStatus = NO_ERROR;
    Std_ReturnType Status = (Std_ReturnType)E_OK;
    LVBMS_Measurements Measurements;
    /*Delay of 100ms*/
    const TickType_t xDelay = 100U / portTICK_PERIOD_MS;

    /*Wake up and configure the battery cell controller*/
    do
    {
        ErrorStatus = LVBMS_SystemConfig();
    } while (ErrorStatus != NO_ERROR);

    /*System is configured, begin measurements*/
    while (1U)
    {
        /*Start conversion*/
        Status = LVBMS_StartMeasurements();
        if (Status == (Std_ReturnType)E_OK)
        {
            Status = LVBMS_ReadMeasurements(&Measurements);
            if (Status == (Std_ReturnType)E_OK && xMeasurementsQueue != NULL)
            {
                /*Send measurements to the queue*/
                xQueueSend(xMeasurementsQueue, (void*)&Measurements, (TickType_t)0);
            }
        }
        vTaskDelay(xDelay);
    }
}

void LVBMS_DashboardTask(void* parameters)
{
     /*Delay of 20ms*/
    const TickType_t xDelay = 20U / portTICK_PERIOD_MS;
    LVBMS_Measurements Measurements;
    while(1)
    {
        if (xQueueReceive(xMeasurementsQueue, (void*)&Measurements, ( TickType_t ) 10 ) == pdPASS)
        {
            printData(&Measurements);
        }
        vTaskDelay(xDelay);
    }
}

void LVBMS_FaultHandlerTask(void* parameters)
{

}

#ifdef __cplusplus
}
#endif

/** @} */

