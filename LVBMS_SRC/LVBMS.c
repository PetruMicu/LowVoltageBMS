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

/*==================================================================================================
 *                                            ENUMS
 ==================================================================================================*/

/*==================================================================================================
 *                                STRUCTURES AND OTHER TYPEDEFS
 ==================================================================================================*/

/*==================================================================================================
 *                                GLOBAL VARIABLE DECLARATIONS
 ==================================================================================================*/

 /**************************************************************************************************
  *                                          Global Info
****************************************************************************************************/
/*These value must be documented somewhere*/
/*OCV-SoC LookUp Tables*/
static const float OCV_LookUpTbl[20U] = {4.1617, 4.0913, 4.0749, 4.0606, 4.0153, 3.9592, 3.9164, 3.8687, 3.8163, 
                             3.7735, 3.7317, 3.6892, 3.6396, 3.5677, 3.5208, 3.4712, 3.3860, 3.2880, 3.2037, 3.0747};
static const float SoC_LookUpTbl[20U] = {1, 0.9503, 0.9007, 0.8510, 0.8013, 0.7517, 0.7020, 0.6524, 0.6027, 0.5530,
                             0.5034, 0.4537, 0.4040, 0.3543, 0.3046, 0.2550, 0.2053, 0.1556, 0.1059, 0.0563};

/*Keeps track of state of the system (idle, charging or discharging)*/
static LVBMS_StateType SystemState;
/*Keeps track of the voltage fault status*/
static volatile LVBMS_FaultStatusType VoltageFaultStatus = NO_FAULT;
/*Keeps track of the current fault status*/
static volatile LVBMS_FaultStatusType CurrentFaultStatus = NO_FAULT;

/*Stores last measurements and cell states*/
static LVBMS_MeasurementsType Measurements;

/*Stores cell fault and balancing information*/
static LVBMS_CellStateType CellInfo = {.CTState = {CELL_OK}, .CTBalEnabled = {false}};
/*Variable used to reduce transmission overhead, each cell state ocupies 2bits*/
static uint16 CTState = 0U;
/*Variable used to reduce transmission overhead, each cell balancing status ocupies 1bit*/
static uint8 CTBalEnabled = 0U;

/*Store the time of CC reading*/
static TickType_t CurrentTime = 0;
static TickType_t OldTime = 0;

/*Stores State of Charge of the battery pack*/
volatile static float SoC = 100;
/*Stores State of Health of the battery pack*/
static float SoH = 100;
/*Stores Depth of Discharge of the battery pack*/
static float DoD = 0;
 /**************************************************************************************************
  *                                          FreeMASTER
****************************************************************************************************/

FMSTR_TSA_TABLE_BEGIN(LVBMS_CellParameters)
   FMSTR_TSA_RO_VAR(Measurements.CTVoltage, FMSTR_TSA_POINTER)
   FMSTR_TSA_RO_VAR(CTState, FMSTR_TSA_UINT16)
   FMSTR_TSA_RO_VAR(CTBalEnabled, FMSTR_TSA_UINT8)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(LVBMS_Parameters)
    FMSTR_TSA_RW_VAR(SystemState, FMSTR_TSA_USERTYPE(LVBMS_StateType))
    FMSTR_TSA_RO_VAR(VoltageFaultStatus, FMSTR_TSA_USERTYPE(LVBMS_FaultStatusType))
    FMSTR_TSA_RO_VAR(CurrentFaultStatus, FMSTR_TSA_USERTYPE(LVBMS_FaultStatusType))
    FMSTR_TSA_RO_VAR(Measurements.Current, FMSTR_TSA_FLOAT)
    FMSTR_TSA_RO_VAR(Measurements.StackVoltage, FMSTR_TSA_FLOAT)
    FMSTR_TSA_RO_VAR(Measurements.PackTemperature, FMSTR_TSA_DOUBLE)
    FMSTR_TSA_RO_VAR(SoC, FMSTR_TSA_FLOAT)
    FMSTR_TSA_RO_VAR(SoH, FMSTR_TSA_FLOAT)
FMSTR_TSA_TABLE_END()


FMSTR_TSA_TABLE_LIST_BEGIN()
    FMSTR_TSA_TABLE(LVBMS_Parameters)
	FMSTR_TSA_TABLE(LVBMS_CellParameters)
FMSTR_TSA_TABLE_LIST_END()

 /**************************************************************************************************
  *                                          FreeRTOS
****************************************************************************************************/

/* Semaphore used to send and receive fault status from battery cell controller*/
extern SemaphoreHandle_t xFaultSemaphore;
/*Mutex used for device communication to ensure exclusive access to Bms_TD_Send*/
extern SemaphoreHandle_t xCommMutex;
/*Mutex used for ensuring exclusive access on Measurements*/
extern SemaphoreHandle_t xMeasurementsMutex;
/*Task handlers for LVBMS tasks*/
extern TaskHandle_t xMeasureHandler;
extern TaskHandle_t xControlHandler;
extern TaskHandle_t xFaultHandler;
extern TaskHandle_t xFaultLED_Handler;
/*==================================================================================================
 *                                    FUNCTION PROTOTYPES
 ==================================================================================================*/ 
static void LVBMS_ExtractData(Phy_TDType* TransactionDescriptor, uint16* ptrData);

LVBMS_FaultStatusType LVBMS_CheckFault(void);
Std_ReturnType LVBMS_ClearFault(void);

/*LVBMS tasks*/
void LVBMS_MeasureTask(void* parameters);
void LVBMS_ControlTask(void* parameters);
void LVBMS_FaultHandlerTask(void* parameters);
void LVBMS_FaultLEDHandlerTask(void* parameters);
void LVBMS_CurrentFaultHandlerTask(void* parameters);

/*==================================================================================================
 *                                    STATIC FUNCTIONS
 ==================================================================================================*/

static void LVBMS_TD_Wait(const Bms_TDType* MessageTD)
{
    volatile Phy_TStatusInfoType *p_TDStatus = (volatile Phy_TStatusInfoType *)(uint32)&MessageTD->PhyTD->Status;
    while((*p_TDStatus == PHY_TS_PENDING))
    {
    }
}

static void LVBMS_ExtractData(Phy_TDType* TransactionDescriptor, uint16* ptrData)
{
    uint8 idx;
    /*Iterate through all response messages and extract data field*/
    for (idx = 0u; idx < TransactionDescriptor->Response.MsgNum; idx++)
    {
        /*Copy data field into ptrData*/
        *(uint16*)(ptrData + idx) = *(uint16*)(TransactionDescriptor->Response.Data + idx*3U);
    }
}

static void LVBMS_TD_Comm(const Bms_TDType *BmsTD, Phy_ErrorStatusType *ErrorStatus)
{
    /*Take communication mutex*/
    xSemaphoreTake(xCommMutex, (TickType_t)portMAX_DELAY);
    
    /*Send TD to the device*/
    Bms_TD_Send(BmsTD, ErrorStatus);

    /* Wait for TD status to be updated by the BMS SPI driver */
    LVBMS_TD_Wait(BmsTD);

    /*Release communication mutex*/
    xSemaphoreGive(xCommMutex);
}

static void LVBMS_LED_Update(void)
{
    switch (SystemState)
    {
    case IDLE:
        Dio_WriteChannel(DioConf_DioChannel_RED, STD_LOW);
        Dio_WriteChannel(DioConf_DioChannel_GRN, STD_LOW);
        Dio_WriteChannel(DioConf_DioChannel_BLU, STD_LOW);
        break;
    case CHARGING:
        Dio_WriteChannel(DioConf_DioChannel_RED, STD_LOW);
        Dio_WriteChannel(DioConf_DioChannel_GRN, STD_HIGH);
        Dio_WriteChannel(DioConf_DioChannel_BLU, STD_HIGH);
        break;
    case DISCHARGING:
        Dio_WriteChannel(DioConf_DioChannel_RED, STD_HIGH);
        Dio_WriteChannel(DioConf_DioChannel_GRN, STD_HIGH);
        Dio_WriteChannel(DioConf_DioChannel_BLU, STD_LOW);
        break;
    default:
        break;
    }
}

static void LVBMS_ComputeSoCSoH(LVBMS_MeasurementsType* LocalMeasurements)
{
    float Delta_DoD = (float)(-1.0) * (float)(LocalMeasurements->DeltaQ / (float)C_RATED) * (float)100.0;
    /*Introduced only for the emulator*/
    if ((IDLE == SystemState) ||
        ((CHARGING == SystemState) && (Delta_DoD > 0U)))
    {
        Delta_DoD = 0U;
    }

    if (LocalMeasurements->Current < 0U)
    {
        /*Discharging*/
        if (LocalMeasurements->StackVoltage >= (float)VMIN)
        {
            /*Normal operation*/
            DoD = DoD + Delta_DoD;
            SoC = SoH - DoD;
        }
        else
        {
            /*Deeply discharged*/
            SoH = DoD;
        }
    }
    else
    {
        /*Charging or regenerative breaking*/
        if ((LocalMeasurements->StackVoltage >= VMAX && LocalMeasurements->StackVoltage <= (VMAX + VMAX* 3/100)) || (SoC >= 100.0))
        {
            /*NOTE: In a real system we would also consider the current*/
            /*Fully charged*/
            SoH = SoC;
        }
        else
        {
            DoD = DoD + Delta_DoD;
            SoC = SoH - DoD;
        }
    }
}

void LVBMS_UpdateCellStateInfo(void)
{
    uint8 idx = 0U;

    CTState = 0U;
    CTBalEnabled = 0U;
    for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
    {
        CTState |= (uint16)((CellInfo.CTState[idx] & 3U) << (idx*2U));
        if (DISCHARGING == SystemState)
        {
        	CTBalEnabled = 0U;
        }
        else
        {
        	CTBalEnabled |= (uint8)((CellInfo.CTBalEnabled[idx] & 1U) << idx);
        }
    }
}

void LVBMS_RecalibrationSoCOcV(LVBMS_MeasurementsType* LocalMeasurements)
{
	uint8 idx, jdx;
    float LocalSoC = 0U;

    for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
    {
        for (jdx = 0U; jdx < 20U; jdx++)
        {
            if (LocalMeasurements->CTVoltage[idx] >= OCV_LookUpTbl[jdx])
            {
                break;
            }
        }
        if (20U == jdx)
        {
        	LocalSoC += 0.005;
        }
        else
        {
        	LocalSoC += SoC_LookUpTbl[jdx];
        }

    }
    /*Compute average*/
    LocalSoC = (LocalSoC / ACTIVE_CELL_NUMBER) * 100.0;
    /*Check if there is a 10% difference*/
    if (abs(LocalSoC - SoC) >= 10.0)
    {
        /*This will be always true for first estimation*/
        SoC = LocalSoC;
    }
    SoH -= 0.00001;
    DoD = (float)100.0 - SoC;
}

Std_ReturnType LVBMS_InitialSocSoHEstimation(void)
{
    LVBMS_MeasurementsType LocalMeasurements;
    Std_ReturnType Status = (Std_ReturnType)E_NOT_OK;
    Phy_ErrorStatusType ErrorStatus;
    
    /*Device will respond with one dummy response and all cells voltages*/
    uint16 Measurements[7U];
    uint8 idx;

    /* Clear TD */
	Status = Bms_TD_Clear(&LVBMS_TD);
    /* Start voltage measurements*/
    Status |= Bcc_772c_MSR_StartConversion(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, MC33772C_ADC_CFG_AVG_2_SAMPLES_ENUM_VAL, &LVBMS_TD);
    /*Send TD to the device*/
    Bms_TD_Send(&LVBMS_TD, &ErrorStatus);
    /* Wait for TD status to be updated by the BMS SPI driver */
    if ((Std_ReturnType)E_OK == Status)
    {
        LVBMS_TD_Wait(&LVBMS_TD);
    }

    if (PHY_NO_ERROR == ErrorStatus)
    {
        /*keep device awake by sending NOP commands until conversion is ready*/
        /* Clear TD */
        Status = Bms_TD_Clear(&LVBMS_TD);
        Status |= Bcc_772c_COM_InsertNop(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD);
        /*insert 2ms delay*/
        Status |= Bms_TD_InsertPhyEvent(&LVBMS_TD, PHY_TIMER, 2000U);
        Status |= Bcc_772c_COM_InsertNop(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD);
        /*Send TD to the device*/
        Status |= Bms_TD_Send(&LVBMS_TD, &ErrorStatus);
        /* Wait for TD status to be updated by the BMS SPI driver */
        LVBMS_TD_Wait(&LVBMS_TD);
        /* Clear TD */
        Status |= Bms_TD_Clear(&LVBMS_TD);
        Status |= Bcc_772c_MSR_GetData(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, BCC_772C_MSR_TYPE_CELL_VOLTAGE, &LVBMS_TD);
        /*Send TD to the device*/
        Status |= Bms_TD_Send(&LVBMS_TD, &ErrorStatus);
        /* Wait for TD status to be updated by the BMS SPI driver */
        LVBMS_TD_Wait(&LVBMS_TD);

        if (((Std_ReturnType)E_OK == Status) && (PHY_NO_ERROR == ErrorStatus))
        {
             LVBMS_ExtractData(LVBMS_TD.PhyTD, Measurements);
            for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
            {
                /*Voltage values are in reversed order: CELL6 ... CELL1*/
                LocalMeasurements.CTVoltage[idx] = (float)CELL_VOLTAGE_VALUE(Measurements[6U - idx]);
            }
            LVBMS_RecalibrationSoCOcV(&LocalMeasurements);
        }
    }

    return Status;
}
/*==================================================================================================
 *                                    GLOBAL FUNCTIONS
 ==================================================================================================*/

/*ICU Callback. If transition is detected on FAULT pin then LVBMS_FaultHandlerTask is unblocked for fault handling*/
void FaultEvent(void)
{
	VoltageFaultStatus = FAULT;
	/*Signal Fault Task that a fault has occured*/
	xSemaphoreGiveFromISR(xFaultSemaphore, NULL);
}

/*==================================================================================================
 *                                    INITIALIZATION HANDLE
 ==================================================================================================*/

void LVBMS_Init(void)
{
	LVBMS_ErrorType ErrorStatus;

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
    Uart_Init(NULL_PTR);
    Spi_SetHWUnitAsyncMode(0u, SPI_INTERRUPT_MODE);
    Gpt_Init(NULL_PTR);
    OsIf_Init(NULL_PTR);
    Icu_Init(NULL_PTR);
    /*Initialize BMS SPI Driver*/
    Bms_SpiIf_Init();
    /*Initialize Bcc_772c Driver*/
    Bcc_772c_Init(NULL_PTR);

    /*Initialize FreeMASTER*/
    FMSTR_SerialSetBaseAddress((FMSTR_ADDR)IP_LPUART_13_BASE);
	FMSTR_Init();

    /*Wake up and configure the battery cell controller*/
    do
    {
        ErrorStatus = LVBMS_SystemConfig();
    } while (ErrorStatus != NO_ERROR);
}

LVBMS_ErrorType LVBMS_SystemConfig(void)
{
    Phy_ErrorStatusType ErrorStatus = PHY_NO_ERROR;
	Std_ReturnType Status = (Std_ReturnType)E_OK;

    /*Defines critical voltage thresholds*/
    Bcc_772c_MsrThresholdValueType Thresholds = 
    {
        .Overvoltage = MC33772C_TH_ALL_CT_ALL_CT_OV_TH_DEFAULT_ENUM_VAL,
        .Undervoltage = MC33772C_ALL_CT_UV_TH_VAL
    };

    /*Stores balancing configurations*/
    Bcc_772c_BalChannelConfigurationType BalConfig = 
    {
        .ChannelNo = BCC_772C_CHANNEL_ALL, /*Configure all cells*/
        .BalChannelEn = false, /*Balancing drivers are initially disabled*/
        .Timer = 15U /*Balancing time 15m*/
    };

    /*Clear the TD before appending messages*/
    Status = Bms_TD_Clear(&LVBMS_TD);

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
    LVBMS_TD_Wait(&LVBMS_TD);

    /*Check if device has been the right address*/
    if (PHY_NO_ERROR == ErrorStatus)
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
    Status = Bcc_772c_MSR_Configure(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, 0, &LVBMS_TD);
    Status |= Bcc_772c_MSR_SetThresholds(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, 
                                         BCC_772C_MSR_TH_TYPE_CT_ALL, 
                                         Thresholds,
                                         &LVBMS_TD);
    Status |= Bcc_772c_FEH_Configure(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, 0, &LVBMS_TD);
    Status |= Bcc_772c_SYS_Configure(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, 0, &LVBMS_TD);
    Status |= Bcc_772c_CC_Configure(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, 0, &LVBMS_TD);
	Status |= Bcc_772c_CC_ControlMeasurements(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, BCC_772C_CC_START, &LVBMS_TD);
    Status |= Bcc_772c_BAL_SetChannelConfiguration(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &BalConfig, &LVBMS_TD);
    
    if (Status == (Std_ReturnType)E_NOT_OK)
    {
        /*Error while packaging the request frames*/
        return ERROR_TD_FILL;
    }

    Bms_TD_Send(&LVBMS_TD, &ErrorStatus);
    
    /* Wait for TD status to be updated by the BMS SPI driver */
    LVBMS_TD_Wait(&LVBMS_TD);

    if (PHY_NO_ERROR != ErrorStatus)
    {
        return ERROR_CONFIGURE;
    }

    /*Estimate SoC for Open Circuit Voltage*/
    Status = LVBMS_InitialSocSoHEstimation();

    if ((Std_ReturnType)E_OK != Status)
    {
        return ERROR_INITIAL_ESTIMATION;
    }

    return NO_ERROR;
}

/*==================================================================================================
 *                                    MEASUREMENTS HANDLE
 ==================================================================================================*/

Std_ReturnType LVBMS_StartMeasurements(void)
{
    Std_ReturnType Status = (Std_ReturnType)E_OK;
    Phy_ErrorStatusType ErrorStatus;
    /* Clear TD */
	Status = Bms_TD_Clear(&LVBMS_TD);

    /* Start voltage measurements*/
    Status |= Bcc_772c_MSR_StartConversion(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, MC33772C_ADC_CFG_AVG_2_SAMPLES_ENUM_VAL, &LVBMS_TD);

    /*Send requests to the device*/
    LVBMS_TD_Comm(&LVBMS_TD, &ErrorStatus);

    /* Clear TD */
    Status |= Bms_TD_Clear(&LVBMS_TD);

    return Status;
}

Std_ReturnType LVBMS_ReadMeasurements(LVBMS_MeasurementsType* LocalMeasurements)
{
    Std_ReturnType Status = (Std_ReturnType)E_OK;
    Phy_ErrorStatusType ErrorStatus;
    /*Structure to store current measurement information*/
    Bcc_772c_CcISenseValueType CCIsense;
    /*Value of current coulomb count*/
    sint32 CCount;
    /*Value of number of samples accumulated in the coulomb counter*/
    uint16 CCSamples;
    /*Average Current calculated with coulomb counting*/
    float AvgCurrent;
    /*Array to store raw MEAS_xxxx register values*/
    uint16 measurementData[20U] = {0U};
    /*Calculated NTC Resistance*/
    volatile double NTC_Resistance;
    uint8 idx;

    /*Clear the TD before appending messages*/
    Status = Bms_TD_Clear(&LVBMS_TD);

    /*Pack multiple read requests to retrieve all measurements*/
    Status |= Bcc_772c_CC_GetData(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD);
    Status |= Bcc_772c_MSR_GetData(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, BCC_772C_MSR_TYPE_STACK_CELL_VOLTAGE, &LVBMS_TD);
    Status |= Bcc_772c_COM_ReadRegisters(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, MC33772C_MEAS_AN1_OFFSET, 1U, &LVBMS_TD);
    /*Send requests to the device*/
    LVBMS_TD_Comm(&LVBMS_TD, &ErrorStatus);

    CurrentTime = xTaskGetTickCount();
    if (PHY_NO_ERROR == ErrorStatus)
    {
    	/* Resolution of this structure is 0.1uOhm/LSB,
    	 * therefore RSHUNT is converted down. Care will be taken
    	 * when calculating the current value
    	 */
        CCIsense.RShunt = RSHUNT / 100U;
        /*Extract Cell voltage values into measurementData*/
        LVBMS_ExtractData(LVBMS_TD.PhyTD, measurementData);
        /*Extract ISense*/
        Bcc_772c_CC_ExtractISense(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &CCIsense, &LVBMS_TD);
        /*Extract Coulomb Count*/
        Bcc_772c_CC_ExtractCoulombCnt(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &CCount, &LVBMS_TD);

        /*Compute Coulomb Counting*/
        CCSamples = measurementData[1U];
        AvgCurrent = (float)((float)CCount / (float)CCSamples) * ((float)BCC_772C_V2RES / (float)RSHUNT);
        LocalMeasurements->DeltaQ = (float)(AvgCurrent * (float)(CurrentTime - OldTime) / 3.6e+6);

        
        if (IDLE == SystemState)
        {
            /*Added to simulate current interruption*/
            LocalMeasurements->Current = 0U;
        }
        /*Current value in mA*/
        else
        {
            LocalMeasurements->Current = (float)(CCIsense.ISense / 1000);
        }
        /*Stack Voltage in V*/
        LocalMeasurements->StackVoltage = (float)STACK_VOLTAGE_VALUE(measurementData[7U]);
        /*Individual Cell Voltages*/
        for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
        {
            /*Voltage values are in reversed order: CELL6 ... CELL1*/
            LocalMeasurements->CTVoltage[idx] = (float)CELL_VOLTAGE_VALUE(measurementData[13U - idx]);
        }
        /*Compute Temperature*/
        NTC_Resistance = (double)((double)RVOLTAGE_DEVIDER / (((double)32767U / (double)(measurementData[15U] & 0x7fff)) - 1U));
        LocalMeasurements->PackTemperature = (double)(1.0 / (double)CONVERT_CELCIUS_TO_KELVIN(25)) + (double)(1.0 / (double)BETA) * log(NTC_Resistance / RNTC);
        LocalMeasurements->PackTemperature = (double)CONVERT_KELVIN_TO_CELSIUS(1.0 / LocalMeasurements->PackTemperature);
    }
    else
    {
        Status = (Std_ReturnType)E_NOT_OK;
    }

    OldTime = CurrentTime;
    
    return Status;
}

void LVBMS_MeasureTask(void* parameters)
{
    (void)parameters;
    Std_ReturnType Status = (Std_ReturnType)E_OK;
    /*Delay of 2ms used for waiting for conversion cycle to finish*/
    const TickType_t xDelay_Measurements = 2U / portTICK_PERIOD_MS;
    /*Store the time of CC reading*/
    TickType_t RecalibrationCurrentTime = 0;
    TickType_t RecalibrationOldTime = 0;
    LVBMS_MeasurementsType LocalMeasurements;

    while (1U)
    {
        RecalibrationCurrentTime = xTaskGetTickCount();
        /*Start conversion*/
        Status = LVBMS_StartMeasurements();
        if ((Std_ReturnType)E_OK == Status)
        {
            /*Wait for conversion to finish*/
            vTaskDelay(xDelay_Measurements);
            Status = LVBMS_ReadMeasurements(&LocalMeasurements);
            if ((Std_ReturnType)E_OK == Status)
            {
                if ((RecalibrationCurrentTime - RecalibrationOldTime) >= (1000U / portTICK_PERIOD_MS))
                {
                    /*Recalibrate every second if battery is idle*/
                    RecalibrationOldTime = RecalibrationCurrentTime;
                    LVBMS_RecalibrationSoCOcV(&LocalMeasurements);
                }
                /*Compute SoC/SoH with CC*/
                LVBMS_ComputeSoCSoH(&LocalMeasurements);
                /*Take mutex*/
                xSemaphoreTake(xMeasurementsMutex, (TickType_t)portMAX_DELAY);
                /*Update global measurements*/
                Measurements = LocalMeasurements;
                /*Give mutex*/
                xSemaphoreGive(xMeasurementsMutex);
            }
        }
        /*Schedule next measurement cycle*/
        vTaskDelay(MEASUREMENTS_DELAY);
    }
}

/*==================================================================================================
 *                                    CONTROL AND BALANCE HANDLE
 ==================================================================================================*/

void LVBMS_CellBalanceControl(void)
{
    Std_ReturnType Status;
    Phy_ErrorStatusType ErrorStatus;
    uint16 BalStatusReg = 0U;
    float MinCellVoltage = 5U;
    uint8 MinCellVoltageIdx = 0U;
    uint8 idx;
    /*Stores balancing configurations*/
    Bcc_772c_BalChannelConfigurationType BalConfig = 
    {
        .ChannelNo = BCC_772C_CHANNEL_ALL, /*Configure all cells*/
        .BalChannelEn = false, /*Balancing drivers are initially disabled*/
        .Timer = 15U /*Balancing time 15m*/
    };

    /*Clear the TD before appending messages*/
    Status = Bms_TD_Clear(&LVBMS_TD_BAL);
    Status |= Bcc_772c_BAL_GetStatus(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD_BAL);
    if ((Std_ReturnType)E_OK == Status)
    {
        LVBMS_TD_Comm(&LVBMS_TD_BAL, &ErrorStatus);
    }
    if (PHY_NO_ERROR == ErrorStatus)
    {
        /*Extract device responses*/
        BalStatusReg = (uint16)(LVBMS_TD_BAL.PhyTD->Response.Data[12U] & 0x3F);
    }

    if ((CHARGING == SystemState) || (IDLE == SystemState))
    {
        /*Perform balancing algorithm only when battery is charging or idle*/
        /*Find minimum cell voltage*/
        for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
        {
            /*Save balancing driver state for each cell*/
            CellInfo.CTBalEnabled[idx] = (bool)((BalStatusReg >> idx) & 1U);
            if (MinCellVoltage >= Measurements.CTVoltage[idx])
            {
                MinCellVoltageIdx = idx;
                MinCellVoltage = Measurements.CTVoltage[idx];
            }
        }

        /*Clear the TD before appending messages*/
        Status = Bms_TD_Clear(&LVBMS_TD_BAL);
        /*Decide and activate cell balancing*/
        for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
        {
            if (idx == MinCellVoltageIdx)
            {
                continue;
            }
            else if ((Std_ReturnType)E_OK == Status)
            {  
                BalConfig.ChannelNo = (Bcc_772c_ChannelNoType)idx;
                BalConfig.BalChannelEn  = false;
                /*Check if voltage level is too high and start balancing if not already started*/
                if ((Measurements.CTVoltage[idx] >= (MinCellVoltage + MAX_VOLTAGE_DIFF)) &&
                    (false == CellInfo.CTBalEnabled[idx]))
                {
                    /*Check if voltage level is too low to be charging*/
                    if (CELL_UV == CellInfo.CTState[idx])
                    {
                        BalConfig.BalChannelEn  = false;
                    }
                    else
                    {
                        BalConfig.BalChannelEn  = true;
                    }
                }
                /*Balance a cell that is beyond the allowed threshold*/
                else if (CELL_OV == CellInfo.CTState[idx])
                {
                    BalConfig.BalChannelEn  = true;
                }
                /*Cell balancing is needed*/
                Status |= Bcc_772c_BAL_SetChannelConfiguration(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &BalConfig, &LVBMS_TD_BAL);
                CellInfo.CTBalEnabled[idx] |= (bool)(BalConfig.BalChannelEn << idx);
            }
            else
            {
                break;
            }
        }

        if (0u != LVBMS_TD_BAL.PhyTD->Request.MsgNum)
        {
            /*Send balancing requests*/
            LVBMS_TD_Comm(&LVBMS_TD_BAL, &ErrorStatus);
        }
    }
    else
    {
        /*Do not balance during discharging*/
    }

    /*Prepare information for FreeMASTER*/
    LVBMS_UpdateCellStateInfo();
}

void LVBMS_ControlState(void)
{
    static TickType_t start = 0;
    static TickType_t stop = 0;

    /*Keep current fault asserted for 5 seconds*/
    if (FAULT == CurrentFaultStatus)
    {
        stop = xTaskGetTickCount();
        if ((stop - start) >= (5000U / portTICK_PERIOD_MS))
        {
            /*Clear fault after 5 seconds have passed*/
            CurrentFaultStatus = NO_FAULT;
        }
    }
    /*Take mutex*/
    xSemaphoreTake(xMeasurementsMutex, (TickType_t)portMAX_DELAY);
    /*Current fault check*/
    if (((CHARGING == SystemState) && ((Measurements.Current > MAX_CHARGE_CURRENT) || Measurements.Current < 0U)) || 
        ((DISCHARGING == SystemState) && ((Measurements.Current < MAX_DISCHARGE_CURRENT) || ((Measurements.Current > MAX_CHARGE_CURRENT))))
       )
    {
        /*Check if fault is already asserted*/
        if ((NO_FAULT == VoltageFaultStatus) && (NO_FAULT == CurrentFaultStatus))
        {
            CurrentFaultStatus = FAULT;
            start = xTaskGetTickCount();
        }
    }
    /*If any fault is detected, take action*/
    if ((FAULT == VoltageFaultStatus) || (FAULT == CurrentFaultStatus) ||
        ((CHARGING != SystemState) && (SoC <= 5.0)) || ((DISCHARGING != SystemState) && (SoC >= 100.0))
       )
    {
        /*Change state and break the circuit*/
        SystemState = IDLE;
    }

    /*Manually tinker with the current. Just for presentation purposes*/
    if (IDLE == SystemState)
    {
        Measurements.Current = 0;
    }
    /*Give Mutex*/
    xSemaphoreGive(xMeasurementsMutex);
}

void LVBMS_ControlTask(void* parameters)
{
	(void)parameters;
	/*Delay of 10ms*/
	const TickType_t xDelay = 2U / portTICK_PERIOD_MS;
    /*Initial state of the battery*/
    SystemState = IDLE;
    while(1)
    {
        /*Check on system and update state if needed*/
        LVBMS_ControlState();
        if (IDLE == SystemState)
        {
        	Dio_WriteChannel(DioConf_DioChannel_RELAY, STD_HIGH);
        }
        else
        {
        	Dio_WriteChannel(DioConf_DioChannel_RELAY, STD_LOW);
        }
        /*For debugging only*/
        LVBMS_LED_Update();
        /*Controls and decide if cell balancing is needed*/
        LVBMS_CellBalanceControl();
        /*Send information to FreeMASTER*/
        FMSTR_Poll();
        vTaskDelay(xDelay);
    }
}

/*==================================================================================================
 *                                    FAULT HANDLE
 ==================================================================================================*/

LVBMS_FaultStatusType LVBMS_CheckFault(void)
{
    Std_ReturnType Status = (Std_ReturnType)E_OK;
    Phy_ErrorStatusType ErrorStatus;
    Bcc_772c_RegisterContentType RW;
    LVBMS_FaultStatusType FaultState = FAULT;
    uint16 Fault1StatusReg = 0U;
    uint16 CellOVFaultReg = 0U;
    uint16 CellUVFaultReg = 0U;
    uint8 idx;

    /*Clear the TD before appending messages*/
    Bms_TD_Clear(&LVBMS_TD_FAULT);

    Status = Bcc_772c_FEH_GetFaultStatus(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD_FAULT);
    LVBMS_TD_Comm(&LVBMS_TD_FAULT, &ErrorStatus);

    if (PHY_NO_ERROR == ErrorStatus)
    {
        /*Store FAULT1_STATUS register content*/
        Fault1StatusReg = LVBMS_TD_FAULT.PhyTD->Response.Data[3U];
        /*Check if undervoltage or overvoltage has been detected*/
        if ((MC33772C_FAULT1_STATUS_CT_UV_FLT(Fault1StatusReg) == 
                MC33772C_FAULT1_STATUS_CT_UV_FLT_UNDERVOLTAGE_ENUM_VAL))
        {
            RW.RegAddr = MC33772C_CELL_UV_FLT_OFFSET;
            RW.RegData = 0U;
            RW.RegMask = 0x3F;
            /*Clear the TD before appending messages*/
            Bms_TD_Clear(&LVBMS_TD_FAULT);
            /*Read CELL_UV_FLT register*/
            Status = Bcc_772c_COM_ReadRegisters(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, MC33772C_CELL_UV_FLT_OFFSET, (uint8)1U, &LVBMS_TD_FAULT);
            /*Clear fault. The fault will be set back by the hardware if fault is still detected*/
            Status |= Bcc_772c_COM_WriteRegister(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &RW, &LVBMS_TD_FAULT);
            if ((Std_ReturnType)E_OK == Status)
            {
                LVBMS_TD_Comm(&LVBMS_TD_FAULT, &ErrorStatus);
            }
            else
            {
                return FAULT;
            }
            
            if (PHY_NO_ERROR == ErrorStatus)
            {
                /*Store CELL_UV_FLT register*/
                CellUVFaultReg = LVBMS_TD_FAULT.PhyTD->Response.Data[3U];
            }
        }
        if ((MC33772C_FAULT1_STATUS_CT_OV_FLT(Fault1StatusReg) ==
                    MC33772C_FAULT1_STATUS_CT_OV_FLT_OVERVOLTAGE_ENUM_VAL))
        {
            RW.RegAddr = MC33772C_CELL_OV_FLT_OFFSET;
            RW.RegData = 0U;
            RW.RegMask = 0x3F;
            /*Clear the TD before appending messages*/
            Bms_TD_Clear(&LVBMS_TD_FAULT);
            /*Read CELL_UV_FLT register*/
            Status = Bcc_772c_COM_ReadRegisters(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, MC33772C_CELL_OV_FLT_OFFSET, (uint8)1U, &LVBMS_TD_FAULT);
            /*Clear fault. The fault will be set back by the hardware if fault is still detected*/
            Status |= Bcc_772c_COM_WriteRegister(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &RW, &LVBMS_TD_FAULT);
            if ((Std_ReturnType)E_OK == Status)
            {
                LVBMS_TD_Comm(&LVBMS_TD_FAULT, &ErrorStatus);
            }
            else
            {
                return FAULT;
            }

            if (PHY_NO_ERROR == ErrorStatus)
            {
                /*Store CELL_OV_FLT register*/
                CellOVFaultReg = LVBMS_TD_FAULT.PhyTD->Response.Data[3U];
            }
        }
        /*Extend based on BCC_772C FEH configurations*/

        for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
        {
            /*Check if OV was detected for this cell*/
            if ((bool)((CellOVFaultReg >> idx) & 1U) == true)
            {
                CellInfo.CTState[idx] = CELL_OV;
            }
            /*Check if UV was detected for this cell*/
            else if ((bool)((CellUVFaultReg >> idx) & 1U) == true)
            {
                CellInfo.CTState[idx] = CELL_UV;
            }
            /*Cell is OK*/
            else
            {
                CellInfo.CTState[idx] = CELL_OK;
            }
        }
    }

    /*If first two bits of FAULT1_STATUS are not set, fault has been fixed.*/
    if(MC33772C_FAULT1_STATUS_POR_NO_POR_ENUM_VAL == (Fault1StatusReg & 3U))
    {
        FaultState = NO_FAULT;
    }
    
    return FaultState;
}

Std_ReturnType LVBMS_ClearFault(void)
{
    Std_ReturnType Status = (Std_ReturnType)E_OK;
    Phy_ErrorStatusType ErrorStatus;
    uint8 idx;
    /*Clear the TD before appending messages*/
    Bms_TD_Clear(&LVBMS_TD_FAULT);

    /*Clear all Fault registers*/
    Status = Bcc_772c_FEH_ClearFaultStatus(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD_FAULT);
    LVBMS_TD_Comm(&LVBMS_TD_FAULT, &ErrorStatus);

    if (PHY_NO_ERROR != ErrorStatus)
    {
        Status = (Std_ReturnType)E_NOT_OK;
    }
    else
    {
        /*Update Cell States to CELL_OK*/
        for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
        {
            CellInfo.CTState[idx] = CELL_OK;
        }
    }
    return Status;
}

void LVBMS_FaultHandlerTask(void* parameters)
{
    (void)parameters;

    /*Enable fault detection*/
    Icu_EnableEdgeDetection(FAULT_ICU);
    Icu_EnableNotification(FAULT_ICU);

    /*Clean start*/
    LVBMS_ClearFault();

    while(1U)
	{
        /*Wait for fault detection*/
		xSemaphoreTake(xFaultSemaphore, portMAX_DELAY);
        /*Start Fault signaling*/
		vTaskResume(xFaultLED_Handler);
		while (VoltageFaultStatus == FAULT)
		{
            /*Check if fault was fixed*/
            VoltageFaultStatus = LVBMS_CheckFault();
            if (NO_FAULT == VoltageFaultStatus)
            {
                /*Clear the fault*/
                if ((Std_ReturnType)E_NOT_OK == LVBMS_ClearFault())
                {
                    /*Fault was not cleared*/
                    VoltageFaultStatus = FAULT;
                    vTaskDelay(FAULT_HANDLER_DELAY);
                }
            }
			else
            {
                vTaskDelay(FAULT_HANDLER_DELAY);
            }
		}
        /*Stop Fault signaling*/
		vTaskSuspend(xFaultLED_Handler);
		Dio_WriteChannel(DioConf_DioChannel_RED_FLT, STD_LOW);
	}
}

void LVBMS_FaultLEDHandlerTask(void* parameters)
{
    (void)parameters;
	while(1U)
	{
		Dio_WriteChannel(DioConf_DioChannel_RED_FLT, STD_HIGH);
		vTaskDelay(LED_FAULT_DELAY);
		Dio_WriteChannel(DioConf_DioChannel_RED_FLT, STD_LOW);
		vTaskDelay(LED_FAULT_DELAY);
	}
}

#ifdef __cplusplus
}
#endif

/** @} */

