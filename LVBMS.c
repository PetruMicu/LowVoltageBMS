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
/* Semaphore used to send and receive fault status from battery cell controller*/
extern SemaphoreHandle_t xFaultSemaphore;
/*Mutex used for device communication to ensure exclusive access to Bms_TD_Send*/
extern SemaphoreHandle_t xCommMutex;
/*Task handlers for LVBMS tasks*/
extern TaskHandle_t xMeasureHandler;
extern TaskHandle_t xControlHandler;
extern TaskHandle_t xFaultHandler;
extern TaskHandle_t xFaultLED_Handler;

/*Store the time of CC reading*/
static TickType_t CurrentTime = 0;
static TickType_t OldTime = 0;

/*Keeps track of the fault status*/
static volatile LVBMS_FaultStatus FaultStatus = NO_FAULT;
/*Stores last measurements and cell states*/
static LVBMS_Measurements Measurements;
/*Stores cell fault and balancing information*/
static LVBMS_CellState CellInfo = {.CTState = {CELL_OK}, .CTBalEnabled = {false}};
/*Variable used to reduce transmission overhead, each cell state ocupies 2bits*/
static uint16 CTState = 0U;
/*Variable used to reduce transmission overhead, each cell balancing status ocupies 1bit*/
static uint8 CTBalEnabled = 0U;
/*Keeps track of state of the system (idle, charging or discharging)*/
static LVBMS_State SystemState;
/*Stores State of Charge of the battery pack*/
static float SoC;
/*Stores State of Health of the battery pack*/
static float SoH;
/*Stores Depth of Discharge of the battery pack*/
static float DoD;

FMSTR_TSA_TABLE_BEGIN(LVBMS_CellParameters)
   FMSTR_TSA_RO_VAR(Measurements.CTVoltage, FMSTR_TSA_POINTER)
   FMSTR_TSA_RO_VAR(CTState, FMSTR_TSA_UINT16)
   FMSTR_TSA_RO_VAR(CTBalEnabled, FMSTR_TSA_UINT8)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(LVBMS_Parameters)
    FMSTR_TSA_RW_VAR(SystemState, FMSTR_TSA_USERTYPE(LVBMS_State))
    FMSTR_TSA_RO_VAR(FaultStatus, FMSTR_TSA_USERTYPE(LVBMS_FaultStatus))
    FMSTR_TSA_RO_VAR(Measurements.Current, FMSTR_TSA_FLOAT)
    FMSTR_TSA_RO_VAR(Measurements.StackVoltage, FMSTR_TSA_FLOAT)
    FMSTR_TSA_RO_VAR(Measurements.PackTemperature, FMSTR_TSA_FLOAT)
    FMSTR_TSA_RO_VAR(SoC, FMSTR_TSA_FLOAT)
    FMSTR_TSA_RO_VAR(SoH, FMSTR_TSA_FLOAT)
FMSTR_TSA_TABLE_END()


FMSTR_TSA_TABLE_LIST_BEGIN()
    FMSTR_TSA_TABLE(LVBMS_Parameters)
	FMSTR_TSA_TABLE(LVBMS_CellParameters)
FMSTR_TSA_TABLE_LIST_END()
/*==================================================================================================
 *                                    FUNCTION PROTOTYPES
 ==================================================================================================*/ 
static void LVBMS_ExtractData(Phy_TDType* TD, uint16* ptrData);

LVBMS_FaultStatus LVBMS_CheckFault(void);
Std_ReturnType LVBMS_ClearFault(void);

/*LVBMS tasks*/
void LVBMS_MeasureTask(void* parameters);
void LVBMS_ControlTask(void* parameters);
void LVBMS_FaultHandlerTask(void* parameters);
void LVBMS_FaultLEDHandlerTask(void* parameters);
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

static void LVBMS_ComputeSoCSoH(LVBMS_Measurements* Results)
{
    float Delta_DoD = (float)(-1.0) * (float)(Results->DeltaQ / (float)C_RATED) * (float)100.0;

    if (Results->Current < 0U)
    {
        /*Discharging*/
        if (Results->StackVoltage > (float)VMIN)
        {
            /*Normal operation*/
            DoD = DoD + Delta_DoD * DISCHARGE_OPERATING_EFFICIENCY;
            SoC = SoH - DoD;
            /*Introduced only for the emulator*/
            if (SoC < 0U)
            {
                SoC = 0U;
            }
        }
        else
        {
            /*Deeply discharged*/
            SoH = DoD;
            /*Introduced only for the emulator*/
            if (SoH < 0U)
            {
                SoH = 0U;
            }
            SoC = SoH;
        }
    }
    else
    {
        /*Charging or regenerative breaking*/
        if ((Results->StackVoltage >= VMAX && Results->StackVoltage <= (VMAX + VMAX* 3/100)) || (SoC >= 100.0))
        {
            /*NOTE: In a real system we would also consider the current*/
            /*Fully charged*/
            SoH = SoC;
        }
        else
        {
            DoD = DoD + Delta_DoD * CHARGE_OPERATING_EFFICIENCY;
            SoC = SoH - DoD;
            /*Introduced only for the emulator*/
            if (SoC > SoH) 
            {
                SoC = SoH;
            }
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
        CTBalEnabled |= (uint8)((CellInfo.CTBalEnabled[idx] & 1U) << idx);
    }
}
/*==================================================================================================
 *                                    GLOBAL FUNCTIONS
 ==================================================================================================*/

void FaultEvent(void)
{
	FaultStatus = FAULT;
	/*Signal Fault Task that a fault has occured*/
	xSemaphoreGiveFromISR(xFaultSemaphore, NULL);
}

void LVBMS_Init(void)
{
	LVBMS_Error ErrorStatus;

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

LVBMS_Error LVBMS_SystemConfig(void)
{
    Phy_ErrorStatusType ErrorStatus = PHY_NO_ERROR;
	Std_ReturnType Status = (Std_ReturnType)E_OK;

    /*Defines critical voltage thresholds*/
    Bcc_772c_MsrThresholdValueType Thresholds = 
    {
        .Overvoltage = MC33772C_TH_ALL_CT_ALL_CT_OV_TH_DEFAULT_ENUM_VAL,
        .Undervoltage = MC33772C_TH_ALL_CT_ALL_CT_UV_TH_DEFAULT_ENUM_VAL
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

    /*Later read these values from I2C EEPROM*/
    SoC = 100.0;
    SoH = 100.0;

    DoD = (float)100.0 - SoC;
    return NO_ERROR;
}

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

Std_ReturnType LVBMS_ReadMeasurements(LVBMS_Measurements* Results)
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
    uint32 NTC_Resistance;
    uint8 idx;

    /*Clear the TD before appending messages*/
    Status = Bms_TD_Clear(&LVBMS_TD);

    /*Pack multiple read requests to retrieve all measurements*/
    Status |= Bcc_772c_CC_GetData(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD);
    Status |= Bcc_772c_MSR_GetData(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, BCC_772C_MSR_TYPE_STACK_CELL_VOLTAGE, &LVBMS_TD);
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

        CCSamples = measurementData[1U];
        AvgCurrent = (float)((float)CCount / (float)CCSamples) * ((float)BCC_772C_V2RES / (float)RSHUNT);
        Results->DeltaQ = (float)(AvgCurrent * (float)(CurrentTime - OldTime) / 3.6e+6);
        /*Current value in mA*/
        Results->Current = (float)(CCIsense.ISense / 1000);
        Results->StackVoltage = (float)STACK_VOLTAGE_VALUE(measurementData[7U]);
        for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
        {
            /*Results are in reversed order: CELL6 ... CELL1*/
            Results->CTVoltage[idx] = (float)CELL_VOLTAGE_VALUE(measurementData[13U - idx]);
        }

        /*NTC_Resistance = (float)((measurementData[15U] & 0x7fff) * 5.0 / 32768.0);*/
        Results->PackTemperature = 0;
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

    while (1U)
    {
        /*Start conversion*/
        Status = LVBMS_StartMeasurements();
        if ((Std_ReturnType)E_OK == Status)
        {
            /*Wait for conversion to finish*/
            vTaskDelay(xDelay_Measurements);
            Status = LVBMS_ReadMeasurements(&Measurements);
            if ((Std_ReturnType)E_OK == Status)
            {
                /*Compute SoC/SoH*/
                LVBMS_ComputeSoCSoH(&Measurements);
                LVBMS_UpdateCellStateInfo();
                /*Send measurements to FreeMASTER*/
                FMSTR_Poll();
            }
        }
        /*Schedule next measurement cycle*/
        vTaskDelay(MEASUREMENTS_DELAY);
    }
}

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
        /*Perform balancing algorithm only when charging*/
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
                /*Check if voltage level is too high and start balancing if not already started*/
                if ((Measurements.CTVoltage[idx] >= (MinCellVoltage + MAX_VOLTAGE_DIFF)) &&
                    (false == CellInfo.CTBalEnabled[idx]))
                {
                    BalConfig.ChannelNo     = (Bcc_772c_ChannelNoType)idx;
                    BalConfig.BalChannelEn  = true;
                    /*Cell balancing is needed*/
                    Status |= Bcc_772c_BAL_SetChannelConfiguration(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &BalConfig, &LVBMS_TD_BAL);
                    CellInfo.CTBalEnabled[idx] |= (bool)(1U << idx);
                }
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

    }
    
}

// void LVBMS_ContactorControl()
// {

// }

void LVBMS_ControlTask(void* parameters)
{
	(void)parameters;
	/*Delay of 10ms*/
	   const TickType_t xDelay = 1000U / portTICK_PERIOD_MS;
    /*Initial state of the battery*/
    SystemState = IDLE;
    while(1)
    {
        /*For debugging only*/
        LVBMS_LED_Update();
        /*Controls load coupling or decoupling based on system parameters*/
        // LVBMS_ContactorControl();
        /*Controls and decide if cell balancing is needed*/
        LVBMS_CellBalanceControl();
        vTaskDelay(xDelay);
    }
}

LVBMS_FaultStatus LVBMS_CheckFault(void)
{
    LVBMS_FaultStatus FaultState = NO_FAULT;
    Std_ReturnType Status = (Std_ReturnType)E_OK;
    Phy_ErrorStatusType ErrorStatus;
    Bcc_772c_RegisterContentType RW;
    uint16 FaultRegister;
    uint8 idx;

    /*Clear the TD before appending messages*/
    Bms_TD_Clear(&LVBMS_TD_FAULT);

    Status = Bcc_772c_FEH_GetFaultStatus(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD_FAULT);
    LVBMS_TD_Comm(&LVBMS_TD_FAULT, &ErrorStatus);

    if (PHY_NO_ERROR == ErrorStatus)
    {
        FaultRegister = LVBMS_TD_FAULT.PhyTD->Response.Data[3U];
        /*Check if undervoltage or overvoltage has been detected*/
        if ((MC33772C_FAULT1_STATUS_CT_UV_FLT(FaultRegister) == 
                MC33772C_FAULT1_STATUS_CT_UV_FLT_UNDERVOLTAGE_ENUM_VAL))
        {
            RW.RegAddr = MC33772C_CELL_UV_FLT_OFFSET;
            RW.RegData = 0U;
            RW.RegMask = 0x3F;
            /*Clear the TD before appending messages*/
            Bms_TD_Clear(&LVBMS_TD_FAULT);

            Status = Bcc_772c_COM_ReadRegisters(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, MC33772C_CELL_UV_FLT_OFFSET, (uint8)1U, &LVBMS_TD_FAULT);
            Status |= Bcc_772c_COM_WriteRegister(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &RW, &LVBMS_TD_FAULT);
            LVBMS_TD_Comm(&LVBMS_TD_FAULT, &ErrorStatus);

            if ((PHY_NO_ERROR == ErrorStatus) && (MC33772C_CELL_UV_FLT_POR_VAL != LVBMS_TD_FAULT.PhyTD->Response.Data[3U]))
            {
                FaultState = FAULT;
                for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
                {
                    /*Stores CELL_OK or CELL_OV*/
                    CellInfo.CTState[idx] = (LVBMS_CellFaultStatus)(((LVBMS_TD_FAULT.PhyTD->Response.Data[3U] >> idx) & 0x0001)*CELL_UV);
                }
            }
        }

        if ((MC33772C_FAULT1_STATUS_CT_OV_FLT(FaultRegister) ==
                    MC33772C_FAULT1_STATUS_CT_OV_FLT_OVERVOLTAGE_ENUM_VAL))
        {
            RW.RegAddr = MC33772C_CELL_OV_FLT_OFFSET;
            RW.RegData = 0U;
            RW.RegMask = 0x3F;
            /*Clear the TD before appending messages*/
            Bms_TD_Clear(&LVBMS_TD_FAULT);

            Status = Bcc_772c_COM_ReadRegisters(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, MC33772C_CELL_OV_FLT_OFFSET, (uint8)1U, &LVBMS_TD_FAULT);
            Status |= Bcc_772c_COM_WriteRegister(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &RW, &LVBMS_TD_FAULT);
            LVBMS_TD_Comm(&LVBMS_TD_FAULT, &ErrorStatus);

            if ((PHY_NO_ERROR == ErrorStatus) && (MC33772C_CELL_OV_FLT_POR_VAL != LVBMS_TD_FAULT.PhyTD->Response.Data[3U]))
            {
                FaultState = FAULT;
                for (idx = 0U; idx < ACTIVE_CELL_NUMBER; idx++)
                {
                    /*Either keeps stored state or stores CELL_UV*/
                    CellInfo.CTState[idx] |= (LVBMS_CellFaultStatus)(((LVBMS_TD_FAULT.PhyTD->Response.Data[3U] >> idx) & 0x0001)*CELL_OV);
                }
            }
        }
    }
    
    return FaultState;
}

Std_ReturnType LVBMS_ClearFault(void)
{
    Std_ReturnType Status;
    Phy_ErrorStatusType ErrorStatus;
    uint8 idx;
    /*Clear the TD before appending messages*/
    Bms_TD_Clear(&LVBMS_TD_FAULT);

    Status = Bcc_772c_FEH_ClearFaultStatus(SPI_BCC_CHAIN_ADDR, SPI_BCC_DEVICE_ADDR, &LVBMS_TD_FAULT);
    LVBMS_TD_Comm(&LVBMS_TD_FAULT, &ErrorStatus);

    if (PHY_NO_ERROR != ErrorStatus)
    {
        return (Std_ReturnType)E_NOT_OK;
    }
    else
    {
        /*Cell States are OK*/
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

    while(1)
	{
        /*Wait for fault detection*/
		xSemaphoreTake(xFaultSemaphore, portMAX_DELAY);
        /*Start Fault signaling*/
		vTaskResume(xFaultLED_Handler);
		while (FaultStatus == FAULT)
		{
            /*Check if fault was fixed*/
            FaultStatus = LVBMS_CheckFault();
            if (NO_FAULT == FaultStatus)
            {
                /*Clear the fault*/
                if ((Std_ReturnType)E_NOT_OK == LVBMS_ClearFault())
                {
                    /*Fault was not cleared*/
                    FaultStatus = FAULT;
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
	while(1)
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

