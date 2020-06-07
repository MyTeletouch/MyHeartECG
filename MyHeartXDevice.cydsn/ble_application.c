/******************************************************************************
* File Name: ble_application.h
*
* Version: 1.20
*
* Description: This file contains BLE related functions.
*
* Related Document: CE222004_PSoC6_BLE_MultiSlave.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*
*******************************************************************************
* Copyright (2017-2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/
/* Header files */
#include "ble_application.h"
#include "ble_custom_service_config.h"
#include "health_thermometer_service.h"
#include "AD8232ECG.h"
#include "rgb_led.h"
#include "uart_debug.h"
#include "math.h"

/* Macro used for application timeout */
#define TIMEOUT (120u)  /* 250 msec * 120 = 30 sec                    */
                        /* MCWDT will generate interrupt at every   */
                        /* 250 msec.                                 */

/* Variable used for custom service */
static uint8_t customServiceData[CUSTOM_SERVICE_128BIT_RW_SERVICE_LEN];
/* Variable used for custom notification */
static uint8_t customNotificationData[CUSTOM_NOTIFICATIN_SERVICE_LEN];
/* Variable used to maintain connection information */
static cy_stc_ble_conn_handle_t appConnHandle[CY_BLE_CONN_COUNT];

/* Flag variables */
static bool mcwdtInterruptFlag = false;
static bool htsIndication = false;
static bool notificationPending = false;
static bool ecgNotify = true;

/** 
 * These static functions are used by the BLE application. These are not available 
 * outside this file. See the respective function definitions for more 
 * details. 
 */
static void StackEventHandler(uint32_t eventType, void* eventParam);
static void CallBackHts(uint32 event, void *eventParam);
static void StartAdvertisement(void);
static void HandleWriteRequest_RgbLed(cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam);
static void HandleWriteRequest_CustomService(cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam);
static void UpdateGattDB(cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam);
static void SendCustomNotification(void);
static void SendECG(void);
static void SendHtsIndication(void);
static void MCWDT_InterruptHandler(void);
static void InitMcwdt(void);

/*******************************************************************************
* Function Name: void BleApplicationInit(void)
********************************************************************************
*
* Summary:
*  Function that performs all required initialization for this project, that 
*  includes
*  - initialization of the BLE component with a custom event handler
*  - initialization of the ADC
*  - initialization of the MCWDT
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void BleApplicationInit(void)
{
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t  bleApiResult;
    
    /* Variable used to store the stack version */
    cy_stc_ble_stack_lib_version_t  stackVersion;
    
    /* Initialize ADC for ECG sensing */
    InitECG();
    /* Initialize MCWDT */
    InitMcwdt();
    
    /* Start Host of BLE Component and register generic event handler */
    bleApiResult = Cy_BLE_Start(StackEventHandler);
    if(bleApiResult == CY_BLE_SUCCESS)
    {
        DebugPrintf("Success   : BLE - Stack initialization\r\n");
    }
    else
    {
        /* BLE stack initialization failed */
        DebugPrintf("Failure!  : BLE  - Stack initialization. Error Code: 0x%X\r\n", bleApiResult);
        /* Halt CM4 */
        CY_ASSERT(0u);
    }
    
    /* Get the stack version */
	bleApiResult = Cy_BLE_GetStackLibraryVersion(&stackVersion);
    
    if(bleApiResult == CY_BLE_SUCCESS)
    {
        DebugPrintf("Info      : Stack Version: %d.%d.%d.%d \r\n", stackVersion.majorVersion, 
        stackVersion.minorVersion, stackVersion.patch, stackVersion.buildNumber);
    }
    else
    {
        DebugPrintf("Failure!  : GetStackLibraryVersion API error 0x%2.2X\r\n", bleApiResult);
    }
    
    /* Register the HTS specific callback handler */
    bleApiResult = Cy_BLE_HTS_RegisterAttrCallback(CallBackHts);
    if (bleApiResult == CY_BLE_SUCCESS )
    {
        DebugPrintf("Info      : BLE HTS registration completed \r\n");
    }
    else
    {
        DebugPrintf("Info      : BLE HTS registration failed! \r\n");
    }
}

/*******************************************************************************
* Function Name: void BleProcessEvent(void)
********************************************************************************
*
* Summary:
*  Function that process the BLE events and handles BLE services
*
* Parameters:  
*  None
*
* Return: 
*  None
*
*******************************************************************************/
void BleProcessEvent(void)
{
    /* Local variables */
    static uint32_t timeoutVar = 0;
    uint8_t activeConnCount; 
    
    /* Process pending BLE event */
    Cy_BLE_ProcessEvents();

    /* Send ECG status */
    if(ecgNotify)
    {
        ecgNotify = true;//todo: for initial test only
        SendECG();
    }
    
    /* Send HTS indication */
    if(htsIndication) 
    {
        SendHtsIndication();
    }
    /* Send notification */
    if(notificationPending)
    {
        notificationPending = false;
        SendCustomNotification();
    }

    /* Process MCWDT interrupt */
    if(mcwdtInterruptFlag)
    {
        mcwdtInterruptFlag = false;
        if(Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_ADVERTISING)
        {
            /* Toggle orange LED */
            Cy_GPIO_Inv(Pin_LED_Orange_PORT, Pin_LED_Orange_NUM);
        }
        activeConnCount = Cy_BLE_GetNumOfActiveConn();
        /* If no device is connected after advertising 30 sec go to hibernate mode */
        if(activeConnCount == 0)
        {
            timeoutVar++;
            if(timeoutVar >= TIMEOUT)
            {
                /* Turn on red LED */
                Cy_GPIO_Clr(Pin_LED_Red_PORT, Pin_LED_Red_NUM);
                Cy_GPIO_Set(Pin_LED_Orange_PORT, Pin_LED_Orange_NUM);
                /* Stop BLE */
                Cy_BLE_Stop();
            }
        }
        else if(activeConnCount == CY_BLE_CONN_COUNT)
        {
            /* Turn on orange LED */
            Cy_GPIO_Clr(Pin_LED_Orange_PORT, Pin_LED_Orange_NUM);
        }
        
    }
}

/*******************************************************************************
* Function Name: bool Ble_LowPowerReadiness(void)
********************************************************************************
*
* Summary:
*  This function returns the low power readiness of the BLE
*
* Parameters:  
*  None
*
* Return: 
*  None
*
*******************************************************************************/
bool Ble_LowPowerReadiness(void)
{
    /* Return "true" if the BLE is on, "false" otherwise */
    return (Cy_BLE_GetState() == CY_BLE_STATE_ON) ? true : false;
}

/*******************************************************************************
* Function Name: static void CallBackHts(uint32_t event, void *eventParam)
********************************************************************************
*
* Summary:
*  This is an event callback function to receive events from the BLE Component,
*  which are specific to Health Thermometer Service.
*
* Parameters:  
*  event:       Event for Health Thermometer Service.
*  eventParams: Event parameter for Health Thermometer Service.
*
* Return: 
*  None
*
*******************************************************************************/
static void CallBackHts(uint32_t event, void *eventParam)
{   
    static uint8_t htsRequestCount = 0;
    
    cy_stc_ble_hts_char_value_t htsCharValue = *(cy_stc_ble_hts_char_value_t*)eventParam;
    
    switch(event)
    {
        /* This event is received when indication are enabled by the central */
        case CY_BLE_EVT_HTSS_INDICATION_ENABLED:
        {
            DebugPrintf("Info      : HTS indication enabled by Device : %X\r\n",\
                CY_BLE_CONN_COUNT - htsCharValue.connHandle.attId);

            /* Tracking how many devices requested for HTS */
            htsRequestCount++;
            /* Set the htsIndication flag */
            htsIndication = true;
            break;
        }
        /* This event is received when indication is disabled by the central */
        case CY_BLE_EVT_HTSS_INDICATION_DISABLED:
        {
            DebugPrintf("Info      : HTS indication disabled by Device : %X\r\n",\
                CY_BLE_CONN_COUNT - htsCharValue.connHandle.attId);
            if(--htsRequestCount == 0)
            {
                /* Reset the htsIndiciation flag */
                htsIndication = false;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    
    /* Remove compiler warning */
    (void)htsCharValue;
}

/*******************************************************************************
* Function Name: static void StackEventHandler(uint32_t eventType, void* eventParam)
********************************************************************************
*
* Summary:
*  This is an event callback function Event callback function to receive events 
*  from BLE stack.
*
* Parameters:  
*  event: event returned
*  eventParameter: link to value of the events returned
*
* Return: 
*  None
*
*******************************************************************************/
static void StackEventHandler(uint32_t eventType, void* eventParam)
{
    /* Variable for storing write request parameter */
    static cy_stc_ble_gatts_write_cmd_req_param_t writeReqParameter;
    
    switch(eventType)
    {
        /** 
         * There are some events generated by the BLE component
         *  that are not required for this code example. 
         */

        /* This event is received when the BLE stack is started */
        case CY_BLE_EVT_STACK_ON:
        {
            DebugPrintf("Info      : BLE - Stack on event\r\n");
            /* Start advertisement */
            StartAdvertisement();
            break;
        }
        /* This event is received when there is a timeout */
        case CY_BLE_EVT_TIMEOUT:
        {
            DebugPrintf("Info      : BLE - Timeout event\r\n");
            break;
        }
        /* This event is received when stack shutdown is complete */
        case CY_BLE_EVT_STACK_SHUTDOWN_COMPLETE:
        {
            DebugPrintf("Info      : BLE - Stack shutdown complete\r\n");
            DebugPrintf("Info      : Entering hibernate mode\r\n");
            WAIT_FOR_UART_TX_COMPLETE;
            /* Enter hibernate node */
            Cy_SysPm_Hibernate();
            break;
        }

        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~ GAP Events ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        /* This event indicates peripheral device has started/stopped advertising */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        {
            DebugPrintf("Info      : BLE - Advertisement start/stop event\r\n");
            break;
        }
        /* This event is generated at the GAP Peripheral end after connection 
           is completed with peer Central device */
        case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
        {
            DebugPrintf("Info      : BLE - GAP device connected\r\n");
            /* start advertisement for next device */
            StartAdvertisement();
            break;
        }
        /* This event is triggered instead of 'CY_BLE_EVT_GAP_DEVICE_CONNECTED', 
           if Link Layer Privacy is enabled in component customizer */
        case CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE:
        {
            DebugPrintf("Info      : BLE - GAP enhance connection complete\r\n");
            /* start advertisement for next device */
            StartAdvertisement();
            break;
        }
        /* This event is generated when disconnected from remote device or 
           failed to establish connection */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {
            DebugPrintf("Info      : BLE - GAP device disconnected\r\n");
            /* start advertisement for next device */
            StartAdvertisement();
            break;
        }

        /*~~~~~~~~~~~~~~~~~~~~~~~~~ GATT Events ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        /* This event is generated at the GAP Peripheral end after connection 
           is completed with peer Central device */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            DebugPrintf("Info      : BLE - GATT device connected\r\n");
            DebugPrintf("Info      : Connected device : %u \r\n", Cy_BLE_GetNumOfActiveConn());
            appConnHandle[(*(cy_stc_ble_conn_handle_t*)eventParam).attId] = 
                (*(cy_stc_ble_conn_handle_t*)eventParam);
            break;
        }
        /* This event is generated at the GAP Peripheral end after 
           disconnection */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            DebugPrintf("Info      : BLE - GATT device disconnected\r\n");
            DebugPrintf("Info      : Connected device : %u \r\n", Cy_BLE_GetNumOfActiveConn());
            
            /* Remove disconnected device from database*/
            appConnHandle[(*(cy_stc_ble_conn_handle_t *)eventParam).attId].attId = \
                                                    CY_BLE_INVALID_CONN_HANDLE_VALUE ;

            appConnHandle[(*(cy_stc_ble_conn_handle_t *)eventParam).attId].bdHandle = \
                                                    CY_BLE_INVALID_CONN_HANDLE_VALUE ;
            break;
        }
        
        /* This event is received when Central device sends a Write command
           on an Attribute */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
        {
            DebugPrintf("Info      : BLE - GATT write request \r\n");
            writeReqParameter = *(cy_stc_ble_gatts_write_cmd_req_param_t *)eventParam;
            
            /* Send the response to the write request received. */
            Cy_BLE_GATTS_WriteRsp(writeReqParameter.connHandle);
            
            if(writeReqParameter.handleValPair.attrHandle == RGB_CCCD_HANDLE)
            {
                HandleWriteRequest_RgbLed(writeReqParameter);
            }
            else if(writeReqParameter.handleValPair.attrHandle == CUSTOM_SERVICE_128BIT_RW_CCC_HANDLE)
            {
                HandleWriteRequest_CustomService(writeReqParameter);
            }
            /* Update flag to send notification */
            notificationPending = true;
            break;
        }
        /* This event is triggered when a read received from GATT 
           client device */
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
        {
            DebugPrintf("Info      : BLE - GATT read request \r\n");
            break;
        }
        
        default: 
        {
            break;
        } 
    }
}

/*******************************************************************************
* Function Name: static void StartAdvertisement(void)
********************************************************************************
*
* Summary:
*  This function starts advertisement if BLE is not already advertising and 
*  number of active connection is less than CY_BLE_CONN_COUNT
*
* Parameters:  
*  None
*
* Return: 
*  None
*
*******************************************************************************/
static void StartAdvertisement(void)
{
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t bleApiResult;    
    if((Cy_BLE_GetNumOfActiveConn() < CY_BLE_CONN_COUNT) && \
    (Cy_BLE_GetAdvertisementState() != CY_BLE_ADV_STATE_ADVERTISING))
    {
        bleApiResult = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, \
            CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        if(bleApiResult != CY_BLE_SUCCESS)
        {
            DebugPrintf("Failure!  : BLE - Start advertisement 0x%X\r\n", bleApiResult);
        }
    }
}

/*******************************************************************************
* Function Name: static void HandleWriteRequest_RgbLed(
*                       cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam)
********************************************************************************
*
* Summary:
*  This function handles write request for RGB LED custom service
*
* Parameters:  
*  cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam: Write request parameter
*
* Return: 
*  None
*
*******************************************************************************/
static void HandleWriteRequest_RgbLed(cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam)
{
    /* Variable used to store RGB LED data */
    static rgb_led_data_t rgbData;
    
    /* Extract the write value sent by the Client for RGB LED Color characteristic */
    memcpy(rgbData.valueArray, writeReqParam.handleValPair.value.val,
        RGB_DATA_LEN);
    
    /* Update GATT database so that the central reads the new user data */
    UpdateGattDB(writeReqParam);
    /* Update custom notification data */
    customNotificationData[CUSTOM_NOTIFICATION_DEVICE_ID_INDEX] = CY_BLE_CONN_COUNT - writeReqParam.connHandle.attId;
    customNotificationData[CUSTOM_NOTIFICATION_SERVICE_ID_INDEX] = CUSTOM_SERVICE_RGB;
    
    /* Update the RGB LED color */
    SetColor(rgbData.colorAndIntensity);
}

/*******************************************************************************
* Function Name: static void HandleWriteRequest_CustomService(
*                   cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam)
********************************************************************************
*
* Summary:
*  This function handles write request for 128-bit read write custom service
*
* Parameters:  
*  cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam: Write request parameter
*
* Return: 
*  None
*
*******************************************************************************/
static void HandleWriteRequest_CustomService(cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam)
{
    /* Extract the write value sent by the Client for RGB LED Color characteristic */
    memcpy(customServiceData, writeReqParam.handleValPair.value.val,
        CUSTOM_SERVICE_128BIT_RW_SERVICE_LEN);
    
    /* Update GATT database so that the central reads the new user data */
    UpdateGattDB(writeReqParam);
    /* Update custom notification data */
    customNotificationData[CUSTOM_NOTIFICATION_DEVICE_ID_INDEX] = CY_BLE_CONN_COUNT - writeReqParam.connHandle.attId;
    customNotificationData[CUSTOM_NOTIFICATION_SERVICE_ID_INDEX] = CUSTOM_SERVICE_128BIT_READ_WRITE;
    
    DebugPrintf("Info      : BLE - 128-bit read write custom service data updated\r\n");
}

/*******************************************************************************
* Function Name: static void UpdateGattDB(
*                       cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam)
********************************************************************************
*
* Summary:
*  This function update the value field of the specified attribute  in the GATT 
*  database of a GATT Server by a peer device.
*
* Parameters:  
*  cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam: Write request paramete
*
* Return: 
*  None
*
*******************************************************************************/
static void UpdateGattDB(cy_stc_ble_gatts_write_cmd_req_param_t writeReqParam)
{
    /* Local variable that stores custom service data parameters */
    cy_stc_ble_gatt_handle_value_pair_t  handleValue = 
    {
        .attrHandle = writeReqParam.handleValPair.attrHandle,
        .value      = writeReqParam.handleValPair.value,
    };
    Cy_BLE_GATTS_WriteAttributeValuePeer(&writeReqParam.connHandle, &handleValue);
}


/*******************************************************************************
* Function Name: static void SendCustomNotification(void)
********************************************************************************
*
* Summary:
*  This function sends custom notification to all the connected device if 
*  notification is enabled. 
*
* Parameters:  
*  None
*
* Return: 
*  None
*
*******************************************************************************/
static void SendCustomNotification(void)
{
    uint32_t i;
    /* Send notification top all the connected device */
    for(i = 0; i < CY_BLE_CONN_COUNT; i++)
    {
        
        if((Cy_BLE_GetConnectionState(appConnHandle[i]) ==  CY_BLE_CONN_STATE_CONNECTED )&&
           (Cy_BLE_GATTS_IsNotificationEnabled(&appConnHandle[i], CUSTOM_NOTIFICATION_CCC_HANDLE)))
        {
            /*  Local variable used for storing notification parameter */
            cy_stc_ble_gatts_handle_value_ntf_t customNotificationHandle = 
            {
                .connHandle = appConnHandle[i],
                .handleValPair.attrHandle   = CUSTOM_NOTIFICATION_CCC_HANDLE,
                .handleValPair.value.val    = customNotificationData,
                .handleValPair.value.len    = CUSTOM_NOTIFICATIN_SERVICE_LEN
            };
            
            /* Send the updated handle as part of attribute for notifications */
            Cy_BLE_GATTS_Notification(&customNotificationHandle);
        }
    }
}

/*******************************************************************************
* Function Name: static void SendECG(void)
********************************************************************************
*
* Summary:
*  This function sends ECG value to central device 
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void SendECG(void)
{
    uint8_t ecg = GetECG();
    customNotificationData[CUSTOM_NOTIFICATION_DEVICE_ID_INDEX] = ecg;
    customNotificationData[CUSTOM_NOTIFICATION_SERVICE_ID_INDEX] = CUSTOM_SERVICE_ECG;

    uint32_t i;
    /* Send notification top all the connected device */
    for(i = 0; i < CY_BLE_CONN_COUNT; i++)
    {
        
        if((Cy_BLE_GetConnectionState(appConnHandle[i]) ==  CY_BLE_CONN_STATE_CONNECTED )&&
           (Cy_BLE_GATTS_IsNotificationEnabled(&appConnHandle[i], CUSTOM_NOTIFICATION_CCC_HANDLE)))
        {
            /*  Local variable used for storing notification parameter */
            cy_stc_ble_gatts_handle_value_ntf_t customNotificationHandle = 
            {
                .connHandle = appConnHandle[i],
                .handleValPair.attrHandle   = CUSTOM_NOTIFICATION_CCC_HANDLE,
                .handleValPair.value.val    = customNotificationData,
                .handleValPair.value.len    = CUSTOM_NOTIFICATIN_SERVICE_LEN
            };
            
            /* Send the updated handle as part of attribute for notifications */
            Cy_BLE_GATTS_Notification(&customNotificationHandle);
        }
    }
}


/*******************************************************************************
* Function Name: static void SendHtsIndication(void)
********************************************************************************
*
* Summary:
*  This function sends HTS indication to central device 
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void SendHtsIndication(void)
{
    uint32_t i;
    
    /* Temporary variables to hold Health Thermometer Characteristic information */
    static uint8 valueArray[HTS_CHARACTERISTIC_SIZE];
    temperature_data_t tempData;    
    float temperature = 36.6f;//GetTemperature(); //Dummy value for now - TODO: Add Thermometer module
    
    /**
     * Convert from IEEE-754 single precision floating point format to
     * IEEE-11073 FLOAT, which is mandated by the health thermometer
     * characteristic. 
     */
    tempData.temeratureValue = (int32_t)(roundf(temperature * IEEE_11073_MANTISSA_SCALER));
    tempData.temperatureArray[IEEE_11073_EXPONENT_INDEX] = IEEE_11073_EXPONENT_VALUE;         
    
    /* Read Health Thermometer Characteristic from GATT DB */
    if(CY_BLE_SUCCESS == Cy_BLE_HTSS_GetCharacteristicValue
                         (CY_BLE_HTS_TEMP_MEASURE,
                          HTS_CHARACTERISTIC_SIZE, valueArray))
    { 
        /* Update temperature value in the characteristic */
        memcpy(&valueArray[HTS_TEMPERATURE_DATA_INDEX], tempData.temperatureArray, HTS_TEMPERATURE_DATA_SIZE);
        
        for(i = 0; i < CY_BLE_CONN_COUNT; i++)
        {
            /* Send indication to the central */
            Cy_BLE_HTSS_SendIndication(appConnHandle[i], 
                                       CY_BLE_HTS_TEMP_MEASURE,
                                       HTS_CHARACTERISTIC_SIZE, valueArray);   
        }
    }
}

/*******************************************************************************
* Function Name: void MCWDT_InterruptHandler(void)
********************************************************************************
* Summary:
*  Interrupt service routine for the MCWDT interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void MCWDT_InterruptHandler(void)
{
    /* Clear the MCWDT peripheral interrupt */
    Cy_MCWDT_ClearInterrupt(MCWDT_Timer_4Hz_HW, CY_MCWDT_CTR0);
    
    /* Set interrupt flag */
    mcwdtInterruptFlag = true;
    
    /* Clear CM4 NVIC pending interrupt for MCWDT */
    NVIC_ClearPendingIRQ(MCWDT_isr_cfg.intrSrc);
}

/*******************************************************************************
* Function Name: static void InitMcwdt(void)
********************************************************************************
*
* Summary:
*  Function that initializes the MCWDT To generate interrupt every 250 msec.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void InitMcwdt(void)
{
    /**
     * Enable 4 Hz free-running MCWDT_Timer_4Hz counter 0. MCWDT_Timer_4Hz_config
     * structure is defined by the MCWDT_PDL component based on parameters entered
     * in the customizer. 
     */
    Cy_MCWDT_Init(MCWDT_Timer_4Hz_HW, &MCWDT_Timer_4Hz_config);
    Cy_MCWDT_Enable(MCWDT_Timer_4Hz_HW, CY_MCWDT_CTR0, 93 /* 2 LFCLK cycles */);
    /* Unmask the MCWDT counter 0 peripheral interrupt */
    Cy_MCWDT_SetInterruptMask(MCWDT_Timer_4Hz_HW, CY_MCWDT_CTR0);
 
    /** 
     * Configure ISR connected to MCWDT_Timer_4Hz interrupt signal. 
     * MCWDT_Timer_4Hz_isr_cfg structure is defined by the SYSINT_PDL component 
     * based on parameters entered in the customizer. 
     */
    Cy_SysInt_Init(&MCWDT_isr_cfg, &MCWDT_InterruptHandler);
    /* Clear CM4 NVIC pending interrupt for MCWDT */
    NVIC_ClearPendingIRQ(MCWDT_isr_cfg.intrSrc);
    /* Enable CM4 NVIC MCWDT interrupt */
    NVIC_EnableIRQ(MCWDT_isr_cfg.intrSrc);
}

/* [] END OF FILE */
