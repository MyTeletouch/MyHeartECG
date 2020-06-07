/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <math.h>
#include "AD8232ECG.h"

/*******************************************************************************
* Function Name: void InitECG(void)
********************************************************************************
* Summary:
*  This functions initializes the ADC and put it to sleep so that entering low
*  power modes won't affect the ADC configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void InitECG(void)
{
    ADC_ECG_Start();
    ADC_ECG_Sleep();
}

/*******************************************************************************
* Function Name: float GetECG(void)
********************************************************************************
* Summary:
*  This functions calculates the ECG value 
*
* Parameters:
*  None
*
* Return:
*  float : ECG value 
*
*******************************************************************************/
uint8_t GetECG(void)
{   
    uint8_t ecg;
    
    /* Wake up the ADC and start conversion */
    ADC_ECG_Wakeup();
    ADC_ECG_StartConvert();
    ADC_ECG_IsEndConversion(CY_SAR_WAIT_FOR_RESULT);
    
    /* Read the ADC value */
    ecg = ADC_ECG_GetResult16(ECG_CHANNEL);
   
    /* Put the ADC to sleep so that entering low power modes won't affect
       the ADC configuration */
    ADC_ECG_Sleep();
    
    /* Return the ecg value */
    return ecg;
}

/* [] END OF FILE */
