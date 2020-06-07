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
/* Include Guard */
#ifndef AD8232ECG_H
#define AD8232ECG_H

#include "project.h"

/* ADC channel used to measure ecg voltages */
#define ECG_CHANNEL     (0x00u)

/* Initialize ADCs */
void InitECG(void);
/* Get the ECG data from AD8232 circuit */ 
uint8_t GetECG(void);

#endif /* AD8232ECG_H */
/* [] END OF FILE */
