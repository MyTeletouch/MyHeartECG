/******************************************************************************
* File Name: uart_debug.h
*
* Version: 1.0
*
* Description: This file contains the macros that are used for UART based 
*              debug
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
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
* written permission of Cypress.
*
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
/******************************************************************************
* This file contains the macros that are used for UART based debug. Note that 
* enabling debug reduces performance and power efficiency. It also increases 
* the code size.
*
* If the debug is enabled, you should set up a serial port terminal emulator 
* such as Tera Term, or HyperTerminal with these settings to view the debug
* information :
*
*   Baud rate     :  115200
*   Data size     :  8-bit
*   Parity        :  None
*   Stop          :  1-bit
*   Flow Control  :  None  
*******************************************************************************/

/* Include guard */
#ifndef UART_DEBUG_H
#define UART_DEBUG_H

/* Header file includes */     
#include "project.h"    
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
  
/**
 * (true) enables UART based debug and (false) disables it. Note that 
 * enabling debug reduces performance, power efficiency and increases 
 * code size. Make sure to enable the UART_DEBUG component in TopDesign 
 * schematic if debug is enabled 
 */    
#define UART_DEBUG_ENABLE    (false)
    
/** 
 * Declare the macros used for UART based debug. Make sure to enable the
 * UART_DEBUG component in TopDesign schematic if debug is enabled 
 */    
#if (UART_DEBUG_ENABLE)
    /* UART component used for STDIO functions */
    #define UART_STDIO          (DEBUG_UART_HW)
        
    /* Initializes the UART component used for STDIO functions */
    #define DebugPrintfInit()  (DEBUG_UART_Start())
    /* Function macro that sends formatted output to STDOUT */
    #define DebugPrintf(...)     (printf(__VA_ARGS__))
           
    #define WAIT_FOR_UART_TX_COMPLETE    while(Cy_SCB_GetNumInTxFifo(DEBUG_UART_SCB__HW) + \
                                    Cy_SCB_GetTxSrValid(DEBUG_UART_SCB__HW))

    #define Uart_LowPower_Readiness() \
        ((DEBUG_UART_GetNumInTxFifo() == 0u)&& \
        (Cy_SCB_GetTxSrValid(DEBUG_UART_SCB__HW)== 0u))
/**
 * Declaration of empty or default value macros if the debug is not enabled
 * for efficient code generation. Make sure to disable the UART_DEBUG 
 * component in TopDesign schematic for reducing leakage 
 */
#else
    #define UART_STDIO              (NULL)  
    #define DebugPrintfInit()         
    #define DebugPrintf(...)
    #define Task_DebugInit()
    #define Task_DebugPrintf(...)
    #define WAIT_FOR_UART_TX_COMPLETE   
    #define Uart_LowPower_Readiness()   (NULL)
#endif  /* UART_DEBUG_ENABLE */ 
    
#endif /* UART_DEBUG_H */ 

/* [] END OF FILE */
