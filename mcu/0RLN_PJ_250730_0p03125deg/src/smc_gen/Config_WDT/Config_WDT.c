/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2022 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name        : Config_WDT.c
* Component Version: 1.11.0
* Device(s)        : R5F566TKFxFP
* Description      : This file implements device driver for Config_WDT.
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "Config_WDT.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_WDT_Create
* Description  : This function initializes the WDT module
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_WDT_Create(void)
{
	/* Set control registers */
	WDT.WDTCR.WORD = _0003_WDT_TIMEOUT_16384 | _00F0_WDT_CLOCK_DIV128 | _0300_WDT_WINDOW_END_0 | 
                     _3000_WDT_WINDOW_START_100;
#ifdef	WDT_USE_NMI
	WDT.WDTRCR.BYTE = _00_WDT_NMI_INTERRUPT;

    /* Enable NMI interrupt */
	ICU.NMIER.BIT.WDTEN = 1U;
#else
	WDT.WDTRCR.BYTE = _80_WDT_RESET_OUTPUT;
#endif

	R_Config_WDT_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_WDT_Restart
* Description  : This function restarts WDT module
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_WDT_Restart(void)
{
	/* Refreshed by writing 00h and then writing FFh */
	WDT.WDTRR = 0x00U;
	WDT.WDTRR = 0xFFU;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
