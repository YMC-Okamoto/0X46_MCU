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
* File Name        : r_smc_interrupt.c
* Version          : 1.2.20
* Device(s)        : R5F566TKFxFP
* Description      : This file implements interrupt setting.
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
#include "r_smc_interrupt.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Interrupt_Create
* Description  : This function Used to set the fast interrupt or group interrupt 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Interrupt_Create(void)
{
    uint32_t * vector_location;
    
    /* Disable group AL0 interrupt*/
    IEN(ICU,GROUPAL0) = 0U;
    
    /* Set fast interrupt */
    ICU.FIR.WORD = _00E8_FAST_VECTOR_NUM;

    /* Find the start of the maskable interrupt vector table */
    vector_location = (uint32_t *)get_intb();

    /* Move to the location of the address of the selected interrupt */
    vector_location += ICU.FIR.BIT.FVCT;

    /* Store the address of this vector in the FINTV register */
    set_fintv((void *) * vector_location);

    /* Enable the fast interrupt feature */
    ICU.FIR.BIT.FIEN = 1U;

    /* Set group AL0 interrupt priority level */
    IPR(ICU,GROUPAL0) = _0A_ICU_PRIORITY_LEVEL10;

    /* Enable group AL0 interrupt */
    IEN(ICU,GROUPAL0) = 1U;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
