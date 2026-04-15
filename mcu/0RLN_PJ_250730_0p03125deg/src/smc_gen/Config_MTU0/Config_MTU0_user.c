/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.	 ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
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
* File Name		   : Config_MTU0_user.c
* Component Version: 1.12.0
* Device(s)		   : R5F566TKFxFP
* Description	   : This file implements device driver for Config_MTU0.
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
#include "Config_MTU0.h"
/* Start user code for include. Do not edit comment generated here */
#include "DD_ENCODER.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"
#include "MCU.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
static volatile u16	u16_MTU0_edgeCycleA;
static volatile u16	u16_MTU0_edgeCycleB;
static volatile u16	u16_MTU0_edgeCycleC;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_MTU0_Create_UserInit
* Description  : This function adds user code after initializing the MTU0 channel
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU0_Create_UserInit(void)
{
	/* Start user code for user init. Do not edit comment generated here */
	/* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_MTU0_tgia0_interrupt
* Description  : This function is TGIA0 interrupt service routine
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_PERIA_INTA208
#pragma interrupt r_Config_MTU0_tgia0_interrupt(vect=VECT(PERIA,INTA208),fint)
#else
#pragma interrupt r_Config_MTU0_tgia0_interrupt(vect=VECT(PERIA,INTA208))
#endif
static void r_Config_MTU0_tgia0_interrupt(void)
{
	/* Start user code for r_Config_MTU0_tgia0_interrupt. Do not edit comment generated here */
	IR(PERIA,INTA208) = 0U;
	IR(PERIA,INTA208) = 0U;
	MTU0.TCNT = 0U;
	V_DD_ENC_EdgeCycle( MTU0.TGRA );
	/* for DEBUG */
	u16_MTU0_edgeCycleA = MTU0.TGRA;
	/* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_MTU0_tgib0_interrupt
* Description  : This function is TGIB0 interrupt service routine
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_PERIA_INTA209
#pragma interrupt r_Config_MTU0_tgib0_interrupt(vect=VECT(PERIA,INTA209),fint)
#else
#pragma interrupt r_Config_MTU0_tgib0_interrupt(vect=VECT(PERIA,INTA209))
#endif
static void r_Config_MTU0_tgib0_interrupt(void)
{
	/* Start user code for r_Config_MTU0_tgib0_interrupt. Do not edit comment generated here */
	IR(PERIA,INTA209) = 0U;
	MTU0.TCNT = 0U;
	V_DD_ENC_EdgeCycle( MTU0.TGRB );
	/* for DEBUG */
	u16_MTU0_edgeCycleB = MTU0.TGRB;
	/* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_MTU0_tgic0_interrupt
* Description  : This function is TGIC0 interrupt service routine
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_PERIA_INTA210
#pragma interrupt r_Config_MTU0_tgic0_interrupt(vect=VECT(PERIA,INTA210),fint)
#else
#pragma interrupt r_Config_MTU0_tgic0_interrupt(vect=VECT(PERIA,INTA210))
#endif
static void r_Config_MTU0_tgic0_interrupt(void)
{
	/* Start user code for r_Config_MTU0_tgic0_interrupt. Do not edit comment generated here */
	IR(PERIA,INTA210) = 0U;
	MTU0.TCNT = 0U;
	V_DD_ENC_EdgeCycle( MTU0.TGRC );
	/* for DEBUG */
	u16_MTU0_edgeCycleC = MTU0.TGRC;
	/* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_MTU0_tciv0_interrupt
* Description  : This function is TCIV0 interrupt service routine
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_PERIA_INTA212
#pragma interrupt r_Config_MTU0_tciv0_interrupt(vect=VECT(PERIA,INTA212),fint)
#else
#pragma interrupt r_Config_MTU0_tciv0_interrupt(vect=VECT(PERIA,INTA212))
#endif
static void r_Config_MTU0_tciv0_interrupt(void)
{
	/* Start user code for r_Config_MTU0_tciv0_interrupt. Do not edit comment generated here */
	IR(PERIA,INTA212) = 0U;
	V_DD_ENC_TimerOverflow();
	/* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
