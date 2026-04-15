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
* File Name		   : Config_MTU6_MTU7.c
* Component Version: 1.11.0
* Device(s)		   : R5F566TKFxFP
* Description	   : This file implements device driver for Config_MTU6_MTU7.
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
#include "Config_MTU6_MTU7.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_MTU6_MTU7_Create
* Description  : This function initializes the MTU6 channel
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU6_MTU7_Create(void)
{
	/* Release MTU channel 6 from stop state */
	MSTP(MTU6) = 0U;

	/* Enable read/write to MTU6, MTU7 registers */
	MTU.TRWERB.BIT.RWE = 1U;

	/* Stop MTU channel 6 counter */
	MTU.TSTRB.BIT.CST6 = 0U;
	MTU.TSTRB.BIT.CST7 = 0U;

	/* Set timer synchronous clear */
	MTU6.TSYCR.BYTE = _00_MTU6_CL0A_OFF | _00_MTU6_CL0B_OFF | _00_MTU6_CL0C_OFF | _00_MTU6_CL0D_OFF | 
					  _00_MTU6_CL1A_OFF | _00_MTU6_CL1B_OFF | _00_MTU6_CL2A_OFF | _00_MTU6_CL2B_OFF;

	/* Set A/D conversion signal output for ADSM0, ADSM1 pins */
	MTU.TADSTRGR0.BYTE = _00_MTU_TADSMEN_DISABLE;
	MTU.TADSTRGR1.BYTE = _00_MTU_TADSMEN_DISABLE;

	/* Set TGIA6 interrupt priority level */
	ICU.SLIAR236.BYTE = 0x1EU;
	IPR(PERIA, INTA236) = _0F_MTU_PRIORITY_LEVEL15;

	/* Set TCIV7 interrupt priority level */
	ICU.SLIAR245.BYTE = 0x27U;
	IPR(PERIA, INTA245) = _0F_MTU_PRIORITY_LEVEL15;

	/* MTU channel 6 is used as complementary PWM mode 1 */
	MTU6.TIER.BYTE = 0x00U;
	MTU7.TIER.BYTE = 0x00U;
	MTU.TITCR1B.BIT.T6AEN = 0U;
	MTU.TITCR1B.BIT.T7VEN = 0U;
	MTU6.TCR.BYTE = _01_MTU_PCLK_4 | _00_MTU_CKEG_RISE | _00_MTU_CKCL_DIS;
	MTU7.TCR.BYTE = _01_MTU_PCLK_4 | _00_MTU_CKEG_RISE;
	MTU6.TCR2.BYTE = _00_MTU_PCLK_1;
	MTU7.TCR2.BYTE = _00_MTU_PCLK_1;
	//MTU.TGCRB.BYTE = _C0_MTU_BDC_FUN | _88_MTU_FB_SW | _90_MTU_P_PWM | _A0_MTU_N_PWM;
	//MTU6.TCNT = _003C_6TCNT_VALUE;
	MTU6.TCNT = 0x0001U;				/* for MTIOC6B PWM output not use deadtime */
	MTU7.TCNT = 0x0000U;
	MTU.TSYRB.BIT.SYNC6 = 0U;
	MTU.TSYRB.BIT.SYNC7 = 0U;
	MTU6.TGRB = _03E7_SUM_VALUE;		/* MTU6 do not use nueral point setting */
	MTU6.TGRD = _03E7_SUM_VALUE;
	MTU7.TGRA = _03E7_SUM_VALUE;
	MTU7.TGRC = _03E7_SUM_VALUE;
	MTU7.TGRB = _03E7_SUM_VALUE;
	MTU7.TGRD = _03E7_SUM_VALUE;
	MTU.TDERB.BIT.TDER = 0U;			/* do not use deadtime */
	MTU.TDDRB = 0x0001U;
	MTU.TCDRB = _03E8_TCDRB_VALUE;
	MTU.TCBRB = _03E8_TCDRB_VALUE;
	MTU6.TGRA = _03E9_SUM_VALUE;
	MTU6.TGRC = _03E9_SUM_VALUE;
	MTU.TOCR1B.BYTE = _40_MTU_PSYE_ENABLE | _08_MTU_TOCL_DISABLE | _04_MTU_TOCS_TOCR2;
	MTU.TOCR2B.BYTE = _80_MTU_TOLBR_T | _01_MTU_OLS1P_LH;
	MTU.TOLBRB.BYTE = MTU.TOCR2B.BYTE & 0x3FU;
	MTU6.TIER.BYTE = _01_MTU_TGIEA_ENABLE | _00_MTU_TGIEB_DISABLE | _00_MTU_TTGE_DISABLE;
	MTU7.TIER.BYTE = _00_MTU_TGIEA_DISABLE | _00_MTU_TGIEB_DISABLE | _10_MTU_TCIEV_ENABLE | _40_MTU_TTGE2_ENABLE;
	MTU.TITMRB.BIT.TITM = 1U;
	MTU.TITCR2B.BYTE = _00_MTU_TRGCOR_4_7_SKIP_COUNT_0;
	MTU6.TMDR1.BYTE = _0D_MTU_CMT1 | _10_MTU_BFA_BUFFER | _20_MTU_BFB_BUFFER;
	MTU.TOERB.BYTE = _C1_MTU_OE6B_ENABLE  | _C0_MTU_OE6D_DISABLE | _C0_MTU_OE7A_DISABLE | 
					 _C0_MTU_OE7C_DISABLE | _C0_MTU_OE7B_DISABLE | _C0_MTU_OE7D_DISABLE;

/* Start user code */
	MTU.TOERB.BYTE = 0xC0U;			/* stop output */
	MTU6.TIORH.BYTE = 0x00U;
	MTU6.TIORL.BYTE = 0x00U;
	MTU7.TIORH.BYTE = 0x00U;
	MTU7.TIORL.BYTE = 0x00U;

	/* Disable read/write to MTU6, MTU7 registers */
	MTU.TRWERB.BIT.RWE = 0U;

	/* Set MTIOC6A pin */
	MPC.PA1PFS.BYTE = 0x01U;
	PORTA.PMR.BYTE |= 0x02U;

	/* Init Up (MTIOC6B) P95 */
	PORT9.PODR.BIT.B5 = 0U;	/* Active High */
	PORT9.PDR.BIT.B5 = 1U;
	PORT9.PMR.BIT.B5 = 0U;
	MPC.P95PFS.BIT.PSEL = 0x01U;
/* end user code */

	R_Config_MTU6_MTU7_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU6_MTU7_Start
* Description  : This function starts the MTU6 channel counter
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU6_MTU7_Start(void)
{
	/* Enable TGIA6 interrupt in ICU */
	IEN(PERIA, INTA236) = 1U;

	/* Enable TCIV7 interrupt in ICU */
	IEN(PERIA, INTA245) = 1U;

	/* Start MTU6, MTU7 channel counter */
	MTU.TSTRB.BYTE |= (_40_MTU_CST6_ON | _80_MTU_CST7_ON);
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU6_MTU7_Stop
* Description  : This function stops the MTU6 channel counter
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU6_MTU7_Stop(void)
{
	/* Disable TGIA6 interrupt in ICU */
	IEN(PERIA, INTA236) = 0U;

	/* Disable TCIV7 interrupt in ICU */
	IEN(PERIA, INTA245) = 0U;

	/* Stop MTU6, MTU7 channel counter */
	MTU.TSTRB.BIT.CST6 = 0U;
	MTU.TSTRB.BIT.CST7 = 0U;
}

/* Start user code for adding. Do not edit comment generated here */
/***********************************************************************************************************************
* Function Name: R_Config_MTU6_MTU7_StartTimerOutput
* Description  : This function starts the MTU output
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU6_MTU7_StartTimerCtrl(void)
{
	/* Enable read/write to MTU6, MTU7 registers */
	MTU.TRWERB.BIT.RWE = 1U;

	//Control Up (MTIOC6B)
	PORT9.PODR.BIT.B5 = 0U; //Active High
	PORT9.PMR.BIT.B5 = 1U;
	//Enable MTU6.MTU7 PWM output
	MTU.TOERB.BYTE = _C1_MTU_OE6B_ENABLE  | _C0_MTU_OE6D_DISABLE | _C0_MTU_OE7A_DISABLE | 
					 _C0_MTU_OE7C_DISABLE | _C0_MTU_OE7B_DISABLE | _C0_MTU_OE7D_DISABLE;

	/* Disable read/write to MTU6, MTU7 registers */
	MTU.TRWERB.BIT.RWE = 0U;
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU6_MTU7_StopTimerOutput
* Description  : This function stops the MTU output
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU6_MTU7_StopTimerCtrl(void)
{
	/* Enable read/write to MTU6, MTU7 registers */
	MTU.TRWERB.BIT.RWE = 1U;

	//Disable MTU6.MTU7 PWM output
	MTU.TOERB.BYTE = 0xC0U;
	//Control Up (MTIOC6B)
	PORT9.PODR.BIT.B5 = 0U; //Active High
	PORT9.PMR.BIT.B5 = 0U;
	/* Disable read/write to MTU6, MTU7 registers */
	MTU.TRWERB.BIT.RWE = 0U;
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU6_MTU7_UpdDuty
* Description  : This function updates duty cycle
* Arguments	   : duty_u -
*					 U phase duty register value
*				 duty_v -
*					 V phase duty register value
*				 duty_w -
*					 W phase duty register value
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU6_MTU7_UpdDuty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w)
{
	MTU6.TGRD = duty_u;
	MTU7.TGRC = duty_v;
	MTU7.TGRD = duty_w;
}
/* End user code. Do not edit comment generated here */
