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
* File Name		   : Config_MTU0.c
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
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_MTU0_Create
* Description  : This function initializes the MTU0 channel
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU0_Create(void)
{
	/* Release MTU channel 0 from stop state */
	MSTP(MTU3) = 0U;

	/* Stop MTU channel 0 counter */
	MTU.TSTRA.BIT.CST0 = 0U;

	/* Set A/D conversion signal output for ADSM0, ADSM1 pins */
	MTU.TADSTRGR0.BYTE = _00_MTU_TADSMEN_DISABLE;
	MTU.TADSTRGR1.BYTE = _00_MTU_TADSMEN_DISABLE;

	/* Set TGIA0 interrupt priority level */
	ICU.SLIAR208.BYTE = 0x01U;
	IPR(PERIA, INTA208) = _0F_MTU_PRIORITY_LEVEL15;

	/* Set TGIB0 interrupt priority level */
	ICU.SLIAR209.BYTE = 0x02U;
	IPR(PERIA, INTA209) = _0F_MTU_PRIORITY_LEVEL15;

	/* Set TGIC0 interrupt priority level */
	ICU.SLIAR210.BYTE = 0x03U;
	IPR(PERIA, INTA210) = _0F_MTU_PRIORITY_LEVEL15;

	/* Set TCIV0 interrupt priority level */
	ICU.SLIAR212.BYTE = 0x05U;
	IPR(PERIA, INTA212) = _0F_MTU_PRIORITY_LEVEL15;

	/* MTU channel 0 is used as normal mode */
	MTU.TSYRA.BIT.SYNC0 = 0U;
	MTU0.TCR.BYTE = _10_MTU_CKEG_BOTH | _00_MTU_CKCL_DIS;				/* —¼ƒGƒbƒW */

	MTU0.TCR2.BYTE = _04_MTU_PCLK_256;

	MTU0.TIER.BYTE = _01_MTU_TGIEA_ENABLE | _02_MTU_TGIEB_ENABLE | _04_MTU_TGIEC_ENABLE | _00_MTU_TGIED_DISABLE | 
					 _10_MTU_TCIEV_ENABLE | _00_MTU_TTGE_DISABLE;
	MTU0.TIER2.BYTE = _00_MTU_TGIEE_DISABLE | _00_MTU_TGIEF_DISABLE | _00_MTU_TTGE2_DISABLE;
	MTU0.NFCR0.BYTE = _01_MTU_NFAEN_ENABLE | _02_MTU_NFBEN_ENABLE | _04_MTU_NFCEN_ENABLE | _00_MTU_NFCS_PCLK_1;
	MTU0.TIORH.BYTE = _0A_MTU_IOA_IB | _A0_MTU_IOB_IB;
	MTU0.TIORL.BYTE = _0A_MTU_IOC_IB | _00_MTU_IOD_DISABLE;
	MTU0.TMDR1.BYTE = _00_MTU_NORMAL;

	MTU0.TGRD = _007C_TGRD0_VALUE;
	MTU0.TGRE = _007C_TGRE0_VALUE;
	MTU0.TGRF = _007C_TGRF0_VALUE;

	/* Set MTIOC0A pin */
	MPC.PB3PFS.BYTE = 0x01U;					/* MTIOC0A	U‘Š */
	PORTB.PMR.BYTE |= 0x08U;

	/* Set MTIOC0B pin */
	MPC.PB2PFS.BYTE = 0x01U;					/* MTIOC0B	V‘Š */
	PORTB.PMR.BYTE |= 0x04U;

	/* Set MTIOC0C pin */
	MPC.PB1PFS.BYTE = 0x01U;					/* MTIOC0C	W‘Š */
	PORTB.PMR.BYTE |= 0x02U;

	R_Config_MTU0_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU0_Start
* Description  : This function starts the MTU0 channel counter
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU0_Start(void)
{
	/* Enable TGIA0 interrupt in ICU */
	IEN(PERIA, INTA208) = 1U;

	/* Enable TGIB0 interrupt in ICU */
	IEN(PERIA, INTA209) = 1U;

	/* Enable TGIC0 interrupt in ICU */
	IEN(PERIA, INTA210) = 1U;

	/* Enable TCIV0 interrupt in ICU */
	IEN(PERIA, INTA212) = 1U;

	/* Start MTU channel 0 counter */
	MTU.TSTRA.BIT.CST0 = 1U;
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU0_Stop
* Description  : This function stops the MTU0 channel counter
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU0_Stop(void)
{
	/* Disable TGIA0 interrupt in ICU */
	IEN(PERIA, INTA208) = 0U;

	/* Disable TGIB0 interrupt in ICU */
	IEN(PERIA, INTA209) = 0U;

	/* Disable TGIC0 interrupt in ICU */
	IEN(PERIA, INTA210) = 0U;

	/* Disable TCIV0 interrupt in ICU */
	IEN(PERIA, INTA212) = 0U;

	/* Stop MTU channel 0 counter */
	MTU.TSTRA.BIT.CST0 = 0U;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
