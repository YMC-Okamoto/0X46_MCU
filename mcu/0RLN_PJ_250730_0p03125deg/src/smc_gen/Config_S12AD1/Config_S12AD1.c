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
* File Name		   : Config_S12AD1.c
* Component Version: 1.12.0
* Device(s)		   : R5F566TKFxFP
* Description	   : This file implements device driver for Config_S12AD1.
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
#include "Config_S12AD1.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_S12AD1_Create
* Description  : This function initializes the S12AD1 channel
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_S12AD1_Create(void)
{
	/* Cancel S12AD1 module stop state */
	MSTP(S12AD1) = 0U;

	/* Disable and clear interrupt flags of S12AD1 module */
	S12AD1.ADCSR.BIT.ADIE = 0U;
	S12AD1.ADCSR.BIT.GBADIE = 0U;
	S12AD1.ADGCTRGR.BIT.GCADIE = 0U;
	S12AD1.ADCMPCR.BIT.CMPAIE = 0U;
	S12AD1.ADCMPCR.BIT.CMPBIE = 0U;

	/* Set S12AD1 control registers */
	S12AD1.ADCSR.WORD = _0000_AD_DBLTRIGGER_DISABLE | _0200_AD_SYNCASYNCTRG_ENABLE | _2000_AD_GROUP_SCAN_MODE;

	/* Set AD conversion start trigger sources */
	S12AD1.ADSTRGR.WORD = 0x003FU | _0700_AD_TRSA_TRGA7N;						/* MTU7谷 */
//	S12AD1.ADSTRGR.WORD = _0007_AD_TRSB_TRGA7N | _0500_AD_TRSA_TRGA4N;			/* 取り敢えずコメント化。MTU7を有効にした時点で有効化 */
//	S12AD1.ADSTRGR.WORD = _0500_AD_TRSA_TRGA4N;									/* MTU4谷 */
//	S12AD1.ADSTRGR.WORD = _0400_AD_TRSA_TRGA3N;									/* MTU3山 */
	S12AD1.ADGCTRGR.BYTE |= (_00_AD_GROUPC_DISABLE);
	S12AD1.ADDISCR.BYTE = _00_AD_DISCONECT_UNUSED;
	S12AD1.ADGSPCR.WORD = _0000_AD_GPPRIORITY_DISABLE;

	/* Set channels and sampling time */
	S12AD1.ADSSTR0 = _1B_AD1_SAMPLING_STATE_0;
	S12AD1.ADSSTR1 = _1B_AD1_SAMPLING_STATE_1;
	S12AD1.ADSSTR2 = _1B_AD1_SAMPLING_STATE_2;
	S12AD1.ADSSTR3 = _1B_AD1_SAMPLING_STATE_3;
	S12AD1.ADANSA0.WORD = _0001_AD_ANx00_GROUPA_USED | _0002_AD_ANx01_GROUPA_USED | _0004_AD_ANx02_GROUPA_USED | _0008_AD_ANx03_GROUPA_USED;

	/* Set AN100 amplifier */
	S12AD1.ADPGADCR0.BIT.P100DEN = 0U;
	S12AD1.ADPGACR.BIT.P100CR = _0001_AD_PATH_ANx_NONE_NONE;

	/* Set AN101 amplifier */
	S12AD1.ADPGADCR0.BIT.P101DEN = 0U;
	S12AD1.ADPGACR.BIT.P101CR = _0001_AD_PATH_ANx_NONE_NONE;

	/* Set AN102 amplifier */
	S12AD1.ADPGADCR0.BIT.P102DEN = 0U;
	S12AD1.ADPGACR.BIT.P102CR = _0001_AD_PATH_ANx_NONE_NONE;
	S12AD1.ADCER.WORD = _0000_AD_AUTO_CLEARING_DISABLE | _0000_AD_SELFTDIAGST_DISABLE | _0000_AD_RIGHT_ALIGNMENT;
	S12AD1.ADADC.BYTE = _00_AD_1_TIME_CONVERSION | _00_AD_ADDITION_MODE;
	S12AD1.ADELCCR.BYTE = _02_ALL_SCAN_COMPLETION;
	S12AD1.ADCSR.WORD |= (_0000_AD_GBADI_DISABLE | _0000_AD_SCAN_END_INTERRUPT_DISABLE);

	/* Set compare control register */
	S12AD1.ADCMPCR.WORD = _0000_AD_WINDOWB_DISABLE | _0000_AD_WINDOWA_DISABLE | _0000_AD_WINDOWFUNCTION_DISABLE;

	/* Set interrupt and priority level */
	IPR(S12AD1, S12GCADI1) = _0F_AD_PRIORITY_LEVEL15;

	/* Set AN100 pin */
	PORT4.PMR.BYTE &= 0xEFU;
	PORT4.PDR.BYTE &= 0xEFU;
	MPC.P44PFS.BYTE = 0x80U;

	/* Set AN101 pin */
	PORT4.PMR.BYTE &= 0xDFU;
	PORT4.PDR.BYTE &= 0xDFU;
	MPC.P45PFS.BYTE = 0x80U;

	/* Set AN102 pin */
	PORT4.PMR.BYTE &= 0xBFU;
	PORT4.PDR.BYTE &= 0xBFU;
	MPC.P46PFS.BYTE = 0x80U;

	/* Set AN103 pin */
	PORT4.PMR.BYTE &= 0x7FU;
	PORT4.PDR.BYTE &= 0x7FU;
	MPC.P47PFS.BYTE = 0x80U;

	R_Config_S12AD1_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_S12AD1_Start
* Description  : This function starts the AD1 converter
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_S12AD1_Start(void)
{
	S12AD1.ADCSR.BIT.TRGE = 1U;
}

/***********************************************************************************************************************
* Function Name: R_Config_S12AD1_Stop
* Description  : This function stops the AD1 converter
* Arguments	   : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_S12AD1_Stop(void)
{
	S12AD1.ADCSR.BIT.TRGE = 0U;
	S12AD1.ADCSR.BIT.ADST = 0U;
}

/***********************************************************************************************************************
* Function Name: R_Config_S12AD1_Get_ValueResult
* Description  : This function gets result from the AD1 converter
* Arguments	   : channel -
*					 channel of data register to be read
*				 buffer -
*					 buffer pointer
* Return Value : None
***********************************************************************************************************************/

void R_Config_S12AD1_Get_ValueResult(ad_channel_t channel, uint16_t * const buffer)
{
	switch (channel)
	{
		case ADSELFDIAGNOSIS:
		{
			*buffer = (uint16_t)(S12AD1.ADRD.WORD);
			break;
		}
		case ADCHANNEL0:
		{
			*buffer = (uint16_t)(S12AD1.ADDR0);
			break;
		}
		case ADCHANNEL1:
		{
			*buffer = (uint16_t)(S12AD1.ADDR1);
			break;
		}
		case ADCHANNEL2:
		{
			*buffer = (uint16_t)(S12AD1.ADDR2);
			break;
		}
		case ADCHANNEL3:
		{
			*buffer = (uint16_t)(S12AD1.ADDR3);
			break;
		}
		case ADCHANNEL4:
		{
			*buffer = (uint16_t)(S12AD1.ADDR4);
			break;
		}
		case ADCHANNEL5:
		{
			*buffer = (uint16_t)(S12AD1.ADDR5);
			break;
		}
		case ADCHANNEL6:
		{
			*buffer = (uint16_t)(S12AD1.ADDR6);
			break;
		}
		case ADCHANNEL7:
		{
			*buffer = (uint16_t)(S12AD1.ADDR7);
			break;
		}
		case ADDATADUPLICATIONA:
		{
			*buffer = (uint16_t)(S12AD1.ADDBLDRA);
			break;
		}
		case ADDATADUPLICATIONB:
		{
			*buffer = (uint16_t)(S12AD1.ADDBLDRB);
			break;
		}
		case ADDATADUPLICATION:
		{
			*buffer = (uint16_t)(S12AD1.ADDBLDR);
			break;
		}
		default:
		{
			break;
		}
	}
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
