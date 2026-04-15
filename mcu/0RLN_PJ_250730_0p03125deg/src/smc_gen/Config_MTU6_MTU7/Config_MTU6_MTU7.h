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
* File Name		   : Config_MTU6_MTU7.h
* Component Version: 1.11.0
* Device(s)		   : R5F566TKFxFP
* Description	   : This file implements device driver for Config_MTU6_MTU7.
***********************************************************************************************************************/

#ifndef CFG_Config_MTU6_MTU7_H
#define CFG_Config_MTU6_MTU7_H

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_mtu3.h"

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define MTU6_PCLK_COUNTER_DIVISION		(4)
#define _003C_6TCNT_VALUE				(0x003CU) /* MTU6.TCNT value */
#define _03E8_TCDRB_VALUE				(0x03E8U) /* MTU.TCDRB value */
#define _0064_6TGRB_VALUE				(0x0064U) /* MTU6.TGRB value */
#define _0064_7TGRA_VALUE				(0x0064U) /* MTU7.TGRA value */
#define _0064_7TGRB_VALUE				(0x0064U) /* MTU7.TGRB value */
#define _03E9_SUM_VALUE					(0x03E9U) /* Timer General Register (TGR) value */
#define _03E7_SUM_VALUE					(0x03E7U) /* Timer General Register (TGR) value */

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_Config_MTU6_MTU7_Create(void);
void R_Config_MTU6_MTU7_Create_UserInit(void);
void R_Config_MTU6_MTU7_Start(void);
void R_Config_MTU6_MTU7_Stop(void);
/* Start user code for function. Do not edit comment generated here */
void R_Config_MTU6_MTU7_StartTimerCount(void);
void R_Config_MTU6_MTU7_StopTimerCount(void);
void R_Config_MTU6_MTU7_StartTimerCtrl(void);
void R_Config_MTU6_MTU7_StopTimerCtrl(void);
void R_Config_MTU6_MTU7_UpdDuty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w);
/* End user code. Do not edit comment generated here */
#endif
