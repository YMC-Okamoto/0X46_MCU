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
* Copyright (C) 2018 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name        : Pin.c
* Version          : 1.0.2
* Device(s)        : R5F566TKFxFP
* Description      : This file implements SMC pin code generation.
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
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Pins_Create
* Description  : This function initializes Smart Configurator pins
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Pins_Create(void)
{
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_MPC);

    /* Set AN000 pin */
    PORT4.PMR.BYTE &= 0xFEU;
    PORT4.PDR.BYTE &= 0xFEU;
    MPC.P40PFS.BYTE = 0x80U;

    /* Set AN001 pin */
    PORT4.PMR.BYTE &= 0xFDU;
    PORT4.PDR.BYTE &= 0xFDU;
    MPC.P41PFS.BYTE = 0x80U;

    /* Set AN002 pin */
    PORT4.PMR.BYTE &= 0xFBU;
    PORT4.PDR.BYTE &= 0xFBU;
    MPC.P42PFS.BYTE = 0x80U;

    /* Set AN003 pin */
    PORT4.PMR.BYTE &= 0xF7U;
    PORT4.PDR.BYTE &= 0xF7U;
    MPC.P43PFS.BYTE = 0x80U;

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

    /* Set AN200 pin */
    PORT5.PMR.BYTE &= 0xFBU;
    PORT5.PDR.BYTE &= 0xFBU;
    MPC.P52PFS.BYTE = 0x80U;

    /* Set AN201 pin */
    PORT5.PMR.BYTE &= 0xF7U;
    PORT5.PDR.BYTE &= 0xF7U;
    MPC.P53PFS.BYTE = 0x80U;

    /* Set AN202 pin */
    PORT5.PMR.BYTE &= 0xEFU;
    PORT5.PDR.BYTE &= 0xEFU;
    MPC.P54PFS.BYTE = 0x80U;

    /* Set AN204 pin */
    PORT5.PMR.BYTE &= 0xFEU;
    PORT5.PDR.BYTE &= 0xFEU;
    MPC.P50PFS.BYTE = 0x80U;

    /* Set AN205 pin */
    PORT5.PMR.BYTE &= 0xFDU;
    PORT5.PDR.BYTE &= 0xFDU;
    MPC.P51PFS.BYTE = 0x80U;

    /* Set AN206 pin */
    PORT6.PMR.BYTE &= 0xFEU;
    PORT6.PDR.BYTE &= 0xFEU;
    MPC.P60PFS.BYTE = 0x80U;

    /* Set AN207 pin */
    PORT6.PMR.BYTE &= 0xFDU;
    PORT6.PDR.BYTE &= 0xFDU;
    MPC.P61PFS.BYTE = 0x80U;

    /* Set AN209 pin */
    PORT6.PMR.BYTE &= 0xF7U;
    PORT6.PDR.BYTE &= 0xF7U;
    MPC.P63PFS.BYTE = 0x80U;

    /* Set CRX0 pin */
    MPC.PB6PFS.BYTE = 0x10U;
    PORTB.PMR.BYTE |= 0x40U;

    /* Set CTX0 pin */
    MPC.PB5PFS.BYTE = 0x10U;
    PORTB.PMR.BYTE |= 0x20U;

    /* Set DA0 pin */
    PORT6.PMR.BYTE &= 0xEFU;
    PORT6.PDR.BYTE &= 0xEFU;
    MPC.P64PFS.BYTE = 0x80U;

    /* Set DA1 pin */
    PORT6.PMR.BYTE &= 0xDFU;
    PORT6.PDR.BYTE &= 0xDFU;
    MPC.P65PFS.BYTE = 0x80U;

    /* Set MISOA pin */
    MPC.PD1PFS.BYTE = 0x0DU;
    PORTD.PMR.BYTE |= 0x02U;

    /* Set MOSIA pin */
    MPC.PD2PFS.BYTE = 0x0DU;
    PORTD.PMR.BYTE |= 0x04U;

    /* Set MTIOC0A pin */
    MPC.PB3PFS.BYTE = 0x01U;
    PORTB.PMR.BYTE |= 0x08U;

    /* Set MTIOC0B pin */
    MPC.PB2PFS.BYTE = 0x01U;
    PORTB.PMR.BYTE |= 0x04U;

    /* Set MTIOC0C pin */
    MPC.PB1PFS.BYTE = 0x01U;
    PORTB.PMR.BYTE |= 0x02U;

    /* Set MTIOC3A pin */
    MPC.P33PFS.BYTE = 0x01U;
    PORT3.PMR.BYTE |= 0x08U;

    /* Set MTIOC3B pin */
    MPC.P71PFS.BYTE = 0x01U;
    PORT7.PMR.BYTE |= 0x02U;

    /* Set MTIOC3D pin */
    MPC.P74PFS.BYTE = 0x01U;
    PORT7.PMR.BYTE |= 0x10U;

    /* Set MTIOC4A pin */
    MPC.P72PFS.BYTE = 0x01U;
    PORT7.PMR.BYTE |= 0x04U;

    /* Set MTIOC4B pin */
    MPC.P73PFS.BYTE = 0x01U;
    PORT7.PMR.BYTE |= 0x08U;

    /* Set MTIOC4C pin */
    MPC.P75PFS.BYTE = 0x01U;
    PORT7.PMR.BYTE |= 0x20U;

    /* Set MTIOC4D pin */
    MPC.P76PFS.BYTE = 0x01U;
    PORT7.PMR.BYTE |= 0x40U;

    /* Set MTIOC6A pin */
    MPC.PA1PFS.BYTE = 0x01U;
    PORTA.PMR.BYTE |= 0x02U;

    /* Set MTIOC6B pin */
    MPC.P95PFS.BYTE = 0x01U;
    PORT9.PMR.BYTE |= 0x20U;

    /* Set RSPCKA pin */
    MPC.PD0PFS.BYTE = 0x0DU;
    PORTD.PMR.BYTE |= 0x01U;

    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_MPC);
}

