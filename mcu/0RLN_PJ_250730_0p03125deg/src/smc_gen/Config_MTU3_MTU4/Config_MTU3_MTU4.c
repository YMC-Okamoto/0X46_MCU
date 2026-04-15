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
* File Name        : Config_MTU3_MTU4.c
* Component Version: 1.11.0
* Device(s)        : R5F566TKFxFP
* Description      : This file implements device driver for Config_MTU3_MTU4.
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
#include "Config_MTU3_MTU4.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_MTU4_Create
* Description  : This function initializes the MTU3 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_MTU4_Create(void)
{
    /* Release MTU channel 3 from stop state */
    MSTP(MTU3) = 0U;

    /* Enable read/write to MTU3, MTU4 registers */
    MTU.TRWERA.BIT.RWE = 1U;

    /* Stop MTU3, MTU4 counter */
    MTU.TSTRA.BIT.CST3 = 0U;
    MTU.TSTRA.BIT.CST4 = 0U;

    /* Set A/D conversion signal output for ADSM0, ADSM1 pins */
    MTU.TADSTRGR0.BYTE = _00_MTU_TADSMEN_DISABLE;
    MTU.TADSTRGR1.BYTE = _00_MTU_TADSMEN_DISABLE;

    /* Set TGIA3 interrupt priority level */
    ICU.SLIAR223.BYTE = 0x10U;
    IPR(PERIA, INTA223) = _0F_MTU_PRIORITY_LEVEL15;

    /* Set TCIV4 interrupt priority level */
    ICU.SLIAR232.BYTE = 0x19U;
    IPR(PERIA, INTA232) = _0F_MTU_PRIORITY_LEVEL15;

    /* MTU channel 3 is used as complementary PWM mode 1 */
    MTU3.TIER.BYTE = 0x00U;
    MTU4.TIER.BYTE = 0x00U;
    MTU.TITCR1A.BIT.T3AEN = 0U;
    MTU.TITCR1A.BIT.T4VEN = 0U;
    MTU3.TCR.BYTE = _01_MTU_PCLK_4 | _00_MTU_CKEG_RISE | _00_MTU_CKCL_DIS;
    MTU4.TCR.BYTE = _01_MTU_PCLK_4 | _00_MTU_CKEG_RISE;
    MTU3.TCR2.BYTE = _00_MTU_PCLK_1;
    MTU4.TCR2.BYTE = _00_MTU_PCLK_1;
    MTU3.TCNT = _003C_3TCNT_VALUE;
    MTU4.TCNT = 0x0000U;
    MTU.TSYRA.BIT.SYNC3 = 0U;
    MTU.TSYRA.BIT.SYNC4 = 0U;
    MTU3.TGRB = _028F_3TGRB_VALUE;
    MTU3.TGRD = _028F_3TGRB_VALUE;
    MTU4.TGRA = _028F_4TGRA_VALUE;
    MTU4.TGRC = _028F_4TGRA_VALUE;
    MTU4.TGRB = _028F_4TGRB_VALUE;
    MTU4.TGRD = _028F_4TGRB_VALUE;
    MTU.TDERA.BIT.TDER = 1U;
    MTU.TDDRA = _003C_TDDRA_VALUE;
    MTU.TCDRA = _04E2_TCDRA_VALUE;
    MTU.TCBRA = _04E2_TCDRA_VALUE;
    MTU3.TGRA = _051E_SUM_VALUE;
    MTU3.TGRC = _051E_SUM_VALUE;

    MTU.TOCR1A.BYTE = _40_MTU_PSYE_ENABLE | _08_MTU_TOCL_DISABLE | _04_MTU_TOCS_TOCR2;
    MTU.TOCR2A.BYTE = _80_MTU_TOLBR_T | _20_MTU_OLS3N_LH | _10_MTU_OLS3P_LH | _08_MTU_OLS2N_LH | _04_MTU_OLS2P_LH | 
                      _02_MTU_OLS1N_LH | _01_MTU_OLS1P_LH;
    MTU.TOLBRA.BYTE = MTU.TOCR2A.BYTE & 0x3FU;
    MTU3.TIER.BYTE = _01_MTU_TGIEA_ENABLE | _00_MTU_TGIEB_DISABLE | _00_MTU_TTGE_DISABLE;
    MTU4.TIER.BYTE = _00_MTU_TGIEA_DISABLE | _00_MTU_TGIEB_DISABLE | _10_MTU_TCIEV_ENABLE | _40_MTU_TTGE2_ENABLE;
    MTU.TITMRA.BIT.TITM = 0U;
    MTU.TITCR2A.BYTE = _00_MTU_TRGCOR_4_7_SKIP_COUNT_0;
    MTU3.TMDR1.BYTE = _0E_MTU_CMT2 | _10_MTU_BFA_BUFFER | _20_MTU_BFB_BUFFER;

/* Start user code */
    MTU.TOERA.BYTE = 0xC0U;         /*stop output*/
    MTU3.TIORH.BYTE = _00_MTU_IOA_DISABLE | _00_MTU_IOB_DISABLE;
    MTU3.TIORL.BYTE = _00_MTU_IOC_DISABLE | _00_MTU_IOD_DISABLE;
    MTU4.TIORH.BYTE = _00_MTU_IOA_DISABLE | _00_MTU_IOB_DISABLE;
    MTU4.TIORL.BYTE = _00_MTU_IOC_DISABLE | _00_MTU_IOD_DISABLE;

    /* Disable read/write to MTU3, MTU4 registers */
    MTU.TRWERA.BIT.RWE = 0U;

    /* Set MTIOC3A pin */
    MPC.P33PFS.BYTE = 0x01U;
    PORT3.PMR.BYTE |= 0x08U;

    /* Init Up (MTIOC3B) : P71 UH */
    PORT7.PODR.BIT.B1 = 0U;	/* Active High */
    PORT7.PDR.BIT.B1 = 1U;
    PORT7.PMR.BIT.B1 = 0U;
    MPC.P71PFS.BIT.PSEL = 0x01U;
    /* Init Un (MTIOC3D) : P74 UL */
    PORT7.PODR.BIT.B4 = 0U;	/* Active High */
    PORT7.PDR.BIT.B4 = 1U;
    PORT7.PMR.BIT.B4 = 0U;
    MPC.P74PFS.BIT.PSEL = 0x01U;
    /* Init Vp (MTIOC4A): P72 VH */
    PORT7.PODR.BIT.B2 = 0U;	/* Active High */
    PORT7.PDR.BIT.B2 = 1U;
    PORT7.PMR.BIT.B2 = 0U;
    MPC.P72PFS.BIT.PSEL = 0x01U;
    /* Init Wp (MTIOC4B) : P73 WH */
    PORT7.PODR.BIT.B3 = 0U;	/* Active High */
    PORT7.PDR.BIT.B3 = 1U;
    PORT7.PMR.BIT.B3 = 0U;
    MPC.P73PFS.BIT.PSEL = 0x01U;
    /* Init Vn (MTIOC4C) : P75 VL */
    PORT7.PODR.BIT.B5 = 0U;	/* Active High */
    PORT7.PDR.BIT.B5 = 1U;
    PORT7.PMR.BIT.B5 = 0U;
    MPC.P75PFS.BIT.PSEL = 0x01U;
    /* Init Wn (MTIOC4D) : P76 WL */
    PORT7.PODR.BIT.B6 = 0U;	/* Active High */
    PORT7.PDR.BIT.B6 = 1U;
    PORT7.PMR.BIT.B6 = 0U;
    MPC.P76PFS.BIT.PSEL = 0x01U;
/* end user code */

    R_Config_MTU3_MTU4_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_MTU4_Start
* Description  : This function starts the MTU3 channel counter
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_MTU4_Start(void)
{
    /* Enable TGIA3 interrupt in ICU */
    IEN(PERIA, INTA223) = 1U;

    /* Enable TCIV4 interrupt in ICU */
    IEN(PERIA, INTA232) = 1U;

    /* Start MTU3, MTU4 channel counter */
    MTU.TSTRA.BYTE |= (_40_MTU_CST3_ON | _80_MTU_CST4_ON);
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_MTU4_Stop
* Description  : This function stops the MTU3 channel counter
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_MTU4_Stop(void)
{
    /* Disable TGIA3 interrupt in ICU */
    IEN(PERIA, INTA223) = 0U;

    /* Disable TCIV4 interrupt in ICU */
    IEN(PERIA, INTA232) = 0U;

    /* Stop MTU3, MTU4 channel counter */
    MTU.TSTRA.BIT.CST3 = 0U;
    MTU.TSTRA.BIT.CST4 = 0U;
}

/* Start user code for adding. Do not edit comment generated here */
/***********************************************************************************************************************
* Function Name: R_Config_MTU3_MTU4_StarttTimerCount
* Description  : This function starts the MTU3 channel counter
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_MTU4_StartTimerCount(void)
{
    /* Enable TGIA3 interrupt in ICU */
    IEN(PERIA, INTA223) = 1U;

 /* Enable TCIV4 interrupt in ICU */
    IEN(PERIA, INTA232) = 1U;

    /* Start MTU3, MTU4 counter */
   // MTU.TCSYSTR.BYTE = (_10_MTR_MTU_SCH3_START | _08_MTR_MTU_SCH4_START);
    MTU.TCSYSTR.BYTE = ( 0x10U | 0x80U );
}
/***********************************************************************************************************************
* Function Name: R_Config_MTU3_MTU4_StopTimerCount
* Description  : This function stops the MTU3 channel counter
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_MTU4_StopTimerCount(void)
{
    /* Disable TGIA3 interrupt in ICU */
    IEN(PERIA, INTA223) = 0U;

    /* Disable TCIV4 interrupt in ICU */
    IEN(PERIA, INTA232) = 0U;

    /* Stop MTU3, MTU4 channel counter */
    MTU.TSTRA.BYTE = MTU.TSTRA.BYTE & 0x3FU;
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_MTU4_StartTimerOutput
* Description  : This function starts the MTU output
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_MTU4_StartTimerCtrl(void)
{
    /* Enable read/write to MTU3, MTU4 registers */
    MTU.TRWERA.BIT.RWE = 1U;

    //Control Up (MTIOC3B)
    PORT7.PODR.BIT.B1 = 0U; //Active High
    PORT7.PMR.BIT.B1 = 1U;
    //Control Un (MTIOC3D)
    PORT7.PODR.BIT.B4 = 0U; //Active High
    PORT7.PMR.BIT.B4 = 1U;
    //Control Vp (MTIOC4A)
    PORT7.PODR.BIT.B2 = 0U; //Active High
    PORT7.PMR.BIT.B2 = 1U;
    //Control Wp (MTIOC4B)
    PORT7.PODR.BIT.B3 = 0U; //Active High
    PORT7.PMR.BIT.B3 = 1U;
    //Control Vn (MTIOC4C)
    PORT7.PODR.BIT.B5 = 0U; //Active High
    PORT7.PMR.BIT.B5 = 1U;
    //Control Wn (MTIOC4D)
    PORT7.PODR.BIT.B6 = 0U; //Active High
    PORT7.PMR.BIT.B6 = 1U;
    //Enable MTU3,MTU4 PWM output
    MTU.TOERA.BYTE = _C1_MTU_OE3B_ENABLE | _C8_MTU_OE3D_ENABLE | _C2_MTU_OE4A_ENABLE | 
                     _D0_MTU_OE4C_ENABLE | _C4_MTU_OE4B_ENABLE | _E0_MTU_OE4D_ENABLE;

    /* Disable read/write to MTU3, MTU4 registers */
    MTU.TRWERA.BIT.RWE = 0U;
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_MTU4_StopTimerOutput
* Description  : This function stops the MTU output
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_MTU4_StopTimerCtrl(void)
{
    /* Enable read/write to MTU3, MTU4 registers */
    MTU.TRWERA.BIT.RWE = 1U;

    //Disable MTU3,MTU4 PWM output
    MTU.TOERA.BYTE = 0xC0U;
    //Control Up (MTIOC3B)
    PORT7.PODR.BIT.B1 = 0U; //Active High
    PORT7.PMR.BIT.B1 = 0U;
    //Control Un (MTIOC3D)
    PORT7.PODR.BIT.B4 = 0U; //Active High
    PORT7.PMR.BIT.B4 = 0U;
    //Control Vp (MTIOC4A)
    PORT7.PODR.BIT.B2 = 0U; //Active High
    PORT7.PMR.BIT.B2 = 0U;
    //Control Wp (MTIOC4B)
    PORT7.PODR.BIT.B3 = 0U; //Active High
    PORT7.PMR.BIT.B3 = 0U;
    //Control Vn (MTIOC4C)
    PORT7.PODR.BIT.B5 = 0U; //Active High
    PORT7.PMR.BIT.B5 = 0U;
    //Control Wn (MTIOC4D)
    PORT7.PODR.BIT.B6 = 0U; //Active High
    PORT7.PMR.BIT.B6 = 0U;

    /* Disable read/write to MTU3, MTU4 registers */
    MTU.TRWERA.BIT.RWE = 0U;
}

/***********************************************************************************************************************
* Function Name: R_Config_MTU3_MTU4_UpdDuty
* Description  : This function updates duty cycle
* Arguments    : duty_u -
*                    U phase duty register value
*                duty_v -
*                    V phase duty register value
*                duty_w -
*                    W phase duty register value
* Return Value : None
***********************************************************************************************************************/

void R_Config_MTU3_MTU4_UpdDuty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w)
{
    MTU3.TGRD = duty_u;
    MTU4.TGRC = duty_v;
    MTU4.TGRD = duty_w;
}
/* End user code. Do not edit comment generated here */
