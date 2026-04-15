/**********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO
 * THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
 *********************************************************************************************************************/
/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "main.h"
#include <machine.h>
#include "ctypedef.h"
#include "Config_TMR0_TMR1.h"
#include "Config_MTU3_MTU4.h"
#include "Config_MTU6_MTU7.h"
#include "Config_MTU0.h"
#include "Config_S12AD2.h"
#include "Config_WDT.h"
#include "Config_DA.h"

#include "MCU.h"

#include "DD_MOTOR.h"
#include "DD_ENCODER.h"

#include "COMMON_FUNCTION.h"
#include "ADC.h"
#include "CAN.h"
#include "MOTOR.h"
#include "BATTERY.h"
#include "MODE_CONDUCTOR.h"
#include "PREVENT_CHATTERING.h"						/* チャタリング防止 */
#include "DETECT_ABNORMAL.h"
#include "LED_CONTROL.h"

#include "DEFINE_MOTOR.h"

#include "BUZZER_CONTROL.h"

#include "CUW.h"

#include "ramdef.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static bool b_MAIN_1msecPassed = FALSE;					/* 1msec経過フラグ */

/* Private function prototypes ------------------------------------------------*/
static void	v_MAIN_10msecLoop(void);
static void	v_MAIN_Initialize(void);

#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

void main(void)
{
	v_MAIN_Initialize();
	while (TRUE)
	{
/*------0.5msec周期-------------*/
		if(B_TMR0_Is500usecPassed() == TRUE)
		{
			V_TMR0_Clear500usecPassed();
			V_ADC_Loop500us();								/* AD変換平均化処理	*/

			if( b_MAIN_1msecPassed == TRUE )
			{
/*----------1msec周期-------------*/
				V_MTR_UpdateRevolutionAve_rpm();			/* モータ回転数平均化処理 */
				();							/* CAN関連1msecループ */
				b_MAIN_1msecPassed = FALSE;
			}
			else
			{
				b_MAIN_1msecPassed = TRUE;
			}
/*----------10msec周期-------------*/
			if( U8_TMR0_Get10msecCount() >= (u8)20 )
			{
				V_TMR0_Set10msecCount((u8)0);				/* 10msカウンタをクリア */
				v_MAIN_10msecLoop();						/* 10msec周期処理 */
			}
/*-----------50msec周期-------------*/
			if( U8_TMR0_Get50msecCount() >= (u8)100 )
			{
				R_Config_WDT_Restart();						/* WDTリスタート */
				V_TMR0_Set50msecCount((u8)0);				/* 50msカウンタをクリア */
			}
/*-----------100msec周期-------------*/
			if( U8_TMR0_Get100msecCount() >= (u8)200 )
			{
				V_TMR0_Set100msecCount((u8)0);				/* 100msカウンタをクリア */
				V_LED_Loop100ms();							/* LED表示制御 */
				V_BUZZER_Loop100ms();						/* ブザー鳴動制御 */
			}
		}
		V_CAN_CyclicTransmit();								/* CAN送信データ */
	}
}


/**
 * @brief 初期化処理
 * @detail メイン関数内でループに入る前に1度だけコールされる
 * @param none
 * @return none
 */
static void	v_MAIN_Initialize(void)
{
	V_DD_MTR_Initialize();							/* 電流制御 & AD取り込み処理用変数の初期化処理 */
	V_DD_ENC_Initialize();							/* 回転数検出初期化	*/

	V_ABN_Initialize();								/* 異常検出変数初期化 */

	V_MDC_Initialize();								/* モード移行演算処理の初期化処理 */
	V_CAN_Initialize();								/* CAN通信初期化 */
#ifdef	CUW_ACTIVITY
	V_CUW_Initialize();								/* Cuw初期化 */
#endif
	/*------- CPU初期化異常の確認 ----------------------------*/
	/*------- 基板検査モードの確認 ---------------------------*/

	V_TMR0_Set10msecCount((u8)0);
	V_TMR0_Set50msecCount((u8)0);
	V_TMR0_Set100msecCount((u8)0);
	setpsw_i();										/* 割込み許可 */
	R_Config_MTU3_MTU4_Start();						/* PWMタイマスタート*/
	R_Config_MTU6_MTU7_Start();						/* PWMタイマスタート*/
	R_Config_MTU0_Start();							/* エンコーダ割込みスタート */
	R_Config_TMR0_TMR1_Start();						/* 0.5msec周期開始 */
	R_Config_DA0_Start();							/* DA0 */
	R_Config_DA1_Start();							/* DA1 */
//	R_Config_S12AD2_Start();

	UNI_GL_testFlg.word = (u16)0;
	U16_GL_dPGainForTest	= (U16_MTR_DAXIS_KP);	/* d軸比例ゲインTEST用 */
	U16_GL_dIGainForTest	= (U16_MTR_DAXIS_KI);	/* d軸積分ゲインTEST用 */
	U16_GL_qPGainForTest	= (U16_MTR_QAXIS_KP);	/* q軸比例ゲインTEST用 */
	U16_GL_qIGainForTest	= (U16_MTR_QAXIS_KI);	/* q軸積分ゲインTEST用 */
	U16_GL_pGainTemp		= (U16_EX_KP);			/* 界磁比例ゲインTEST用 */
	U16_GL_iGainTemp		= (U16_EX_KI);			/* 界磁積分ゲインTEST用 */
	U16_GL_pGainTest		= (U16_EX_KP);			/* 界磁比例ゲインTEST用 */
	U16_GL_iGainTest		= (U16_EX_KI);			/* 界磁積分ゲインTEST用 */
}


/**
 * @brief 10msec周期処理
 * @detail メイン関数から10msec周期でコールされる
 * @param none
 * @return none
 */
static void v_MAIN_10msecLoop(void)
{
	V_CUW_Loop10ms();								/* CuW10msecループ */
	V_CHT_Loop10ms();								/* チャタリング */
	V_BAT_Loop10ms();								/* バッテリ関連 */
	V_ABN_Loop10ms();								/* 異常検出処理 */
	V_CAN_Loop10ms();								/* CAN関連10msecループ */
	V_MDC_Loop10ms();								/* モード移行処理 */
	V_MTR_Loop10ms();								/* 走行可否判断 */
}


#ifdef __cplusplus
void abort(void)
{

}
#endif
