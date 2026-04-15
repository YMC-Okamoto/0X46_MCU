/*==========================================================================*/
/**
 * @file	DD_ENCODER.h
 * @brief	モータ回転数演算処理ヘッダ
 * @author	Power Conversion System Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __DD_ENCODER_H
#define __DD_ENCODER_H
/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"
#include "DEFINE_SETTING.h"						/* IFDEF設定 */

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Global variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void V_DD_ENC_EdgeCycle( u16 a_u16_encCycleTimerValue );			/* エンコーダ周期計算の割り込み処理 */
extern void V_DD_ENC_TimerOverflow(void);								/* エンコーダ周期タイマがOverflowの割り込み処理 */
extern void	V_DD_ENC_Initialize(void);									/* エンコーダ処理の初期化 */
extern void	V_DD_ENC_InitializeForRestart(void);						/* エンコーダ処理の初期化(再起動用) */
extern u8	U8_DD_ENC_GetEncPatternCheckerSum(void);					/* エンコーダパターンチェッカーSum値取得 */

extern bool	B_DD_ENC_IsEncEdge(void);									/* エンコーダエッジ状態取得 */
extern u32	U32_DD_ENC_GetEncTimerValue(void);							/* エンコーダ周期タイマ値取得 */
extern bool	B_DD_ENC_IsRequestCalcMotorRevolution(void);				/* 回転数演算要求取得 */
extern void	V_DD_ENC_ClearRequestCalcMotorRevolution(void);				/* 回転数演算要求クリア */

#endif	/* __DD_ENCODER_H */
