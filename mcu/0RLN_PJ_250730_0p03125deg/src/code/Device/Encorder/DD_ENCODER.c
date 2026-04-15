/**
 * @brief エンコーダ処理
 * @detail TAUJ3のエッジ割込みを利用したパルス周期計測
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11.xx
 */
#define __DD_ENCODER_C
/* Includes ------------------------------------------------------------------*/

#include "DEFINE_MOTOR.h"
#include "DEFINE_SYSTEM.h"
#include "COMMON_FUNCTION.h"		/* 汎用関数群 */

/* DDL */
#include "DD_ENCODER.h"
#include "DD_MOTOR.h"

#include "MOTOR.h"
#include "mcu.h"
#include "ramdef.h"

/* Private define ------------------------------------------------------------*/
#define	ENC_PATTERN_CHECKER_MAX		(u8)6			/* ENCパターンチェッカーIndex最大値 */

/* Private variables ---------------------------------------------------------*/
static bool	b_DD_ENC_isFirstEdgeInterrupt	= TRUE;																/* 有効エッジ割り込み初回フラグ[初回：TRUE] */
static volatile u32	u32_DD_ENC_encTimerValueBuffer[7] = {(u32)0,(u32)0,(u32)0,(u32)0,(u32)0,(u32)0,(u32)0};		/* エンコーダ周期のバッファ*/
static u8	u8_DD_ENC_encPatternChecker_sum	= (u8)0;															/* 回転数計算用パターンチェッカー総計 */
static bool	b_DD_ENC_isEncEdge = FALSE;																			/* エンコーダ有効エッジ変化フラグ */
static bool	b_DD_ENC_isRequestCalcMotorRevolution = FALSE;														/* モータ回転数演算要求 */
static bool	b_DD_ENC_isEncTimerOverflow = FALSE;																/* エンコーダタイマオーバーフロー(MTU0) */
static u32	u32_DD_ENC_encTimerValue = (u32)(U32_ENC_MAX_TIMER_VALUE);											/* エンコーダ周期(タイマ値) */
static u8	u8_DD_ENC_encTimerOverflowCounter = (u8)0;															/* エンコーダタイマオーバーフローカウンタ(オーバーフロー回数) */
static u8	u8_DD_ENC_encPatternChecker[6] = { (u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0 };							/* 回転数計算用パターンチェッカー */
static u8	u8_DD_ENC_encPatternCheckerIndex = (u8)0;															/* 回転数計算用パターンチェッカーIndex */
/* for DEBUG */
static u8	u8_DD_ENC_encPatternChecker_sumMax	= (u8)0;																/* 回転数計算用パターンチェッカー総計 */


/**
 * @brief エンコーダ エッジ処理
 * @detail  からコールされる
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11.xx
 */
void V_DD_ENC_EdgeCycle( u16 a_u16_encCycleTimerValue )
{
	bool a_b_isEdgeValid = FALSE;
	u8 *a_pu8 = (u8)0;

	b_DD_ENC_isEncEdge = !b_DD_ENC_isEncEdge;																	/* 有効エッジ変化フラグON(反転) */

	if( b_DD_ENC_isFirstEdgeInterrupt == TRUE )																	/* 有効エッジ割り込みが初回かどうかの判断 */
	{																											/* 初回の割り込み */
		b_DD_ENC_isFirstEdgeInterrupt = FALSE;																	/* 初回だと⇒割り込み計数カウンタアップ */
		a_b_isEdgeValid = FALSE;																				/* 一番目のカウンタのデータを採用しない */
	}
	else
	{																											/* 有効エッジ割り込み発生 */
		a_b_isEdgeValid = TRUE;
	}

	if( u8_DD_ENC_encPatternCheckerIndex < (ENC_PATTERN_CHECKER_MAX) )
	{
		u8_DD_ENC_encPatternChecker[u8_DD_ENC_encPatternCheckerIndex]
		= U8_DD_MTR_GetEncPatternChecker(u8_DD_ENC_encPatternCheckerIndex);
		u8_DD_ENC_encPatternCheckerIndex++;
	}

	if ( a_b_isEdgeValid == TRUE )
	{											/*** 有効の場合：通常の計算 ***/
		if( b_DD_ENC_isEncTimerOverflow == TRUE )																/* エンコーダタイマオーバフローか */
		{																										/* オーバフローと判断して→オーバフロー処理 */
			/* オーバーフロー分を加算 */
			u32_DD_ENC_encTimerValue = (u32)a_u16_encCycleTimerValue + (u32)(U16_MAX) * (u32)u8_DD_ENC_encTimerOverflowCounter + (u32)1;
			u8_DD_ENC_encTimerOverflowCounter = (u8)0;															/* オーバーフローカウンタ初期化 */
		}
		else
		{
			u32_DD_ENC_encTimerValue = a_u16_encCycleTimerValue;												/* タイマの値を採用する */
			u8_DD_ENC_encTimerOverflowCounter = (u8)0;															/* オーバーフローカウンタ初期化 */
		}

		if( u8_DD_ENC_encPatternChecker_sum >= (U8_MTR_NUM_ENCODER_PATTERNS) )
		{
			u8_DD_ENC_encPatternChecker_sum = (u8)0;															/* エンコーダパターン和を初期化 */
			u8_DD_ENC_encPatternCheckerIndex = (u8)0;
		}
		a_pu8 = u8_DD_ENC_encPatternChecker;																	/* エンコーダのパターンチェッカーへのポインタ */
		u8_DD_ENC_encPatternChecker_sum += *( a_pu8 + u8_DD_ENC_encPatternCheckerIndex );						/* エンコーダパターンの和を求める */
		/* for DEBUG */
		u8_DD_ENC_encPatternChecker_sumMax = CMF_GETMAX( u8_DD_ENC_encPatternChecker_sumMax,u8_DD_ENC_encPatternChecker_sum );

		V_DD_MTR_ClearEncPatternChecker();																		/* 回転数計算用パターンチェッカーをクリア */
		b_DD_ENC_isRequestCalcMotorRevolution = TRUE;															/* 演算要求フラグON*/
	}
	else
	{											/*** 無効の場合 ***/
		V_DD_MTR_ClearEncPatternChecker();																		/* 回転数計算用パターンチェッカーをクリア */
	}
}


/**
 * @brief エンコーダ オーバフロー処理
 * @detail 
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11.xx
 */
void V_DD_ENC_TimerOverflow(void)
{
	b_DD_ENC_isEncTimerOverflow = TRUE;																			/* エンコーダタイマオーバーフローSET */
	u8_DD_ENC_encTimerOverflowCounter++;																		/* エンコーダタイマオーバーフロー回数INC */

	if( u8_DD_ENC_encTimerOverflowCounter >= (U8_ENC_TIMEROVERFLOW_COUNT_MAX) )									/* オーバフロー最大回数以上 */
	{
		u32_DD_ENC_encTimerValue = (U32_ENC_MAX_TIMER_VALUE);													/* 最大値でクリップ */
		u8_DD_ENC_encTimerOverflowCounter = (u8)0;																/* オーバーフローカウンタ初期化 */
		b_DD_ENC_isEncTimerOverflow = FALSE;																	/* エンコーダタイマオーバーフローClear */
	}
}


/**
 * @brief エンコーダパターンチェッカーSum値取得
 * @param none
 * @return u8_DD_ENC_encPatternChecker_sum
 */
u8	U8_DD_ENC_GetEncPatternCheckerSum(void)
{
	return u8_DD_ENC_encPatternChecker_sum;
}


/**
 * @brief エンコーダエッジ状態取得
 * @param none
 * @return b_DD_ENC_isEncEdge
 */
bool	B_DD_ENC_IsEncEdge(void)
{
	return b_DD_ENC_isEncEdge;
}

/**
 * @brief エンコーダ周期タイマ値取得
 * @param none
 * @return u32_DD_ENC_encTimerValue
 */
u32	U32_DD_ENC_GetEncTimerValue(void)
{
	return u32_DD_ENC_encTimerValue;
}

/**
 * @brief 回転数演算要求取得
 * @param none
 * @return b_DD_ENC_isRequestCalcMotorRevolution
 */
bool	B_DD_ENC_IsRequestCalcMotorRevolution(void)
{
	return	b_DD_ENC_isRequestCalcMotorRevolution;
}

/**
 * @brief 回転数演算要求クリア
 * @param none
 * @return b_DD_ENC_isRequestCalcMotorRevolution
 */
void	V_DD_ENC_ClearRequestCalcMotorRevolution(void)
{
	b_DD_ENC_isRequestCalcMotorRevolution = FALSE;																/* 回転数演算要求フラグをクリア */
}

/**
 * @brief エンコーダ処理の初期化
 * @param none
 * @return none
 */
void V_DD_ENC_Initialize(void)
{
	V_CMF_Memset(u8_DD_ENC_encPatternChecker, (u8)0, (u8)6);													/* 回転数計算用パターンチェッカー初期化 */
}

