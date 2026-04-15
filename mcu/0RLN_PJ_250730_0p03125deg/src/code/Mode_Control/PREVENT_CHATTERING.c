/**
 * @file PREVENT_CHATTERING.c
 * @brief SW入力チャタリング防止
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "MCU.h"
#include "pin.h"
#include "DEFINE_SYSTEM.h"							/* 定数定義 */
#include "PREVENT_CHATTERING.h"						/* チャタリング防止 */

#include "ramdef.h"

/* Private define ------------------------------------------------------------*/
#define U8_CHT_COUNT_TYPE1		(u8)3						/* チャタリング判定カウンタ	10[ms] × 3 ＝ 30[ms] */
#define U8_CHT_COUNT_TYPE2		(u8)10						/* チャタリング判定カウンタ	10[ms] × 10 ＝ 100[ms] */
#define U8_CHT_COUNT_TYPE3		(u8)150						/* チャタリング判定カウンタ	150回 */

#define U8_ENC_EXAM_LEBEL		(u8)0x00					/* 完成検査移行判断ENCポートレベル */
/* Private variables ---------------------------------------------------------*/
#ifdef	DEBUG_WATCH_ACTIVITY

bool	b_CHT_isKeySwOn= FALSE;							/* キーSW ON (SW ON = TRUE) */
bool	b_CHT_isAccelSwOn = FALSE;						/* アクセルSW ON (SW ON = TRUE) */
bool	b_CHT_isBackSwOn= FALSE;						/* BACK SW ON (SW ON = TRUE) */
bool	b_CHT_isFwdSwOn = FALSE;						/* FWD SW ON (SW ON = TRUE) */
bool	b_CHT_isBrOn	= FALSE;						/* BR ON (SW ON = TRUE) */
bool	b_CHT_isChgOn	= FALSE;						/* Charger ON (SW ON = TRUE) */

#else

static bool	b_CHT_isKeySwOn= FALSE;							/* キーSW ON (SW ON = TRUE) */
static bool	b_CHT_isAccelSwOn = FALSE;						/* アクセルSW ON (SW ON = TRUE) */
static bool	b_CHT_isBackSwOn= FALSE;						/* BACK SW ON (SW ON = TRUE) */
static bool	b_CHT_isFwdSwOn = FALSE;						/* FWD SW ON (SW ON = TRUE) */
static bool	b_CHT_isBrOn	= FALSE;						/* BR ON (SW ON = TRUE) */
static bool	b_CHT_isChgOn	= FALSE;						/* Charger ON (SW ON = TRUE) */

#endif

static u8	u8_CHT_chatteringTimerForKeySw = (u8)0;			/* キーSWチャタリングタイマ */
static u8	u8_CHT_chatteringTimerForAccelSw = (u8)0;		/* アクセルSWチャタリングタイマ */
static u8	u8_CHT_chatteringTimerForBackSw = (u8)0;		/* BACK SWチャタリングタイマ */
static u8	u8_CHT_chatteringTimerForFwdSw = (u8)0;			/* FWD SWチャタリングタイマ */
static u8	u8_CHT_chatteringTimerForBr = (u8)0;			/* BRチャタリングタイマ */
static u8	u8_CHT_chatteringTimerForChg = (u8)0;			/* Chargerチャタリングタイマ */

/* Private function prototypes -----------------------------------------------*/
static bool	b_CHT_JugdementChattering(u8 a_u8_JudgeTime, bool a_b_isLebelRaw, bool a_b_isLebelResult, u8 *a_u8_chatteringTimer, bool a_b_logic);	/* SW状態判断用関数 */


/**
 * @brief SW入力チャタリング処理
 * @detail 各部SW入力のチャタリング判定をする
 * @note 10msec周期
 * @param none
 * @return none
 */
void V_CHT_Loop10ms(void)
{
	b_CHT_isKeySwOn		= b_CHT_JugdementChattering( (U8_CHT_COUNT_TYPE1), (bool)PI_KEY_SW(),	b_CHT_isKeySwOn,	&u8_CHT_chatteringTimerForKeySw,	FALSE );		/* キーSWチャタリング判断 */
	b_CHT_isAccelSwOn	= b_CHT_JugdementChattering( (U8_CHT_COUNT_TYPE1), (bool)PI_ACCEL_SW(),	b_CHT_isAccelSwOn,	&u8_CHT_chatteringTimerForAccelSw,	FALSE );		/* アクセルSWチャタリング判断 */
	b_CHT_isBackSwOn	= b_CHT_JugdementChattering( (U8_CHT_COUNT_TYPE1), (bool)PI_BACK_SW(),	b_CHT_isBackSwOn,	&u8_CHT_chatteringTimerForBackSw,	FALSE );		/* BACK SWチャタリング判断 */
	b_CHT_isFwdSwOn		= b_CHT_JugdementChattering( (U8_CHT_COUNT_TYPE1), (bool)PI_FWD_SW(),	b_CHT_isFwdSwOn,	&u8_CHT_chatteringTimerForFwdSw,	FALSE );		/* FWDチSWャタリング判断 */
	b_CHT_isBrOn		= b_CHT_JugdementChattering( (U8_CHT_COUNT_TYPE1), (bool)PI_BR(),		b_CHT_isBrOn,		&u8_CHT_chatteringTimerForBr,		FALSE );		/* BRチャタリング判断 */
	b_CHT_isChgOn		= b_CHT_JugdementChattering( (U8_CHT_COUNT_TYPE1), (bool)PI_CHG(),		b_CHT_isChgOn,		&u8_CHT_chatteringTimerForChg,		FALSE );		/* Chargerチャタリング判断 */
}

/**
 * @brief 入力ポートのHi/Low状態を判断する
 * @detail bit単位で各部SW入力のチャタリング判定をする
 * @param a_u8_JudgeTime:		SW状態ONまたはOFFの判定連続時間閾値
 * @param a_b_isLebelRaw:		現在ポートの信号入力
 * @param a_b_isLebelResult:	現在のSW判断結果
 * @param *a_u8_chlting_timer：	タイマへのポインタ
 * @param a_b_logic:			TRUE :正論理	ポートhigh→状態ON
 * 								FALSE:論理反転	ポートlow→状態ON
 * @return a_b_isLebelResult：	SW状態の判断結果
 */
static bool b_CHT_JugdementChattering( u8 a_u8_JudgeTime, bool a_b_isLebelRaw, bool a_b_isLebelResult, u8 *a_u8_chatteringTimer, bool a_b_logic)
{
	if( a_b_logic == FALSE )
	{
		if( a_b_isLebelResult == TRUE )
		{
			if( a_b_isLebelRaw == TRUE )
			{																	/* 現在の信号はHigh、SW状態はONの場合 */
				( *a_u8_chatteringTimer )++;									/* タイマ インクリメント */
			}
			else
			{
				( *a_u8_chatteringTimer ) = (u8)0;
			}
		}
		else
		{
			if( a_b_isLebelRaw == FALSE )
			{
				( *a_u8_chatteringTimer )++;									/* タイマ インクリメント */
			}
			else
			{
				( *a_u8_chatteringTimer ) = (u8)0;
			}
		}
	}
	else
	{
		if( a_b_isLebelResult == FALSE )
		{
			if( a_b_isLebelRaw == TRUE )
			{																	/* 現在の信号はHigh、SW状態はONの場合 */
				( *a_u8_chatteringTimer )++;									/* タイマ インクリメント */
			}
			else
			{
				( *a_u8_chatteringTimer ) = (u8)0;
			}
		}
		else
		{
			if( a_b_isLebelRaw == FALSE )
			{
				( *a_u8_chatteringTimer )++;									/* タイマ インクリメント */
			}
			else
			{
				( *a_u8_chatteringTimer ) = (u8)0;
			}
		}
	}

	if( ( *a_u8_chatteringTimer ) >= a_u8_JudgeTime )							/* 不一致の時間の確認 */
	{																			/* 閾値を超えた場合 */
		a_b_isLebelResult = !a_b_isLebelResult;									/* SW状態変化の判断 */
		( *a_u8_chatteringTimer ) = (u8)0;										/* 同時にタイマをクリアする */
	}

	return a_b_isLebelResult;													/* 判断結果を出力する */
}


/**
 * @brief キーSW状態提供
 * @param none
 * @return b_CHT_isKeySwOn
 */
bool	B_CHT_IsKeySwOn(void)
{
	return	b_CHT_isKeySwOn;
}

/**
 * @brief アクセルSW状態提供
 * @param none
 * @return b_CHT_isAccelSwOn
 */
bool	B_CHT_IsAccelSwOn(void)
{
	return	b_CHT_isAccelSwOn;
}

/**
 * @brief BACK SW状態提供
 * @param none
 * @return b_CHT_isBackSwOn
 */
bool	B_CHT_IsBackSwOn(void)
{
	return	b_CHT_isBackSwOn;
}

/**
 * @brief FWD SW状態提供
 * @param none
 * @return b_CHT_isFwdSwOn
 */
bool	B_CHT_IsFwdSwOn(void)
{
	return	b_CHT_isFwdSwOn;
}

/**
 * @brief BR状態提供
 * @param none
 * @return b_CHT_isBrOn
 */
bool	B_CHT_IsBrOn(void)
{
	return	b_CHT_isBrOn;
}

/**
 * @brief Charge状態提供
 * @param none
 * @return b_CHT_isBrOn
 */
bool	B_CHT_IsChgOn(void)
{
	return	b_CHT_isChgOn;
}

#if 0
/**
 * @brief SW入力初期化処理
 * @detail 各部SW入力の初期化をする
 * @param none
 * @return none
 */
void V_CHT_Initialize(void)
{
	/*----- キーSW状態の初期化 -----*/
	b_CHT_isKeySwOn = FALSE;				/* キーSW = OFF */
}
#endif
