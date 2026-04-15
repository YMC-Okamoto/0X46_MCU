/**
 * @file DETECT_BATTERY.c
 * @brief バッテリ関連異常検出
 * @note 50ms周期
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "TYPEDEF_MODE.h"
#include "TYPEDEF_ABN.h"
#include "DEFINE_SETTING.h"						/* IFDEF設定 */
#include "DEFINE_ABN.h"							/* 定数定義 */
#include "DEFINE_SYSTEM.h"						/* 定数定義 */
#include "DEFINE_MOTOR.h"						/* 定数定義 */
#include "COMMON_FUNCTION.h"					/* 汎用関数群 */
#include "DETECT_ABNORMAL.h"
#include "DETECT_BATTERY.h"
#include "DETECT_MOTOR.h"
#include "DD_MOTOR.h"
#include "BATTERY.h"
#include "ADC.h"
#include "MODE_CONDUCTOR.h"
#include "DIAG_SET.h"
#include "CAN.h"

/* for DEBUG */
#include "ramdef.h"

/* Private define -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static u8	u8_DET_BAT_abnBattVoltCounter = (u8)0;							/* 電池電圧差異常検出タイマ */
static u8	u8_DET_BAT_overVoltCounter = (u8)0;								/* 過電圧異常検出タイマ */
static u8	u8_DET_BAT_overVoltReleaseCounter = (u8)0;						/* 過電圧異常解除タイマ */
static u8	u8_DET_BAT_shutdownVoltCounter = (u8)0;							/* 低電圧シャットダウン検出タイマ */
static u8	u8_DET_BAT_shutdownVoltReleaseCounter = (u8)0;					/* 低電圧シャットダウン解除タイマ */

static UNI_ABN_WARNINGFLG	uni_DET_BAT_warningFlg;
static UNI_ABN_ERRORFLG		uni_DET_BAT_errorFlg;

#pragma bit_order right
#pragma unpack

static union
{
	u8			byte;
	struct
	{
		u8	b_abnormalOverVoltage			:1;								/* b0:過電圧異常			(1:過電圧) */
		u8	b_abnormalBatterytVoltage		:1;								/* b1:電池電圧差異常		(1:電池電圧とインバータ電圧の差が5V以上) */
		u8									:6;								/* b2-7:空き */
	}bit;
}uni_DET_BAT_abnormalFlg;

static union
{
	u8			byte;
	struct
	{
		u8	b_warningVoltageDropped			:1;								/* b0:低電圧警告		(1:電圧低下=走行不可) */
		u8	b_warningShutdownVoltageDropped	:1;								/* b1:シャットダウン	(1:低電圧シャットダウン) */
		u8									:6;								/* b2-7:空き */
	}bit;
}uni_DET_BAT_battCautionFlg;

/* Private function prototypes -----------------------------------------------*/
static void	v_DET_BAT_DetectOverVoltage(void);								/* 過電圧異常判定 */
static void	v_DET_BAT_DetectUnderVoltage(void);								/* 低電圧検出判定 */
static void	v_DET_BAT_DetectBatteryVoltage(void);							/* 電池電圧差異常判定 */

/**
 * @brief バッテリ異常関連の変数初期化処理
 * @param none
 * @return none
 */
void V_DET_BAT_Initialize(void)
{
	u8_DET_BAT_overVoltCounter = (u8)0;														/* 過電圧異常検出タイマ初期化 */
	u8_DET_BAT_shutdownVoltCounter = (u8)0;													/* 低電圧シャットダウン検出タイマ初期化 */
	u8_DET_BAT_shutdownVoltReleaseCounter = (u8)0;											/* 低電圧シャットダウン解除タイマ初期化 */

	uni_DET_BAT_abnormalFlg.byte		= (u8)0;
	uni_DET_BAT_battCautionFlg.byte		= (u8)0;

	uni_DET_BAT_warningFlg.ulong = (u32)0;													/* CAN送信用警告フラグ初期化 */
	uni_DET_BAT_errorFlg.ulong = (u32)0;													/* CAN送信用異常フラグ初期化 */
}


/**
 * @brief バッテリ異常関連の異常検出周期処理
 * @param none
 * @return none
 * @detail 各異常・警告を検出する
 * @note 10msec周期
 */
void V_DET_BAT_Loop10ms(void)
{
	EN_MDC_MODE	a_en_mode = En_MDC_GetMode();												/* 動作モード取得 */

	bool	a_b_isDetectActivity = B_MDC_IsDetectActivity();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( a_b_isDetectActivity	== FALSE )
	{																						/* 初期異常検出遅延中⇒異常検出しない */
		return;
	}

	if( a_en_mode == MODE_SHUTDOWN_WAIT )
	{																						/* 遮断待機モード⇒異常検出しない */
		return;
	}

/*===== 遮断待機モード以外 ======*/
/*===== 以下は起動中のみ検出判定を行う ======*/
/*----- 電圧関連 -----*/
	v_DET_BAT_DetectOverVoltage();															/* 過電圧異常判定 */
	v_DET_BAT_DetectUnderVoltage();															/* 低電圧検出判定 */

	switch( a_en_mode )
	{
	case MODE_RUN:
		v_DET_BAT_DetectBatteryVoltage();
		break;

	default:
		break;
	}
}				/**/


/**
 * @brief 過電圧異常判定
 * @param none
 * @return none
 */
static void v_DET_BAT_DetectOverVoltage(void)
{																							/* 回生ありのため、過電圧異常は削除 */
	u16 a_u16_battVoltAve_10mV = U16_DD_MTR_GetBattVolt_10mV();							/* バッテリ電圧取得 */

	if( uni_DET_BAT_abnormalFlg.bit.b_abnormalOverVoltage == FALSE )						/* 過電圧異常フラグ検知 */
	{																						/* 過電圧異常フラグOFF場合→過電圧異常検知 */
		if( B_ABN_GetResultAbnormal( &u8_DET_BAT_overVoltCounter, a_u16_battVoltAve_10mV, (U8_ABN_UPPER_LIM_COMPARISON), (U16_OVER_BATT_VOLT_10MV), (U16_ABN_ABNORMAL_COUNT_30MS) ) == FALSE )		/* 異常判断結果を確認 */
		{																					/* 過電圧異常の場合 */
			uni_DET_BAT_abnormalFlg.bit.b_abnormalOverVoltage = TRUE;						/* タイマの閾値超えると→過電圧異常フラグをSET */
			uni_DET_BAT_errorFlg.bit.b_abnormalOverVoltage = TRUE;							/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_OVER_VOLT)  );							/* ダイアグコードの格納 */
		}
	}
	else
	{																						/* 過電圧異常解除条件検知 異常判定ロジックを流用 */
		if( B_ABN_GetResultAbnormal( &u8_DET_BAT_overVoltReleaseCounter, a_u16_battVoltAve_10mV, (U8_ABN_LOWER_LIM_COMPARISON), (U16_OVER_BATT_RELEASE_VOLT_10MV), (U16_ABN_ABNORMAL_COUNT_30MS) ) == FALSE )		/* 異常判断結果を確認 */
		{																					/* 過電圧解除の場合 */
			uni_DET_BAT_abnormalFlg.bit.b_abnormalOverVoltage = FALSE;						/* 過電圧異常フラグをクリア */
		}

	}
}


/**
 * @brief 低電圧異常判定
 * @param none
 * @return none
 */
static void v_DET_BAT_DetectUnderVoltage(void)
{
	u16	a_u16_battVoltAve_10mV = U16_BAT_GetBattVoltAve_10mV();							/* バッテリ電圧取得 */

/*======= 低電圧シャットダウン電圧判定 ==========*/
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( uni_DET_BAT_battCautionFlg.bit.b_warningVoltageDropped == FALSE )					/* 低電圧シャットダウンフラグを確認 */
	{
	/* 低電圧シャットダウンフラグOFFの場合 */
		if( B_ABN_GetResultAbnormal( &u8_DET_BAT_shutdownVoltCounter, a_u16_battVoltAve_10mV, (U8_ABN_LOWER_LIM_COMPARISON), (U16_LOWVOLTAGE_SHUTDOWN_BATT_VOLT_10MV), (U16_ABN_ABNORMAL_COUNT_30MS) ) == FALSE )	/* 異常判断結果を確認 */
		{																					/* シャットダウン電圧異常 */
			uni_DET_BAT_battCautionFlg.bit.b_warningVoltageDropped = TRUE;					/* 低電圧警告フラグをON */

			if( a_u16_battVoltAve_10mV <= (U16_LOWVOLTAGE_SHUTDOWN_BATT_VOLT_10MV) )
			{
				uni_DET_BAT_battCautionFlg.bit.b_warningShutdownVoltageDropped = TRUE;		/* 低電圧シャットダウンフラグをON */
			}
		}
	}
	else
	{
		if( a_u16_battVoltAve_10mV >= (U16_LOWVOLTAGE_RELEASE_BATT_VOLT_10MV) )
		{
			u8_DET_BAT_shutdownVoltReleaseCounter++;
			if( u8_DET_BAT_shutdownVoltReleaseCounter >= (U16_ABN_ABNORMAL_COUNT_30MS) )
			{
				uni_DET_BAT_battCautionFlg.bit.b_warningVoltageDropped = FALSE;				/* 低電圧警告フラグをクリア */
				uni_DET_BAT_battCautionFlg.bit.b_warningShutdownVoltageDropped = FALSE;		/* 低電圧シャットダウンフラグをクリア */
				u8_DET_BAT_shutdownVoltReleaseCounter = (u8)0;								/* 低電圧シャットダウン解除タイマ初期化 */
			}
		}
	}
/*======= その他の場合 ==========*/
}

/**
 * @brief 電池電圧差異常判定
 * @param none
 * @return none
 */
static void v_DET_BAT_DetectBatteryVoltage(void)
{
	s32	a_s32_diffVolt	= (s32)U16_BAT_GetBattVoltAve_10mV() - (s32)U16_BAT_GetInvVoltAve_10mV();
	u32	a_u32_diffVolt	= (u32)(s32)CMF_GETABS( a_s32_diffVolt );
	u16	a_u16_diffVolt_10mv = (u16)(u32)CMF_GETMIN( a_u32_diffVolt, (u32)0xFFFF );

	EN_MDC_PRECHARGE_MODE	a_en_prechargeMode = En_MDC_GetPrechargeMode();					/* プリチャージモード取得 */

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	/* プリチャージ未完了の場合、異常判定は行わない */
	if( a_en_prechargeMode != MD_PRECHARGE_FINISH )
	{
		return;
	}

	if( uni_DET_BAT_abnormalFlg.bit.b_abnormalBatterytVoltage == TRUE )
	{
		if( B_ABN_GetResultAbnormal( &u8_DET_BAT_abnBattVoltCounter, a_u16_diffVolt_10mv, (U8_ABN_UPPER_LIM_COMPARISON), (U16_DETECT_BATT_VOLT_DIFF_10MV), (U16_ABN_ABNORMAL_COUNT_1S) ) == FALSE )		/* 異常判断結果を確認 */
		{
			uni_DET_BAT_abnormalFlg.bit.b_abnormalBatterytVoltage = TRUE;					/* 電池電圧差異常フラグをSET */
			uni_DET_BAT_errorFlg.bit.b_abnormalBatterytVoltage = TRUE;						/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_ABN_BATT_VOLT) );						/* ダイアグコードの格納 */
		}
	}
	else
	{
		if( B_ABN_GetResultAbnormal( &u8_DET_BAT_abnBattVoltCounter, a_u16_diffVolt_10mv, (U8_ABN_LOWER_LIM_COMPARISON), (U16_DETECT_BATT_VOLT_DIFF_10MV), (U16_ABN_ABNORMAL_COUNT_1S) ) == FALSE )		/* 異常判断結果を確認 */
		{
			uni_DET_BAT_abnormalFlg.bit.b_abnormalBatterytVoltage = FALSE;					/* 電池電圧差異常フラグをクリア */
		}
	}
}

/**
 * @brief 低電圧警告状態提供
 * @param none
 * @return uni_DET_BAT_battCautionFlg.bit.b_warningVoltageDropped
 */
bool	B_DET_BAT_IsWarningVoltageDropped(void)
{
	return	uni_DET_BAT_battCautionFlg.bit.b_warningVoltageDropped;
}

/**
 * @brief 低電圧シャットダウン状態提供
 * @param none
 * @return uni_DET_BAT_battCautionFlg.bit.b_warningShutdownVoltageDropped
 */
bool	B_DET_BAT_IsShutdownVoltageDropped(void)
{
	return	uni_DET_BAT_battCautionFlg.bit.b_warningShutdownVoltageDropped;
}

/**
 * @brief (DET_BATクラス)異常フラグクリア
 * @param none
 * @return none
 */
void	V_DET_BAT_ClearAbnormalFlg(void)
{
	uni_DET_BAT_abnormalFlg.byte = (u8)0;
}

/**
 * @brief (DET_BATクラス)異常フラグ提供
 * @param none
 * @return uni_DET_BAT_abnormalFlg.byte
 */
u8	U8_DET_BAT_IsAbnormalFlg(void)
{
	return	(u8)uni_DET_BAT_abnormalFlg.byte;
}

/**
 * @brief バッテリ警告フラグクリア
 * @param none
 * @return none
 */
void	V_DET_BAT_ClearBattWarningFlg(void)
{
	uni_DET_BAT_battCautionFlg.byte = (u8)0;
}

/**
 * @brief (DET_BATクラス)異常フラグ提供
 * @param none
 * @return uni_DET_BAT_errorFlg.ulong
 */
u32	U32_DET_BAT_GetErrorFlg(void)
{
	return	(u32)uni_DET_BAT_errorFlg.ulong;
}

/**
 * @brief (DET_BATクラス)警告フラグ提供
 * @param none
 * @return uni_DET_BAT_warningFlg.ulong
 */
u32	U32_DET_BAT_GetWarningFlg(void)
{
	return	(u32)uni_DET_BAT_warningFlg.ulong;
}
