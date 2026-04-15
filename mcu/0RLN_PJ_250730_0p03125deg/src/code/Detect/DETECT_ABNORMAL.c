/**
 * @file DETECT_ABNORMAL.c
 * @brief 異常検出
 * @note 50ms周期
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "TYPEDEF_MODE.h"
#include "TYPEDEF_ABN.h"
#include "DEFINE_SYSTEM.h"		/*定数定義*/
#include "DEFINE_ABN.h"
#include "DEFINE_SETTING.h"
#include "COMMON_FUNCTION.h"

/* DDL */
#include "DD_MOTOR.h"

#include "DETECT_ABNORMAL.h"
#include "DETECT_MOTOR.h"
#include "DETECT_BATTERY.h"
#include "DIAG_SET.h"
#include "ADC.h"
#include "CAN.h"
#include "BATTERY.h"
#include "MODE_CONDUCTOR.h"

/* for DEBUG */
#include "ramdef.h"

/* Private define -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#ifdef	DEBUG_WATCH_ACTIVITY

u8	u8_ABN_fetThermistorOpenCounter		= (u8)0;			/* サーミスタ断線異常検出タイマ */
u8	u8_ABN_capThermistorOpenCounter		= (u8)0;			/* サーミスタ断線異常検出タイマ */
u8	u8_ABN_relayErrorCounter			= (u8)0;			/* リレー駆動異常検出タイマ */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
u8	u8_ABN_motThermistorOpenCounter		= (u8)0;			/* サーミスタ断線異常検出タイマ */
#endif

#ifdef	DETECT_ACCEL_ACTIVITY
u8	u8_ABN_accelOutOfRangeCounter		= (u8)0;			/* アクセル電圧逸脱異常検出タイマ */
u8	u8_ABN_accelOutOfRangeAtBootCounter	= (u8)0;			/* 初期アクセル電圧逸脱異常検出タイマ */
u8	u8_ABN_accelDetectAtBootCounter		= (u8)0;			/* 初期アクセル異常判定時間カウント用タイマ */
bool	b_ABN_isJudgedDetectAccelAtBoot = FALSE;				/* 初期アクセル異常判定済み */
#endif

UNI_ABN_WARNINGFLG	uni_ABN_warningFlg;						/* VCU送信専用 */
UNI_ABN_ERRORFLG		uni_ABN_errorFlg;						/* VCU送信専用 */

#pragma bit_order right
#pragma unpack

union
{
	u16			word;
	struct
	{
		u16	b_abnormalPrechargeWaitOver		:1;		/* b0:プリチャージ異常 */
		u16	b_abnormalMosThermistorOpen		:1;		/* b1:FETサーミスタ断線異常			(1:サーミスタ断線) */
		u16	b_abnormalCapThermistorOpen		:1;		/* b2:コンデンササーミスタ断線異常	(1:サーミスタ断線) */
		u16	b_abnormalAccelOutOfRangeAtBoot	:1;		/* b3:初期アクセル電圧逸脱異常		(1:逸脱) */
		u16	b_abnormalAccelOutOfRange		:1;		/* b4:アクセル電圧逸脱異常			(1:逸脱) */
		u16	b_abnormalMotThermistorOpen		:1;		/* b5:モータサーミスタ断線異常		(1:サーミスタ断線) */
		u16	b_abnormalCanCommunication		:1;		/* b6:CAN通信異常 */
		u16	b_abnormalDischargeWaitOver		:1;		/* b7:ディスチャージ異常 */
		u16	b_abnormalRelay					:1;		/* b8:リレー駆動異常 */
		u16									:7;		/* b9-b15:空き */
	}bit;
}uni_ABN_abnormalFlg;

#else

static u8	u8_ABN_fetThermistorOpenCounter		= (u8)0;			/* サーミスタ断線異常検出タイマ */
static u8	u8_ABN_capThermistorOpenCounter		= (u8)0;			/* サーミスタ断線異常検出タイマ */
static u8	u8_ABN_relayErrorCounter			= (u8)0;			/* リレー駆動異常検出タイマ */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
static u8	u8_ABN_motThermistorOpenCounter		= (u8)0;			/* サーミスタ断線異常検出タイマ */
#endif

#ifdef	DETECT_ACCEL_ACTIVITY
static u8	u8_ABN_accelOutOfRangeCounter		= (u8)0;			/* アクセル電圧逸脱異常検出タイマ */
static u8	u8_ABN_accelOutOfRangeAtBootCounter	= (u8)0;			/* 初期アクセル電圧逸脱異常検出タイマ */
static u8	u8_ABN_accelDetectAtBootCounter		= (u8)0;			/* 初期アクセル異常判定時間カウント用タイマ */
static bool	b_ABN_isJudgedDetectAccelAtBoot = FALSE;				/* 初期アクセル異常判定済み */
#endif

static UNI_ABN_WARNINGFLG	uni_ABN_warningFlg;						/* VCU送信専用 */
static UNI_ABN_ERRORFLG		uni_ABN_errorFlg;						/* VCU送信専用 */

#pragma bit_order right
#pragma unpack

static union
{
	u16			word;
	struct
	{
		u16	b_abnormalPrechargeWaitOver		:1;		/* b0:プリチャージ異常 */
		u16	b_abnormalMosThermistorOpen		:1;		/* b1:FETサーミスタ断線異常			(1:サーミスタ断線) */
		u16	b_abnormalCapThermistorOpen		:1;		/* b2:コンデンササーミスタ断線異常	(1:サーミスタ断線) */
		u16	b_abnormalAccelOutOfRangeAtBoot	:1;		/* b3:初期アクセル電圧逸脱異常		(1:逸脱) */
		u16	b_abnormalAccelOutOfRange		:1;		/* b4:アクセル電圧逸脱異常			(1:逸脱) */
		u16	b_abnormalMotThermistorOpen		:1;		/* b5:モータサーミスタ断線異常		(1:サーミスタ断線) */
		u16	b_abnormalCanCommunication		:1;		/* b6:CAN通信異常 */
		u16	b_abnormalDischargeWaitOver		:1;		/* b7:ディスチャージ異常 */
		u16	b_abnormalRelay					:1;		/* b8:リレー駆動異常 */
		u16									:7;		/* b9-b15:空き */
	}bit;
}uni_ABN_abnormalFlg;

#endif
/* Private function prototypes -----------------------------------------------*/
static void	v_ABN_DetectFetThermistorOpen(void);									/* FETサーミスタ断線異常判定 */
static void	v_ABN_DetectCapThermistorOpen(void);									/* コンデンササーミスタ断線異常判定 */
static void v_ABN_DetectRelay(void);												/* リレー駆動異常判定 */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
static void	v_ABN_DetectMotThermistorOpen(void);									/* モータサーミスタ断線異常判定 */
#endif

#ifdef	DETECT_ACCEL_ACTIVITY
static void	v_ABN_DetectAccelOutOfRange(void);										/* アクセル電圧逸脱異常判定 */
static void	v_ABN_DetectAccelAtBoot(u16 a_u16_adValueAccel);						/* 起動時アクセル電圧逸脱異常判定 */
static void	v_ABN_DetectAccel(u16 a_u16_adValueAccel);								/* アクセル電圧逸脱異常判定 */
#endif

/*----- CAN通信異常 -----*/


/**
 * @brief 異常検出周期処理
 * @param none
 * @return none
 * @detail 各異常・警告を検出する
 * @note 10msec周期
 */
void V_ABN_Loop10ms(void)
{
	EN_MDC_MODE	a_en_mode = En_MDC_GetMode();										/* 動作モード取得 */

	bool	a_b_isDetectActivity = B_MDC_IsDetectActivity();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( a_b_isDetectActivity	== FALSE )
	{																				/* 初期異常検出遅延中⇒異常検出しない */
		return;
	}

	if( a_en_mode == MODE_SHUTDOWN_WAIT )
	{																				/* 遮断待機モード⇒異常検出しない */
		return;
	}

/*===== 遮断待機モード以外 ======*/
/*===== 以下は起動中のみ検出判定を行う ======*/

/*----- 電圧関連 -----*/
	V_DET_BAT_Loop10ms();															/* バッテリ関連異常判定 */

/*----- モータ関連 -----*/
	V_DET_MTR_Loop10ms();															/* モータ関連異常判定 */

/*----- リレー関連 -----*/
	v_ABN_DetectRelay();															/* リレー駆動異常判定 */

/*----- センサ関連 -----*/
#ifdef	MOTOR_THERMISTOR_ACTIVITY
	v_ABN_DetectMotThermistorOpen();												/* モータサーミスタ断線異常判定 */
#endif

	v_ABN_DetectFetThermistorOpen();												/* FETサーミスタ断線異常判定 */
	v_ABN_DetectCapThermistorOpen();												/* コンデンササーミスタ断線異常判定 */

#ifdef	DETECT_ACCEL_ACTIVITY
	v_ABN_DetectAccelOutOfRange();													/* アクセル異常判定 */
#endif

	V_MDC_DetectPrecharge();														/* プリチャージ異常判定 */

	V_MDC_DetectDischarge();														/* ディスチャージ異常判定 */

	V_CAN_DetectCan();																/* CAN関連 */
}

/**
 * @brief リレー駆動異常判定
 * @param none
 * @return none
 */
static void v_ABN_DetectRelay(void)
{
	s16	a_s16_diffVolt_10mv	= (s16)U16_BAT_GetBattVoltAve_10mV() - (s16)U16_BAT_GetInvVoltAve_10mV();
	u16	a_u16_diffVolt_10mv	= (u16)(s16)CMF_GETABS( a_s16_diffVolt_10mv );

	EN_MDC_PRECHARGE_MODE	a_en_prechargeMode = En_MDC_GetPrechargeMode();			/* プリチャージモード取得 */

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	/* プリチャージ未完了の場合、異常判定は行わない */
	if( a_en_prechargeMode != MD_PRECHARGE_FINISH )
	{
		return;
	}

	/* 既に異常の場合、異常判定は行わない */
	if( uni_ABN_abnormalFlg.bit.b_abnormalRelay == TRUE )
	{
		return;
	}

	if( B_ABN_GetResultAbnormal( &u8_ABN_relayErrorCounter, a_u16_diffVolt_10mv, (U8_ABN_UPPER_LIM_COMPARISON), (U16_ABN_BATT_VOLT_DIFF_10MV), (U16_ABN_ABNORMAL_COUNT_1S) ) == FALSE )	/* 異常判断結果を確認 */
	{
		uni_ABN_abnormalFlg.bit.b_abnormalRelay = TRUE;									/* リレー駆動異常フラグをSET */
		V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_RELAY_ERR) );							/* ダイアグコードの格納 */
	}
}

/**
 * @brief 異常検出変数初期化
 * @param none
 * @return none
 */
void V_ABN_Initialize(void)
{
/*--- 異常フラグリセット -------------------------*/
	uni_ABN_abnormalFlg.word = (u16)0;

	u8_ABN_fetThermistorOpenCounter		= (u8)0;										/* サーミスタ断線異常検出タイマをリセット */
	u8_ABN_capThermistorOpenCounter		= (u8)0;										/* サーミスタ断線異常検出タイマをリセット */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
	u8_ABN_motThermistorOpenCounter		= (u8)0;										/* サーミスタ断線異常検出タイマをリセット */
#endif

#ifdef	DETECT_ACCEL_ACTIVITY
	u8_ABN_accelOutOfRangeCounter		= (u8)0;										/* アクセル電圧逸脱異常検出タイマをリセット */
	u8_ABN_accelOutOfRangeAtBootCounter	= (u8)0;										/* 初期アクセル電圧逸脱異常検出タイマをリセット */
	u8_ABN_accelDetectAtBootCounter		= (u8)0;										/* 初期アクセル異常判定時間カウント用タイマをリセット */
#endif

	uni_ABN_warningFlg.ulong = (u32)0;													/* CAN送信用警告フラグ初期化 */
	uni_ABN_errorFlg.ulong = (u32)0;													/* CAN送信用異常フラグ初期化 */
	V_DET_MTR_Initialize();
	V_DET_BAT_Initialize();
}


/**
 * @brief 異常カウンタの判定
 * @param *a_pu8_count:異常カウンタへのポインター
 * @param a_s16_cmpItem:異常警告判定相手
 * @param a_u8_cmpType:異常比較種類
 * @param a_s16_threshold:異常とする閾値
 * @param a_u16_countMax:異常継続最大時間
 * @return a_b_result:異常判定結果 TRUE=正常 FALSE=異常
 * @note a_u8_cmpType:(U8_ABN_UPPER_LIM_COMPARISON):異常閾値を超えると異常になる / (U8_ABN_LOWER_LIM_COMPARISON):異常閾値を下回ると異常になる
 */
bool B_ABN_GetResultAbnormal( u8 *a_pu8_count, s16 a_s16_cmpItem, u8 a_u8_cmpType, s16 a_s16_threshold, u16 a_u16_countMax )
{
	bool a_b_result;																/* 異常判定結果 */

	if( a_u8_cmpType == (U8_ABN_UPPER_LIM_COMPARISON) )								/* 異常比較種類を確認 */
	{																				/* 異常閾値を超えると異常になる場合 */
		if( a_s16_cmpItem < a_s16_threshold )										/* 異常判定の閾値と比較 */
		{																			/* 閾値より小さい場合 */
			*a_pu8_count = (u8)0;													/* 異常カウンタクリア */
		}
		else
		{																			/* 閾値より大きい場合 */
			( *a_pu8_count )++;														/* 異常カウントアップ */
		}
	}
	else
	{																				/* 異常閾値より小さいと異常になる場合 */
		if( a_s16_cmpItem > a_s16_threshold )										/* 異常判定の閾値と比較 */
		{																			/* 閾値より大きい場合 */
			*a_pu8_count = (u8)0;													/* 異常カウンタクリア */
		}
		else
		{																			/* 閾値より小さい場合 */
			( *a_pu8_count )++;														/* 異常カウントアップ */
		}
	}

	if( *a_pu8_count >= a_u16_countMax )											/* 異常継続時間は異常時間閾値と比較 */
	{																				/* 異常閾値を超えた場合 */
		a_b_result = FALSE;															/* 異常の判断 */
		*a_pu8_count = a_u16_countMax;												/* カウンタを最大値でクリップ */
	}
	else
	{																				/* 異常閾値以下の場合 */
		a_b_result = TRUE;															/* 正常の判断 */
	}

	return a_b_result;																/* 判断結果を返す */
}

/**
 * @brief FETサーミスタ断線異常判定
 * @param none
 * @return none
 */
static void v_ABN_DetectFetThermistorOpen(void)
{

	u16 a_u16_adValueFetThermistor = U16_ADC_GetAdValueAveFetThermistor();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if ( (a_u16_adValueFetThermistor < (U16_THERM_ADC_RELEASE_RANGE_MAX) )
	  && (a_u16_adValueFetThermistor > (U16_THERM_ADC_RELEASE_RANGE_MIN) ) )		/* サーミスタ電圧の範囲を確認 */
	{																				/* サーミスタ電圧が復帰範囲内の場合 */
		u8_ABN_fetThermistorOpenCounter = (u8)0;									/* サーミスタ断線異常検出タイマをリセット */
		uni_ABN_abnormalFlg.bit.b_abnormalMosThermistorOpen = FALSE;				/* サーミスタ断線異常フラグをクリア */
		uni_ABN_warningFlg.bit.b_warningMosThermistorOpen = FALSE;					/* CAN送信用サーミスタ断線警告フラグ */
		uni_ABN_errorFlg.bit.b_abnormalMosThermistorOpen = FALSE;					/* CAN送信用サーミスタ断線異常フラグ */
		return;
	}

	if( (a_u16_adValueFetThermistor < (U16_THERM_ADC_RANGE_MAX)) && (a_u16_adValueFetThermistor > (U16_THERM_ADC_RANGE_MIN)) )	/* サーミスタ電圧の範囲を確認 */
	{																				/* サーミスタ電圧が正常範囲内の場合 */
		u8_ABN_fetThermistorOpenCounter = (u8)0;									/* サーミスタ断線異常検出タイマをリセット */
	}
	else
	{																				/* サーミスタ電圧は正常範囲を超えた場合 */
		u8_ABN_fetThermistorOpenCounter++;											/* サーミスタ断線異常検出タイマを更新 */
		if( u8_ABN_fetThermistorOpenCounter >= (u8)(U16_ABN_ABNORMAL_COUNT_250MS) )	/* 断線異常連続時間を確認 */
		{																			/* 連続時間が閾値を超えた場合 */
			if( uni_ABN_abnormalFlg.bit.b_abnormalMosThermistorOpen == TRUE )		/* サーミスタ断線異常フラグを確認 */
			{
				uni_ABN_abnormalFlg.bit.b_abnormalMosThermistorOpen = TRUE;			/* サーミスタ断線異常フラグをSET */
				uni_ABN_warningFlg.bit.b_warningMosThermistorOpen = TRUE;			/* CAN送信用サーミスタ断線警告フラグ */
				uni_ABN_errorFlg.bit.b_abnormalMosThermistorOpen = TRUE;			/* CAN送信用サーミスタ断線異常フラグ */
				V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_FET_THERMISTOR_OPEN) );		/* ダイアグコードの格納 */
			}
		}
	}
}

/**
 * @brief コンデンササーミスタ断線異常判定
 * @param none
 * @return none
 */
static void v_ABN_DetectCapThermistorOpen(void)
{

	u16 a_u16_adValueCapThermistor = U16_ADC_GetAdValueAveCapThermistor();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if ( (a_u16_adValueCapThermistor < (U16_THERM_ADC_RELEASE_RANGE_MAX) )
	  && (a_u16_adValueCapThermistor > (U16_THERM_ADC_RELEASE_RANGE_MIN) ) )		/* サーミスタ電圧の範囲を確認 */
	{																				/* サーミスタ電圧が復帰範囲内の場合 */
		u8_ABN_capThermistorOpenCounter = (u8)0;									/* サーミスタ断線異常検出タイマをリセット */
		uni_ABN_abnormalFlg.bit.b_abnormalCapThermistorOpen = FALSE;				/* サーミスタ断線異常フラグをクリア */
		uni_ABN_warningFlg.bit.b_warningCapThermistorOpen = FALSE;					/* CAN送信用サーミスタ断線警告フラグ */
		uni_ABN_errorFlg.bit.b_abnormalCapThermistorOpen = FALSE;					/* CAN送信用サーミスタ断線異常フラグ */
		return;
	}

	if( (a_u16_adValueCapThermistor < (U16_THERM_ADC_RANGE_MAX)) && (a_u16_adValueCapThermistor > (U16_THERM_ADC_RANGE_MIN)) )	/* サーミスタ電圧の範囲を確認 */
	{																				/* サーミスタ電圧が正常範囲内の場合 */
		u8_ABN_capThermistorOpenCounter = (u8)0;									/* サーミスタ断線異常検出タイマをリセット */
	}
	else
	{																				/* サーミスタ電圧は正常範囲を超えた場合 */
		u8_ABN_capThermistorOpenCounter++;											/* サーミスタ断線異常検出タイマを更新 */
		if( u8_ABN_capThermistorOpenCounter >= (u8)(U16_ABN_ABNORMAL_COUNT_250MS) )	/* 断線異常連続時間を確認 */
		{	
			if( uni_ABN_abnormalFlg.bit.b_abnormalCapThermistorOpen == TRUE )		/* サーミスタ断線異常フラグを確認 */
			{
				uni_ABN_abnormalFlg.bit.b_abnormalCapThermistorOpen = TRUE;			/* サーミスタ断線異常フラグをSET */
				uni_ABN_warningFlg.bit.b_warningCapThermistorOpen = TRUE;			/* CAN送信用サーミスタ断線警告フラグ */
				uni_ABN_errorFlg.bit.b_abnormalCapThermistorOpen = TRUE;			/* CAN送信用サーミスタ断線異常フラグ */
				V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_CAP_THERMISTOR_OPEN) );		/* ダイアグコードの格納 */
			}																		/* 連続時間が閾値を超えた場合 */
		}
	}

}

#ifdef	MOTOR_THERMISTOR_ACTIVITY
/**
 * @brief モータサーミスタ断線異常判定
 * @param none
 * @return none
 */
static void v_ABN_DetectMotThermistorOpen(void)
{

	u16	a_u16_adValueMotThermistor = U16_ADC_GetAdValueAveMotThermistor();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if ( (a_u16_adValueMotThermistor < (U16_MTR_THERM_ADC_RELEASE_RANGE_MAX) )
	  && (a_u16_adValueMotThermistor > (U16_MTR_THERM_ADC_RELEASE_RANGE_MIN) ) )	/* サーミスタ電圧の範囲を確認 */
	{																				/* サーミスタ電圧が復帰範囲内の場合 */
		u8_ABN_motThermistorOpenCounter = (u8)0;									/* サーミスタ断線異常検出タイマをリセット */
		uni_ABN_abnormalFlg.bit.b_abnormalMotThermistorOpen = FALSE;				/* サーミスタ断線異常フラグをクリア */
		uni_ABN_warningFlg.bit.b_warningMtrThermistorOpen = FALSE;					/* CAN送信用サーミスタ断線警告フラグ */
		return;
	}

	if( (a_u16_adValueMotThermistor < (U16_MTR_THERM_ADC_RANGE_MAX)) && (a_u16_adValueMotThermistor > (U16_MTR_THERM_ADC_RANGE_MIN)) )	/* サーミスタ電圧の範囲を確認 */
	{																				/* サーミスタ電圧が正常範囲内の場合 */
		u8_ABN_motThermistorOpenCounter = (u8)0;									/* サーミスタ断線異常検出タイマをリセット */
	}
	else
	{																				/* サーミスタ電圧は正常範囲を超えた場合 */
		u8_ABN_motThermistorOpenCounter++;											/* サーミスタ断線異常検出タイマを更新 */
		if( u8_ABN_motThermistorOpenCounter >= (u8)(U16_ABN_ABNORMAL_COUNT_250MS) )	/* 断線異常連続時間を確認 */
		{																			/* 連続時間を閾値を超えた場合 */
			if( uni_ABN_abnormalFlg.bit.b_abnormalMotThermistorOpen == TRUE )		/* サーミスタ断線異常フラグを確認 */
			{	
				uni_ABN_abnormalFlg.bit.b_abnormalMotThermistorOpen = TRUE;			/* サーミスタ断線異常フラグをSET */
				uni_ABN_warningFlg.bit.b_warningMtrThermistorOpen = TRUE;			/* CAN送信用サーミスタ断線警告フラグ */
				V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_MOT_THERMISTOR_OPEN) );		/* ダイアグコードの格納 */
			}
		}
	}
}
#endif

#ifdef	DETECT_ACCEL_ACTIVITY
/**
 * @brief アクセル異常判定
 * @param none
 * @return none
 */
static void	v_ABN_DetectAccelOutOfRange(void)
{
	EN_MDC_MODE	a_en_mode = En_MDC_GetMode();										/* 動作モード取得 */
	u16	a_u16_adValueAccel = U16_ADC_GetAdValueAveAccel();

	switch( a_en_mode )
	{
	case MODE_STANDBY:
		v_ABN_DetectAccelAtBoot( a_u16_adValueAccel );
		break;
	case MODE_RUN:
#ifdef	APS_FAULT_INVALID

#else
		v_ABN_DetectAccel( a_u16_adValueAccel );
#endif
		break;
	default:
		break;
	}
}

static void	v_ABN_DetectAccelAtBoot( u16 a_u16_adValueAccel )
{
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( (uni_ABN_abnormalFlg.bit.b_abnormalAccelOutOfRangeAtBoot == TRUE) ||		/* 初期アクセル異常フラグを確認 */
		(b_ABN_isJudgedDetectAccelAtBoot == TRUE) )									/* 初期アクセル異常判定済みフラグを確認 */
	{
		return;
	}

	/* 判定時間のカウント */
	u8_ABN_accelDetectAtBootCounter++;												/* アクセル異常検出タイマを更新 */
	if( u8_ABN_accelDetectAtBootCounter >= (u8)(U16_ABN_ABNORMAL_COUNT_250MS) )
	{
		b_ABN_isJudgedDetectAccelAtBoot = TRUE;
		u8_ABN_accelDetectAtBootCounter = (u8)0;
		u8_ABN_accelOutOfRangeAtBootCounter = (u8)0;
		return;
	}

	/* 異常範囲の判定及び異常時間のカウント */
	if( a_u16_adValueAccel < (U16_ACCEL_ADC_RANGE_AT_BOOT) )						/* アクセル電圧の範囲を確認 */
	{																				/* アクセル電圧は正常範囲の場合 */
		u8_ABN_accelOutOfRangeAtBootCounter = (u8)0;								/* アクセル異常検出タイマをリセット */
	}
	else
	{																				/* アクセル電圧は正常範囲を超えた場合 */
		u8_ABN_accelOutOfRangeAtBootCounter++;										/* アクセル異常検出タイマを更新 */
		if( u8_ABN_accelOutOfRangeAtBootCounter >= (u8)(U16_ABN_ABNORMAL_COUNT_100MS) )	/* 断線異常連続時間を確認 */
		{																			/* 連続時間を閾値を超えた場合 */
			uni_ABN_abnormalFlg.bit.b_abnormalAccelOutOfRangeAtBoot = TRUE;			/* アクセル異常フラグをSET */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_ACCEL_AT_BOOT) );				/* ダイアグコードの格納 */
		}
	}
}

static void	v_ABN_DetectAccel(u16 a_u16_adValueAccel)								/* アクセル電圧逸脱異常判定 */
{

#ifdef APS_GROUND_FAULT_INVALID														/* APS地絡異常無効 */
	if( a_u16_adValueAccel < (U16_ACCEL_ADC_RANGE_UPPER) )							/* アクセル電圧の範囲を確認 */
#else
	if( (a_u16_adValueAccel < (U16_ACCEL_ADC_RANGE_UPPER)) && (a_u16_adValueAccel > (U16_ACCEL_ADC_RANGE_LOWER)) )	/* アクセル電圧の範囲を確認 */
#endif
	{																				/* アクセル電圧は正常範囲の場合 */
		u8_ABN_accelOutOfRangeCounter = (u8)0;										/* アクセル異常検出タイマをリセット */
		uni_ABN_abnormalFlg.bit.b_abnormalAccelOutOfRange = FALSE;					/* アクセル異常フラグをクリア */
	}
	else
	{																				/* アクセル電圧は正常範囲を超えた場合 */
		u8_ABN_accelOutOfRangeCounter++;											/* アクセル異常検出タイマを更新 */
		if( u8_ABN_accelOutOfRangeCounter >= (u8)(U16_ABN_ABNORMAL_COUNT_250MS) )	/* 断線異常連続時間を確認 */
		{																			/* 連続時間を閾値を超えた場合 */
			uni_ABN_abnormalFlg.bit.b_abnormalAccelOutOfRange = TRUE;				/* アクセル異常フラグをSET */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_ACCEL) );						/* ダイアグコードの格納 */
		}
	}
}
#endif

/**
 * @brief (ABNクラス)異常フラグクリア
 * @param none
 * @return none
 */
void	V_ABN_ClearAbnormalFlg(void)
{
	uni_ABN_abnormalFlg.word = (u16)0;
}

/**
 * @brief (ABNクラス)異常フラグ確認
 * @param none
 * @return uni_ABN_abnormalFlg.word
 */
u16	U16_ABN_IsAbnormalFlg(void)
{
	return	(u16)uni_ABN_abnormalFlg.word;
}

/**
 * @brief CAN通信異常解除
 * @param none
 * @return none
 */
void	V_ABN_ClearAbnormalCanCommunication(void)
{
	uni_ABN_abnormalFlg.bit.b_abnormalCanCommunication = FALSE;
}

/**
 * @brief CAN通信異常セット
 * @param none
 * @return none
 */
void	V_ABN_SetAbnormalCanCommunication(void)
{
	uni_ABN_abnormalFlg.bit.b_abnormalCanCommunication = TRUE;
}

#ifdef	DETECT_ACCEL_ACTIVITY
/**
 * @brief 初期アクセル異常判定状態提供
 * @param none
 * @return b_ABN_isJudgedDetectAccelAtBoot
 */
bool	B_ABN_IsJudgedDetectAccelAtBoot(void)
{
	return	b_ABN_isJudgedDetectAccelAtBoot;
}

/**
 * @brief 初期アクセル異常判定済みフラグ解除
 * @param none
 * @return none
 */
void	V_ABN_ReleaseJudgedDetectAccelAtBoot(void)
{
	b_ABN_isJudgedDetectAccelAtBoot = FALSE;
}
#endif

/**
 * @brief CAN通信異常状態提供
 * @param none
 * @return uni_ABN_abnormalFlg.bit.b_abnormalCanCommunication
 */
bool	B_ABN_IsAbnormalCanCommunication(void)
{
	return	uni_ABN_abnormalFlg.bit.b_abnormalCanCommunication;
}

/**
 * @brief プリチャージ異常セット
 * @param none
 * @return none
 */
void	V_ABN_SetAbnormalPrechargeWaitOver(void)
{
	uni_ABN_abnormalFlg.bit.b_abnormalPrechargeWaitOver = TRUE;
	uni_ABN_errorFlg.bit.b_abnormalPrechargeWaitOver = TRUE;
}

/**
 * @brief プリチャージ異常状態提供
 * @param none
 * @return uni_ABN_abnormalFlg.bit.b_abnormalPrechargeWaitOver
 */
bool	B_ABN_IsAbnormalPrechargeWaitOver(void)
{
	return	uni_ABN_abnormalFlg.bit.b_abnormalPrechargeWaitOver;
}

/**
 * @brief ディスチャージ異常セット
 * @param none
 * @return none
 */
void	V_ABN_SetAbnormalDischargeWaitOver(void)
{
	uni_ABN_abnormalFlg.bit.b_abnormalDischargeWaitOver = TRUE;
}

/**
 * @brief ディスチャージ異常状態提供
 * @param none
 * @return uni_ABN_abnormalFlg.bit.b_abnormalDischargeWaitOver
 */
bool	B_ABN_IsAbnormalDischargeWaitOver(void)
{
	return	uni_ABN_abnormalFlg.bit.b_abnormalDischargeWaitOver;
}

/**
 * @brief FETサーミスタ異常状態提供
 * @param none
 * @return uni_ABN_abnormalFlg.bit.b_abnormalMosThermistorOpen
 */
bool	B_ABN_IsAbnormalMosThermistorOpen(void)
{
	return	uni_ABN_abnormalFlg.bit.b_abnormalMosThermistorOpen;
}

/**
 * @brief コンデンササーミスタ異常状態提供
 * @param none
 * @return uni_ABN_abnormalFlg.bit.b_abnormalCapThermistorOpen
 */
bool	B_ABN_IsAbnormalCapThermistorOpen(void)
{
	return	uni_ABN_abnormalFlg.bit.b_abnormalCapThermistorOpen;
}

/**
 * @brief 異常状態提供
 * @param none
 * @return a_b_isAbn
 */
bool	B_ABN_IsSomethingAbnormal(void)
{
	bool	a_b_isAbn = TRUE;

	u8	a_u8_isBattAbnormal	= U8_DET_BAT_IsAbnormalFlg();
	u8	a_u8_isMtrAbnormal	= U8_DET_MTR_IsAbnormalFlg();
	u8	a_u8_isMtrAbnormal2	= U8_DET_MTR_IsAbnormalFlg2();

#ifdef	VCU_ERROR_ACTIVITY
	if( (B_CAN_IsVcuAbnormal()	== FALSE)		&&
		(uni_ABN_abnormalFlg.word	== (u16)0)	&&							/* 異常状態を確認 */
		(a_u8_isBattAbnormal		== (u8)0)	&&							/* 異常状態を確認 */
		(a_u8_isMtrAbnormal			== (u8)0)	&&							/* 異常状態を確認 */
		(a_u8_isMtrAbnormal2		== (u8)0) )								/* 異常状態を確認 */
#else
	if( (uni_ABN_abnormalFlg.word	== (u16)0)	&&							/* 異常状態を確認 */
		(a_u8_isBattAbnormal		== (u8)0)	&&							/* 異常状態を確認 */
		(a_u8_isMtrAbnormal			== (u8)0)	&&							/* 異常状態を確認 */
		(a_u8_isMtrAbnormal2		== (u8)0) )								/* 異常状態を確認 */
#endif
	{
		a_b_isAbn = FALSE;
	}

	return	a_b_isAbn;
}

/**
 * @brief	異常フラグ集約＆提供
 * @param	none
 * @return	uni_ABN_errorFlg.ulong
 * @note	異常復帰については別途考える
 */
u32	U32_ABN_GetErrorFlg(void)
{
	UNI_ABN_ERRORFLG	a_uni_ABN_canErrorFlg;
	u32	a_u32_battErrFlg = U32_DET_BAT_GetErrorFlg();
	u32	a_u32_mtrErrFlg = U32_DET_MTR_GetErrorFlg();
	a_uni_ABN_canErrorFlg.ulong = uni_ABN_errorFlg.ulong | a_u32_battErrFlg | a_u32_mtrErrFlg;

	return	(u32)a_uni_ABN_canErrorFlg.ulong;
}

/**
 * @brief	警告フラグ集約＆提供
 * @param	none
 * @return	uni_ABN_warningFlg.ulong
 * @note	復帰については別途考える
 */
u32	U32_ABN_GetWarningFlg(void)
{
	UNI_ABN_WARNINGFLG	a_uni_ABN_canWarningFlg;
	u32	a_u32_battWarnFlg = U32_DET_BAT_GetWarningFlg();
	u32	a_u32_mtrWarnFlg = U32_DET_MTR_GetWarningFlg();
	a_uni_ABN_canWarningFlg.ulong = uni_ABN_warningFlg.ulong | a_u32_battWarnFlg | a_u32_mtrWarnFlg;

	return	(u32)a_uni_ABN_canWarningFlg.ulong;
}
