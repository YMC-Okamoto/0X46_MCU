/**
 * @file DETECT_MOTOR.c
 * @brief モータ関連異常検出
 * @note 50ms周期
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "TYPEDEF_MODE.h"
#include "TYPEDEF_ABN.h"
#include "DEFINE_SYSTEM.h"			/*定数定義*/
#include "DEFINE_MOTOR.h"			/*定数定義*/
#include "DEFINE_ABN.h"
#include "DEFINE_SETTING.h"
#include "COMMON_MACRO.h"			/* 汎用関数群 */

/* DDL */
#include "DD_MOTOR.h"
#include "DD_EXCITER.h"

#include "DETECT_ABNORMAL.h"
#include "DETECT_MOTOR.h"
#include "DETECT_BATTERY.h"
#include "ADC.h"
#include "DIAG_SET.h"
#include "MOTOR.h"
#include "BATTERY.h"
#include "PREVENT_CHATTERING.h"		/* チャタリング防止 */

#include "MODE_CONDUCTOR.h"

/* for DEBUG */
#include "ramdef.h"

/* Private define -----------------------------------------------------------*/
/*** 温度異常 ***/
#define	S16_DET_MTR_MOSFET_OVERHEAT_0P1DEGC				(s16)1200		/* FET基板過熱異常検出値		120.0[℃] */
#define	S16_DET_MTR_MOSFET_OVERHEAT_RELEASE_0P1DEGC		(s16)760		/* FET基板過熱異常解除値		76.0[℃] */
#define	S16_DET_MTR_CAPACITOR_OVERHEAT_0P1DEGC			(s16)1000		/* コンデンサ過熱異常検出値		100.0[℃] */
#define	S16_DET_MTR_CAPACITOR_LOWTEMP_0P1DEGC			(s16)-200		/* コンデンサ低温異常検出値		-20.0[℃] */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
#define	S16_DET_MTR_MOTOR_OVERHEAT_0P1DEGC				(s16)1550		/* モータ過熱異常検出値（仮）	155.0[℃] 暫定 */
#define	S16_DET_MTR_MOTOR_OVERHEAT_RELEASE_0P1DEGC		(s16)1350		/* モータ過熱異常解除値（仮）	135.0[℃] 暫定 */
#endif

/* Private variables ---------------------------------------------------------*/
static u8	u8_DET_MTR_MOSFETOverheatCounter = (u8)0;					/* FET過熱異常検出タイマ */
static u8	u8_DET_MTR_capacitorOverheatCounter = (u8)0;				/* コンデンサ過熱異常検出タイマ */
static u8	u8_DET_MTR_MOSFETOverheatReleaseCounter = (u8)0;			/* 過熱異常(FET)解除タイマ */
static u8	u8_DET_MTR_capacitorOverheatReleaseCounter = (u8)0;			/* 過熱異常(コンデンサ)解除タイマ */
static u8	u8_DET_MTR_capacitorLowTempCounter = (u8)0;					/* 低温異常(コンデンサ)検出タイマ */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
static u8	u8_DET_MTR_motorOverheatCounter = (u8)0;					/* 過熱異常(モータ)検出タイマ */
static u8	u8_DET_MTR_motorOverheatReleaseCounter = (u8)0;				/* 過熱異常(モータ)解除タイマ */
#endif

#ifdef	DETECT_OVERREVOLUTION_ACTIVITY
static u8	u8_DET_MTR_overRevolutionCounter = (u8)0;					/* 過回転異常検出タイマ  */
static u8	u8_DET_MTR_overRevolutionReleaseCounter = (u8)0;			/* 過回転異常解除タイマ  */
static u8	u8_DET_MTR_backRevotuiionCounter	 = (u8)0;				/* 逆回転カウンタ */
#endif

static u8	u8_DET_MTR_ctUOffsetAbnormalCounter = (u8)0;				/* U相CTオフセット異常カウンタ */
static u8	u8_DET_MTR_ctVOffsetAbnormalCounter = (u8)0;				/* V相CTオフセット異常カウンタ */

static UNI_ABN_WARNINGFLG	uni_DET_MTR_warningFlg;
static UNI_ABN_ERRORFLG		uni_DET_MTR_errorFlg;

static UNI_DET_MTR_ABNORMALFLG	uni_DET_MTR_abnormalFlg;
static UNI_DET_MTR_ABNORMALFLG2	uni_DET_MTR_abnormalFlg2;
static UNI_DET_MTR_WARNINGFLG	uni_DET_MTR_cautionFlg;

static bool	b_DET_MTR_clearOverCurrent = FALSE;							/* 過電流異常クリア要求フラグ */

static bool	b_DET_MTR_isKeySwOn		= (SW_OFF);							/* キーSW ON状態 */
static bool	b_DET_MTR_isKeySwOnLast	= (SW_OFF);							/* 前回キーSW ON状態 */

static enum{
	EN_KEY_INIT_STATE = 0,
	EN_KEY_ON_STATE,
	EN_KEY_OFF_STATE,
	EN_KEY_OFF_TO_ON_STATE,
	EN_KEY_ON_TO_OFF_STATE,
}en_KEY_state = EN_KEY_INIT_STATE;


/* Private function prototypes -----------------------------------------------*/
static void v_DET_MTR_DetectMtrOverCurrent(void);						/* モータ過電流異常判定 */
static void v_DET_MTR_DetectEncoder(void);								/* エンコーダ異常判定 */
static void v_DET_MTR_DetectExOverCurrent(void);						/* エキサイタ過電流異常判定 */
static void v_DET_MTR_DetectUPhaseWireOpen(void);						/* U相断線異常判定 */
static void v_DET_MTR_DetectVPhaseWireOpen(void);						/* V相断線異常判定 */
static void v_DET_MTR_DetectWPhaseWireOpen(void);						/* W相断線異常判定 */

static void v_DET_MTR_DetectMosFETOverheat(void);						/* FET過熱異常判定 */
static void v_DET_MTR_DetectCapacitorOverheat(void);					/* コンデンサ過熱異常判定 */
static void v_DET_MTR_DetectCapacitorLowTemp(void);						/* 低温異常(コンデンサ部)判定 */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
static void v_DET_MTR_DetectMotorOverheat(void);						/* 過熱異常(モータ部)判定 */
#endif

#ifdef	DETECT_OVERREVOLUTION_ACTIVITY
static void v_DET_MTR_OverRevolution(void);								/* 過回転異常判定 */
#endif
static void	v_DET_MTR_DetectCtOffset(void);								/* モータ電流用CTオフセット異常判定 */
static void v_DET_MTR_DetectCtUOffset(void);							/* U相電流 オフセット異常判定 */
static void v_DET_MTR_DetectctVOffset(void);							/* V相電流 オフセット異常判定 */


/**
 * @brief モータ系異常検出周期処理
 * @param none
 * @return none
 * @detail 各異常・警告を検出する
 * @note 50msec周期
 */
void V_DET_MTR_Loop10ms(void)
{
	EN_MDC_MODE	a_en_mode = En_MDC_GetMode();										/* 動作モード取得 */

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( a_en_mode == MODE_SHUTDOWN_WAIT )
	{																				/* 遮断待機モード⇒異常検出しない */
		return;
	}

	/** KEYステート設定 **/
	b_DET_MTR_isKeySwOnLast = b_DET_MTR_isKeySwOn;									/* 前回状態保存 */
	b_DET_MTR_isKeySwOn = B_CHT_IsKeySwOn();										/* 現在状態取得 */

	if( (b_DET_MTR_isKeySwOnLast == (SW_OFF)) && (b_DET_MTR_isKeySwOn == (SW_ON)) )
	{
		en_KEY_state = EN_KEY_OFF_TO_ON_STATE;
	}
	else if( (b_DET_MTR_isKeySwOnLast == (SW_ON)) && (b_DET_MTR_isKeySwOn == (SW_OFF)) )
	{
		en_KEY_state = EN_KEY_ON_TO_OFF_STATE;
	}
	else if( b_DET_MTR_isKeySwOn == (SW_ON) )
	{
		en_KEY_state = EN_KEY_ON_STATE;
	}
	else if( b_DET_MTR_isKeySwOn == (SW_OFF) )
	{
		en_KEY_state = EN_KEY_OFF_STATE;
	}
	else
	{
			/* MISRA-C 2004 14.10 対応 */
	}

/*===== 遮断待機モード以外 ======*/
/*===== 以下は起動中のみ検出判定を行う ======*/

/*----- モータ関連 -----*/
	v_DET_MTR_DetectEncoder();														/* エンコーダ異常判定 */

	v_DET_MTR_DetectMtrOverCurrent();												/* モータ過電流異常判定 */
	v_DET_MTR_DetectExOverCurrent();												/* エキサイタ過電流異常判定 */

	v_DET_MTR_DetectUPhaseWireOpen();												/* U相断線異常判定 */
	v_DET_MTR_DetectVPhaseWireOpen();												/* V相断線異常判定 */
	v_DET_MTR_DetectWPhaseWireOpen();												/* W相断線異常判定 */

/*----- センサ関連 -----*/
	v_DET_MTR_DetectMosFETOverheat();												/* FET過熱異常判定 */
	v_DET_MTR_DetectCapacitorOverheat();											/* コンデンサ過熱異常判定 */
	v_DET_MTR_DetectCapacitorLowTemp();												/* 低温異常( パワー部)判定 */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
	v_DET_MTR_DetectMotorOverheat();												/* 過熱異常( モータ部)判定 */
#endif

#ifdef	DETECT_OVERREVOLUTION_ACTIVITY
	v_DET_MTR_OverRevolution();														/* 過回転異常判定 */
#endif
	v_DET_MTR_DetectCtOffset();														/* モータ電流用CTオフセット異常判定 */

}				/**/


/**
 * @brief モータ系異常検出変数初期化
 * @param none
 * @return none
 */
void V_DET_MTR_Initialize(void)
{
	u8_DET_MTR_MOSFETOverheatCounter = (u8)0;										/* FET過熱異常検出タイマ初期化 */
	u8_DET_MTR_capacitorOverheatCounter = (u8)0;									/* 過熱異常(パワー部 )検出タイマ初期化 */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
	u8_DET_MTR_motorOverheatCounter = (u8)0;										/* 過熱異常(モータ部 )検出タイマ初期化 */
#endif

#ifdef	DETECT_OVERREVOLUTION_ACTIVITY
	u8_DET_MTR_overRevolutionCounter = (u8)0;										/* 過回転異常検出タイマ  */
	u8_DET_MTR_backRevotuiionCounter = (u8)0;										/* モータ逆転カウンタ初期化 */
#endif

	u8_DET_MTR_ctUOffsetAbnormalCounter = (u8)0;									/* U相CTオフセット異常カウンタ初期化 */
	u8_DET_MTR_ctVOffsetAbnormalCounter = (u8)0;									/* V相CTオフセット異常カウンタ初期化 */

	uni_DET_MTR_abnormalFlg.byte = (u8)0;											/* モータ系異常フラグ初期化 */
	uni_DET_MTR_abnormalFlg2.byte = (u8)0;											/* モータ系異常フラグ初期化 */
	uni_DET_MTR_cautionFlg.byte = (u8)0;											/* モータ系警告フラグ初期化 */

	uni_DET_MTR_warningFlg.ulong = (u32)0;											/* CAN送信用警告フラグ初期化 */
	uni_DET_MTR_errorFlg.ulong = (u32)0;											/* CAN送信用異常フラグ初期化 */
}

/**
 * @brief エンコーダ異常判定
 * @note
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectEncoder(void)
{
	if( B_DD_MTR_IsAbnormanEncoder() == TRUE )										/* エンコーダ異常フラグを確認 */
	{
		if( uni_DET_MTR_abnormalFlg.bit.b_abnormalEncoder == FALSE )
		{
			uni_DET_MTR_abnormalFlg.bit.b_abnormalEncoder = TRUE;					/* エンコーダ異常フラグをSET */
			uni_DET_MTR_errorFlg.bit.b_abnormalEncoder = TRUE;						/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_ENC_ABN) );						/* ダイアグコードの格納 */
		}
	}
}

/**
 * @brief モータ過電流異常判定
 * @note
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectMtrOverCurrent(void)
{
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( (en_KEY_state == EN_KEY_ON_TO_OFF_STATE) || (en_KEY_state == EN_KEY_OFF_STATE) )
	{
		uni_DET_MTR_abnormalFlg.bit.b_abnormalMtrOverCurrent = FALSE;				/* モータ過電流異常フラグをClear */
		uni_DET_MTR_errorFlg.bit.b_abnormalMtrOverCurrent = FALSE;					/* CAN送信用異常フラグ */
		b_DET_MTR_clearOverCurrent = TRUE;
		return;
	}
	/*** KEY SW(MainSW)がONの時はここに来る ***/
	b_DET_MTR_clearOverCurrent = FALSE;												/* ONの時はフラグをクリアしておく */

	if( B_DD_MTR_IsAbnormalOverCurrent() == TRUE )									/* モータ過電流異常フラグを確認 */
	{
		if( uni_DET_MTR_abnormalFlg.bit.b_abnormalMtrOverCurrent == FALSE )
		{
			uni_DET_MTR_abnormalFlg.bit.b_abnormalMtrOverCurrent = TRUE;			/* モータ過電流異常フラグをSET */
			uni_DET_MTR_errorFlg.bit.b_abnormalMtrOverCurrent = TRUE;				/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_MTR_OVER_CUR) );					/* ダイアグコードの格納 */
		}
	}
}

/**
 * @brief エキサイタ過電流異常判定
 * @note
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectExOverCurrent(void)
{
	if( B_DD_EX_IsAbnormalOverCurrent() == TRUE )									/* エキサイタ過電流異常フラグを確認 */
	{
		if( uni_DET_MTR_abnormalFlg2.bit.b_abnormalExOverCurrent == FALSE )
		{
			uni_DET_MTR_abnormalFlg2.bit.b_abnormalExOverCurrent = TRUE;			/* エキサイタ過電流異常フラグをSET */
			uni_DET_MTR_errorFlg.bit.b_abnormalExOverCurrent = TRUE;				/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_EX_OVER_CUR) );					/* ダイアグコードの格納 */
		}
	}
}

/**
 * @brief U相断線異常判定
 * @note
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectUPhaseWireOpen(void)
{
	if( B_DD_MTR_IsAbnormalUPhaseWireOpen() == TRUE )								/* U相断線異常フラグを確認 */
	{
		if( uni_DET_MTR_abnormalFlg2.bit.b_abnormalUPhaseWireOpen == FALSE )
		{
			uni_DET_MTR_abnormalFlg2.bit.b_abnormalUPhaseWireOpen = TRUE;			/* U相断線異常フラグをSET */
			uni_DET_MTR_errorFlg.bit.b_abnormalPhaseWireOpen = TRUE;				/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_U_PHASE_WIRE_OPEN) );			/* ダイアグコードの格納 */
		}
	}
}

/**
 * @brief V相断線異常判定
 * @note
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectVPhaseWireOpen(void)
{
	if( B_DD_MTR_IsAbnormalVPhaseWireOpen() == TRUE )								/* V相断線異常フラグを確認 */
	{
		if( uni_DET_MTR_abnormalFlg2.bit.b_abnormalVPhaseWireOpen == FALSE )
		{
			uni_DET_MTR_abnormalFlg2.bit.b_abnormalVPhaseWireOpen = TRUE;			/* V相断線異常フラグをSET */
			uni_DET_MTR_errorFlg.bit.b_abnormalPhaseWireOpen = TRUE;				/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_V_PHASE_WIRE_OPEN) );			/* ダイアグコードの格納 */
		}
	}
}

/**
 * @brief W相断線異常判定
 * @note
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectWPhaseWireOpen(void)
{
	if( B_DD_MTR_IsAbnormalWPhaseWireOpen() == TRUE )								/* W相断線異常フラグを確認 */
	{
		if( uni_DET_MTR_abnormalFlg2.bit.b_abnormalWPhaseWireOpen == FALSE )
		{	uni_DET_MTR_abnormalFlg2.bit.b_abnormalWPhaseWireOpen = TRUE;			/* W相断線異常フラグをSET */
			uni_DET_MTR_errorFlg.bit.b_abnormalPhaseWireOpen = TRUE;				/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_W_PHASE_WIRE_OPEN) );			/* ダイアグコードの格納 */
		}
	}
}


/**
 * @brief FET基板過熱異常判定
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectMosFETOverheat(void)
{
	s16	a_s16_overheatThreshold = (S16_DET_MTR_MOSFET_OVERHEAT_0P1DEGC);
	s16	a_s16_FETTemp_0p1degC = S16_ADC_GetFetTemperature_0p1degC();
	bool a_b_MosThermistorOpen_fg = B_ABN_IsAbnormalMosThermistorOpen();

	if ( a_b_MosThermistorOpen_fg == TRUE ) 										/* サーミスタ断線時は検出しない*/
	{
		return ;
	} 

	if( uni_DET_MTR_abnormalFlg.bit.b_abnormalMosFetOverheat == FALSE )				/* FET基板過熱異常フラグを確認 */
	{																				/* FET基板過熱異常ではない場合 */
		if( B_ABN_GetResultAbnormal( &u8_DET_MTR_MOSFETOverheatCounter, a_s16_FETTemp_0p1degC,  (U8_ABN_UPPER_LIM_COMPARISON), a_s16_overheatThreshold, (U16_ABN_ABNORMAL_COUNT_250MS) ) == FALSE )	/*異常判断結果を確認*/
		{
																					/* FET基板過熱異常 */
			uni_DET_MTR_abnormalFlg.bit.b_abnormalMosFetOverheat = TRUE;			/* FET基板過熱異常フラグをSET */
			uni_DET_MTR_errorFlg.bit.b_abnormalMosOverheat = TRUE;					/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_OVERHEAT_INV) );					/* ダイアグコードの格納 */
		}
	}
	else
	{
		if( B_ABN_GetResultAbnormal( &u8_DET_MTR_MOSFETOverheatReleaseCounter, a_s16_FETTemp_0p1degC,  (U8_ABN_LOWER_LIM_COMPARISON), S16_DET_MTR_MOSFET_OVERHEAT_RELEASE_0P1DEGC, (U16_ABN_ABNORMAL_COUNT_250MS) ) == FALSE )	/*異常判断結果を確認*/
		{																			/* FET基板過熱異常 */
			uni_DET_MTR_abnormalFlg.bit.b_abnormalMosFetOverheat = FALSE;			/* FET基板過熱異常フラグをクリア */
			uni_DET_MTR_errorFlg.bit.b_abnormalMosOverheat = FALSE;					/* CAN送信用異常フラグ */
		}
	}
}

/**
 * @brief コンデンサ過熱異常判定
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectCapacitorOverheat(void)
{
	s16	a_s16_overheatThreshold = (S16_DET_MTR_CAPACITOR_OVERHEAT_0P1DEGC);
	s16	a_s16_capTemp_0p1degC = S16_ADC_GetCapTemperature_0p1degC();
	bool a_b_CapThermistorOpen_fg = B_ABN_IsAbnormalCapThermistorOpen();

	if ( a_b_CapThermistorOpen_fg == TRUE )											/* サーミスタ断線時は検出しない*/
	{
		return ;
	} 

	if( uni_DET_MTR_abnormalFlg.bit.b_abnormalCapacitorOverheat == FALSE )			/* コンデンサ基板過熱異常フラグを確認 */
	{																				/* コンデンサ基板過熱異常ではない場合 */
		if( B_ABN_GetResultAbnormal( &u8_DET_MTR_capacitorOverheatCounter, a_s16_capTemp_0p1degC,  (U8_ABN_UPPER_LIM_COMPARISON), a_s16_overheatThreshold, (U16_ABN_ABNORMAL_COUNT_250MS) ) == FALSE )	/*異常判断結果を確認*/
		{
																					/* コンデンサ基板過熱異常 */
			uni_DET_MTR_abnormalFlg.bit.b_abnormalCapacitorOverheat = TRUE;			/* コンデンサ基板過熱異常フラグをSET */
			uni_DET_MTR_errorFlg.bit.b_abnormalCapOverheat = TRUE;					/* CAN送信用異常フラグ */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_OVERHEAT_CAP) );					/* ダイアグコードの格納 */
		}
	}
	else
	{
		if( B_ABN_GetResultAbnormal( &u8_DET_MTR_capacitorOverheatReleaseCounter, a_s16_capTemp_0p1degC,  (U8_ABN_LOWER_LIM_COMPARISON), S16_DET_MTR_MOSFET_OVERHEAT_RELEASE_0P1DEGC, (U16_ABN_ABNORMAL_COUNT_250MS) ) == FALSE )	/*異常判断結果を確認*/
		{																			/* コンデンサ基板過熱異常 */
			uni_DET_MTR_abnormalFlg.bit.b_abnormalCapacitorOverheat = FALSE;			/* コンデンサ基板過熱異常フラグをクリア */
			uni_DET_MTR_errorFlg.bit.b_abnormalCapOverheat = FALSE;						/* CAN送信用異常フラグ */
		}
	}
}

/**
 * @brief 低温異常(コンデンサ基板)判定
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectCapacitorLowTemp(void)
{
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/

	if( en_KEY_state == EN_KEY_ON_TO_OFF_STATE )
	{
		uni_DET_MTR_abnormalFlg.bit.b_abnormalCapacitorLowTemp = FALSE;				/* コンデンサ基板低温異常フラグをClear */
		uni_DET_MTR_errorFlg.bit.b_abnormalCapLowTemp = FALSE;						/* CAN送信用異常フラグ */
		u8_DET_MTR_capacitorLowTempCounter = (u8)0;									/* コンデンサ基板低温異常検出タイマClear */
		return;
	}

	s16	a_s16_capTemp_0p1degC = S16_ADC_GetCapTemperature_0p1degC();

	if( uni_DET_MTR_abnormalFlg.bit.b_abnormalCapacitorLowTemp == FALSE )			/* コンデンサ基板低温異常フラグを確認 */
	{																				/* コンデンサ基板低温異常ではない場合 */
		if( B_ABN_GetResultAbnormal( &u8_DET_MTR_capacitorLowTempCounter, a_s16_capTemp_0p1degC,  (U8_ABN_LOWER_LIM_COMPARISON), S16_DET_MTR_CAPACITOR_LOWTEMP_0P1DEGC, (U16_ABN_ABNORMAL_COUNT_250MS) ) == FALSE )	/*異常判断結果を確認*/
		{
			if( en_KEY_state == EN_KEY_OFF_STATE )
			{
					/* KEY OFFの時は異常としない。但し、判定は実施しカウンタは動作させる */
			}
			else
			{
																					/* コンデンサ基板低温異常 */
				uni_DET_MTR_abnormalFlg.bit.b_abnormalCapacitorLowTemp = TRUE;		/* コンデンサ基板低温異常フラグをSET */
				uni_DET_MTR_errorFlg.bit.b_abnormalCapLowTemp = TRUE;				/* CAN送信用異常フラグ */
				V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_LOW_TEMP_CAP) );				/* ダイアグコードの格納 */
			}
		}
	}
}

#ifdef	MOTOR_THERMISTOR_ACTIVITY
/**
 * @brief 過熱異常(モータ部)判定
 * @param none
 * @return none
 */
static void v_DET_MTR_DetectMotorOverheat(void)
{
	s16	a_s16_motorTemp_0p1degC = S16_ADC_GetMotorTemperature_0p1degC();

	if( uni_DET_MTR_abnormalFlg.bit.b_abnormalMotorOverheat == FALSE )				/* モータ部過熱異常フラグを確認 */
	{																				/* モータ部過熱異常ではない場合 */
		if( B_ABN_GetResultAbnormal( &u8_DET_MTR_motorOverheatCounter, a_s16_motorTemp_0p1degC, (U8_ABN_UPPER_LIM_COMPARISON), (S16_DET_MTR_MOTOR_OVERHEAT_0P1DEGC), (U16_ABN_ABNORMAL_COUNT_250MS) ) == FALSE )	/*異常判断結果を確認*/
		{																			/* モータ部部過熱異常 */
			uni_DET_MTR_abnormalFlg.bit.b_abnormalMotorOverheat = TRUE;				/* 過熱異常フラグをSET */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_OVERHEAT_MTR) );					/* ダイアグコードの格納 */
		}
	}
	else
	{
		if( B_ABN_GetResultAbnormal( &u8_DET_MTR_motorOverheatReleaseCounter, a_s16_motorTemp_0p1degC, (U8_ABN_LOWER_LIM_COMPARISON), (S16_DET_MTR_MOTOR_OVERHEAT_RELEASE_0P1DEGC), (U16_ABN_ABNORMAL_COUNT_250MS) ) == FALSE )	/*異常判断結果を確認*/
		{																			/* モータ部部過熱異常 */
			uni_DET_MTR_abnormalFlg.bit.b_abnormalMotorOverheat = FALSE;				/* 過熱異常フラグをSET */
		}
	}
}
#endif

#ifdef	DETECT_OVERREVOLUTION_ACTIVITY
/**
 * @brief 過回転異常判定
 * @param none
 * @return none
 */
static void v_DET_MTR_OverRevolution(void)
{
	s16 a_s16_motorRevloution_rpm = S16_DD_MTR_GetRevolution_rpm();

	if( uni_DET_MTR_abnormalFlg2.bit.b_abnormalOverRevolution == FALSE )			/* 過回転異常フラグの確認 */
	{																				/* 過回転異常ではない場合 */
		if( B_ABN_GetResultAbnormal( &u8_DET_MTR_overRevolutionCounter, a_s16_motorRevloution_rpm, (U8_ABN_UPPER_LIM_COMPARISON), (U16_MTR_OVER_REVOLUTION_RPM), (U16_ABN_ABNORMAL_COUNT_250MS) ) == FALSE )	/* 異常判断 */
		{																			/* 過回転異常発生 */
			uni_DET_MTR_abnormalFlg2.bit.b_abnormalOverRevolution = TRUE;			/* 過回転異常フラグをSET */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_OVER_REVOLUTION) );				/* ダイアグコードの格納 */
		}
		/* 逆転時 */
		if(a_s16_motorRevloution_rpm <  (S16_MTR_OVER_BACK_REVOLUTION_RPM))
		{
			u8_DET_MTR_backRevotuiionCounter++;										/* カウンタをインクリメント */
			if( u8_DET_MTR_backRevotuiionCounter >= (U16_ABN_ABNORMAL_COUNT_250MS))	/* 閾値以上 */
			{
				u8_DET_MTR_backRevotuiionCounter = (U16_ABN_ABNORMAL_COUNT_250MS);
				uni_DET_MTR_abnormalFlg2.bit.b_abnormalOverRevolution = TRUE;		/* 過回転異常フラグをSET */
				V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_OVER_REVOLUTION) );			/* ダイアグコードの格納 */	
			}
		}
		else
		{
			u8_DET_MTR_backRevotuiionCounter = (u8)0;								/* カウンタクリア */
		}
	}
	else
	{
		if ( ( a_s16_motorRevloution_rpm < ( U16_MTR_OVER_REVOLUTION_RELEASE_RPM ) )
		  && ( a_s16_motorRevloution_rpm <  (S16_MTR_OVER_BACK_REVOLUTION_RELEASE_RPM) ) )	/* 復帰判断 */
		{																			
			u8_DET_MTR_overRevolutionReleaseCounter++;										/* カウンタをインクリメント */
			if( u8_DET_MTR_overRevolutionReleaseCounter >= (U16_ABN_ABNORMAL_COUNT_250MS))	/* 閾値以上 */
			{
				u8_DET_MTR_overRevolutionReleaseCounter = (U16_ABN_ABNORMAL_COUNT_250MS);
				uni_DET_MTR_abnormalFlg2.bit.b_abnormalOverRevolution = FALSE;			/* 過回転異常フラグをクリア */
			}
		}
		else
		{
			u8_DET_MTR_overRevolutionReleaseCounter = (u8)0;
		}
	}
}
#endif

/**
 * @brief モータ電流用CTオフセット異常判定
 * @param none
 * @return none
 */
static void	v_DET_MTR_DetectCtOffset(void)		/* モータ電流用CTオフセット異常判定 */
{
	s16 a_s16_revolution_rpm = S16_DD_MTR_GetRevolution_rpm();
	bool	a_b_isSomethingAbnormal = B_ABN_IsSomethingAbnormal();
	bool	a_b_isGateOn = B_DD_MTR_IsGateOn();

	/* この時点での判定がDD_MTRクラスで変更されたとしてもCtのAD値は判定時から変化無い為、問題なし	*/
	/* （50msec処理中にAD変換結果平均化の1msec処理の割込みは掛からない為)							*/
	if(	(a_b_isSomethingAbnormal == FALSE) &&										/* 異常状態を確認 */
		(a_b_isGateOn == FALSE) &&													/* 現在ゲートオフ状態 */
		(a_s16_revolution_rpm == (s16)0) )											/* モータ回転数0rpm */
	{
		v_DET_MTR_DetectCtUOffset();												/* U相電流用CTオフセット異常判定 */
		v_DET_MTR_DetectctVOffset();												/* V相電流用CTオフセット異常判定 */
	}
}

/**
 * @brief U相電流用CTオフセット異常判定
 * @param none
 * @return none
 */
static void	v_DET_MTR_DetectCtUOffset(void)
{
	u16		a_u16_adCtUOffset = U16_ADC_GetAdValueAveCtUPhase();
	bool	a_b_isGateOn = B_DD_MTR_IsGateOn();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( en_KEY_state == EN_KEY_ON_TO_OFF_STATE )
	{
		uni_DET_MTR_abnormalFlg.bit.b_abnormalCtUOffset = FALSE;					/* U相CTオフセット異常フラグをClear */
		uni_DET_MTR_errorFlg.bit.b_abnormalCtOffset = FALSE;						/* CAN送信用異常フラグ */
		V_DD_EX_CanselForcedGateOff();												/* エキサイタ強制ゲートオフを解除 */
		u8_DET_MTR_ctUOffsetAbnormalCounter = (u8)0;								/* オフセット異常検出タイマClear */
		return;
	}

	if( uni_DET_MTR_abnormalFlg.bit.b_abnormalCtUOffset == TRUE )					/* U相CTオフセット異常フラグを確認 */
	{
		return;
	}

	if( a_b_isGateOn == FALSE ) 													/* ゲートオフ状態フラグを確認(電流制御周期で変化する可能性がある為直前で取得する) */
	{																				/* ゲートオフしている */
		if( ( a_u16_adCtUOffset > (U16_DET_MTR_CT_OFFSET_UPPER) ) ||				/* U相AD値は上限値より大きい or */
			( a_u16_adCtUOffset < (U16_DET_MTR_CT_OFFSET_LOWER) ) )					/* U相AD値は下限値より小さい */
		{																			/* U相CTオフセット異常 */
			u8_DET_MTR_ctUOffsetAbnormalCounter++;									/* オフセット異常検出タイマ+1 */
			/* 最大値でクリップ */
			u8_DET_MTR_ctUOffsetAbnormalCounter = (u8)CMF_CLIP_MAX( u8_DET_MTR_ctUOffsetAbnormalCounter, (u8)(U16_ABN_ABNORMAL_COUNT_2S) );
			if( u8_DET_MTR_ctUOffsetAbnormalCounter >= (u8)(U16_ABN_ABNORMAL_COUNT_2S) )	/* 異常タイマの確認 */
			{																		/* 閾値を超えた場合 */
				if( en_KEY_state == EN_KEY_OFF_STATE )
				{
					/* KEY OFFの時は異常としない。但し、判定は実施しカウンタは動作させる */
				}
				else
				{
					uni_DET_MTR_abnormalFlg.bit.b_abnormalCtUOffset = TRUE;			/* U相CTオフセット異常フラグをSET */
					uni_DET_MTR_errorFlg.bit.b_abnormalCtOffset = TRUE;				/* CAN送信用異常フラグ */
					V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_CT_UOFFSET) );			/* ダイアグコードの格納 */
				}
			}
		}
		else
		{																			/* U相オフセット値は正常範囲以内の場合 */
			u8_DET_MTR_ctUOffsetAbnormalCounter = (u8)0;							/* オフセット異常検出タイマリセット */
		}
	}
	else
	{																				/* ゲートオフではな い*/
		u8_DET_MTR_ctUOffsetAbnormalCounter = (u8)0;								/* オフセット異常検出タイマリセット */
	}
}

/**
 * @brief V相電流用CTオフセット異常判定
 * @param none
 * @return none
 */
static void	v_DET_MTR_DetectctVOffset(void)
{
	u16		a_u16_adctVOffset = U16_ADC_GetAdValueAveCtVPhase();
	bool	a_b_isGateOn = B_DD_MTR_IsGateOn();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( en_KEY_state == EN_KEY_ON_TO_OFF_STATE )
	{
		uni_DET_MTR_abnormalFlg.bit.b_abnormalCtVOffset = FALSE;					/* V相CTオフセット異常フラグをClear */
		uni_DET_MTR_errorFlg.bit.b_abnormalCtOffset = FALSE;						/* CAN送信用異常フラグ */
		V_DD_EX_CanselForcedGateOff();												/* エキサイタ強制ゲートオフを解除 */
		u8_DET_MTR_ctVOffsetAbnormalCounter = (u8)0;								/* オフセット異常検出タイマClear */
		return;
	}

	if( uni_DET_MTR_abnormalFlg.bit.b_abnormalCtVOffset == TRUE )					/* V相CTオフセット異常フラグを確認 */
	{
		return;
	}

	if( a_b_isGateOn == FALSE ) 													/* ゲートオフ状態フラグを確認(電流制御周期で変化する可能性がある為直前で取得する) */
	{																				/* ゲートオフしている*/
		if( ( a_u16_adctVOffset > (U16_DET_MTR_CT_OFFSET_UPPER) ) ||				/* V相AD値は上限値より大きい or */
			( a_u16_adctVOffset < (U16_DET_MTR_CT_OFFSET_LOWER) ) )					/* V相AD値は下限値より小さい */
		{																			/* V相CTオフセット異常 */
			u8_DET_MTR_ctVOffsetAbnormalCounter++;									/* オフセット異常検出タイマ+1 */
			/* 最大値でクリップ */
			u8_DET_MTR_ctVOffsetAbnormalCounter = (u8)CMF_CLIP_MAX( u8_DET_MTR_ctVOffsetAbnormalCounter, (u8)(U16_ABN_ABNORMAL_COUNT_2S) );
			if( u8_DET_MTR_ctVOffsetAbnormalCounter >= (u8)(U16_ABN_ABNORMAL_COUNT_2S) )	/* 異常タイマの確認 */
			{																		/* 閾値を超えた場合 */
				if( en_KEY_state == EN_KEY_OFF_STATE )
				{
					/* KEY OFFの時は異常としない。但し、判定は実施しカウンタは動作させる */
				}
				else
				{
					uni_DET_MTR_abnormalFlg.bit.b_abnormalCtVOffset = TRUE;			/* V相CTオフセット異常フラグをSET */
					uni_DET_MTR_errorFlg.bit.b_abnormalCtOffset = TRUE;				/* CAN送信用異常フラグ */
					V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_CT_VOFFSET) );			/* ダイアグコードの格納 */
				}
			}
		}
		else
		{																			/* V相オフセット値は正常範囲以内の場合 */
			u8_DET_MTR_ctVOffsetAbnormalCounter = (u8)0;							/* オフセット異常検出タイマリセット */
		}
	}
	else
	{																				/* ゲートオフではない */
		u8_DET_MTR_ctVOffsetAbnormalCounter = (u8)0;								/* オフセット異常検出タイマリセット */
	}
}

/**
 * @brief エンコーダ異常状態取得
 * @param none
 * @return uni_DET_MTR_abnormalFlg.bit.b_abnormalEncoder
 */
bool	B_DET_MTR_IsAbnormanEncoder(void)
{
	return uni_DET_MTR_abnormalFlg.bit.b_abnormalEncoder;
}

/**
 * @brief エンコーダ異常セット
 * @param none
 * @return none
 */
void	V_DET_MTR_SetAbnormalEncoder(void)
{
	uni_DET_MTR_abnormalFlg.bit.b_abnormalEncoder = TRUE;							/* エンコーダ異常セット */
}

/**
 * @brief 過電流異常状態提供
 * @param none
 * @return uni_DET_MTR_abnormalFlg.bit.b_abnormalMtrOverCurrent
 */
bool	B_DET_MTR_IsAbnormalOverCurrent(void)
{
	return uni_DET_MTR_abnormalFlg.bit.b_abnormalMtrOverCurrent;
}

/**
 * @brief 過電流異常セット
 * @param none
 * @return uni_DET_MTR_abnormalFlg.bit.b_abnormalMtrOverCurrent
 */
void	V_DET_MTR_SetAbnormalOverCurrent(void)
{
	uni_DET_MTR_abnormalFlg.bit.b_abnormalMtrOverCurrent = TRUE;
}

/**
 * @brief U相モータ断線異常セット
 * @param none
 * @return uni_DET_MTR_abnormalFlg2.bit.b_abnormalUPhaseWireOpen
 */
void	V_DET_MTR_SetAbnormalUPhaseWireOpen(void)
{
	uni_DET_MTR_abnormalFlg2.bit.b_abnormalUPhaseWireOpen = TRUE;
}

/**
 * @brief V相モータ断線異常セット
 * @param none
 * @return uni_DET_MTR_abnormalFlg2.bit.b_abnormalVPhaseWireOpen
 */
void	V_DET_MTR_SetAbnormalVPhaseWireOpen(void)
{
	uni_DET_MTR_abnormalFlg2.bit.b_abnormalVPhaseWireOpen = TRUE;
}

/**
 * @brief W相モータ断線異常セット
 * @param none
 * @return uni_DET_MTR_abnormalFlg2.bit.b_abnormalWPhaseWireOpen
 */
void	V_DET_MTR_SetAbnormalWPhaseWireOpen(void)
{
	uni_DET_MTR_abnormalFlg2.bit.b_abnormalWPhaseWireOpen = TRUE;
}

/**
 * @brief エキサイタ過電流異常セット
 * @param none
 * @return uni_DET_MTR_abnormalFlg.bit.b_abnormalMtrOverCurrent
 */
void	V_DET_MTR_SetAbnormalExOverCurrent(void)
{
	uni_DET_MTR_abnormalFlg2.bit.b_abnormalExOverCurrent = TRUE;
}

/**
 * @brief (DET_MTRクラス)異常フラグクリア
 * @param none
 * @return none
 */
void	V_DET_MTR_ClearAbnormalFlg(void)
{
	uni_DET_MTR_abnormalFlg.byte = (u8)0;
}

/**
 * @brief (DET_MTRクラス)異常フラグ2クリア
 * @param none
 * @return none
 */
void	V_DET_MTR_ClearAbnormalFlg2(void)
{
	uni_DET_MTR_abnormalFlg2.byte = (u8)0;
}

/**
 * @brief (DET_MTRクラス)警告フラグクリア
 * @param none
 * @return none
 */
void	V_DET_MTR_ClearWarningFlg(void)
{
	uni_DET_MTR_cautionFlg.byte = (u8)0;
}

/**
 * @brief (DET_MTRクラス)異常フラグ提供
 * @param none
 * @return uni_DET_MTR_abnormalFlg.byte
 */
u8	U8_DET_MTR_IsAbnormalFlg(void)
{
	return	(u8)uni_DET_MTR_abnormalFlg.byte;
}

/**
 * @brief (DET_MTRクラス)異常フラグ2提供
 * @param none
 * @return uni_DET_MTR_abnormalFlg.byte
 */
u8	U8_DET_MTR_IsAbnormalFlg2(void)
{
	return	(u8)uni_DET_MTR_abnormalFlg2.byte;
}

/**
 * @brief (DET_MTRクラス)異常フラグ提供
 * @param none
 * @return uni_DET_MTR_errorFlg.ulong
 */
u32	U32_DET_MTR_GetErrorFlg(void)
{
	return	(u32)uni_DET_MTR_errorFlg.ulong;
}

/**
 * @brief (DET_MTRクラス)警告フラグ提供
 * @param none
 * @return uni_DET_MTR_warningFlg.ulong
 */
u32	U32_DET_MTR_GetWarningFlg(void)
{
	return	(u32)uni_DET_MTR_warningFlg.ulong;
}
/**
 * @brief 過電流異常クリア要求
 * @param none
 * @return none
 */
bool	B_DET_MTR_ClearOverCurrent(void)
{
	return	b_DET_MTR_clearOverCurrent;
}
