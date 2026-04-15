/**
 * @brief 界磁制御処理
 * @detail 相電流取込、電気角推定、ベクトル演算、タイマ指令値算出、電圧監視、ロータ回転数算出
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "Config_MTU6_MTU7.h"
#include "Config_S12AD1.h"
#include "Config_DA.h"

#include "MCU.h"
#include "DEFINE_SETTING.h"
#include "DEFINE_SYSTEM.h"
#include "DEFINE_MOTOR.h"
#include "DEFINE_ABN.h"
#include "COMMON_FUNCTION.h"			/* 汎用関数群 */
#include "COMMON_MACRO.h"

/* DDL */
#include "DD_EXCITER.h"
#include "DD_MOTOR.h"

#include "DETECT_ABNORMAL.h"
#include "DETECT_MOTOR.h"
#include "DETECT_BATTERY.h"
#include "MOTOR.h"
#include "ADC.h"
#include "DIAG_SET.h"
#include "MODE_CONDUCTOR.h"
#include "BATTERY.h"
#include "PREVENT_CHATTERING.h"		/* チャタリング防止 */

/* for DEBUG */
#include "ramdef.h"

/* Private define ------------------------------------------------------------*/
#define	U8_DD_EX_OVERCURRENT_COUNT				(u8)20									/* 過電流異常検出連続時間 */

#define	U8_EX_AD1_MAX_WAITING_COUNT				(u8)120									/* AD1ループAD変換待機カウンタ最大数 */

#define	S16_DD_EX_OVER_CURRENT_THRESHOLD_10MA	(s16)2000								/* エキサイタ電流過電流異常検出値 20A */

#define	S16_DD_EX_IFREF_DISCHARGE_10MA			(s16)1000								/* ディスチャージ用If指令値 */
#define	U16_EX_DISCHARGE_PWM_TIMER_MIN			(u16)100								/* ディスチャージ用タイマ制限値(Duty制限) */

/* Private variables ---------------------------------------------------------*/
#ifdef	DEBUG_WATCH_ACTIVITY

bool	b_DD_EX_isStartAD1Convert = FALSE;											/* AD1変換開始フラグ */
bool	b_DD_EX_isDelayAD1Convert = FALSE;											/* AD変換遅延状態フラグ */
u8	u8_DD_EX_ad1FinishedCounterMax = (u8)0;										/* AD1チャンネルAD変換終了カウンタ最大値 */

u16	u16_DD_EX_proportionalGain = (U16_EX_KP);									/* 比例ゲイン */
u16	u16_DD_EX_integralGain = (U16_EX_KI);										/* 積分ゲイン */
//u16	u16_DD_EX_pGainTemp = (U16_EX_KP);											/* 比例ゲイン */
//u16	u16_DD_EX_iGainTemp = (U16_EX_KI);											/* 積分ゲイン */
bool	b_DD_EX_isStartRequested = FALSE;											/* モータ駆動要求 */
bool	b_DD_EX_isPermitted = TRUE;													/* ゲートON許可 */
bool	b_DD_EX_isGateOn = FALSE;													/* ゲートON状態 */
bool	b_DD_EX_forcedGateOff = FALSE;												/* 強制ゲートOFF要求 */
bool	b_DD_EX_emergencyStop = FALSE;												/* 緊急停止要求 */
s16	s16_DD_EX_ifRef_10mA = (s16)0;												/* If指令値[10mA] */
s16	s16_DD_EX_ifRefFinal_10mA = (s16)0;											/* If指令値(制限後)[10mA] */
s16	s16_DD_EX_ifRefAtDischarge_10mA = (S16_DD_EX_IFREF_DISCHARGE_10MA);			/* ディスチャージ用If指令値[10mA] */
s32	s32_DD_EX_integralVf_10mV = (s32)0;											/* 積分項上位16bit */

s16	s16_DD_EX_vfMax_10mV = (U16_ADC_REFERENCE_BATT_VOLT_10MV);					/* 界磁電圧Vf最大値 物理量[10mV] */
s16	s16_DD_EX_vfTemp = (s16)0;													/* 界磁電圧Vf タイマ仮値 */
s16	s16_DD_EX_vf = (s16)0;														/* 界磁電圧Vf タイマ値 */
s16	s16_DD_EX_if_10mA = (s16)0;													/* 界磁電流 */
s16	s16_DD_EX_vf_10mV = (s16)0;													/* 界磁電圧 */
s16	s16_DD_EX_eCommand = (s16)0;												/* 上アーム用指令値 */
s16	s16_DD_EX_eCommandTemp = (s16)0;											/* 上アーム用指令値(仮) */

u16	u16_DD_EX_pwmTimerMax = (U16_EX_PWM_TIMER_MAX);								/* 界磁用PWMタイマ最大値(タイマ値レベルでは最小値) */
u16	u16_DD_EX_pwmTimerMin = (U16_EX_PWM_TIMER_MIN);								/* 界磁用PWMタイマ最小値(タイマ値レベルでは最大値) */

u8	u8_DD_EX_overCurrentJudgeCount	= (u8)0;									/* エキサイタ過電流異常検出タイマ */
bool	b_DD_EX_abnormalOverCurrent = FALSE;										/* エキサイタ過電流異常フラグ */

ST_ADC_ADVALUE1	st_DD_EX_adValue1 = {(u16)0,(u16)0,(u16)0,(u16)0};				/* AD1データ */
ST_ADC_ADVALUE1*	pst_DD_EX_adValue1 = &st_DD_EX_adValue1;						/* AD1データの参照先 */

EN_MDC_DISCHARGE_MODE	en_DD_EX_dischargeMode	= MD_DISCHARGE_INITIAL;			/* 現在ディスチャージモード */
bool	b_DD_EX_isDischargeRun = FALSE;												/* ディスチャージ実施 */
bool	b_DD_EX_isDischargePreparingFinish = FALSE;									/* ディスチャージ終了準備 */

/* for DEBUG 後で消す */
bool	b_DD_EX_isNotGateOffForEvalution = FALSE;									/* 界磁OFFしない：評価用 */

#else

static bool	b_DD_EX_isStartAD1Convert = FALSE;											/* AD1変換開始フラグ */
static bool	b_DD_EX_isDelayAD1Convert = FALSE;											/* AD変換遅延状態フラグ */
static u8	u8_DD_EX_ad1FinishedCounterMax = (u8)0;										/* AD1チャンネルAD変換終了カウンタ最大値 */

static u16	u16_DD_EX_proportionalGain = (U16_EX_KP);									/* 比例ゲイン */
static u16	u16_DD_EX_integralGain = (U16_EX_KI);										/* 積分ゲイン */
//static u16	u16_DD_EX_pGainTemp = (U16_EX_KP);											/* 比例ゲイン */
//static u16	u16_DD_EX_iGainTemp = (U16_EX_KI);											/* 積分ゲイン */
static bool	b_DD_EX_isStartRequested = FALSE;											/* モータ駆動要求 */
static bool	b_DD_EX_isPermitted = TRUE;													/* ゲートON許可 */
static bool	b_DD_EX_isGateOn = FALSE;													/* ゲートON状態 */
static bool	b_DD_EX_forcedGateOff = FALSE;												/* 強制ゲートOFF要求 */
static bool	b_DD_EX_emergencyStop = FALSE;												/* 緊急停止要求 */
static s16	s16_DD_EX_ifRef_10mA = (s16)0;												/* If指令値[10mA] */
static s16	s16_DD_EX_ifRefFinal_10mA = (s16)0;											/* If指令値(制限後)[10mA] */
static s16	s16_DD_EX_ifRefAtDischarge_10mA = (S16_DD_EX_IFREF_DISCHARGE_10MA);			/* ディスチャージ用If指令値[10mA] */
static s32	s32_DD_EX_integralVf_10mV = (s32)0;											/* 積分項上位16bit */

static s16	s16_DD_EX_vfMax_10mV = (U16_ADC_REFERENCE_BATT_VOLT_10MV);					/* 界磁電圧Vf最大値 物理量[10mV] */
static s16	s16_DD_EX_vfTemp = (s16)0;													/* 界磁電圧Vf タイマ仮値 */
static s16	s16_DD_EX_vf = (s16)0;														/* 界磁電圧Vf タイマ値 */
static s16	s16_DD_EX_if_10mA = (s16)0;													/* 界磁電流 */
static s16	s16_DD_EX_vf_10mV = (s16)0;													/* 界磁電圧 */
static s16	s16_DD_EX_eCommand = (s16)0;												/* 上アーム用指令値 */
static s16	s16_DD_EX_eCommandTemp = (s16)0;											/* 上アーム用指令値(仮) */

static u16	u16_DD_EX_pwmTimerMax = (U16_EX_PWM_TIMER_MAX);								/* 界磁用PWMタイマ最大値(タイマ値レベルでは最小値) */
static u16	u16_DD_EX_pwmTimerMin = (U16_EX_PWM_TIMER_MIN);								/* 界磁用PWMタイマ最小値(タイマ値レベルでは最大値) */

static u8	u8_DD_EX_overCurrentJudgeCount	= (u8)0;									/* エキサイタ過電流異常検出タイマ */
static bool	b_DD_EX_abnormalOverCurrent = FALSE;										/* エキサイタ過電流異常フラグ */

static ST_ADC_ADVALUE1	st_DD_EX_adValue1 = {(u16)0,(u16)0,(u16)0,(u16)0};				/* AD1データ */
static ST_ADC_ADVALUE1*	pst_DD_EX_adValue1 = &st_DD_EX_adValue1;						/* AD1データの参照先 */

static EN_MDC_DISCHARGE_MODE	en_DD_EX_dischargeMode	= MD_DISCHARGE_INITIAL;			/* 現在ディスチャージモード */
static bool	b_DD_EX_isDischargeRun = FALSE;												/* ディスチャージ実施 */
static bool	b_DD_EX_isDischargePreparingFinish = FALSE;									/* ディスチャージ終了準備 */

static bool	b_DD_EX_isNotGateOffForEvalution = FALSE;									/* 界磁OFFするかしないか：Defaultは界磁OFFする */

#endif
/* Private function prototypes -----------------------------------------------*/
static void v_DD_EX_DetectOverCurrent(void);											/* エキサイタ過電流異常判定 */
static void v_DD_EX_CurrentPIControl(void);												/* PI演算 */
static void	v_DD_EX_ClipVoltageCommand(void);											/* UVW指令値制限 */


/**
 * @brief 電流制御 & AD取り込み処理
 * @detail モータの電流制御 & AD取り込みを行う(谷割込)
 * @note 電気角推定、AD取り込み、モータ電流単位変換、過電流判定、モータ単相断線判定
 * @param none
 * @return none
 */
void	INTRPT_DD_EX_FieldControlValley(void)
{
	/* for DEBUG */
	PO_GPIO3_HI();

#pragma unpack

/*-----AD変換結果取り込み処理-----*/
	if( (S12AD1.ADCSR.BIT.ADST) == TRUE )
	{
		b_DD_EX_isStartAD1Convert = TRUE;
	}

/*-----AD1変換結果取り込み処理-----*/
	u8	a_u8_adCounter = (u8)0;
	while ( (S12AD1.ADCSR.BIT.ADST) == TRUE )
	{																									/* AD1ループスキャン完了してない */
		if( a_u8_adCounter < (U8_EX_AD1_MAX_WAITING_COUNT) )
		{
			a_u8_adCounter++;																			/* 待ち */
			/* for DEBUG */
			u8_DD_EX_ad1FinishedCounterMax = (u8)CMF_GETMAX(a_u8_adCounter,u8_DD_EX_ad1FinishedCounterMax);
		}
		else
		{																								/* 最大待ち時間が超えると */
			b_DD_EX_isDelayAD1Convert = TRUE;															/* AD変換遅延状態フラグON */
			break;
		}
	}

	if( (b_DD_EX_isDelayAD1Convert == FALSE) && (b_DD_EX_isStartAD1Convert == TRUE) )
	{
		/*----AD1取り込み----*/
		R_Config_S12AD1_Get_ValueResult(ADCHANNEL0,&st_DD_EX_adValue1.u16_adValueVu);					/* Ｕ相電圧ＡＤ(生) */
		R_Config_S12AD1_Get_ValueResult(ADCHANNEL1,&st_DD_EX_adValue1.u16_adValueVv);					/* Ｖ相電圧ＡＤ(生) */
		R_Config_S12AD1_Get_ValueResult(ADCHANNEL2,&st_DD_EX_adValue1.u16_adValueVw);					/* Ｗ相電圧ＡＤ(生) */
		R_Config_S12AD1_Get_ValueResult(ADCHANNEL3,&st_DD_EX_adValue1.u16_adValueIf);					/* 界磁電流ＡＤ(生) */
		b_DD_EX_isStartAD1Convert = FALSE;
	}

/*----界磁電流取り込み＆単位変換----*/
	/* オフセット取得 */
	ST_ADC_ADVALUE_AVE	a_st_adValueAve	=
	{(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_EX_IF_OFFSET_STANDARD_ADVALUE)};	/* AD値平均化データ */

	a_st_adValueAve = St_MTR_GetAdValueAve();															/* 電流AD値平均データ取得 */

	/* オフセット減算 */
	s16	a_s16_if = (s16)((s16)st_DD_EX_adValue1.u16_adValueIf - (s16)a_st_adValueAve.u16_adValueAveIf);
	/* 電流(0.1A)への単位変換 */
	s16_DD_EX_if_10mA = (s16)(((s32)a_s16_if * (s32)(S16_EX_IF_RANGE_MAX_10MA)) >> (s32)11);

	bool a_b_isDetectActivity = B_MDC_IsDetectActivity();

	if( a_b_isDetectActivity == TRUE )
	{
/*----エキサイタ過電流異常の判断----*/
		v_DD_EX_DetectOverCurrent();
	}
	/* for DEBUG */
	PO_GPIO3_LO();
}

/**
 * @brief エキサイタ過電流異常判定
 * @param none
 * @return none
 */
static void v_DD_EX_DetectOverCurrent(void)
{
/*----エキサイタ過電流判定----*/

	s16	a_s16_overCurrentThreshold = (S16_DD_EX_OVER_CURRENT_THRESHOLD_10MA);
	bool a_b_isDetectActivity = B_MDC_IsDetectActivity();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( (a_b_isDetectActivity == FALSE) ||																/* 異常判定不許可又？ */
		(b_DD_EX_abnormalOverCurrent == TRUE) )															/* 既に異常状態？ */
	{
		return;
	}

	if( ( s16_DD_EX_if_10mA >= a_s16_overCurrentThreshold ) || ( s16_DD_EX_if_10mA <= (-a_s16_overCurrentThreshold) ) )
	{
		if( u8_DD_EX_overCurrentJudgeCount < (U8_DD_EX_OVERCURRENT_COUNT) )
		{
			u8_DD_EX_overCurrentJudgeCount++;															/* 過電流異常検出タイマを更新 */
		}
		else
		{																								/* 判断閾値以上 */
			b_DD_EX_abnormalOverCurrent = TRUE;															/* エキサイタ過電流異常セット */
			u8_DD_EX_overCurrentJudgeCount = (u8)0;														/* 過電流異常検出タイマをリセット */
		}
	}
	else
	{
		u8_DD_EX_overCurrentJudgeCount = (u8)0;															/* 過電流異常検出タイマをリセット */
	}
}


/**
 * @brief 電流指令値の設定
 * @detail 界磁電流PI演算
 * @note ゲートON/OFF判断
 * @param none
 * @return none
 */
void INTRPT_DD_EX_FieldControlPeak(void)
{
	/* for DEBUG */
	PO_GPIO3_HI();

	en_DD_EX_dischargeMode = En_MDC_GetDischargeMode();													/* ディスチャージモード取得 */

	if( en_DD_EX_dischargeMode == MD_DISCHARGE_RUN )													/* ディスチャージ通電 */
	{
		b_DD_EX_isDischargeRun = TRUE;
		u16_DD_EX_pwmTimerMin = (U16_EX_DISCHARGE_PWM_TIMER_MIN);										/* ディスチャージ用タイマ制限値(Duty制限) */
	}
	else if( en_DD_EX_dischargeMode == MD_DISCHARGE_PREPARING_FINISH )									/* ディスチャージ終了準備 */
	{
		b_DD_EX_isDischargeRun = TRUE;
		b_DD_EX_isDischargePreparingFinish = TRUE;														/* ディスチャージ終了準備 */
		u16_DD_EX_pwmTimerMin = (U16_EX_DISCHARGE_PWM_TIMER_MIN);										/* ディスチャージ用タイマ制限値(Duty制限) */
	}
	else
	{
		b_DD_EX_isDischargeRun = FALSE;
		b_DD_EX_isDischargePreparingFinish = FALSE;
		u16_DD_EX_pwmTimerMin = (U16_EX_PWM_TIMER_MIN);
	}

/*-----ゲートオフ指示判定-----*/
#ifdef	EMERGENCY_STOP_ACTIVITY
//	if( B_CHT_IsBrOn() == FALSE )				/***** 緊急停止対応 *****/
	if( B_CHT_IsChgOn() == TRUE )				/***** 緊急停止対応(暫定) *****/
	{
		b_DD_EX_emergencyStop = TRUE;
		V_DD_EX_ExecuteGateOff();																		/* エキサイタGateOffを実施する */
	}
	else if( (b_DD_EX_abnormalOverCurrent == TRUE)	||													/* エキサイタ過電流 */
			((b_DD_EX_forcedGateOff	== TRUE) && (b_DD_EX_isDischargeRun	== FALSE)) )					/* 強制ゲートオフ & ディスチャージ中ではない */
	{	/** エキサイタのみゲートOFF指示 **/
		V_DD_EX_ExecuteGateOff();																		/* エキサイタGateOffを実施する */
	}
	else if( ((b_DD_EX_isStartRequested == TRUE) && (b_DD_EX_isPermitted == TRUE))	||					/* モータ駆動要求＆モータ駆動許可 */
			(b_DD_EX_isDischargeRun == TRUE) )															/* ディスチャージ要求 */
	{	/** ゲートON指示 **/
		if(b_DD_EX_isGateOn == FALSE)
		{
			R_Config_MTU6_MTU7_StartTimerCtrl();														/* MTUOC6B出力 */
			b_DD_EX_isGateOn = TRUE;																	/* ゲートON状態 */
		}

		v_DD_EX_CurrentPIControl();																		/* PI演算 */

		/* PI演算結果をタイマ仮値に代入 */
		s16_DD_EX_eCommandTemp = s16_DD_EX_vf;
		v_DD_EX_ClipVoltageCommand();																	/* PWM制限 */
		PO_E_FET_ON();
		R_Config_MTU6_MTU7_UpdDuty( (u16)s16_DD_EX_eCommand, (U16_EX_PWM_TIMER_MAX), (U16_EX_PWM_TIMER_MAX) );
	}
	else
	{	/** 全てゲートOFF指示 **/
		V_DD_EX_ExecuteGateOff();																		/* エキサイタGateOffを実施する */
		V_DD_MTR_ExecuteGateOff();																		/* モータGateOffを実施する */
	}
#else
	if( (b_DD_EX_abnormalOverCurrent == TRUE)	||														/* エキサイタ過電流 */
		((b_DD_EX_forcedGateOff	== TRUE) && (b_DD_EX_isDischargeRun	== FALSE)) )						/* 強制ゲートオフ & ディスチャージ中ではない */
	{	/** エキサイタのみゲートOFF指示 **/
		V_DD_EX_ExecuteGateOff();																		/* エキサイタGateOffを実施する */
	}
	if( UNI_GL_testFlg.bit.b_ifForcedGateOff == TRUE )													/* エキサイタ強制GateOff */
	{	/** エキサイタのみゲートOFF指示 **/
		V_DD_EX_ExecuteGateOff();																		/* エキサイタGateOffを実施する */
	}

	else if( ((b_DD_EX_isStartRequested == TRUE) && (b_DD_EX_isPermitted == TRUE))	||					/* モータ駆動要求＆モータ駆動許可 */
			(b_DD_EX_isDischargeRun == TRUE) )															/* ディスチャージ要求 */
	{	/** ゲートON指示 **/
		if(b_DD_EX_isGateOn == FALSE)
		{
			R_Config_MTU6_MTU7_StartTimerCtrl();														/* MTUOC6B出力 */
			b_DD_EX_isGateOn = TRUE;																	/* ゲートON状態 */
		}
		/* for DEBUG */
		PO_GPIO3_HI();

		v_DD_EX_CurrentPIControl();																		/* PI演算 */

		/* PI演算結果をタイマ仮値に代入 */
		s16_DD_EX_eCommandTemp = s16_DD_EX_vf;
		v_DD_EX_ClipVoltageCommand();																	/* PWM制限 */
		PO_E_FET_ON();
		R_Config_MTU6_MTU7_UpdDuty( (u16)s16_DD_EX_eCommand, (U16_EX_PWM_TIMER_MAX), (U16_EX_PWM_TIMER_MAX) );
	}
	else
	{	/** 全てゲートOFF指示 **/
		V_DD_EX_ExecuteGateOff();																		/* エキサイタGateOffを実施する */
		V_DD_MTR_ExecuteGateOff();																		/* モータGateOffを実施する */
	}
#endif
	/* for DEBUG */
	PO_GPIO3_LO();
}


/**
 * @brief GateOffする
 * @param none
 * @return a_s16_idTemp
 */
void	V_DD_EX_ExecuteGateOff(void)																	/* モータGateOffを実施する */
{
	s16_DD_EX_ifRef_10mA = (s16)0;																		/* If指令値=0 */
	s16_DD_EX_ifRefFinal_10mA = (s16)0;																	/* If指令値=0 */

	if( b_DD_EX_isNotGateOffForEvalution == FALSE )														/* 界磁OFFするかどうか */
	{
		PO_E_FET_OFF();																					/* 界磁OFF */
	}
	R_Config_MTU6_MTU7_StopTimerCtrl();

	s32_DD_EX_integralVf_10mV = (s32)0;																	/* Vf積分項クリア */

	/* PWMデューティのタイマセット */
	R_Config_MTU6_MTU7_UpdDuty((U16_EX_PWM_TIMER_MAX), (U16_EX_PWM_TIMER_MAX), (U16_EX_PWM_TIMER_MAX));	/* 各相デューティを０に設定 */

	b_DD_EX_isGateOn = FALSE;																			/* ゲートOFF状態 */

	/* for DEBUG */
	PO_GPIO3_LO();
}


/**
 * @brief 界磁電流PI演算
 * @param none
 * @return none
 */
static void v_DD_EX_CurrentPIControl(void)
{
/*-----ImLimitよりieRefリミット-----*/
	s16_DD_EX_vfMax_10mV = (s16)U16_BAT_GetBattVoltAve_10mV();

	s16 a_s16_ifRefTemp_10mA = (S16_EX_IF_MAX_10MA);

	if( UNI_GL_testFlg.bit.b_currentLimitActivity == TRUE )
	{
		a_s16_ifRefTemp_10mA = (S16_EX_IF_MAX2_10MA);
	}

	if( s16_DD_EX_ifRef_10mA <= a_s16_ifRefTemp_10mA )
	{
		a_s16_ifRefTemp_10mA = s16_DD_EX_ifRef_10mA;
	}

	if( b_DD_EX_isDischargeRun == TRUE )
	{
		if( b_DD_EX_isDischargePreparingFinish == FALSE )
		{
			a_s16_ifRefTemp_10mA = s16_DD_EX_ifRefAtDischarge_10mA;
		}
		else
		{
			a_s16_ifRefTemp_10mA = (s16)0;
		}
	}
	s16_DD_EX_ifRefFinal_10mA = a_s16_ifRefTemp_10mA;
/*-----PI演算-----*/
	/***** ゲインテスト用 *****/
	if( UNI_GL_testFlg.bit.b_ifGainOld == TRUE )
	{
		U16_GL_pGainTemp	= (U16_EX_KP2);																/* 旧界磁電流比例ゲインセット */
		U16_GL_iGainTemp	= (U16_EX_KI2);																/* 旧界磁電流積分ゲインセット */
	}
	else
	{
		U16_GL_pGainTemp	= (U16_EX_KP);																/* 新界磁電流比例ゲインセット */
		U16_GL_iGainTemp	= (U16_EX_KI);																/* 新界磁電流積分ゲインセット */
	}

	/***** ゲインテスト用 *****/
	if( UNI_GL_testFlg.bit.b_ifGainTestActivity == TRUE )
	{
		u16_DD_EX_proportionalGain	= U16_GL_pGainTest;													/* 界磁電流比例ゲインセット */
		u16_DD_EX_integralGain		= U16_GL_iGainTest;													/* 界磁電流積分ゲインセット */
	}
	else
	{
		u16_DD_EX_proportionalGain	= U16_GL_pGainTemp;													/* 界磁電流比例ゲインセット */
		u16_DD_EX_integralGain		= U16_GL_iGainTemp;													/* 界磁電流積分ゲインセット */
	}

	s16 a_s16_ifDiff	= (s16)(s16_DD_EX_ifRefFinal_10mA - s16_DD_EX_if_10mA);							/* 界磁電流の偏差を取得 */
	s32 a_s32_propoVf	= (s32)(((s32)a_s16_ifDiff * (s32)u16_DD_EX_proportionalGain) >> (s32)8);		/* 界磁電流偏差の比例項を計算する */
	s16	a_s16_propoVf	= (s16)S32_CMF_GETLIMIT( a_s32_propoVf, (s32)(-s16_DD_EX_vfMax_10mV), (s32)s16_DD_EX_vfMax_10mV );	/* 最大値と最小値にクリップ */
	s32 a_s32_integVf	= (s32)((s32)a_s16_ifDiff * (s32)u16_DD_EX_integralGain + s32_DD_EX_integralVf_10mV);	/* 界磁電流積分項の計算(8ビット） */
	s16	a_s16_integVf	= (s16)(a_s32_integVf >> (s32)16);												/* 積分項の計算(32ビット）→ 16ビット右シフト */

/*----積分項の制限----*/
	if( a_s16_integVf > s16_DD_EX_vfMax_10mV )															/* 積分項を閾値と比較 */
	{														/* 最大値を超えた場合 */
		a_s16_integVf = s16_DD_EX_vfMax_10mV;															/* 積分項を最大値に設定する */
		s32_DD_EX_integralVf_10mV = (s32)((s32)s16_DD_EX_vfMax_10mV << (s32)16);						/* 積分項32ビット保存 */
	}
	else if ( a_s16_integVf < (-s16_DD_EX_vfMax_10mV) )
	{														/* 最小値より小さい場合 */
		a_s16_integVf = (-s16_DD_EX_vfMax_10mV);														/* 積分項を最小値に設定する */
		s32_DD_EX_integralVf_10mV = (s32)((s32)(-s16_DD_EX_vfMax_10mV) << (s32)16);						/* 積分項32ビット保存 */
	}
	else
	{														/* 制限値の間にいる */
		s32_DD_EX_integralVf_10mV = a_s32_integVf;														/* 現在の積分項を保存 */
	}


//	s32	a_s32_integralVfMax	= (s32)s16_DD_EX_vfMax_10mV << (s32)16;
//
//	if( a_s32_integVf > a_s32_integralVfMax )															/* 積分項を閾値と比較 */
//	{														/** 最大値を超えた場合 **/
//		a_s32_integVf = a_s32_integralVfMax;															/* 積分項を最大値に設定する */
//		s32_DD_EX_integralVf_10mV = a_s32_integralVfMax;												/* 積分項32ビット保存 */
//	}
//	else if ( a_s32_integVf < (-a_s32_integralVfMax) )
//	{														/** 最小値より小さい場合 **/
//		a_s32_integVf = (-a_s32_integralVfMax);															/* 積分項を最小値に設定する */
//		s32_DD_EX_integralVf_10mV = -a_s32_integralVfMax;												/* 積分項32ビット保存 */
//	}
//	else
//	{														/** 制限値の間にいる **/
//		s32_DD_EX_integralVf_10mV = a_s32_integVf;														/* 現在の積分項を保存 */
//	}
//
//	s16	a_s16_integVf	= (s16)(a_s32_integVf >> (s32)16);												/* 積分項の計算(32ビット）→ 16ビット右シフト */

/*----比例項+積分項の制限-----*/
	s16 a_s16_ePI = a_s16_propoVf + a_s16_integVf;														/* 比例項＋積分項項 */

/*----Vfを制限する----*/
	if( a_s16_ePI > s16_DD_EX_vfMax_10mV )																/* 最大値と比較する */
	{														/** 最大値より大きい場合 **/
		s16_DD_EX_vf_10mV = s16_DD_EX_vfMax_10mV;														/* Vfを最大値に設定 */
	}
	else if ( a_s16_ePI < (s16)0 )							/** 最小値と比較 **/
	{														/** 最小値より小さい場合 **/
		s16_DD_EX_vf_10mV = (s16)0;																		/* Vfを最小値に設定する */
	}
	else
	{														/** 制限値の間にある場合 **/
		s16_DD_EX_vf_10mV = a_s16_ePI;																	/* Vf計算値を採用する */
	}

/*-----Vfを物理量からタイマ値に変換する-----*/
	if( s16_DD_EX_vfMax_10mV == (s16)0 )
	{
		s16_DD_EX_vfTemp = (s16)0;
	}
	else
	{
		s16_DD_EX_vfTemp = (s16)((s32)(U16_EX_PWM_TIMER_MAX) * (s32)s16_DD_EX_vf_10mV / (s32)s16_DD_EX_vfMax_10mV );
	}

/*-----タイマ計算結果の反転----*/
	s16_DD_EX_vf = (s16)((U16_EX_PWM_TIMER_MAX) - s16_DD_EX_vfTemp);									/* ここで反転 */
	nop();
}


/**
 * @brief UVW指令値制限
 * @param none
 * @return none
 */
static void	v_DD_EX_ClipVoltageCommand(void)
{
/*-----PWMデューティ制限 最大値クリップ ------*/
	if( s16_DD_EX_eCommandTemp >= (s16)u16_DD_EX_pwmTimerMax )
	{
		s16_DD_EX_eCommand = (s16)u16_DD_EX_pwmTimerMax;
	}
/*-----PWMデューティ制限 最小値クリップ ------*/
	else if( s16_DD_EX_eCommandTemp <= (s16)u16_DD_EX_pwmTimerMin )
	{
		s16_DD_EX_eCommand = (s16)u16_DD_EX_pwmTimerMin;
	}
	else
	{
		s16_DD_EX_eCommand = s16_DD_EX_eCommandTemp;													/* その他：計算値を採用 */
	}
}


/**
 * @brief ゲートON/FF状態提供
 * @param none
 * @return b_DD_EX_isGateOn
 */
bool	B_DD_EX_IsGateOn(void)
{
	return b_DD_EX_isGateOn;
}

/**
 * @brief モータ駆動を禁止する
 * @param none
 * @return none
 */
void	V_DD_EX_Forbid(void)
{
	b_DD_EX_isPermitted = FALSE;
}

/**
 * @brief 界磁駆動を許可する
 * @param none
 * @return none
 */
void	V_DD_EX_Permit(void)
{
	b_DD_EX_isPermitted = TRUE;
}

/**
 * @brief 界磁駆動を要求する
 * @param none
 * @return none
 */
void	V_DD_EX_StartControl(void)
{
	b_DD_EX_isStartRequested = TRUE;
}

/**
 * @brief ゲートOFFを要求する
 * @param none
 * @return none
 */
void	V_DD_EX_StopControl(void)
{
	b_DD_EX_isStartRequested = FALSE;
}

/**
 * @brief エキサイタ強制ゲートOFFを要求する
 * @param none
 * @return none
 */
void	V_DD_EX_ForcedGateOff(void)
{
	b_DD_EX_forcedGateOff = TRUE;
}

/**
 * @brief エキサイタ強制ゲートOFFを解除する
 * @param none
 * @return none
 */
void	V_DD_EX_CanselForcedGateOff(void)
{
	b_DD_EX_forcedGateOff = FALSE;
}

/**
 * @brief AD1データ提供
 */
ST_ADC_ADVALUE1	St_DD_EX_GetAdValue1(void)
{
	return	*pst_DD_EX_adValue1;
}

/**
 * @brief If指令値セット[10mA]
 * @param a_s16_idRef
 * @param a_s16_iqRef
 * @return none
 */
void	V_DD_EX_SetIfRef_10mA(s16 a_s16_ifRef)
{
	s16_DD_EX_ifRef_10mA = a_s16_ifRef;
}

/**
 * @brief 最終If指令値提供[10mA]
 * @param none
 * @return s16_DD_EX_ifRef_10mA
 */
s16	S16_DD_EX_GetIfRef_10mA(void)
{
	return	s16_DD_EX_ifRef_10mA;
}

/**
 * @brief Ifフィードバック値提供[10mA]
 * @param none
 * @return s16_DD_EX_ifRef_10mA
 */
s16	S16_DD_EX_GetIfFb_10mA(void)
{
	return	s16_DD_EX_if_10mA;
}

/**
 * @brief 界磁電圧提供[10mV]
 * @param none
 * @return s16_DD_EX_vf_10mV
 */
s16	S16_DD_EX_GetVf_10mV(void)
{
	return	s16_DD_EX_vf_10mV;
}

/**
 * @brief エキサイタ過電流異常状態提供
 * @param none
 * @return b_DD_EX_abnormalOverCurrent
 */
bool	B_DD_EX_IsAbnormalOverCurrent(void)
{
	return	b_DD_EX_abnormalOverCurrent;
}

/**
 * @brief 緊急停止要求提供
 * @param none
 * @return b_DD_EX_emergencyStop
 */
bool	B_DD_EX_IsEmergencyStop(void)
{
	return	b_DD_EX_emergencyStop;
}

/**
 * @brief 界磁積分バッファ提供[10mV]
 * @param none
 * @return s32_DD_EX_integralVf_10mV
 */
s32	S32_DD_EX_GetIntegralVf_10mV(void)
{
	return	s32_DD_EX_integralVf_10mV;
}

/**
 * @brief 界磁電圧最大値提供[10mV]
 * @param none
 * @return s16_DD_EX_vfMax_10mV
 */
s16	S16_DD_EX_GetVfMax_10mV(void)
{
	return	s16_DD_EX_vfMax_10mV;
}

/**
* @brief 電流制御 & AD取り込み処理用変数の初期化処理(再起動用)
 * @param none
 * @return none
 */
void V_DD_EX_InitializeForRestart(void)
{
	b_DD_EX_isStartRequested = FALSE;

	u16_DD_EX_proportionalGain = (U16_EX_KP);															/* 界磁電流比例ゲイン */
	u16_DD_EX_integralGain = (U16_EX_KI);																/* 界磁電流積分ゲイン */

	s16_DD_EX_vfMax_10mV = (s16)0;																		/* 界磁電流電圧Vd最大値 物理量[10mV] */

	s16_DD_EX_if_10mA = (s16)0;																			/* 界磁電流 */

	s16_DD_EX_ifRef_10mA = (s16)0;																		/* If指令値[10mA] */
}
