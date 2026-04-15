/**
 * @brief ベクトル演算処理
 * @detail 相電流取込、電気角推定、ベクトル演算、タイマ指令値算出、電圧監視、ロータ回転数算出
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11.xx
 * RX66Tに対応　華　2023.09
 */
/* Includes ------------------------------------------------------------------*/
#include "Config_MTU3_MTU4.h"
#include "Config_S12AD0.h"
#include "Config_S12AD1.h"
#include "Config_S12AD2.h"
#include "Config_DA.h"

#include "DEFINE_SETTING.h"
#include "MCU.h"
#include "TYPEDEF_MOTOR.h"
#include "TYPEDEF_MODE.h"
#include "DEFINE_SYSTEM.h"
#include "DEFINE_MOTOR.h"
#include "DEFINE_ABN.h"
#include "COMMON_FUNCTION.h"		/* 汎用関数群 */
#include "COMMON_MACRO.h"
#include "TRIGONOMETRIC_FUNCTION.h"
#include "PREVENT_CHATTERING.h"						/* チャタリング防止 */

/* DDL */
#include "DD_MOTOR.h"
#include "DD_ENCODER.h"
#include "DD_EXCITER.h"

#include "DETECT_ABNORMAL.h"
#include "DETECT_MOTOR.h"
#include "DETECT_BATTERY.h"
#include "MOTOR.h"
#include "ADC.h"
#include "DIAG_SET.h"
#include "MODE_CONDUCTOR.h"
#include "BATTERY.h"
#include "VQLIMIT_TABLE.h"
#include "IMLIMIT_TABLE.h"

/* for DEBUG */
#include "ramdef.h"

/* Private define ------------------------------------------------------------*/
#define	U16_MTR_VQ_BIAS_COEF_100MV				(u16)4100									/* 左は実際値 以下設計値 線間誘起電圧補正係数 60rpmで5.12V → 0.0853 [V/rpm] / 0.86 ＝ 1.06 [0.1V/rpm] ≒ 4342/4096	BIAS値演算時にモータ回転数を掛けて12bit右シフト */

#define	U16_MTR_EANGLE_ESTIMATE					(u16)((u16)60 * (u16)256)					/* 電気角補正値上限 60度 */
#define	U16_MTR_EANGLE_ESTIMATE_0P5				(u16)((u16)60 * (u16)128)					/* 電気角補正値上限 60度 */
#define	U16_MTR_EANGLE_ESTIMATE_MAX_DEG			(u16)60										/* 電気角補正値上限 */
#define	U16_MTR_EANGLE_ESTIMATE_MAX_0P03125DEG	(u16)(U16_MTR_EANGLE_ESTIMATE_MAX_DEG)*32	/* 電気角補正値上限 60deg */

//#define	U16_MTR_MAX_ENC_COUNT					(u16)8000								/* 極低速での電気角推定可能時間に相当するタイマ値	0.5[s]×16000[Hz] = 8000) */
#define	U16_MTR_MAX_ENC_COUNT					(u16)6000									/* 極低速での電気角推定可能時間に相当するタイマ値	0.5[s]×12000[Hz] = 6000) */

#define	U8_ABNORMAL_ENC_COUNT					(u8)5										/* エンコーダ異常判定しきい値 */
#define	S8_MTR_VIBRATE_COUNTUP					(s8)4										/* ロータ振動判定カウントアップ値 */
#define	S8_MTR_VIBRATE_COUNTDOWN				(s8)2										/* ロータ振動判定カウントダウン値 */
#define	S8_MTR_VIBRATE_COUNTMAX					(s8)20										/* ロータ振動判定カウンタ最大値 */
#define	S8_MTR_VIBRATE_COUNTMIN					(s8)0										/* ロータ振動判定カウンタ最小値 */

#define	S32_MTR_OVER_CURRENT_THRESHOLD_10MA		(s32)55000									/* モータ電流過電流異常検出値(+)	  550A 暫定 */
#define	U8_MTR_OVERCURRENT_COUNT				(u8)2										/* 過電流異常検出連続時間：62.5μs × 2 ＝ 125μs */

#define	U8_MTR_AD0_MAX_WAITING_COUNT			(u8)120										/* AD0ループAD変換待機カウンタ最大数 */

/*** デッドタイム補償の設定 */
#define	U16_MTR_DEADTIME_PWM_TIMER_LOW			(u16)0										/* デッドタイム補償 Duty_max( 416 ) * 2.5/ 83 = 12*/
#define	U16_MTR_DEADTIME_PWM_TIME				(u16)6										/* デッドタイム補償 Duty_max( 416 ) * 2.5/ 83 /2 = 6*/
#define	U16_MTR_DEADTIME_VQ_100MV				(u16)250									/* Vq25Vまで補償 */

/*** ゲート電圧の設定 ***/
#define	U16_MTR_GATE_OFF_BATT_VOLT_10MV			(u16)6300									/* 過電圧によるゲートオフ要求電圧 */ /* 暫定 */
#define	U16_MTR_GATE_ON_BATT_VOLT_10MV			(u16)((U16_MTR_GATE_OFF_BATT_VOLT_10MV) - (u16)200)	/* 過電圧によるゲートオフ解除電圧 */ /* 暫定 */
/*----------	エンコーダ異常パターン	-----------------*/
#define	U8_MTR_NG_ENC_PATTERN1					(u8)0										/* エンコーダ未定義パターン1 */
#define	U8_MTR_NG_ENC_PATTERN2					(u8)7										/* エンコーダ未定義パターン2 */

/* 3相電流pp取得（モータ断線検出用） */
#define	S32_MTR_CT_PP_DIFF_JUDGE_IQREF_10MA		((s32)500)									/* U,V,W振幅判定開始閾値 [10mA] */
#define	S16_MTR_CT_PP_DIFF_JUDGE_MOTREV_RPM		((s16)30)									/* U,V,W振幅判定開始モータ回転数 [rpm] */
#define	U32_MTR_WIRE_OPEN_JUDGE_TIME_MS			((u32)640)									/* モータ断線異常検出時間 [msec] */
#define	U16_MTR_WIRE_OPEN_JUDGE_COUNT			((u16)((U32_MTR_WIRE_OPEN_JUDGE_TIME_MS) * (u32)(U16_MTR_PWM_FREQ_KHZ)))	/* 640msモータ断線確定時間 */

#define	U16_MTR_INTEGRAL_VDQ_MAX_10MV			(u16)(U16_DISCHRGSTOP_DEC_BATT_VOLT_10MV) * 12 / 10	/* 積分項リミット値：最低電源電圧（放電停止電圧）×1.2(マージン) に設定 */

/* Private variables ---------------------------------------------------------*/
typedef enum {
	ENUM_IMIN_PHASE_U = 0,																/* モータ電流の最小値はＵ線（断線の疑いがあると選択される）エラー時のUVW特定用 */
	ENUM_IMIN_PHASE_V,																	/* モータ電流の最小値はＶ線（断線の疑いがあると選択される）エラー時のUVW特定用 */
	ENUM_IMIN_PHASE_W																	/* モータ電流の最小値はＷ線（断線の疑いがあると選択される）エラー時のUVW特定用 */
} ENUM_DD_MTR_IMIN_PHASE;

#ifdef	DEBUG_WATCH_ACTIVITY

ENUM_DD_MTR_IMIN_PHASE en_DD_MTR_iMinPhase = ENUM_IMIN_PHASE_U;					/* モーター電流の最小値が検出された相 */
enum {																			/* モータ相電流最大最小値探索 */
	ENUM_DD_MTR_PP_SEARCH_INIT = 0,														/* 開始 */
	ENUM_DD_MTR_PP_SEARCH_RUNNING														/* 実施中 */
} en_DD_MTR_ppSearchStatus = ENUM_DD_MTR_PP_SEARCH_INIT;								/* モーター電流pp値探索ステータス */
/* モータ断線検出用 */
enum {
	ENUM_WIRE_OPEN_NORMAL = 0,															/* 正常 */
	ENUM_WIRE_OPEN_ABNORMAL																/* 断線確定 */
} en_DD_MTR_wireOpenEachStatus = ENUM_WIRE_OPEN_NORMAL;									/* モータ断線状態（UVW各々） */

/* 6Stepへの切り替わり判定用 */
enum {																			/* 6Step状態 */
	ENUM_6STEP_STATE0 = 0,																/* 初回 */
	ENUM_6STEP_STATE1,																	/* 初回判定済み */
	ENUM_6STEP_STATE2																	/* 通常電気角補正積算値参照済み */
} en_DD_MTR_6StepStatus = ENUM_6STEP_STATE0;

EN_MDC_DISCHARGE_MODE	en_DD_MTR_dischargeMode	= MD_DISCHARGE_INITIAL;			/* 現在ディスチャージモード */

bool	b_DD_MTR_isStartAD0Convert = FALSE;											/* AD0変換開始フラグ */
bool	b_DD_MTR_isDelayAD0Convert = FALSE;											/* AD0変換遅延状態フラグ */
u8	u8_DD_MTR_ad0FinishedCounterMax = (u8)0;									/* AD0チャンネルAD変換終了カウンタ最大値 */

s32	s32_DD_MTR_iUPp_10mA = (s32)0;												/* U相電流PP(TOP-BOTTOM)値 */
s32	s32_DD_MTR_iVPp_10mA = (s32)0;												/* V相電流PP(TOP-BOTTOM)値 */
s32	s32_DD_MTR_iWPp_10mA = (s32)0;												/* W相電流PP(TOP-BOTTOM)値 */
s32	s32_DD_MTR_iUTopBuff_10mA = (s32)0;											/* U相電流TOP仮値 */
s32	s32_DD_MTR_iVTopBuff_10mA = (s32)0;											/* V相電流TOP仮値 */
s32	s32_DD_MTR_iWTopBuff_10mA = (s32)0;											/* W相電流TOP仮値 */
s32	s32_DD_MTR_iUBtmBuff_10mA = (s32)0;											/* U相電流BOT仮値 */
s32	s32_DD_MTR_iVBtmBuff_10mA = (s32)0;											/* V相電流BOT仮値 */
s32	s32_DD_MTR_iWBtmBuff_10mA = (s32)0;											/* W相電流BOT仮値 */
s32	s32_DD_MTR_ctPpMAX_10mA = (s32)0;											/* U,V,Wモータ電流の振幅の最大値 単位10mA */
s32	s32_DD_MTR_ctPpMIN_10mA = (s32)0;											/* U,V,Wモータ電流の振幅の最小値 単位10mA */

s16	s16_DD_MTR_iUTopBuff_100mA = (s16)0;											/* U相電流TOP仮値[100mA] */
s16	s16_DD_MTR_iVTopBuff_100mA = (s16)0;											/* V相電流TOP仮値[100mA] */
s16	s16_DD_MTR_iWTopBuff_100mA = (s16)0;											/* V相電流TOP仮値[100mA] */
s16	s16_DD_MTR_iUBtmBuff_100mA = (s16)0;											/* U相電流BOT仮値[100mA] */
s16	s16_DD_MTR_iVBtmBuff_100mA = (s16)0;											/* V相電流BOT仮値[100mA] */
s16	s16_DD_MTR_iWBtmBuff_100mA = (s16)0;											/* V相電流BOT仮値[100mA] */

s16	s16_DD_MTR_iU_100mA = (s16)0;												/* U相電流[100mA] */
s16	s16_DD_MTR_iV_100mA = (s16)0;												/* V相電流[100mA] */
s16	s16_DD_MTR_iW_100mA = (s16)0;												/* W相電流[100mA] */

u8	u8_DD_MTR_encPhaseCountForPpDetection = (u8)1;								/* pp計算用位相カウンタ 初期値は1とする エンコーダパターン数が6なので、カウンタは1-6で数えることとする */
u16	u16_DD_MTR_wireOpenJudgeCount_62p5us = (u16)0;								/* モータ断線の疑い有カウント(640ms) */

u16	u16_DD_MTR_dAxisProportionalGain	= (U16_MTR_DAXIS_KP);					/* d軸比例ゲイン */
u16	u16_DD_MTR_dAxisIntegralGain		= (U16_MTR_DAXIS_KI);					/* d軸積分ゲイン */
u16	u16_DD_MTR_qAxisProportionalGain	= (U16_MTR_QAXIS_KP);					/* q軸比例ゲイン */
u16	u16_DD_MTR_qAxisIntegralGain		= (U16_MTR_QAXIS_KI);					/* q軸積分ゲイン */
u16	u16_DD_MTR_dPGainTemp = (U16_MTR_DAXIS_KP);									/* d軸比例ゲインtemp */
u16	u16_DD_MTR_dIGainTemp = (U16_MTR_DAXIS_KI);									/* d軸積分ゲインtemp */
u16	u16_DD_MTR_qPGainTemp = (U16_MTR_QAXIS_KP);									/* q軸比例ゲインtemp */
u16	u16_DD_MTR_qIGainTemp = (U16_MTR_QAXIS_KI);									/* q軸積分ゲインtemp */
//u16	u16_DD_MTR_dPGainForTest = (U16_MTR_DAXIS_KP);								/* d軸比例ゲインTEST用 */
//u16	u16_DD_MTR_qPGainForTest = (U16_MTR_QAXIS_KP);								/* q軸比例ゲインTEST用 */
bool	b_DD_MTR_isStartRequested = FALSE;											/* モータ駆動要求 */
bool	b_DD_MTR_isPermitted = TRUE;												/* ゲートON許可 */
bool	b_DD_MTR_isGateOn = FALSE;													/* ゲートON状態 */
s32	s32_DD_MTR_integralVd_10mV = (s32)0;										/* d軸積分項上位32bit */
s32	s32_DD_MTR_integralVq_10mV = (s32)0;										/* q軸積分項上位32bit */

u8	u8_DD_MTR_encPattern = (u8)0;												/* 現在のエンコーダパターン出力(換算した結果）*/
u8	u8_DD_MTR_encPatternLast = (u8)0;											/* 前回のエンコーダパターン */
bool	b_DD_MTR_isEncReverseRotate = FALSE;										/* 現サイクル回転方向	0:正転	1:逆転 */
bool	b_DD_MTR_isEncReverseRotateLast = FALSE;									/* 前サイクル回転方向	0:正転	1:逆転 */
bool	b_DD_MTR_isMotorReverseRotate = FALSE;										/* モータ回転方向(エンコーダ正転/逆転カウンタ3回以上で確定)	0:正転	1:逆転 */
u8	u8_DD_MTR_encReverseRotateCounter = (u8)0;									/* エンコーダ逆転カウンタ */
u8	u8_DD_MTR_encNormalRotateCounter = (u8)0;									/* エンコーダ正転カウンタ */
u32	u32_DD_MTR_encTimerValue60 = (u32)0;										/* 60deg間のタイマ値 form DD_ENCODER */
u16	u16_DD_MTR_eangle_0p03125deg = (u16)0;										/* 推定電気角[0.03125deg] */
u16	u16_DD_MTR_eangleMax_0p03125deg = (u16)0;									/* 最大推定電気角[0.03125deg] */
volatile u16	u16_DD_MTR_eangleLast_deg = (u16)0;									/* 推定電気角前回値 */
volatile u16	u16_DD_MTR_eangleLast_0p03125deg = (u16)0;							/* 推定電気角前回値 */
volatile s16	s16_DD_MTR_eangleDiff_deg = (s16)0;									/* 電気角推定差分[deg] */
volatile s16	s16_DD_MTR_eangleDiff_0p03125deg = (s16)0;							/* 電気角推定差分[0.03125deg] */
u16	u16_DD_MTR_eangleBase_deg = (u16)0;											/* ベース電気角[deg] */
u16	u16_DD_MTR_estimateEangleDelta = (u16)0;									/* 単位補正電気角[256×deg] */
u16	u16_DD_MTR_estimateEangle_0p03125deg = (u16)0;								/* 補正電気角[0.03125deg] */
u16	u16_DD_MTR_estimateEangleMax_0p03125deg = (u16)0;								/* 最大補正電気角[0.03125deg] */
u16	u16_DD_MTR_estimateEangleSum = (u16)0;										/* 補正電気角の暫定値[256×deg](総和値) */
u16	u16_DD_MTR_lowSpdEstimateEangleDelta = (u16)0;								/* 補正単位電気角(極遅速度時)[256×deg] */
u16	u16_DD_MTR_lowSpdEstimateEangleSum = (u16)0;								/* 補正電気角(極遅速度時)[256×deg](総和値) */
u16	u16_DD_MTR_eangleOffset_0p03125deg = (U16_MTR_EANGLE_OFFSET_0P03125DEG);	/* 電気角オフセット補正値[0.03125deg] */
u16	u16_DD_MTR_eangleMapOffset_0p03125deg = (U16_MTR_EANGLE_OFFSET_0P03125DEG);	/* 電気角オフセット補正値[0.03125deg] */
u16	u16_DD_MTR_eangleOffsetForTest_0p03125deg = (u16)0;							/* テスト用電気角オフセット補正値[0.03125deg] */
u16	u16_DD_MTR_eangleOffsetForTest_deg = (u16)0;							/* テスト用電気角オフセット補正値[0.03125deg] */
bool	b_DD_MTR_isLowSpdSetFinished = FALSE;										/* 極低速時補正角度の設定フラグ 1:設定完了 0:設定しない */
bool	b_DD_MTR_isRotorVibration = FALSE;											/* ロータ振動フラグ */
s8	s8_DD_MTR_rotorVibrationCounter = (s8)0;									/* ロータ振動判断カウンタ */
bool	b_DD_MTR_overRegenerativeVoltage = FALSE;									/* 回生電圧Over 1：Over */
bool	b_DD_MTR_detectWireOpenActivate = TRUE;										/* モータ単相断線異常有効化：default=有効 */
bool	b_DD_MTR_detectEncoderActivate = TRUE;										/* エンコーダ異常有効化：default=有効 */

s16	s16_DD_MTR_revolutionArray_rpm[(U8_MTR_NUM_ENCODER_PATTERNS)] =				/* モータ回転数6移動平均バッファ */
									{ (s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0, };

u8	u8_DD_MTR_overCurrentJudgeCount	= (u8)0;									/* 過電流異常検出タイマ */
u8	u8_DD_MTR_encAbnormalCount	= (u8)0;										/* エンコーダ異常判定カウンタ */
s16	s16_DD_MTR_revolutionRaw_rpm = (s16)0;										/* 符号付モータ回転数生値[rpm] */
s16	s16_DD_MTR_revolution_rpm = (s16)0;											/* 符号付モータ回転数[rpm] */
s16	s16_DD_MTR_revolutionAve6_rpm = (s16)0;										/* 符号付モータ回転数[rpm] */
//static s16	s16_DD_MTR_vqBias_10mV = (s16)0;										/* 誘起電圧バイアス[10mV] */

s16	s16_DD_MTR_vdMax_10mV = (s16)((u32)(U16_ADC_REFERENCE_BATT_VOLT_10MV) * (u32)(U16_MTR_OVERMODULATION_COEF) >> (u32)11);	/* D軸電圧Vd最大値 物理量[10mV] */
s16	s16_DD_MTR_vqMax_10mV = (s16)((u32)(U16_ADC_REFERENCE_BATT_VOLT_10MV) * (u32)(U16_MTR_OVERMODULATION_COEF) >> (u32)11);	/* Q軸電圧Vq最大値 物理量[10mV] */
s16	s16_DD_MTR_vdMin_10mV = (s16)(-(u32)(U16_ADC_REFERENCE_BATT_VOLT_10MV) * (u32)(U16_MTR_OVERMODULATION_COEF) >> (u32)11);	/* D軸電圧Vd最大値 物理量[10mV] */
s32	s32_DD_MTR_imLim_10mA = (S32_MTR_IM_MAX_10MA);								/* q軸電流制限値 */
s32	s32_DD_MTR_idRefFinal_10mA = (s32)0;										/* 最終d軸電流指令値 */
s32	s32_DD_MTR_iqRefFinal_10mA = (s32)0;										/* 最終q軸電流指令値 */
s16	s16_DD_MTR_iqRefFinal_100mA = (s16)0;										/* 最終q軸電流指令値 */

s16	s16_DD_MTR_vd = (s16)0;														/* D軸電圧Vd タイマ値 */
s16	s16_DD_MTR_vq = (s16)0;														/* Q軸電圧Vq タイマ値 */
s16	s16_DD_MTR_vu = (s16)0;														/* U相電圧指令 （タイマ値）*/
s16	s16_DD_MTR_vv = (s16)0;														/* V相電圧指令 （タイマ値）*/
s16	s16_DD_MTR_vw = (s16)0;														/* W相電圧指令 （タイマ値）*/
s16	s16_DD_MTR_uCommand = (s16)0;												/* U相上アーム用指令値 */
s16	s16_DD_MTR_vCommand = (s16)0;												/* V相上アーム用指令値 */
s16	s16_DD_MTR_wCommand = (s16)0;												/* W相上アーム用指令値 */
s16	s16_DD_MTR_uCommandTemp = (s16)0;											/* U相上アーム用指令値(仮) */
s16	s16_DD_MTR_vCommandTemp = (s16)0;											/* V相上アーム用指令値(仮) */
s16	s16_DD_MTR_wCommandTemp = (s16)0;											/* W相上アーム用指令値(仮) */

volatile s16	s16_DD_MTR_battCurrentByShunt_10mA = (s16)0;						/* バッテリ電流 */
u16	u16_DD_MTR_battVolt_10mV = (u16)0;											/* バッテリ電圧[10mV] */
volatile u16	u16_DD_MTR_battVoltRaw_10mV = (u16)0;								/* 補正前バッテリ電圧[10mV] */
volatile u16	u16_DD_MTR_battVoltAve_10mV = (u16)0;								/* フィルタ後のバッテリ電圧[10mV] */
volatile u16	u16_DD_MTR_invVolt_10mV = (u16)0;									/* インバータ電圧[10mV] */

u16	u16_DD_MTR_encCyclePwm = (u16)0xFFFF;										/* エンコーダ周期PWM 最大値に設定 */
u16	u16_DD_MTR_encCyclePwmCounter = (u16)0xFFFF;								/* エンコーダ周期PWMカウンタ 最大値に設定 */
volatile u16	u16_DD_MTR_encCyclePwmCounterLast = (u16)0xFFFF;					/* エンコーダ周期PWMカウンタ前回値 */
u8	u8_DD_MTR_encPatternChecker[6] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};		/* 回転数計算用パターンチェッカー */

ST_ADC_ADVALUE0	st_DD_MTR_adValue0 = {(u16)0,(u16)0,(u16)0,(u16)0};				/* AD0データ */
ST_ADC_ADVALUE0*	pst_DD_MTR_adValue0 = &st_DD_MTR_adValue0;						/* AD0データの参照先 */

ST_ADC_DQ32	st_DD_MTR_dq32	= {(s32)0,(s32)0,(s32)0,(s32)0};					/* DQ軸データ */
ST_ADC_DQ32*	pst_DD_MTR_dq32	= &st_DD_MTR_dq32;									/* DQ軸データの参照先 */

ST_MTR_PHASE_CURRENT	st_DD_MTR_phaseCurrent	= {(s32)0,(s32)0,(s32)0};		/* 相電流データ */
////static ST_MTR_PHASE_CURRENT*	pst_DD_MTR_phaseCurrent	= &st_DD_MTR_phaseCurrent;		/* 相電流データの参照先 */
//
//ST_MTR_PWM_TIMER		st_DD_MTR_pwmTimer		= {(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0};	/* PWMタイマ値 */
////static ST_MTR_PWM_TIMER*	pst_DD_MTR_pwmTimer	= &st_DD_MTR_pwmTimer;					/* PWMタイマ値 */
//
//ST_MTR_CURRENT_PP	st_DD_MTR_currentPp		= {(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0};	/* 相電流P-P検出用データ */
////static ST_MTR_CURRENT_PP*	pst_DD_MTR_currentPp	= &st_DD_MTR_currentPp;				/* 相電流P-P検出用データの参照先 */

#pragma bit_order right
#pragma unpack

union
{
	u8			byte;
	struct
	{
		u8	b_abnormalOverCurrent		:1;			/* b0:過電流異常				(1:過電流) */
		u8	b_abnormalEncoder			:1;			/* b1:エンコーダ異常			(1:パターン未定義) */
		u8	b_abnormalUPhaseWireOpen	:1;			/* b2:U相モータ断線異常			(1:U相単相断線) */
		u8	b_abnormalVPhaseWireOpen	:1;			/* b3:V相モータ断線異常			(1:V相単相断線) */
		u8	b_abnormalWPhaseWireOpen	:1;			/* b4:W相モータ断線異常			(1:W相単相断線) */
		u8								:3;			/* b5-7:空き */
	}bit;
}uni_DD_MTR_abnormalFlg;

#else

static ENUM_DD_MTR_IMIN_PHASE en_DD_MTR_iMinPhase = ENUM_IMIN_PHASE_U;					/* モーター電流の最小値が検出された相 */
static enum {																			/* モータ相電流最大最小値探索 */
	ENUM_DD_MTR_PP_SEARCH_INIT = 0,														/* 開始 */
	ENUM_DD_MTR_PP_SEARCH_RUNNING														/* 実施中 */
} en_DD_MTR_ppSearchStatus = ENUM_DD_MTR_PP_SEARCH_INIT;								/* モーター電流pp値探索ステータス */
/* モータ断線検出用 */
static enum {
	ENUM_WIRE_OPEN_NORMAL = 0,															/* 正常 */
	ENUM_WIRE_OPEN_ABNORMAL																/* 断線確定 */
} en_DD_MTR_wireOpenEachStatus = ENUM_WIRE_OPEN_NORMAL;									/* モータ断線状態（UVW各々） */

/* 6Stepへの切り替わり判定用 */
static enum {																			/* 6Step状態 */
	ENUM_6STEP_STATE0 = 0,																/* 初回 */
	ENUM_6STEP_STATE1,																	/* 初回判定済み */
	ENUM_6STEP_STATE2																	/* 通常電気角補正積算値参照済み */
} en_DD_MTR_6StepStatus = ENUM_6STEP_STATE0;

static EN_MDC_DISCHARGE_MODE	en_DD_MTR_dischargeMode	= MD_DISCHARGE_INITIAL;			/* 現在ディスチャージモード */

static bool	b_DD_MTR_isStartAD0Convert = FALSE;											/* AD0変換開始フラグ */
static bool	b_DD_MTR_isDelayAD0Convert = FALSE;											/* AD0変換遅延状態フラグ */
static u8	u8_DD_MTR_ad0FinishedCounterMax = (u8)0;									/* AD0チャンネルAD変換終了カウンタ最大値 */

static s32	s32_DD_MTR_iUPp_10mA = (s32)0;												/* U相電流PP(TOP-BOTTOM)値 */
static s32	s32_DD_MTR_iVPp_10mA = (s32)0;												/* V相電流PP(TOP-BOTTOM)値 */
static s32	s32_DD_MTR_iWPp_10mA = (s32)0;												/* W相電流PP(TOP-BOTTOM)値 */
static s32	s32_DD_MTR_iUTopBuff_10mA = (s32)0;											/* U相電流TOP仮値 */
static s32	s32_DD_MTR_iVTopBuff_10mA = (s32)0;											/* V相電流TOP仮値 */
static s32	s32_DD_MTR_iWTopBuff_10mA = (s32)0;											/* W相電流TOP仮値 */
static s32	s32_DD_MTR_iUBtmBuff_10mA = (s32)0;											/* U相電流BOT仮値 */
static s32	s32_DD_MTR_iVBtmBuff_10mA = (s32)0;											/* V相電流BOT仮値 */
static s32	s32_DD_MTR_iWBtmBuff_10mA = (s32)0;											/* W相電流BOT仮値 */
static s32	s32_DD_MTR_ctPpMAX_10mA = (s32)0;											/* U,V,Wモータ電流の振幅の最大値 単位10mA */
static s32	s32_DD_MTR_ctPpMIN_10mA = (s32)0;											/* U,V,Wモータ電流の振幅の最小値 単位10mA */

static s16	s16_DD_MTR_iUTopBuff_100mA = (s16)0;										/* U相電流TOP仮値[100mA] */
static s16	s16_DD_MTR_iVTopBuff_100mA = (s16)0;										/* V相電流TOP仮値[100mA] */
static s16	s16_DD_MTR_iWTopBuff_100mA = (s16)0;										/* V相電流TOP仮値[100mA] */
static s16	s16_DD_MTR_iUBtmBuff_100mA = (s16)0;										/* U相電流BOT仮値[100mA] */
static s16	s16_DD_MTR_iVBtmBuff_100mA = (s16)0;										/* V相電流BOT仮値[100mA] */
static s16	s16_DD_MTR_iWBtmBuff_100mA = (s16)0;										/* V相電流BOT仮値[100mA] */

static s16	s16_DD_MTR_iU_100mA = (s16)0;												/* U相電流[100mA] */
static s16	s16_DD_MTR_iV_100mA = (s16)0;												/* V相電流[100mA] */
static s16	s16_DD_MTR_iW_100mA = (s16)0;												/* W相電流[100mA] */

static u8	u8_DD_MTR_encPhaseCountForPpDetection = (u8)1;								/* pp計算用位相カウンタ 初期値は1とする エンコーダパターン数が6なので、カウンタは1-6で数えることとする */
static u16	u16_DD_MTR_wireOpenJudgeCount_62p5us = (u16)0;								/* モータ断線の疑い有カウント(640ms) */

static u16	u16_DD_MTR_dAxisProportionalGain	= (U16_MTR_DAXIS_KP);					/* d軸比例ゲイン */
static u16	u16_DD_MTR_dAxisIntegralGain		= (U16_MTR_DAXIS_KI);					/* d軸積分ゲイン */
static u16	u16_DD_MTR_qAxisProportionalGain	= (U16_MTR_QAXIS_KP);					/* q軸比例ゲイン */
static u16	u16_DD_MTR_qAxisIntegralGain		= (U16_MTR_QAXIS_KI);					/* q軸積分ゲイン */
static u16	u16_DD_MTR_dPGainTemp = (U16_MTR_DAXIS_KP);									/* d軸比例ゲインtemp */
static u16	u16_DD_MTR_dIGainTemp = (U16_MTR_DAXIS_KI);									/* d軸積分ゲインtemp */
static u16	u16_DD_MTR_qPGainTemp = (U16_MTR_QAXIS_KP);									/* q軸比例ゲインtemp */
static u16	u16_DD_MTR_qIGainTemp = (U16_MTR_QAXIS_KI);									/* q軸積分ゲインtemp */
//static u16	u16_DD_MTR_dPGainForTest = (U16_MTR_DAXIS_KP);								/* d軸比例ゲインTEST用 */
//static u16	u16_DD_MTR_qPGainForTest = (U16_MTR_QAXIS_KP);								/* q軸比例ゲインTEST用 */
static bool	b_DD_MTR_isStartRequested = FALSE;											/* モータ駆動要求 */
static bool	b_DD_MTR_isPermitted = TRUE;												/* ゲートON許可 */
static bool	b_DD_MTR_isGateOn = FALSE;													/* ゲートON状態 */
static s32	s32_DD_MTR_integralVd_10mV = (s32)0;										/* d軸積分項上位32bit */
static s32	s32_DD_MTR_integralVq_10mV = (s32)0;										/* q軸積分項上位32bit */

static u8	u8_DD_MTR_encPattern = (u8)0;												/* 現在のエンコーダパターン出力(換算した結果）*/
static u8	u8_DD_MTR_encPatternLast = (u8)0;											/* 前回のエンコーダパターン */
static bool	b_DD_MTR_isEncReverseRotate = FALSE;										/* 現サイクル回転方向	0:正転	1:逆転 */
static bool	b_DD_MTR_isEncReverseRotateLast = FALSE;									/* 前サイクル回転方向	0:正転	1:逆転 */
static bool	b_DD_MTR_isMotorReverseRotate = FALSE;										/* モータ回転方向(エンコーダ正転/逆転カウンタ3回以上で確定)	0:正転	1:逆転 */
static u8	u8_DD_MTR_encReverseRotateCounter = (u8)0;									/* エンコーダ逆転カウンタ */
static u8	u8_DD_MTR_encNormalRotateCounter = (u8)0;									/* エンコーダ正転カウンタ */
static u32	u32_DD_MTR_encTimerValue60 = (u32)0;										/* 60deg間のタイマ値 form DD_ENCODER */
static u16	u16_DD_MTR_eangle_0p03125deg = (u16)0;										/* 推定電気角[0.03125deg] */
static u16	u16_DD_MTR_eangleMax_0p03125deg = (u16)0;									/* 最大推定電気角[0.03125deg] */
static volatile u16	u16_DD_MTR_eangleLast_deg = (u16)0;									/* 推定電気角前回値 */
static volatile u16	u16_DD_MTR_eangleLast_0p03125deg = (u16)0;							/* 推定電気角前回値 */
static volatile s16	s16_DD_MTR_eangleDiff_deg = (s16)0;									/* 電気角推定差分[deg] */
static volatile s16	s16_DD_MTR_eangleDiff_0p03125deg = (s16)0;							/* 電気角推定差分[0.03125deg] */
static u16	u16_DD_MTR_eangleBase_deg = (u16)0;											/* ベース電気角[deg] */
static u16	u16_DD_MTR_estimateEangleDelta = (u16)0;									/* 単位補正電気角[256×deg] */
static u16	u16_DD_MTR_estimateEangle_0p03125deg = (u16)0;								/* 補正電気角[0.03125deg] */
static u16	u16_DD_MTR_estimateEangleMax_0p03125deg = (u16)0;							/* 最大補正電気角[0.03125deg] */
static u16	u16_DD_MTR_estimateEangleSum = (u16)0;										/* 補正電気角の暫定値[256×deg](総和値) */
static u16	u16_DD_MTR_lowSpdEstimateEangleDelta = (u16)0;								/* 補正単位電気角(極遅速度時)[256×deg] */
static u16	u16_DD_MTR_lowSpdEstimateEangleSum = (u16)0;								/* 補正電気角(極遅速度時)[256×deg](総和値) */
static u16	u16_DD_MTR_eangleOffset_0p03125deg = (U16_MTR_EANGLE_OFFSET_0P03125DEG);	/* 電気角オフセット補正値[0.03125deg] */
static u16	u16_DD_MTR_eangleMapOffset_0p03125deg = (U16_MTR_EANGLE_OFFSET_0P03125DEG);	/* 電気角オフセット補正値[0.03125deg] */
static u16	u16_DD_MTR_eangleOffsetForTest_0p03125deg = (u16)0;							/* テスト用電気角オフセット補正値[0.03125deg] */
static u16	u16_DD_MTR_eangleOffsetForTest_deg = (u16)0;								/* テスト用電気角オフセット補正値[0.03125deg] */
static bool	b_DD_MTR_isLowSpdSetFinished = FALSE;										/* 極低速時補正角度の設定フラグ 1:設定完了 0:設定しない */
static bool	b_DD_MTR_isRotorVibration = FALSE;											/* ロータ振動フラグ */
static s8	s8_DD_MTR_rotorVibrationCounter = (s8)0;									/* ロータ振動判断カウンタ */
static bool	b_DD_MTR_overRegenerativeVoltage = FALSE;									/* 回生電圧Over 1：Over */
static bool	b_DD_MTR_detectWireOpenActivate = TRUE;										/* モータ単相断線異常有効化：default=有効 */
static bool	b_DD_MTR_detectEncoderActivate = TRUE;										/* エンコーダ異常有効化：default=有効 */

static s16	s16_DD_MTR_revolutionArray_rpm[(U8_MTR_NUM_ENCODER_PATTERNS)] =				/* モータ回転数6移動平均バッファ */
										{ (s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0, };

static u8	u8_DD_MTR_overCurrentJudgeCount	= (u8)0;									/* 過電流異常検出タイマ */
static u8	u8_DD_MTR_encAbnormalCount	= (u8)0;										/* エンコーダ異常判定カウンタ */
static s16	s16_DD_MTR_revolutionRaw_rpm = (s16)0;										/* 符号付モータ回転数生値[rpm] */
static s16	s16_DD_MTR_revolution_rpm = (s16)0;											/* 符号付モータ回転数[rpm] */
static s16	s16_DD_MTR_revolutionAve6_rpm = (s16)0;										/* 符号付モータ回転数[rpm] */
//static s16	s16_DD_MTR_vqBias_100mV = (s16)0;										/* 誘起電圧バイアス[0.1V] */

static s16	s16_DD_MTR_vdMax_10mV = (s16)((u32)(U16_ADC_REFERENCE_BATT_VOLT_10MV) * (u32)(U16_MTR_OVERMODULATION_COEF) >> (u32)11);	/* D軸電圧Vd最大値 物理量[10mV] */
static s16	s16_DD_MTR_vqMax_10mV = (s16)((u32)(U16_ADC_REFERENCE_BATT_VOLT_10MV) * (u32)(U16_MTR_OVERMODULATION_COEF) >> (u32)11);	/* Q軸電圧Vq最大値 物理量[10mV] */
static s16	s16_DD_MTR_vdMin_10mV = (s16)(-(u32)(U16_ADC_REFERENCE_BATT_VOLT_10MV) * (u32)(U16_MTR_OVERMODULATION_COEF) >> (u32)11);	/* D軸電圧Vd最大値 物理量[10mV] */
static s32	s32_DD_MTR_imLim_10mA = (S32_MTR_IM_MAX_10MA);								/* q軸電流制限値 */
static s32	s32_DD_MTR_idRefFinal_10mA = (s32)0;										/* 最終d軸電流指令値 */
static s32	s32_DD_MTR_iqRefFinal_10mA = (s32)0;										/* 最終q軸電流指令値 */
static s16	s16_DD_MTR_iqRefFinal_100mA = (s16)0;										/* 最終q軸電流指令値 */

static s16	s16_DD_MTR_vd = (s16)0;														/* D軸電圧Vd タイマ値 */
static s16	s16_DD_MTR_vq = (s16)0;														/* Q軸電圧Vq タイマ値 */
static s16	s16_DD_MTR_vu = (s16)0;														/* U相電圧指令 （タイマ値）*/
static s16	s16_DD_MTR_vv = (s16)0;														/* V相電圧指令 （タイマ値）*/
static s16	s16_DD_MTR_vw = (s16)0;														/* W相電圧指令 （タイマ値）*/
static s16	s16_DD_MTR_uCommand = (s16)0;												/* U相上アーム用指令値 */
static s16	s16_DD_MTR_vCommand = (s16)0;												/* V相上アーム用指令値 */
static s16	s16_DD_MTR_wCommand = (s16)0;												/* W相上アーム用指令値 */
static s16	s16_DD_MTR_uCommandTemp = (s16)0;											/* U相上アーム用指令値(仮) */
static s16	s16_DD_MTR_vCommandTemp = (s16)0;											/* V相上アーム用指令値(仮) */
static s16	s16_DD_MTR_wCommandTemp = (s16)0;											/* W相上アーム用指令値(仮) */

static volatile s16	s16_DD_MTR_battCurrentByShunt_10mA = (s16)0;						/* バッテリ電流 */
static u16	u16_DD_MTR_battVolt_10mV = (u16)0;											/* バッテリ電圧[10mV] */
static volatile u16	u16_DD_MTR_battVoltRaw_10mV = (u16)0;								/* 補正前バッテリ電圧[10mV] */
static volatile u16	u16_DD_MTR_battVoltAve_10mV = (u16)0;								/* フィルタ後のバッテリ電圧[10mV] */
static volatile u16	u16_DD_MTR_invVolt_10mV = (u16)0;									/* インバータ電圧[10mV] */

static u16	u16_DD_MTR_encCyclePwm = (u16)0xFFFF;										/* エンコーダ周期PWM 最大値に設定 */
static u16	u16_DD_MTR_encCyclePwmCounter = (u16)0xFFFF;								/* エンコーダ周期PWMカウンタ 最大値に設定 */
static volatile u16	u16_DD_MTR_encCyclePwmCounterLast = (u16)0xFFFF;					/* エンコーダ周期PWMカウンタ前回値 */
static u8	u8_DD_MTR_encPatternChecker[6] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};		/* 回転数計算用パターンチェッカー */

static ST_ADC_ADVALUE0	st_DD_MTR_adValue0 = {(u16)0,(u16)0,(u16)0,(u16)0};				/* AD0データ */
static ST_ADC_ADVALUE0*	pst_DD_MTR_adValue0 = &st_DD_MTR_adValue0;						/* AD0データの参照先 */

static ST_ADC_DQ32	st_DD_MTR_dq32	= {(s32)0,(s32)0,(s32)0,(s32)0};					/* DQ軸データ */
static ST_ADC_DQ32*	pst_DD_MTR_dq32	= &st_DD_MTR_dq32;									/* DQ軸データの参照先 */

static ST_MTR_PHASE_CURRENT	st_DD_MTR_phaseCurrent	= {(s32)0,(s32)0,(s32)0};			/* 相電流データ */
////static ST_MTR_PHASE_CURRENT*	pst_DD_MTR_phaseCurrent	= &st_DD_MTR_phaseCurrent;		/* 相電流データの参照先 */
//
//static ST_MTR_PWM_TIMER		st_DD_MTR_pwmTimer		= {(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0};	/* PWMタイマ値 */
////static ST_MTR_PWM_TIMER*	pst_DD_MTR_pwmTimer	= &st_DD_MTR_pwmTimer;					/* PWMタイマ値 */
//
//static ST_MTR_CURRENT_PP	st_DD_MTR_currentPp		= {(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0};	/* 相電流P-P検出用データ */
////static ST_MTR_CURRENT_PP*	pst_DD_MTR_currentPp	= &st_DD_MTR_currentPp;				/* 相電流P-P検出用データの参照先 */

#pragma bit_order right
#pragma unpack

static union
{
	u8			byte;
	struct
	{
		u8	b_abnormalOverCurrent		:1;			/* b0:過電流異常				(1:過電流) */
		u8	b_abnormalEncoder			:1;			/* b1:エンコーダ異常			(1:パターン未定義) */
		u8	b_abnormalUPhaseWireOpen	:1;			/* b2:U相モータ断線異常			(1:U相単相断線) */
		u8	b_abnormalVPhaseWireOpen	:1;			/* b3:V相モータ断線異常			(1:V相単相断線) */
		u8	b_abnormalWPhaseWireOpen	:1;			/* b4:W相モータ断線異常			(1:W相単相断線) */
		u8								:3;			/* b5-7:空き */
	}bit;
}uni_DD_MTR_abnormalFlg;

#endif

/* Private function prototypes -----------------------------------------------*/
static u16	u16_DD_MTR_EstimateEAngle(void);											/* 電気角推定 */
static bool b_DD_MTR_DetectEncPattern(void);											/* エンコーダパターン 及び 正逆転判断 */
static void v_DD_MTR_IsEncPatternChanged(void);											/* エンコーダパターン変化時の処理 */
static void v_DD_MTR_IsEncPatternNotChanged(void);										/* エンコーダパターン不変時の処理 */
static void v_DD_MTR_CheckRotorDirection(void);											/* ロータ回転方向判定 */
static void v_DD_MTR_CheckRotorVivration(void);											/* ロータ振動の判定 */
static void v_DD_MTR_EstimateEangleAtNormalSpd(void);									/* 通常速度で電気角度補正推定 */
static void v_DD_MTR_EstimateEangleAtLowSpd(void);										/* 極低速で電気角度補正推定 */
static s32	s32_DD_MTR_IdClarkeAndParkTransform(u16 a_u16_eangle_0p03125deg);			/* Id算出：Clarke変換＋Park変換(UVW → αβ → dq) */
static s32	s32_DD_MTR_IqClarkeAndParkTransform(u16 a_u16_eangle_0p03125deg);			/* Iq算出：Clarke変換＋Park変換(UVW → αβ → dq) */
static void v_DD_MTR_CurrentPIControl(void);											/* dq軸PI演算 */
static void v_DD_MTR_InverseParkAndClarkeTransform(u16 a_u16_eangle_0p03125deg);		/* 逆Park&逆Clarke変換(dq → αβ → UVW) */
static void v_DD_MTR_OverModulation(void);												/* 過変調処理（UVW中性点変換）& UVW指令値制限 */
#ifdef DEADTIME_COMPENSATION
static void v_DD_MTR_DeadtimeCompensation(void);										/* デッドタイム補償 */
#endif
static void	v_DD_MTR_ClipVoltageCommand(void);											/* UVW指令値制限 */
static void v_DD_MTR_MonitoringBattVolt(void);											/* バッテリ電圧監視 */
static void	v_DD_MTR_CalcMotorRevolution_rpm(void);										/* モータ回転数演算[rpm] */
static s16	s16_DD_MTR_CalcRev6Average(void);											/* モータ回転数6移動平均処理 */
static void	v_DD_MTR_RevolutionClear(void);												/* モータ回転数クリア */

static void v_DD_MTR_DetectOverCurrent(void);											/* モータ過電流異常の判断 */
static void v_DD_MTR_CalcCurrentPp(void);												/* 3相電流pp値計算 */
static void v_DD_MTR_DetectWireOpen(void);												/* モータ単相オープン異常判定 */
static void	v_DD_MTR_DaOutput(void);													/* DA出力設定 */

/**
 * @brief 電流制御 & AD取り込み処理
 * @detail モータの電流制御 & AD取り込みを行う(谷割込)
 * @note 電気角推定、AD取り込み、モータ電流単位変換、過電流判定、モータ単相断線判定
 * @param none
 * @return none
 */
void	INTRPT_DD_MTR_MotorControlValley(void)
{
	/* for DEBUG */
	PO_GPIO2_HI();

#pragma unpack

	if( (S12AD.ADCSR.BIT.ADST) == TRUE )
	{
		b_DD_MTR_isStartAD0Convert = TRUE;
	}

/*------------ モータ回転数取得 ------------*/
	v_DD_MTR_CalcMotorRevolution_rpm();

/*------------ 電気角推定 ------------*/
	u16_DD_MTR_eangleLast_0p03125deg = u16_DD_MTR_eangle_0p03125deg;									/* 推定電気角前回値を保存 for DEBUG */
	u16_DD_MTR_eangle_0p03125deg = u16_DD_MTR_EstimateEAngle();											/* 電気角推定 */

	u16_DD_MTR_eangleMax_0p03125deg = (u16)S32_CMF_GETMAX( (s32)u16_DD_MTR_eangleMax_0p03125deg, (s32)u16_DD_MTR_eangle_0p03125deg );

/*-----AD0変換結果取り込み処理-----*/
	u8	a_u8_adCounter = (u8)0;
	while ( (S12AD.ADCSR.BIT.ADST) == TRUE )
	{																									/* AD0ループスキャン完了してない */
		if( a_u8_adCounter < (U8_MTR_AD0_MAX_WAITING_COUNT) )
		{
			a_u8_adCounter++;																			/* 待ち */
			/* for DEBUG */
			u8_DD_MTR_ad0FinishedCounterMax = (u8)CMF_GETMAX(a_u8_adCounter,u8_DD_MTR_ad0FinishedCounterMax);
		}
		else
		{																								/* 最大待ち時間が超えると */
			b_DD_MTR_isDelayAD0Convert = TRUE;															/* AD変換遅延状態フラグON */
			break;
		}
	}

	if( (b_DD_MTR_isDelayAD0Convert == FALSE) && (b_DD_MTR_isStartAD0Convert == TRUE) )
	{
		/*----AD0取り込み----*/
		R_Config_S12AD0_Get_ValueResult(ADCHANNEL0,&st_DD_MTR_adValue0.u16_adValueCtU);					/* Ｕ相電流ＡＤ(生) */
		R_Config_S12AD0_Get_ValueResult(ADCHANNEL1,&st_DD_MTR_adValue0.u16_adValueCtV);					/* Ｖ相電流ＡＤ(生) */
		R_Config_S12AD0_Get_ValueResult(ADCHANNEL2,&st_DD_MTR_adValue0.u16_adValueBattV);				/* バッテリ電圧ＡＤ(生) */
		R_Config_S12AD0_Get_ValueResult(ADCHANNEL3,&st_DD_MTR_adValue0.u16_adValueInvV);				/* インバータ電流ＡＤ(生) */
		b_DD_MTR_isStartAD0Convert = FALSE;
	}

/*----モータ電流取り込み＆単位変換----*/
	/* オフセット取得 */
	ST_ADC_ADVALUE_AVE	a_st_adValueAve	=
	{(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_EX_IF_OFFSET_STANDARD_ADVALUE)};	/* AD値平均化データ */

	a_st_adValueAve = St_MTR_GetAdValueAve();															/* 電流AD値平均データ取得 */

/******** CT取付方法が逆の場合、こちらを使用 ********/
	/* CTの取付方向が逆である為、オフセット減算、ゲイン補正後に"-1"を掛ける */
	s16	a_s16_iU = (s16)(((s16)st_DD_MTR_adValue0.u16_adValueCtU - (s16)a_st_adValueAve.u16_adValueAveCtU) * (s16)(-1));
	s16	a_s16_iV = (s16)(((s16)st_DD_MTR_adValue0.u16_adValueCtV - (s16)a_st_adValueAve.u16_adValueAveCtV) * (s16)(-1));
	/* 電流(10mA)への単位変換 */
	st_DD_MTR_phaseCurrent.s32_iU_10mA = (s32)(((s32)a_s16_iU * (s32)(U32_MTR_CT_RANGE_MAX_10MA)) >> (s32)11);
	st_DD_MTR_phaseCurrent.s32_iV_10mA = (s32)(((s32)a_s16_iV * (s32)(U32_MTR_CT_RANGE_MAX_10MA)) >> (s32)11);
	st_DD_MTR_phaseCurrent.s32_iW_10mA = (s32)((s32)(-1) * (st_DD_MTR_phaseCurrent.s32_iU_10mA + st_DD_MTR_phaseCurrent.s32_iV_10mA));
	bool a_b_isDetectActivity = B_MDC_IsDetectActivity();

	if( a_b_isDetectActivity == TRUE )
	{
/*----モータ過電流異常の判断----*/
		v_DD_MTR_DetectOverCurrent();
/* モータ単相断線異常判定 */
		if( b_DD_MTR_detectWireOpenActivate == TRUE )
		{
			v_DD_MTR_CalcCurrentPp();																	/* 3相電流pp値計算（モータ単相断線異常判定準備）*/
			v_DD_MTR_DetectWireOpen();
		}
	}
#ifdef	DA_EANGLE
	v_DD_MTR_DaOutput();
#endif
	/* for DEBUG */
	PO_GPIO2_LO();
}


/**
 * @brief 電気角推定
 * @param none
 * @return a_u16_eangle
 */
static u16	u16_DD_MTR_EstimateEAngle(void)
{
	bool a_b_isDetectActivity = B_MDC_IsDetectActivity();

	if( b_DD_MTR_DetectEncPattern() == TRUE )															/* エンコーダパターンチェック */
	{																									/* パターンが異常なしの場合 */
		if( u8_DD_MTR_encPatternLast == u8_DD_MTR_encPattern )
		{																								/* エンコーダのパターン変化無し */
			v_DD_MTR_IsEncPatternNotChanged();															/* エンコーダパタン変化無しの処理へ */
		}
		else
		{																								/* エンコーダパターン変化した場合 */
			v_DD_MTR_IsEncPatternChanged();																/* エンコーダパタン変化の処理へ */
		}
	}
	else
	{																									/* パタン異常の場合 */
		if( (a_b_isDetectActivity == TRUE) &&															/* 異常判定許可及びモータ系異常なし */
			(uni_DD_MTR_abnormalFlg.byte == (u8)0) &&
			(b_DD_MTR_detectEncoderActivate == TRUE) )
		{
			if( u8_DD_MTR_encAbnormalCount <  (U8_ABNORMAL_ENC_COUNT) )
			{
				u8_DD_MTR_encAbnormalCount++;															/* エンコーダ異常判定カウンタを更新 */
			}
			else
			{																							/* エンコーダ異常として判断する */
				V_DD_EX_ForcedGateOff();																/* エキサイタ強制ゲートOFFを要求する */
				uni_DD_MTR_abnormalFlg.bit.b_abnormalEncoder = TRUE;									/* エンコーダ異常セット */
				u8_DD_MTR_encAbnormalCount = (u8)0;														/* エンコーダ異常判定カウンタ初期化 */
			}
		}
		v_DD_MTR_IsEncPatternNotChanged();																/* エンコーダパタン変化無しの処理へ */
	}

/*-------共通電気角度の計算----*/
	if( b_DD_MTR_isLowSpdSetFinished == TRUE )															/* 極低速の場合疑似120度通電 */
	{
		u16_DD_MTR_eangleOffset_0p03125deg = (U16_MTR_EANGLE_LOWSPD_OFFSET_0P03125DEG);					/* ENCエッジに対し60度遅角させ60度補正する */
	}
	else																								/* 極低速から抜けた場合，オフセットを戻す */
	{
		u16_DD_MTR_eangleOffset_0p03125deg = u16_DD_MTR_eangleMapOffset_0p03125deg;
	}

	/* 電気角度の計算 */
	u16 a_u16_eangle = (u16)( ( u16_DD_MTR_eangleBase_deg << (u16)5 ) + u16_DD_MTR_estimateEangle_0p03125deg + u16_DD_MTR_eangleOffset_0p03125deg + (u16_DD_MTR_eangleOffsetForTest_deg << (u16)5) );
	for( ;a_u16_eangle >= (U16_ENC_EANGLE_MAX_0P03125DEG); )
	{																									/* 電気角度は360度以上の場合 */
		a_u16_eangle = (u16)( a_u16_eangle - (U16_ENC_EANGLE_MAX_0P03125DEG) );							/* 電気角度処理 */
	}

	/* for DEBUG */
	u16_DD_MTR_estimateEangleMax_0p03125deg = (u16)S32_CMF_GETMAX( (s32)u16_DD_MTR_estimateEangleMax_0p03125deg, (s32)u16_DD_MTR_estimateEangle_0p03125deg );


	return	a_u16_eangle;
}


/**
 * @brief エンコーダパターン変化時の処理
 * @param none
 * @return none
 */
static void v_DD_MTR_IsEncPatternChanged(void)
{
	u8 *a_pu8;

	a_pu8 = u8_DD_MTR_encPatternChecker;																/* エンコーダパターンチェッカーへのポインタ */

	*(a_pu8 + (u8_DD_MTR_encPattern - (u8)1) ) = (u8)1;													/* 回転数計算用エンコーダパターンチェッカーの設定 */

	v_DD_MTR_CheckRotorDirection();																		/* ロータ回転方向判定 */
	v_DD_MTR_CheckRotorVivration();																		/* ロータ振動判定 */

/*----各データの設定-----*/
	u16_DD_MTR_encCyclePwmCounterLast	= u16_DD_MTR_encCyclePwm;										/* エンコーダ周期PWMカウンタ前回値を保存 60rpm = 95 */
	u16_DD_MTR_encCyclePwm				= u16_DD_MTR_encCyclePwmCounter;								/* エンコーダ周期PWMをコピー */

	u16_DD_MTR_encCyclePwmCounter		= (u16)1;														/* エンコーダ周期PWMカウンタ初期化 */

	/*----MTU0による60°毎の電気角推定(×256)-----*/
	u32_DD_MTR_encTimerValue60 = U32_DD_ENC_GetEncTimerValue();
	u16_DD_MTR_estimateEangleDelta = (u16)(u32)((U32_ENC_EANGLE60_TIMER_VALUE) / u32_DD_MTR_encTimerValue60);

/*-----極低速時の補正初期設定----------*/
	if( u16_DD_MTR_encCyclePwm >= (U16_MTR_MAX_PWM_TIMER_VALUE) )										/* 正弦波切換のENCタイマ値 ≒ 5rpm */
	{
		if( b_DD_MTR_isEncReverseRotate == FALSE )
		{													/** 正転の場合 **/
			u16_DD_MTR_lowSpdEstimateEangleSum = (u16)0;												/* 補正電気角(極遅速度時)をクリア */
			u16_DD_MTR_estimateEangle_0p03125deg = (u16)0;
		}
		else
		{													/** 逆転の場合 **/
			u16_DD_MTR_lowSpdEstimateEangleSum = (U16_MTR_EANGLE_ESTIMATE);								/* 補正電気角(極遅速度時)を上限値に設定 */
		}
		b_DD_MTR_isLowSpdSetFinished = TRUE;															/* 極低速で補正初期値設定完了フラグ */
	}
	else																								/* 初期状態又はオーバーフロー後エッジが2回来た場合 */
	{
		b_DD_MTR_isLowSpdSetFinished = FALSE;															/* 極低速で補正初期値設定フラグオフ */
	}

/*-----電気角の初期設定----------*/
	if( b_DD_MTR_isEncReverseRotate == FALSE )
	{														/** 正転時の初期化 **/
		u16_DD_MTR_estimateEangle_0p03125deg = (u16)( u16_DD_MTR_estimateEangleDelta >> (u16)3 );		/* 電気角度補正初期化 */
		u16_DD_MTR_estimateEangleSum = u16_DD_MTR_estimateEangleDelta;									/* 電気角度補正初期化(×8192) */
		u8_DD_MTR_encReverseRotateCounter = (u8)0;														/* エンコーダ逆転カウンタクリア */

		u8_DD_MTR_encNormalRotateCounter++;
		if( u8_DD_MTR_encNormalRotateCounter > (u8)0x7F )
		{
			u8_DD_MTR_encNormalRotateCounter = (u8)0x7F;												/* 0x7Fでクリップ */
		}

		if( u8_DD_MTR_encNormalRotateCounter >= (u8)3 )
		{
			b_DD_MTR_isMotorReverseRotate = FALSE;
		}
	}
	else
	{														/** 逆転時の初期化 **/
		u16_DD_MTR_estimateEangle_0p03125deg = (u16)( (u16)1920 -  (u16_DD_MTR_estimateEangleDelta >> (u16)3) );	/* 電気角度補正初期化 */
		u16_DD_MTR_estimateEangleSum = (u16)( (U16_MTR_EANGLE_ESTIMATE) - u16_DD_MTR_estimateEangleDelta );	/* 電気角度補正初期化(×8192) */
		u8_DD_MTR_encNormalRotateCounter = (u8)0;														/* エンコーダ正転カウンタクリア */

		u8_DD_MTR_encReverseRotateCounter++;															/* エンコーダ逆転カウンタ+1 */
		if( u8_DD_MTR_encReverseRotateCounter > (u8)0x7F )
		{
			u8_DD_MTR_encReverseRotateCounter = (u8)0x7F;												/* 0x7Fでクリップ */
		}

		if( u8_DD_MTR_encReverseRotateCounter >= (u8)3 )
		{
			b_DD_MTR_isMotorReverseRotate = TRUE;
		}
	}
}


/**
 * @brief ロータ回転方向判定
 * @param none
 * @return none
 */
static void v_DD_MTR_CheckRotorDirection(void)
{
	struct ST_ENC_PATTERN_TABLE const *a_pu16;

	a_pu16 = u16_DD_MTR_encPatternTable + (u16)u8_DD_MTR_encPattern;									/* エンコーダテーブルの現在のパターンへのポインタ */
	u16_DD_MTR_eangleBase_deg = a_pu16 -> u16_eangleBase;												/* 現在のベース電気角度を読込 */

/*-------正転逆転の判断-------*/
	if( u8_DD_MTR_encPatternLast == (u8)a_pu16 -> u16_encPatternNormalRotate )
	{																									/* 前回のエンコーダパターンは正転方向と同じ */
		b_DD_MTR_isEncReverseRotate = FALSE;															/* 逆転フラグクリア */
	}
	else if( u8_DD_MTR_encPatternLast == (u8)a_pu16 -> u16_encPatternReverseRotate )
	{																									/* 前回のエンコーダパターンは逆転方向と同じ */
		b_DD_MTR_isEncReverseRotate = TRUE;																/* 逆転フラグオン */
	}
	else
	{																									/* 前回のエンコーダパターンが一致しなくとも,回転方向判断を省略して処理を進める */
																										/* 処理無し */
																										/* MISRA-C 2004 14.10 対応 */
	}
}


/**
 * @brief ロータ振動判定
 * @param none
 * @return none
 */
static void v_DD_MTR_CheckRotorVivration(void)
{
/*------ロータ振動判定------------*/
	if( b_DD_MTR_isEncReverseRotateLast != b_DD_MTR_isEncReverseRotate )								/* 現在と前回の回転方向の比較 */
	{																									/* 回転方向変化あり */
		s8_DD_MTR_rotorVibrationCounter += (S8_MTR_VIBRATE_COUNTUP);									/* ロータ振動判定カウンタをUP */
		if( s8_DD_MTR_rotorVibrationCounter >= (S8_MTR_VIBRATE_COUNTMAX) )
		{
			s8_DD_MTR_rotorVibrationCounter = (S8_MTR_VIBRATE_COUNTMAX);								/* ロータ振動判定カウンタを最大値に設定 */
			b_DD_MTR_isRotorVibration = TRUE;															/* ロータ振動フラグをセット */
		}
	}
	else
	{																									/* 回転方向変化なし */
		s8_DD_MTR_rotorVibrationCounter -= (S8_MTR_VIBRATE_COUNTDOWN);									/* ロータ振動判定カウンタをDOWN */
		if( s8_DD_MTR_rotorVibrationCounter <= (s8)0 )
		{																								/* 最小値より小さい */
			s8_DD_MTR_rotorVibrationCounter = (s8)0;													/* ロータ振動判定カウンタを最小値に設定 */
			b_DD_MTR_isRotorVibration = FALSE;															/* ロータ振動フラグをクリア */
		}
	}
	b_DD_MTR_isEncReverseRotateLast = b_DD_MTR_isEncReverseRotate;										/* 現在回転方向の保存 */
}


/**
 * @brief エンコーダパターン不変時の処理
 * @param none
 * @return none
 */
static void v_DD_MTR_IsEncPatternNotChanged(void)
{
	u16_DD_MTR_encCyclePwmCounter++;																	/* エンコーダ周期PWMカウンタを+1 */

	if( u16_DD_MTR_encCyclePwmCounter >= (U16_MTR_MAX_ENC_COUNT) )										/* エンコーダ周期PWMカウンタを最大値と比較(推定可能周期の確認) */
	{																									/* 推定範囲を超えた場合⇒車両停止と判断 */
		u16_DD_MTR_encCyclePwmCounter = (U16_MTR_MAX_ENC_COUNT);										/* エンコーダ周期PWMカウンタを最大値で固定 */
		u16_DD_MTR_encCyclePwmCounterLast = (U16_MTR_MAX_ENC_COUNT);									/* エンコーダ周期PWMカウンタ前回値を最大値で固定 */
	}

	if( u16_DD_MTR_encCyclePwmCounter >= (U16_MTR_MAX_PWM_TIMER_VALUE) )								/* 正弦波切換のENCタイマ値 ≒ 5rpm */
	{
		b_DD_MTR_isLowSpdSetFinished = TRUE;
		if( en_DD_MTR_6StepStatus == ENUM_6STEP_STATE0 )												/* 初回? */
		{
			en_DD_MTR_6StepStatus = ENUM_6STEP_STATE1;													/* 初回判定済み */
		}
		else
		{
			en_DD_MTR_6StepStatus = ENUM_6STEP_STATE2;													/* 通常電気角補正積算参照済み */
		}
	}

	if ( b_DD_MTR_isLowSpdSetFinished == TRUE )															/* エンコーダタイマOVF且つ極低速の角度補正完了 */
	{																									/* 極低速の場合 */
		v_DD_MTR_EstimateEangleAtLowSpd();																/* 極低速度の電気角度補正の推定 */
	}
	else
	{
		v_DD_MTR_EstimateEangleAtNormalSpd();															/* 通常速度の電気角度補正の推定 */
	}
}


/**
 * @brief 通常速度時の電気角度補正推定
 * @param none
 * @return none
 */
static void v_DD_MTR_EstimateEangleAtNormalSpd(void)
{
	s16 a_s16_eangleSum = (s16)0;
	en_DD_MTR_6StepStatus = ENUM_6STEP_STATE0;															/* 初期化 */
/*-------電気角補正の計算---------*/
	if( b_DD_MTR_isEncReverseRotate == FALSE )
	{																					/** 正転の場合 **/
		a_s16_eangleSum = (s16)(u16)( u16_DD_MTR_estimateEangleSum + u16_DD_MTR_estimateEangleDelta );	/* 単位角を加算 */
		if ( a_s16_eangleSum > (s16)(U16_MTR_EANGLE_ESTIMATE) )											/* 補正角最大値（正転側）と比較 */
		{																								/* 補正角最大値を超えた場合 */
			u16_DD_MTR_estimateEangleSum = (U16_MTR_EANGLE_ESTIMATE);									/* 補正角を最大値に設定 */
		}
		else
		{																								/* 補正角最大値を超えてない場合 */
			u16_DD_MTR_estimateEangleSum = (u16)a_s16_eangleSum;										/* 計算した補正角値を採用する */
		}
	}
	else
	{																					/** 逆転の場合 **/
		a_s16_eangleSum = (s16)u16_DD_MTR_estimateEangleSum - (s16)u16_DD_MTR_estimateEangleDelta;		/* 単位角を減算 */
		if( a_s16_eangleSum < (s16)0 )																	/* 補正角最小値（逆転側）と比較 */
		{																								/* 補正角最小値より小さい場合 */
			u16_DD_MTR_estimateEangleSum = (u16)0;														/* 補正角を最小値に設定 */
		}
		else
		{
			u16_DD_MTR_estimateEangleSum = (u16)a_s16_eangleSum;										/* 計算した補正角値を採用する */
		}
	}

	u16_DD_MTR_estimateEangle_0p03125deg = (u16)( u16_DD_MTR_estimateEangleSum >> (u16)3 );				/* 補正電気角の単位変換 */
	/* 補正電気角の単位変換 */
	if( u16_DD_MTR_estimateEangle_0p03125deg > (U16_MTR_EANGLE_ESTIMATE_MAX_0P03125DEG) )
	{
		u16_DD_MTR_estimateEangle_0p03125deg = (U16_MTR_EANGLE_ESTIMATE_MAX_0P03125DEG);
	}
}


/**
 * @brief 極低速時の電気角度補正推定
 * @param none
 * @return none
 */
static void v_DD_MTR_EstimateEangleAtLowSpd(void)
{
	s16 a_s16_eangleSum = (s16)0;

	if( en_DD_MTR_6StepStatus == ENUM_6STEP_STATE1 )													/* 初回判定済み？ */
	{
		u16_DD_MTR_lowSpdEstimateEangleSum = u16_DD_MTR_estimateEangleSum;								/* 通常電気角補正積算を参照する */
		en_DD_MTR_6StepStatus = ENUM_6STEP_STATE2;
	}
	u16_DD_MTR_lowSpdEstimateEangleDelta = (u16)(u32)( (U32_ENC_EANGLE60_MAX) / (u32)u16_DD_MTR_encCyclePwmCounter );
/*------極低速の電気角度補正の計算------*/
	a_s16_eangleSum = (s16)( u16_DD_MTR_lowSpdEstimateEangleSum + u16_DD_MTR_lowSpdEstimateEangleDelta );	/* 補正電気角(極遅速度時)を加算 */

	if( a_s16_eangleSum > (s16)(U16_MTR_EANGLE_ESTIMATE) )												/* 電気角補正値の上限値(120*256 = U16_MTR_EANGLE_ESTIMATE[60deg]分)比較 */
	{
		u16_DD_MTR_lowSpdEstimateEangleSum = (U16_MTR_EANGLE_ESTIMATE);									/* 補正電気角(極遅速度時)を最大値に設定 */
	}
	else if(a_s16_eangleSum < (s16)0 )
	{
		u16_DD_MTR_lowSpdEstimateEangleSum = (s16)0;
	}
	else
	{
		u16_DD_MTR_lowSpdEstimateEangleSum = (u16)a_s16_eangleSum;										/* 計算した値を採用する */
	}

	s16 a_s16_eangleTemp_0p03125deg = (s16)( u16_DD_MTR_lowSpdEstimateEangleSum >> (u16)3 );			/* 補正電気角(極遅速度時)を単位換算(0～U16_MTR_EANGLE_ESTIMATE → 0～1920) */
	if( a_s16_eangleTemp_0p03125deg > (s16)(U16_MTR_EANGLE_ESTIMATE_MAX_0P03125DEG) )
	{
		u16_DD_MTR_estimateEangle_0p03125deg = (U16_MTR_EANGLE_ESTIMATE_MAX_0P03125DEG);				/* 補正電気角(極遅速度時)を最大値60度に設定 */
	}
	else if( a_s16_eangleTemp_0p03125deg < (s16)0 )
	{
		u16_DD_MTR_estimateEangle_0p03125deg = (u16)0;													/* 補正電気角(極遅速度時)を最小値0に設定 */
	}
	else
	{
		u16_DD_MTR_estimateEangle_0p03125deg = (u16)a_s16_eangleTemp_0p03125deg;						/* 計算した極低速の電気角補正角を偶数化 */
	}
}

/**
 * @brief モータ過電流異常判定
 * @param none
 * @return none
 */
static void v_DD_MTR_DetectOverCurrent(void)
{
/*----モータ過電流判定----*/

	s32	a_s32_overCurrentThreshold = (S32_MTR_OVER_CURRENT_THRESHOLD_10MA);
	bool a_b_isDetectActivity = B_MDC_IsDetectActivity();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( B_DET_MTR_ClearOverCurrent() == TRUE )
	{
		b_DD_MTR_isPermitted = TRUE;																	/* モータ駆動を許可する */
		uni_DD_MTR_abnormalFlg.bit.b_abnormalOverCurrent = FALSE;										/* 過電流異常をクリア */
		V_DD_EX_CanselForcedGateOff();																	/* エキサイタ強制ゲートオフを解除 */
		u8_DD_MTR_overCurrentJudgeCount = (u8)0;														/* 過電流異常検出タイマをリセット */
		return;
	}

	if( (a_b_isDetectActivity == FALSE) ||																/* 異常判定不許可又？ */
		(uni_DD_MTR_abnormalFlg.byte != (u8)0) )														/* 既に異常状態？ */
	{
		return;
	}

	s32	a_s32_iU_10mA = st_DD_MTR_phaseCurrent.s32_iU_10mA;
	s32	a_s32_iV_10mA = st_DD_MTR_phaseCurrent.s32_iV_10mA;
	s32	a_s32_iW_10mA = st_DD_MTR_phaseCurrent.s32_iW_10mA;

	if( (a_s32_iU_10mA >= a_s32_overCurrentThreshold) || (a_s32_iU_10mA <= (-a_s32_overCurrentThreshold)) ||
		(a_s32_iV_10mA >= a_s32_overCurrentThreshold) || (a_s32_iV_10mA <= (-a_s32_overCurrentThreshold)) ||
		(a_s32_iW_10mA >= a_s32_overCurrentThreshold) || (a_s32_iW_10mA <= (-a_s32_overCurrentThreshold)) )
	{
		if( u8_DD_MTR_overCurrentJudgeCount < (U8_MTR_OVERCURRENT_COUNT) )
		{
			u8_DD_MTR_overCurrentJudgeCount++;															/* 過電流異常検出タイマを更新 */
		}
		else
		{																								/* 判断閾値以上 */
			V_DD_EX_ForcedGateOff();																	/* エキサイタ強制ゲートOFFを要求する */
			uni_DD_MTR_abnormalFlg.bit.b_abnormalOverCurrent = TRUE;									/* 過電流異常セット */
			u8_DD_MTR_overCurrentJudgeCount = (u8)0;													/* 過電流異常検出タイマをリセット */
		}
	}
	else
	{
		u8_DD_MTR_overCurrentJudgeCount = (u8)0;														/* 過電流異常検出タイマをリセット */
	}
}

/**
 * @brief モータ回転数演算
 * @param none
 * @return none
 */
static void	v_DD_MTR_CalcMotorRevolution_rpm(void)
{
	s16 a_s16_revolution_rpm = (s16)0;
	u32 a_u32_enc60TimerValue = U32_DD_ENC_GetEncTimerValue();											/* 60deg間隔の値 */
	if( (a_u32_enc60TimerValue >= (U32_ENC_MAX_TIMER_VALUE)) ||											/* エンコーダタイマオーバーフロー */
		(u16_DD_MTR_encCyclePwm >= (U16_MTR_MAX_PWM_TIMER_VALUE)) )										/* 正弦波切換のENCタイマ値 ≒ 5rpm */

	{
		v_DD_MTR_RevolutionClear();																		/* 回転数をクリア */
		return;
	}

	u32 a_u32_encTimerValue = a_u32_enc60TimerValue * (u32)(U8_MTR_NUM_ENCODER_PATTERNS);				/* 360degに換算 */
	u8	a_u8_encPatternChekerSum = U8_DD_ENC_GetEncPatternCheckerSum();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if(B_DD_ENC_IsRequestCalcMotorRevolution() == FALSE)
	{																									/* 回転数演算要求あり */
		return;																							/* 回転数演算しない */
	}

	V_DD_ENC_ClearRequestCalcMotorRevolution();															/* 演算要求フラグをクリア */

	/* モータ回転数生値演算 */
	if( a_u32_encTimerValue == (u32)0 )
	{
		s16_DD_MTR_revolutionRaw_rpm = (s16)0;															/* 回転数をクリア */
		u16_DD_MTR_estimateEangleDelta = (u16)0;														/* 単位補正電気角をクリア */
		v_DD_MTR_RevolutionClear();																		/* 回転数をクリア */
	}
	else if( a_u32_encTimerValue < (u32)(U16_ENC_CYCLE_MIN_TIMER_VALUE) )
	{
		s16_DD_MTR_revolutionRaw_rpm = (s16)(U16_MTR_REVOLUTION_MAX_RPM) * (s16)2;						/* 最大回転数を設定 */
		u16_DD_MTR_estimateEangleDelta = (U16_ENC_EANGLE_ESTIMATE_UNIT_MAX_DEG);						/* 単位補正電気角を最大固定値に設定 */
	}
	else
	{
		s16_DD_MTR_revolutionRaw_rpm = (s16)(s32)( (s32)(U32_ENC_TIMER_RESOLUTON_60HZ) / (s32)((s32)(U16_MTR_POLEPAIRS) * (s32)a_u32_encTimerValue) );
	}
	s16_DD_MTR_revolutionAve6_rpm = s16_DD_MTR_CalcRev6Average();										/* モータ回転数の6移動平均を算出 */

	/* 1周期間のモータ回転数演算 */
	if( U8_DD_ENC_GetEncPatternCheckerSum() == (U8_MTR_NUM_ENCODER_PATTERNS) )							/* 6パターン揃う */
	{																									/* 電気角度の補正を行う */
		if( a_u32_encTimerValue < (u32)(U16_ENC_CYCLE_MIN_TIMER_VALUE) )
		{
			s16_DD_MTR_revolution_rpm = (s16)(U16_MTR_REVOLUTION_MAX_RPM) * (s16)2;						/* 最大回転数を設定 */
			u16_DD_MTR_estimateEangleDelta = (U16_ENC_EANGLE_ESTIMATE_UNIT_MAX_DEG);					/* 単位補正電気角を最大固定値に設定 */
		}
		else
		{
			a_s16_revolution_rpm = (s16)(s32)( (s32)(U32_ENC_TIMER_RESOLUTON_60HZ) / (s32)((s32)(U16_MTR_POLEPAIRS) * (s32)a_u32_encTimerValue) );
			if (b_DD_MTR_isMotorReverseRotate == FALSE)
			{
				s16_DD_MTR_revolution_rpm = (a_s16_revolution_rpm + s16_DD_MTR_revolution_rpm) >> (s16)1;	/* 移動平均追加 */
			}
			else
			{
				s16_DD_MTR_revolution_rpm = (-a_s16_revolution_rpm + s16_DD_MTR_revolution_rpm ) >> (s16)1;	/* 逆転時はマイナス */
			}
		}
	}
}


/*******************************************************************************
	モータ回転数の6移動平均を算出
*******************************************************************************/
static s16	s16_DD_MTR_CalcRev6Average(void)
{
	if (b_DD_MTR_isMotorReverseRotate == TRUE)
	{
		s16_DD_MTR_revolutionRaw_rpm = -s16_DD_MTR_revolutionRaw_rpm;
	}
	s16_DD_MTR_revolutionArray_rpm[0] = s16_DD_MTR_revolutionArray_rpm[1];
	s16_DD_MTR_revolutionArray_rpm[1] = s16_DD_MTR_revolutionArray_rpm[2];
	s16_DD_MTR_revolutionArray_rpm[2] = s16_DD_MTR_revolutionArray_rpm[3];
	s16_DD_MTR_revolutionArray_rpm[3] = s16_DD_MTR_revolutionArray_rpm[4];
	s16_DD_MTR_revolutionArray_rpm[4] = s16_DD_MTR_revolutionArray_rpm[5];
	s16_DD_MTR_revolutionArray_rpm[5] = s16_DD_MTR_revolutionRaw_rpm;

	s32 a_s32_sum_revolution;
	a_s32_sum_revolution =	s16_DD_MTR_revolutionArray_rpm[0] +
							s16_DD_MTR_revolutionArray_rpm[1] +
							s16_DD_MTR_revolutionArray_rpm[2] +
							s16_DD_MTR_revolutionArray_rpm[3] +
							s16_DD_MTR_revolutionArray_rpm[4] +
							s16_DD_MTR_revolutionArray_rpm[5];

	return (s16)(a_s32_sum_revolution / (s32)(U8_MTR_NUM_ENCODER_PATTERNS));
}


/**
 * @brief モータ回転数クリア
 * @param none
 * @return none
 */
static void	v_DD_MTR_RevolutionClear(void)
{
	s16_DD_MTR_revolutionArray_rpm[0] = (s16)0;
	s16_DD_MTR_revolutionArray_rpm[1] = (s16)0;
	s16_DD_MTR_revolutionArray_rpm[2] = (s16)0;
	s16_DD_MTR_revolutionArray_rpm[3] = (s16)0;
	s16_DD_MTR_revolutionArray_rpm[4] = (s16)0;
	s16_DD_MTR_revolutionArray_rpm[5] = (s16)0;
	s16_DD_MTR_revolutionAve6_rpm = (s16)0;
	s16_DD_MTR_revolutionRaw_rpm = (s16)0;
	s16_DD_MTR_revolution_rpm = (s16)0;
}

/**
 * @brief 電流指令値の設定
 * @detail ベクトル演算(山割込)
 * @note ゲートON/OFF判断、ベクトル演算、モータ回転数演算、バッテリ電圧単位変換
 * @param none
 * @return none
 */
void INTRPT_DD_MTR_MotorControlPeak(void)
{
	/* for DEBUG */
	PO_GPIO2_HI();

	en_DD_MTR_dischargeMode = En_MDC_GetDischargeMode();												/* ディスチャージモード取得 */

/*------------ バッテリ電圧監視 ------------*/
	v_DD_MTR_MonitoringBattVolt();																		/* バッテリ電圧監視 */

/*------------  Clarke変換＋Park変換 ------------ */
	st_DD_MTR_dq32.s32_idFb_10mA = s32_DD_MTR_IdClarkeAndParkTransform(u16_DD_MTR_eangle_0p03125deg);	/* Clarke変換＋Park変換 */
	st_DD_MTR_dq32.s32_iqFb_10mA = s32_DD_MTR_IqClarkeAndParkTransform(u16_DD_MTR_eangle_0p03125deg);	/* Clarke変換＋Park変換 */

	/* Vdmax計算 */
	s16_DD_MTR_vdMax_10mV = (s16)((u32)(U16_MTR_OVERMODULATION_COEF) * (u32)u16_DD_MTR_battVolt_10mV >> (u32)11 );	/* 電源電圧よりVd最大制限を求める	過変調を考慮 */
	s16_DD_MTR_vdMin_10mV = -s16_DD_MTR_vdMax_10mV;

/*-----ゲートオフ指示判定-----*/
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( b_DD_MTR_overRegenerativeVoltage == TRUE )														/* 回生中の過電圧判定 */
	{
//		V_DD_EX_StopControl();																			/* 界磁停止指示 */
		V_DD_EX_ExecuteGateOff();
		V_DD_MTR_ExecuteGateOff();																		/* モータGateOffを実施する */
		if( b_DD_MTR_isPermitted == FALSE )
		{
			/* PremitがFALSEなら復帰不可なので全リレーをOFF */
//			PRECHA_FET_OFF();																			/* 各FET OFF */
	//		MAIN_RELAY_OFF();
		}
		return;
	}

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( ((UNI_GL_testFlg.bit.b_forcedGateOff == TRUE) && (B_CHT_IsAccelSwOn() == FALSE)	) ||
		(uni_DD_MTR_abnormalFlg.bit.b_abnormalOverCurrent == TRUE) )
	{
		V_DD_EX_ExecuteGateOff();
		V_DD_MTR_ExecuteGateOff();																		/* モータGateOffを実施する */
		return;
	}

	if( (b_DD_MTR_isStartRequested == TRUE) && (b_DD_MTR_isPermitted == TRUE) )
	{																								  	/* ゲートオン状態 */
		if( b_DD_MTR_isGateOn == FALSE )
		{
			R_Config_MTU3_MTU4_StartTimerCtrl();
			b_DD_MTR_isGateOn = TRUE;																	/* ゲートON状態 */
		}
		v_DD_MTR_CurrentPIControl();																	/* dq軸PI演算 */
		v_DD_MTR_InverseParkAndClarkeTransform(u16_DD_MTR_eangle_0p03125deg);							/* 逆Park&逆Clarke変換(dq → αβ → UVW)  */
		v_DD_MTR_OverModulation();																		/* 過変調処理（UVW中性点変換）*/
	/*--- デッドタイム補償 ---*/
#ifdef DEADTIME_COMPENSATION
		v_DD_MTR_DeadtimeCompensation();																/* デッドタイム補償 */
#endif
		v_DD_MTR_ClipVoltageCommand();																	/* UVW指令値制限 */

		/* PWMデューティのタイマセット */
		R_Config_MTU3_MTU4_UpdDuty((u16)s16_DD_MTR_uCommand, (u16)s16_DD_MTR_vCommand, (u16)s16_DD_MTR_wCommand);
	}
	else
	{																									/* 遮断状態 */
		if( (en_DD_MTR_dischargeMode == MD_DISCHARGE_RUN) ||											/* ディスチャージ通電 */
			(en_DD_MTR_dischargeMode == MD_DISCHARGE_PREPARING_FINISH) )								/* ディスチャージ終了準備 */
		{
		}
		else
		{
//			V_DD_EX_StopControl();																		/* 界磁停止指示 */
			V_DD_EX_ExecuteGateOff();
		}
		V_DD_MTR_ExecuteGateOff();																		/* モータGateOffを実施する */
		if( b_DD_MTR_isPermitted == FALSE )
		{
			/* PremitがFALSEなら復帰不可なので全リレーをOFF */
//			PRECHA_FET_OFF();																			/* 各FET OFF */
	//		MAIN_RELAY_OFF();
		}
	}

	/* for DEBUG */
	PO_GPIO2_LO();
}


/**
 * @brief GateOffする
 * @param none
 * @return none
 */
void	V_DD_MTR_ExecuteGateOff(void)																	/* モータGateOffを実施する */
{
/* for DEBUG */
//	PO_GPIO3_HI();
	R_Config_MTU3_MTU4_StopTimerCtrl();			/*?*/

	/** 積分項クリア **/
	s32_DD_MTR_integralVq_10mV = (s32)0;																/* Vq積分項クリア */
	s32_DD_MTR_integralVd_10mV = (s32)0;																/* Vd積分項クリア */

	s16_DD_MTR_uCommand = (s16)(U16_MTR_PWM_NEUTRAL_POINT);												/* 各相デューティを中性点に設定 */
	s16_DD_MTR_vCommand = (s16)(U16_MTR_PWM_NEUTRAL_POINT);
	s16_DD_MTR_wCommand = (s16)(U16_MTR_PWM_NEUTRAL_POINT);

	/* PWMデューティのタイマセット */
	R_Config_MTU3_MTU4_UpdDuty((u16)s16_DD_MTR_uCommand, (u16)s16_DD_MTR_vCommand, (u16)s16_DD_MTR_wCommand);

	b_DD_MTR_isGateOn = FALSE;																			/* ゲートOFF状態 */

/* for DEBUG */
//	PO_GPIO3_LO();
}




/**
 * @brief Id Clarke & Park Transform
 * @param a_u16_eangle_0p03125deg
 * @return a_s32_idTmp
 */
/*-----Id Clarke & Park Transform-----*/
static	s32	s32_DD_MTR_IdClarkeAndParkTransform(u16 a_u16_eangle_0p03125deg)
{
	s32 a_s32_idTerm1 = (s32)( st_DD_MTR_phaseCurrent.s32_iU_10mA * (s32)TRIG_CALC_DQ_TRANSFORM_A(a_u16_eangle_0p03125deg) );	/* idTerm1 = -2/√3×sin(θ+120)*Iu */
	s32 a_s32_idTerm2 = (s32)( st_DD_MTR_phaseCurrent.s32_iW_10mA * (s32)TRIG_CALC_DQ_TRANSFORM_B(a_u16_eangle_0p03125deg) );	/* idTerm2 = 2/√3×sin(θ)*Iw */
	s32	a_s32_idTmp = (s32)( ((s64)a_s32_idTerm1 + (s64)a_s32_idTerm2) >> (s64)14 );						/* Id = -2/√3×sin(θ+120)*Iu + 2/√3×sin(θ)*Iw */

	return	a_s32_idTmp;
}


/**
 * @brief Iq Clarke & Park Transform
 * @param a_u16_eangle_0p03125deg
 * @return a_s32_iqTmp
 */
/*-----Iq Clarke & Park Transform-----*/
static	s32	s32_DD_MTR_IqClarkeAndParkTransform(u16 a_u16_eangle_0p03125deg)
{
	s32 a_s32_iqTerm1 = (s32)( st_DD_MTR_phaseCurrent.s32_iU_10mA * (s32)TRIG_CALC_DQ_TRANSFORM_C(a_u16_eangle_0p03125deg) );	/* iqTerm1= -2/√3×cos(θ+120)*Iu */
	s32 a_s32_iqTerm2 = (s32)( st_DD_MTR_phaseCurrent.s32_iW_10mA * (s32)TRIG_CALC_DQ_TRANSFORM_D(a_u16_eangle_0p03125deg) );	/* iqTerm2= 2/√3×cos(θ)*Iw */
	s32	a_s32_iqTmp = (s32)( ((s64)a_s32_iqTerm1 + (s64)a_s32_iqTerm2) >> (s64)14 );					/* Iq = -2/√3×cos(θ+120)*Iu + 2/√3×cos(θ)*Iw */

	return	a_s32_iqTmp;
}

/**
 * @brief dq軸PI演算
 * @param none
 * @return none
 */
static void v_DD_MTR_CurrentPIControl(void)
{
/*-----ImLimitよりidRefリミット-----*/	/* D軸なので-1を掛ける */
	s32 a_s32_idRefTemp_10mA = (s32)0;
	if( st_DD_MTR_dq32.s32_idRef_10mA >= (s32)0 )
	{
		a_s32_idRefTemp_10mA = s32_DD_MTR_imLim_10mA;
		if( st_DD_MTR_dq32.s32_idRef_10mA <= a_s32_idRefTemp_10mA )
		{
			a_s32_idRefTemp_10mA = st_DD_MTR_dq32.s32_idRef_10mA;
		}
	}
	else
	{
		a_s32_idRefTemp_10mA = (s32)(-1) * s32_DD_MTR_imLim_10mA;
		if( st_DD_MTR_dq32.s32_idRef_10mA >= a_s32_idRefTemp_10mA )
		{
			a_s32_idRefTemp_10mA = st_DD_MTR_dq32.s32_idRef_10mA;
		}
	}

	s32_DD_MTR_idRefFinal_10mA = a_s32_idRefTemp_10mA;
/*-----d軸PI演算-----*/
	u16_DD_MTR_dAxisIntegralGain		= u16_DD_MTR_dIGainTemp;										/* d軸積分ゲインをセット */

	/***** d軸ゲインテスト用 *****/
	if( UNI_GL_testFlg.bit.b_idqGainOld == TRUE )
	{
		u16_DD_MTR_dAxisProportionalGain	= (U16_MTR_DAXIS_KP2);										/* 旧d軸電流比例ゲインセット */
		u16_DD_MTR_dAxisIntegralGain		= (U16_MTR_DAXIS_KI2);										/* 旧d軸電流積分ゲインセット */
	}
	else
	{
		u16_DD_MTR_dAxisProportionalGain	= (U16_MTR_DAXIS_KP);										/* 新d軸電流比例ゲインセット */
		u16_DD_MTR_dAxisIntegralGain		= (U16_MTR_DAXIS_KI);										/* 新d軸電流積分ゲインセット */
	}

	/***** d軸ゲインテスト用 *****/
	if( UNI_GL_testFlg.bit.b_dqGainTestActivity == TRUE )
	{
		u16_DD_MTR_dAxisProportionalGain	= U16_GL_dPGainForTest;
		u16_DD_MTR_dAxisIntegralGain		= U16_GL_dIGainForTest;
	}

	/* d軸可変ゲイン */
	if( UNI_GL_testFlg.bit.b_variableGainTestActivity == TRUE )
	{
		u16_DD_MTR_dPGainTemp = U16_MTR_GetDAxisPropoGain();											/* 2DマップにてIfRefよりd軸KPを算出 */
		u16_DD_MTR_dAxisProportionalGain	= u16_DD_MTR_dPGainTemp;									/* d軸比例ゲインをセット */
	}

	s32	a_s32_idDiff	= a_s32_idRefTemp_10mA - st_DD_MTR_dq32.s32_idFb_10mA;							/* d軸の偏差を取得 */
	s32	a_s32_propoVd	= (s32)((a_s32_idDiff * (s32)u16_DD_MTR_dAxisProportionalGain) >> (s32)8);		/* d軸偏差の比例項を計算する → 16ビットへキャスト */
	s16	a_s16_propoVd	= (s16)S32_CMF_GETLIMIT( a_s32_propoVd, (s32)(-s16_DD_MTR_vdMax_10mV), (s32)s16_DD_MTR_vdMax_10mV );	/* 最大値と最小値にクリップ */
	s32	a_s32_integVd	= (s32)(a_s32_idDiff * (s32)u16_DD_MTR_dAxisIntegralGain + (s32)s32_DD_MTR_integralVd_10mV);	/* d軸積分項の計算(32ビット）→ 16ビットへキャスト */
	s16	a_s16_integVd	= (s16)(a_s32_integVd >> (s32)16);												/* d軸積分項の計算(32ビット）→ 16ビット右シフト */

	S32_GL_idDiff = a_s32_idDiff;

/*----d軸積分項の制限----*/
	if( a_s16_integVd > s16_DD_MTR_vdMax_10mV )															/* 積分項を閾値と比較 */
	{														/* 最大値を超えた場合 */
		a_s16_integVd = s16_DD_MTR_vdMax_10mV;															/* 積分項を最大値に設定する */
		s32_DD_MTR_integralVd_10mV = (s32)((s32)s16_DD_MTR_vdMax_10mV << (s32)16);						/* 積分項32ビット保存 */
	}
	else if ( a_s16_integVd < (-s16_DD_MTR_vdMax_10mV) )
	{														/* 最小値より小さい場合 */
		a_s16_integVd = (-s16_DD_MTR_vdMax_10mV);														/* 積分項を最小値に設定する */
		s32_DD_MTR_integralVd_10mV = (s32)((s32)(-s16_DD_MTR_vdMax_10mV) << (s32)16);					/* 積分項32ビット保存 */
	}
	else
	{														/* 制限値の間にいる */
		s32_DD_MTR_integralVd_10mV = a_s32_integVd;														/* 現在の積分項を保存 */
	}

/*----d軸比例項+積分項の制限-----*/
	s16	a_s16_dAxisPI = ( a_s16_propoVd + a_s16_integVd );												/* 比例項＋積分項項*/

	S16_GL_propoVd = a_s16_propoVd;
	S16_GL_integVd = a_s16_integVd;
/*----Vdを制限する----*/
	s16 a_s16_vqMaxTemp = (s16)0;

	if( a_s16_dAxisPI > s16_DD_MTR_vdMax_10mV )															/* 最大値と比較する */
	{														/** 最大値より大きい場合 **/
		st_DD_MTR_dq32.s32_vd_10mV = (s32)s16_DD_MTR_vdMax_10mV;										/* Vdを最大値に設定 */
	}
	else if ( a_s16_dAxisPI < s16_DD_MTR_vdMin_10mV )													/* 最小値と比較 */
	{														/** 最小値より小さい場合 **/
		st_DD_MTR_dq32.s32_vd_10mV = (s32)( s16_DD_MTR_vdMin_10mV );									/* Vdを最小値に設定する */
	}
	else
	{														/** 制限値の間にある場合 **/
		st_DD_MTR_dq32.s32_vd_10mV = (s32)a_s16_dAxisPI;												/* Vdを計算値を採用する */
	}

	u32	a_u32_vdTemp = U32_CMF_GETABS( st_DD_MTR_dq32.s32_vd_10mV );									/* 絶対値を取る */

	u16 a_u16_vdTab = (u16)(u32)( (s32)a_u32_vdTemp * (s32)(U16_ADC_REFERENCE_BATT_VOLT_10MV) / (s32)u16_DD_MTR_battVolt_10mV >> (s32)1 );	/* テーブルの位置へ換算する */
	a_u16_vdTab = (u16)S32_CMF_GETMIN( (s32)a_u16_vdTab, (s32)(U16_VQLIM_TABLE_SIZE_NUM) );				/* テーブル最大数でクリップ */
	U16_GL_vqTable = a_u16_vdTab;
	a_s16_vqMaxTemp = (s16)((s32)u16_TAB_vqLim[a_u16_vdTab] * (s32)u16_DD_MTR_battVolt_10mV / (s32)(U16_ADC_REFERENCE_BATT_VOLT_10MV));	/* Vq最大制限を求める */
	s16_DD_MTR_vqMax_10mV = a_s16_vqMaxTemp;

/*-----iqRefリミット-----*/
	a_s32_idRefTemp_10mA = (s32)U32_CMF_GETABS( a_s32_idRefTemp_10mA );									/* 絶対値を取る */
	u16 a_u16_iqTab = (u16)(a_s32_idRefTemp_10mA >> (s32)3);											/* テーブルの位置へ換算する */
	a_u16_iqTab = (u16)S32_CMF_GETMIN( (s32)a_u16_iqTab, (s32)(U16_IMLIM_TABLE_SIZE_NUM) );				/* テーブル最大数でクリップ */
	u32 a_u32_iqLimTemp	= (u32)u16_TAB_imLim[a_u16_iqTab];
	s32 a_s32_iqLimTemp_10mA = (s32)(a_u32_iqLimTemp << (u32)1);										/* 10mAへ単位変換 */
//	s32 a_s32_iqLimTemp_10mA = (s32)((s32)u16_TAB_imLim[a_u16_iqTab] << (s32)1);						/* 10mAへ単位変換 */

	s32 a_s32_iqRefTemp_10mA = (S32_MTR_IQ_MAX_10MA);													/* Iq指令最大値[0.1A] */

	if( UNI_GL_testFlg.bit.b_currentLimitActivity == TRUE )
	{	
		a_s32_iqRefTemp_10mA = (S32_MTR_IQ_MAX2_10MA);													/* Iq指令最大値[0.1A] */
	}

	if( s32_DD_MTR_imLim_10mA <= a_s32_iqRefTemp_10mA )
	{
		a_s32_iqRefTemp_10mA = s32_DD_MTR_imLim_10mA;
	}

	if( a_s32_iqLimTemp_10mA <= a_s32_iqRefTemp_10mA )													/* 制限テーブルとの比較 */
	{
		a_s32_iqRefTemp_10mA = a_s32_iqLimTemp_10mA;
	}

	if( st_DD_MTR_dq32.s32_iqRef_10mA <= a_s32_iqRefTemp_10mA )											/* iq指令値との比較 '20.6.30修正 */
	{
		a_s32_iqRefTemp_10mA = st_DD_MTR_dq32.s32_iqRef_10mA;
	}
	s32_DD_MTR_iqRefFinal_10mA = a_s32_iqRefTemp_10mA;
	s16_DD_MTR_iqRefFinal_100mA = (s16)(s32_DD_MTR_iqRefFinal_10mA / (s32)10);

/*-----q軸PI演算-----*/
	u16_DD_MTR_qAxisIntegralGain		= u16_DD_MTR_qIGainTemp;										/* q軸積分ゲインをセット */
	/***** q軸ゲインテスト用 *****/
	if( UNI_GL_testFlg.bit.b_idqGainOld == TRUE )
	{
		u16_DD_MTR_qAxisProportionalGain	= (U16_MTR_QAXIS_KP2);										/* 旧q軸電流比例ゲインセット */
		u16_DD_MTR_qAxisIntegralGain		= (U16_MTR_QAXIS_KI2);										/* 旧q軸電流積分ゲインセット */
	}
	else
	{
		u16_DD_MTR_qAxisProportionalGain	= (U16_MTR_QAXIS_KP); 										/* 新q軸電流比例ゲインセット */
		u16_DD_MTR_qAxisIntegralGain		= (U16_MTR_QAXIS_KI); 										/* 新q軸電流積分ゲインセット */
	}

	/***** q軸ゲインテスト用 *****/
	if( UNI_GL_testFlg.bit.b_dqGainTestActivity == TRUE )
	{
		u16_DD_MTR_qAxisProportionalGain	= U16_GL_qPGainForTest;
		u16_DD_MTR_qAxisIntegralGain		= U16_GL_qIGainForTest;
	}

	/* q軸可変ゲイン */
	if( UNI_GL_testFlg.bit.b_variableGainTestActivity == TRUE )
	{
		u16_DD_MTR_qPGainTemp = U16_MTR_GetQAxisPropoGain();											/* 2DマップにてIfRefよりq軸KPを算出 */
		u16_DD_MTR_qAxisProportionalGain	= u16_DD_MTR_qPGainTemp;									/* q軸比例ゲインをセット */
	}

	s32 a_s32_iqDiff	= a_s32_iqRefTemp_10mA - st_DD_MTR_dq32.s32_iqFb_10mA;							/* q軸の偏差を取得 */
	s32 a_s32_propoVq	= (s32)((a_s32_iqDiff * (s32)u16_DD_MTR_qAxisProportionalGain) >> (s32)8);		/* q軸偏差の比例項を計算する → 16ビットへキャスト */
	s16	a_s16_propoVq	= (s16)S32_CMF_GETLIMIT( a_s32_propoVq, (s32)(-s16_DD_MTR_vqMax_10mV), (s32)s16_DD_MTR_vqMax_10mV );	/* 最大値と最小値にクリップ */
	s32 a_s32_integVq	= a_s32_iqDiff * (s32)u16_DD_MTR_qAxisIntegralGain + s32_DD_MTR_integralVq_10mV;	/* q軸積分項の計算(32ビット）*/
	s16 a_s16_integVq	= (s16)(s32)( a_s32_integVq >> (s32)16 );										/* q軸積分項の計算(32ビット）→ 16ビット右シフト */

	S32_GL_iqDiff = a_s32_iqDiff;
/*----q軸積分項の制限----*/
	if( a_s16_integVq > a_s16_vqMaxTemp )																/* 積分項を閾値と比較 */
	{																									/* 最大値を超えた場合 */
		a_s16_integVq = a_s16_vqMaxTemp;																/* 積分項を最大値に設定する */
		s32_DD_MTR_integralVq_10mV = (s32)((s32)s16_DD_MTR_vqMax_10mV << (s32)16);						/* 積分項32ビット保存 (前16ビット：積分項の上位、後16ビット：積分項の下位ゼロ）*/
	}
	else if ( a_s16_integVq < (-s16_DD_MTR_vqMax_10mV) )
	{																									/* 最小値より小さい場合 */
		a_s16_integVq = (-a_s16_vqMaxTemp);																/* 積分項を最小値に設定する */
		s32_DD_MTR_integralVq_10mV = (s32)((s32)(-s16_DD_MTR_vqMax_10mV) << (s32)16);					/* 積分項32ビット保存 */
	}
	else
	{																									/* 制限値の間にいる */
		s32_DD_MTR_integralVq_10mV = a_s32_integVq;														/* 現在の積分項を保存 */
	}

/*----q軸比例項+積分項の制限-----*/
//	s16 a_s16_qAxisPI = (s16)( a_s16_propoVq + a_s16_integVq + s16_DD_MTR_vqBias_100mV );				/* 比例項 ＋ 積分項項 + Vqバイアス */
	s16 a_s16_qAxisPI = (s16)( a_s16_propoVq + a_s16_integVq );											/* 比例項 ＋ 積分項項 */

	S16_GL_propoVq = a_s16_propoVq;
	S16_GL_integVq = a_s16_integVq;
/*----Vqを制限する----*/
	if( a_s16_qAxisPI > s16_DD_MTR_vqMax_10mV )															/* 最大値と比較する */
	{																									/* 最大値より大きい場合 */
		st_DD_MTR_dq32.s32_vq_10mV = s16_DD_MTR_vqMax_10mV;												/* Vqを最大値に設定 */
	}
	else if ( a_s16_qAxisPI < (-s16_DD_MTR_vqMax_10mV) )												/* 最小値と比較 */
	{																									/* 最小値より小さい場合 */
		st_DD_MTR_dq32.s32_vq_10mV = (s16)(-s16_DD_MTR_vqMax_10mV);										/* Vqを最小値に設定する */
	}
	else
	{																									/* 制限値の間にある場合 */
		st_DD_MTR_dq32.s32_vq_10mV = a_s16_qAxisPI;														/* Vqは計算値を採用する */
	}

/*-----Vd、Vqを物理量からタイマ値に変換する-----*/
	s16_DD_MTR_vd = (s16)((s32)(U16_MTR_PWM_NEUTRAL_POINT) * st_DD_MTR_dq32.s32_vd_10mV / (s32)u16_DD_MTR_battVoltAve_10mV );	/* Vdの電圧値からDutyタイマ値へのへの変換 */
	s16_DD_MTR_vq = (s16)((s32)(U16_MTR_PWM_NEUTRAL_POINT) * st_DD_MTR_dq32.s32_vq_10mV / (s32)u16_DD_MTR_battVoltAve_10mV );	/* Vqの電圧値からDutyタイマ値へのへの変換 */

/*-----過過変調対応 電流歪みを許容した場合-----*/
	if( s16_DD_MTR_vq >= (s16)(U16_MTR_VD_MAX))
	{
		s16_DD_MTR_vq = (s16)(U16_MTR_VD_MAX);
	}
	else if( s16_DD_MTR_vq <= (s16)((s16)(-1) * (s16)(U16_MTR_VD_MAX)) )
	{
		s16_DD_MTR_vq = (s16)((s16)(-1) * (s16)(U16_MTR_VD_MAX));
	}
	else
	{
																										/* 過過変調以外は何もしない */
																										/* MISRA-C 2004 14.10 対応 */
	}
}

/**
 * @brief InverseParkAndClarkeTransform (dq → αβ → UVW)
 * @param a_u16_eangle_deg
 * @return none
 */
static void v_DD_MTR_InverseParkAndClarkeTransform(u16 a_u16_eangle_0p03125deg)
{
/*----- dq → αβ → UVW -----*/
	s32 a_s32_vu1 = (s32)( (s32)s16_DD_MTR_vd * (s32)TRIG_CALC_UVW_TRANSFORM_A(a_u16_eangle_0p03125deg) );	/* Vu1= -cosθ*Vd */
	s32 a_s32_vu2 = (s32)( (s32)s16_DD_MTR_vq * (s32)TRIG_CALC_UVW_TRANSFORM_B(a_u16_eangle_0p03125deg) );	/* Vu2= sinθ*Vq */
	s16_DD_MTR_vu = (s16)( ((s64)a_s32_vu1 + (s64)a_s32_vu2) >> (s64)14 );									/* Vu = -cosθ*Vd + sinθ*Vq */

	s32 a_s32_vw1 = (s32)( (s32)s16_DD_MTR_vd * (s32)TRIG_CALC_UVW_TRANSFORM_C(a_u16_eangle_0p03125deg) );	/* Vw1= -cos(θ+2π/3)*Vd */
	s32 a_s32_vw2 = (s32)( (s32)s16_DD_MTR_vq * (s32)TRIG_CALC_UVW_TRANSFORM_D(a_u16_eangle_0p03125deg) );	/* Vw2= sin(θ+2π/3)*Vq */
	s16_DD_MTR_vw = (s16)( ((s64)a_s32_vw1 + (s64)a_s32_vw2) >> (s64)14 );									/* Vw = -cos(θ+2π/3)*Vd + sin(θ+2π/3)*Vq */

	s16_DD_MTR_vv = (s16)( -( s16_DD_MTR_vu + s16_DD_MTR_vw ) );							/* Vvの算出 */
}


/**
 * @brief 過変調処理（包絡線中間電圧×1/2加算）& UVW中性点処理
 * @param none
 * @return none
 */
static void	v_DD_MTR_OverModulation(void)
{
/*----中性点変換(中間電圧1/2加算方式)-----*/
	s16 a_s16_vMax = s16_DD_MTR_vu;															/* 最大値として仮置き */
	if( a_s16_vMax <= s16_DD_MTR_vv )
	{
		a_s16_vMax = s16_DD_MTR_vv;
	}
	if( a_s16_vMax <= s16_DD_MTR_vw )
	{
		a_s16_vMax = s16_DD_MTR_vw;
	}

	s16 a_s16_vMin = s16_DD_MTR_vu;															/* 最小値として仮置き */
	if( a_s16_vMin >= s16_DD_MTR_vv )
	{
		a_s16_vMin = s16_DD_MTR_vv;
	}
	if( a_s16_vMin >= s16_DD_MTR_vw )
	{
		a_s16_vMin = s16_DD_MTR_vw;
	}

	s16 a_s16_neutralVoltageHalf = -( a_s16_vMax + a_s16_vMin ) >> (s16)1;								/* 中間電圧/2 */

	a_s16_neutralVoltageHalf = a_s16_neutralVoltageHalf + (s16)(U16_MTR_PWM_NEUTRAL_POINT);				/* 中性点加算値を保存する */

	/* モータの巻線方向が逆である為、ここで反転する */
	s16_DD_MTR_uCommandTemp = (S16_MTR_PWM_TIMER_MAX) - (s16_DD_MTR_vu + a_s16_neutralVoltageHalf);		/* PWMタイマ上側指令値の計算 */
	s16_DD_MTR_vCommandTemp = (S16_MTR_PWM_TIMER_MAX) - (s16_DD_MTR_vv + a_s16_neutralVoltageHalf);
	s16_DD_MTR_wCommandTemp = (S16_MTR_PWM_TIMER_MAX) - (s16_DD_MTR_vw + a_s16_neutralVoltageHalf);
}

#ifdef DEADTIME_COMPENSATION
/**
 * @brief デッドタイム補償
 * @param none
 * @return none
 */

static void v_DD_MTR_DeadtimeCompensation(void)
{
	s16 a_s16_deadtime_uPwmTimer = (s16)0;																/* U相デッドタイム補償値 */
	s16 a_s16_deadtime_vPwmTimer = (s16)0;																/* V相デッドタイム補償値 */	
	s16 a_s16_deadtime_wPwmTimer = (s16)0;																/* W相デッドタイム補償値 */	
	s16 a_s16_deattimeLevelTimer = (s16)(U16_MTR_DEADTIME_PWM_TIME) * (s16)2;							/* デッドタイム線形補間レベル(暫定） */

	/* U相デッドタイム補償計算 */
	if( ( a_s16_deattimeLevelTimer >= s16_DD_MTR_vu ) &&  ( s16_DD_MTR_vu >= -a_s16_deattimeLevelTimer ) )	/* デッドタイム線形補間レベル内 */
	{
		a_s16_deadtime_uPwmTimer = (s16)( s16_DD_MTR_vu * (s16)(U16_MTR_DEADTIME_PWM_TIME) ) / a_s16_deattimeLevelTimer;	/* 線形補間 */
	}
	else if( s16_DD_MTR_vu > (s16)0 )																	/* 上側でデッドタイム敷居値より大きい */
	{
		a_s16_deadtime_uPwmTimer = (s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償を加算 */
	}
	else if( s16_DD_MTR_vu < (s16)0 )																	/* 下側でデッドタイム敷居値より小さい */
	{
		a_s16_deadtime_uPwmTimer = -(s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償を減算 */
	}

	/* V相デッドタイム補償計算 */
	if( ( a_s16_deattimeLevelTimer >= s16_DD_MTR_vv ) &&  ( s16_DD_MTR_vv >= -a_s16_deattimeLevelTimer ) )	/* デッドタイム線形補間レベル内 */ 
	{
		a_s16_deadtime_vPwmTimer = (s16)( s16_DD_MTR_vv * (s16)(U16_MTR_DEADTIME_PWM_TIME) ) / a_s16_deattimeLevelTimer;	/* 線形補間 */	
	}
	else if( s16_DD_MTR_vv > (s16)0 )																	/* 上側でデッドタイム敷居値より大きい */
	{
		a_s16_deadtime_vPwmTimer = (s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償を加算 */
	}
	else if( s16_DD_MTR_vv < (s16)0 )																	/* 下側でデッドタイム敷居値より小さい */
	{
		a_s16_deadtime_vPwmTimer = -(s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償を減算 */
	}

	/* W相デッドタイム補償計算 */
	if( ( a_s16_deattimeLevelTimer >= s16_DD_MTR_vw ) &&  ( s16_DD_MTR_vw >= -a_s16_deattimeLevelTimer ) )	/* デッドタイム線形補間レベル内 */
	{
		a_s16_deadtime_wPwmTimer = (s16)( s16_DD_MTR_vw * (s16)(U16_MTR_DEADTIME_PWM_TIME) ) / a_s16_deattimeLevelTimer;	/* 線形補間 */		
	}
	else if( s16_DD_MTR_vw > (s16)0 )																	/* 上側でデッドタイム敷居値より大きい */
	{
		a_s16_deadtime_wPwmTimer = (s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償を加算 */
	}
	else if( s16_DD_MTR_vw < (s16)0 )																	/* 下側でデッドタイム敷居値より小さい */
	{
		a_s16_deadtime_wPwmTimer = -(s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償を減算 */
	}

/* クリップ */
	if( a_s16_deadtime_uPwmTimer >= (s16)(U16_MTR_DEADTIME_PWM_TIME))									/* デッドタイム補償値以上 */
	{
		a_s16_deadtime_uPwmTimer = (s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償値でクリップ */
	}
	else if( a_s16_deadtime_uPwmTimer <= -(s16)(U16_MTR_DEADTIME_PWM_TIME))								/* デッドタイム補償値以下 */
	{
		a_s16_deadtime_uPwmTimer = -(s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償値でクリップ */
	}
	else
	{
	}

	if( a_s16_deadtime_vPwmTimer >= (s16)(U16_MTR_DEADTIME_PWM_TIME))									/* デッドタイム補償値以上 */
	{
		a_s16_deadtime_vPwmTimer = (s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償値でクリップ */
	}
	else if( a_s16_deadtime_vPwmTimer <= -(s16)(U16_MTR_DEADTIME_PWM_TIME))								/* デッドタイム補償値以下 */
	{
		a_s16_deadtime_vPwmTimer = -(s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償値でクリップ */
	}
	else
	{
	}

	if( a_s16_deadtime_wPwmTimer >= (s16)(U16_MTR_DEADTIME_PWM_TIME))									/* デッドタイム補償値以上 */
	{
		a_s16_deadtime_wPwmTimer = (s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償値でクリップ */
	}
	else if( a_s16_deadtime_wPwmTimer <= -(s16)(U16_MTR_DEADTIME_PWM_TIME))								/* デッドタイム補償値以下 */
	{
		a_s16_deadtime_wPwmTimer = -(s16)(U16_MTR_DEADTIME_PWM_TIME);									/* デッドタイム補償値でクリップ */
	}
	else
	{
	}

	if( b_DD_MTR_isLowSpdSetFinished == TRUE )															/* 極低速時はデッドタイム補償値クリア */
	{
		a_s16_deadtime_uPwmTimer = (s16)0;
		a_s16_deadtime_vPwmTimer = (s16)0;
		a_s16_deadtime_wPwmTimer = (s16)0;
	}

	if( (s16_DD_MTR_uCommandTemp <= (s16)((S16_MTR_PWM_TIMER_MAX) - (s16)(U16_MTR_DEADTIME_PWM_TIME))) &&
		(s16_DD_MTR_uCommandTemp >= (s16)((U16_MTR_PWM_TIMER_MIN) + (s16)(U16_MTR_DEADTIME_PWM_TIME))))	/* タイマ上下限からデッドタイム補償分だけ余裕がある */
	{
		s16_DD_MTR_uCommandTemp -=  a_s16_deadtime_uPwmTimer;											/* 出力を反転しているため，ここも反転 */
	}
	if( (s16_DD_MTR_vCommandTemp <= (s16)((S16_MTR_PWM_TIMER_MAX) - (s16)(U16_MTR_DEADTIME_PWM_TIME))) &&
		(s16_DD_MTR_vCommandTemp >= (s16)((U16_MTR_PWM_TIMER_MIN) + (s16)(U16_MTR_DEADTIME_PWM_TIME))))	/* タイマ上下限からデッドタイム補償分だけ余裕がある */
	{
		s16_DD_MTR_vCommandTemp -=  a_s16_deadtime_vPwmTimer;											/* 出力を反転しているため，ここも反転 */
	}
	if( (s16_DD_MTR_wCommandTemp <= (s16)((S16_MTR_PWM_TIMER_MAX) - (s16)(U16_MTR_DEADTIME_PWM_TIME))) &&
		(s16_DD_MTR_wCommandTemp >= (s16)((U16_MTR_PWM_TIMER_MIN) + (s16)(U16_MTR_DEADTIME_PWM_TIME))))	/* タイマ上下限からデッドタイム補償分だけ余裕がある */
	{
		s16_DD_MTR_wCommandTemp -=  a_s16_deadtime_wPwmTimer;											/* 出力を反転しているため，ここも反転 */
	}
}
#endif		/* end of "DEADTIME_COMPENSATION" */

/**
 * @brief UVW指令値制限
 * @param none
 * @return none
 */
static void	v_DD_MTR_ClipVoltageCommand(void)
{
/*-----PWMデューティ制限 最大値クリップ ------*/
	if( s16_DD_MTR_uCommandTemp >= (S16_MTR_PWM_TIMER_MAX) )
	{
		s16_DD_MTR_uCommandTemp = (S16_MTR_PWM_TIMER_MAX);
	}

	if( s16_DD_MTR_vCommandTemp >= (S16_MTR_PWM_TIMER_MAX) )
	{
		s16_DD_MTR_vCommandTemp = (S16_MTR_PWM_TIMER_MAX);
	}

	if( s16_DD_MTR_wCommandTemp >= (S16_MTR_PWM_TIMER_MAX) )
	{
		s16_DD_MTR_wCommandTemp = (S16_MTR_PWM_TIMER_MAX);
	}

/*-----PWMデューティ制限 最小値クリップ ------*/
	if( s16_DD_MTR_uCommandTemp <= (s16)(U16_MTR_PWM_TIMER_MIN) )
	{
		s16_DD_MTR_uCommand = (s16)(U16_MTR_PWM_TIMER_MIN);
	}
	else
	{
		s16_DD_MTR_uCommand = s16_DD_MTR_uCommandTemp;							/* その他：計算値を採用 */
	}

	if( s16_DD_MTR_vCommandTemp <= (s16)(U16_MTR_PWM_TIMER_MIN) )
	{
		s16_DD_MTR_vCommand = (s16)(U16_MTR_PWM_TIMER_MIN);
	}
	else
	{
		s16_DD_MTR_vCommand = s16_DD_MTR_vCommandTemp;							/* その他：計算値を採用 */
	}

	if( s16_DD_MTR_wCommandTemp <= (s16)(U16_MTR_PWM_TIMER_MIN) )
	{
		s16_DD_MTR_wCommand = (s16)(U16_MTR_PWM_TIMER_MIN);
	}
	else
	{
		s16_DD_MTR_wCommand = s16_DD_MTR_wCommandTemp;							/* その他：計算値を採用 */
	}
}

/**
 * @brief バッテリ電圧監視
 * @param none
 * @return none
 */
static void	v_DD_MTR_MonitoringBattVolt(void)
{
	u16_DD_MTR_invVolt_10mV		= (u16)((u32)st_DD_MTR_adValue0.u16_adValueInvV * (u32)(U16_SYS_BAT_VDC_MAX_10MV) >> (u32)12);
	u16_DD_MTR_battVoltRaw_10mV	= (u16)((u32)st_DD_MTR_adValue0.u16_adValueBattV * (u32)(U16_SYS_BAT_VDC_MAX_10MV) >> (u32)12);
	/* 下限クリップ */
	if( u16_DD_MTR_battVoltRaw_10mV <= (U16_LOWVOLTAGE_SHUTDOWN_BATT_VOLT_10MV) )
	{
		u16_DD_MTR_battVolt_10mV = (U16_LOWVOLTAGE_SHUTDOWN_BATT_VOLT_10MV);
	}
	else
	{
		u16_DD_MTR_battVolt_10mV = u16_DD_MTR_battVoltRaw_10mV;
	}

	/* バッテリ電圧とインバータ電圧の高い方をゲートオフ判定に使用する */
	u16	a_u16_maxInvVolt_10mV = u16_DD_MTR_invVolt_10mV;
	if( u16_DD_MTR_invVolt_10mV < u16_DD_MTR_battVolt_10mV )
	{
		a_u16_maxInvVolt_10mV = u16_DD_MTR_battVolt_10mV;
	}

	u16_DD_MTR_battVoltAve_10mV = U16_BAT_GetBattVoltAve_10mV();										/* フィルタ後の電源電圧取得[0.1V] */

/*------------------- 過電圧によるゲートオフ判定 -------------------*/
	if( b_DD_MTR_isGateOn == FALSE )
	{																									/* ゲートオフ中 */
		if( b_DD_MTR_overRegenerativeVoltage == TRUE )
		{																								/* 回生中電圧超過フラグONの場合 */
			if( (a_u16_maxInvVolt_10mV <= (U16_MTR_GATE_ON_BATT_VOLT_10MV)) ||
				(st_DD_MTR_dq32.s32_iqRef_10mA > (s32)0) )
			{																							/* 電圧は閾値電圧より小さい場合 注:指令値は正の場合で回生中電圧超過フラグOFF */
				b_DD_MTR_overRegenerativeVoltage = FALSE;												/* 回生中電圧超過フラグOFF */
			}
		}
	}
	else
	{																									/* ゲートオン中 */
		if( (a_u16_maxInvVolt_10mV >= (U16_MTR_GATE_OFF_BATT_VOLT_10MV)) && (st_DD_MTR_dq32.s32_iqRef_10mA < (s32)0) )
		{																								/* 回生中、電圧は閾値を超えた */
			st_DD_MTR_dq32.s32_idRef_10mA = (s32)0;
			st_DD_MTR_dq32.s32_iqRef_10mA = (s32)0;
			b_DD_MTR_isStartRequested = FALSE;															/* モータ制御停止要求 */
			b_DD_MTR_overRegenerativeVoltage = TRUE;													/* 回生中電圧が超えてる */
		}
	}
}

/**
 * @brief 3相電流PP計算
 * @param none
 * @return none
 * @note 62.5usec周期
 */
static void v_DD_MTR_CalcCurrentPp(void)
{
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	/* モータ回転数が規定値未満又はロータ逆転又は振動している場合 */
	if( (s16_DD_MTR_revolution_rpm < (s32)20) || (b_DD_MTR_isEncReverseRotate == TRUE) || (b_DD_MTR_isRotorVibration == TRUE) )
	{
		en_DD_MTR_ppSearchStatus	= ENUM_DD_MTR_PP_SEARCH_INIT;										/* 初期化 */
		s32_DD_MTR_iUPp_10mA		= (s32)0;															/* P-P値を初期化 */
		s32_DD_MTR_iVPp_10mA		= (s32)0;															/* P-P値を初期化 */
		s32_DD_MTR_iWPp_10mA		= (s32)0;															/* P-P値を初期化 */
		s32_DD_MTR_iUTopBuff_10mA	= (s32)0;															/* TOP仮値を初期化 */
		s32_DD_MTR_iVTopBuff_10mA	= (s32)0;															/* TOP仮値を初期化 */
		s32_DD_MTR_iWTopBuff_10mA	= (s32)0;															/* TOP仮値を初期化 */
		s32_DD_MTR_iUBtmBuff_10mA	= (s32)0;															/* BOTTOM仮値を初期化 */
		s32_DD_MTR_iVBtmBuff_10mA	= (s32)0;															/* BOTTOM仮値を初期化 */
		s32_DD_MTR_iWBtmBuff_10mA	= (s32)0;															/* BOTTOM仮値を初期化 */
		return;
	}

	/* モータ回転数が規定値以上の場合（モータが回転している場合） */
	/* 正転かつロータ振動が無い場合 */
	s32 a_s32_tempU = st_DD_MTR_phaseCurrent.s32_iU_10mA;												/* U相電流取得 */
	s32 a_s32_tempW = st_DD_MTR_phaseCurrent.s32_iW_10mA;												/* W相電流取得 */
	s32 a_s32_tempV = st_DD_MTR_phaseCurrent.s32_iV_10mA;												/* V相電流取得 */

	switch (en_DD_MTR_ppSearchStatus)
	{
	case ENUM_DD_MTR_PP_SEARCH_INIT:																	/* 開始の場合 */
		u8_DD_MTR_encPhaseCountForPpDetection = (u8)1;													/* パターンカウンタ：1に初期化 エンコーダパターン数が6なので、カウンタは1-6で数えることとする */
		s32_DD_MTR_iUTopBuff_10mA = a_s32_tempU;														/* 現在の相電流を仮TOP値に設定 */
		s32_DD_MTR_iVTopBuff_10mA = a_s32_tempV;														/* 現在の相電流を仮TOP値に設定 */
		s32_DD_MTR_iWTopBuff_10mA = a_s32_tempW;														/* 現在の相電流を仮TOP値に設定 */
		s32_DD_MTR_iUBtmBuff_10mA = a_s32_tempU;														/* 現在の相電流を仮BOTTOM値に設定 */
		s32_DD_MTR_iVBtmBuff_10mA = a_s32_tempV;														/* 現在の相電流を仮BOTTOM値に設定 */
		s32_DD_MTR_iWBtmBuff_10mA = a_s32_tempW;														/* 現在の相電流を仮BOTTOM値に設定 */
		en_DD_MTR_ppSearchStatus = ENUM_DD_MTR_PP_SEARCH_RUNNING;										/* 探索中へ */
		break;

	case ENUM_DD_MTR_PP_SEARCH_RUNNING:																	/* 探索実施中の場合 */
		if( s32_DD_MTR_iUTopBuff_10mA < a_s32_tempU )													/* 仮TOP値を更新 */
		{
			s32_DD_MTR_iUTopBuff_10mA = a_s32_tempU;
		}
		if( s32_DD_MTR_iVTopBuff_10mA < a_s32_tempV )													/* 仮TOP値を更新 */
		{
			s32_DD_MTR_iVTopBuff_10mA = a_s32_tempV;
		}
		if( s32_DD_MTR_iWTopBuff_10mA < a_s32_tempW )													/* 仮TOP値を更新 */
		{
			s32_DD_MTR_iWTopBuff_10mA = a_s32_tempW;
		}
		if( s32_DD_MTR_iUBtmBuff_10mA > a_s32_tempU )													/* 仮BOTTOM値を更新 */
		{
			s32_DD_MTR_iUBtmBuff_10mA = a_s32_tempU;
		}
		if( s32_DD_MTR_iVBtmBuff_10mA > a_s32_tempV )													/* 仮BOTTOM値を更新 */
		{
			s32_DD_MTR_iVBtmBuff_10mA = a_s32_tempV;
		}
		if( s32_DD_MTR_iWBtmBuff_10mA > a_s32_tempW )													/* 仮BOTTOM値を更新 */
		{
			s32_DD_MTR_iWBtmBuff_10mA = a_s32_tempW;
		}

		if ( u8_DD_MTR_encPatternLast != u8_DD_MTR_encPattern )											/* パターン変化あり？ */
		{
			u8_DD_MTR_encPhaseCountForPpDetection ++;													/* パターンカウンタInc */
			if( u8_DD_MTR_encPhaseCountForPpDetection > (U8_MTR_NUM_ENCODER_PATTERNS) )
			{
				u8_DD_MTR_encPhaseCountForPpDetection = (U8_MTR_NUM_ENCODER_PATTERNS);					/* パターンカウンタClip */
			}
		}

		if ( u8_DD_MTR_encPhaseCountForPpDetection >= (U8_MTR_NUM_ENCODER_PATTERNS) )					/* 電気角が1周した場合 */
		{
			s32_DD_MTR_iUPp_10mA	= s32_DD_MTR_iUTopBuff_10mA - s32_DD_MTR_iUBtmBuff_10mA;			/* P-P値を更新 */
			s32_DD_MTR_iVPp_10mA	= s32_DD_MTR_iVTopBuff_10mA - s32_DD_MTR_iVBtmBuff_10mA;			/* P-P値を更新 */
			s32_DD_MTR_iWPp_10mA	= s32_DD_MTR_iWTopBuff_10mA - s32_DD_MTR_iWBtmBuff_10mA;			/* P-P値を更新 */
			en_DD_MTR_ppSearchStatus = ENUM_DD_MTR_PP_SEARCH_INIT;										/* 開始へ */
		}
		break;
	default:
		break;
	}

	s16_DD_MTR_iUTopBuff_100mA = (s16)(s32_DD_MTR_iUTopBuff_10mA / (s32)10);
	s16_DD_MTR_iVTopBuff_100mA = (s16)(s32_DD_MTR_iVTopBuff_10mA / (s32)10);
	s16_DD_MTR_iWTopBuff_100mA = (s16)(s32_DD_MTR_iWTopBuff_10mA / (s32)10);
	s16_DD_MTR_iUBtmBuff_100mA = (s16)(s32_DD_MTR_iUBtmBuff_10mA / (s32)10);
	s16_DD_MTR_iVBtmBuff_100mA = (s16)(s32_DD_MTR_iVBtmBuff_10mA / (s32)10);
	s16_DD_MTR_iWBtmBuff_100mA = (s16)(s32_DD_MTR_iWBtmBuff_10mA / (s32)10);
}

/**
 * @brief モータ断線チェック
 * @param none
 * @return none
 * @note 62.5usec周期
 * @note 3相各相の電流振幅で最大振幅が最小振幅の3倍より大きければ断線の疑い。ある一定時間続いた場合エラーとする。
 */
static void v_DD_MTR_DetectWireOpen(void)																/* モータ断線判定（83.3us毎）*/
{
	ENUM_DD_MTR_IMIN_PHASE	a_en_minUVW;

	bool a_b_isDetectActivity = B_MDC_IsDetectActivity();

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( uni_DD_MTR_abnormalFlg.byte != (u8)0 )															/* 既に異常状態？ */
	{
		return;
	}

	/* UVWの振幅の最大値，最小値 */
	if ( s32_DD_MTR_iUPp_10mA > s32_DD_MTR_iVPp_10mA )						/* まずはUとVを比較して */
	{
		s32_DD_MTR_ctPpMAX_10mA = s32_DD_MTR_iUPp_10mA;						/* MAXはU（仮）*/
		s32_DD_MTR_ctPpMIN_10mA = s32_DD_MTR_iVPp_10mA;						/* MINはV（仮）*/
		a_en_minUVW = ENUM_IMIN_PHASE_V;																/* MinはVと記憶（仮）*/

		if ( s32_DD_MTR_iUPp_10mA < s32_DD_MTR_iWPp_10mA )					/* UとWで比較MAXは？ */
		{
			s32_DD_MTR_ctPpMAX_10mA = s32_DD_MTR_iWPp_10mA;					/* WならMAXはWに決定、MINはVに決定 */
		}
		else if ( s32_DD_MTR_iVPp_10mA > s32_DD_MTR_iWPp_10mA )				/* VとWで比較MINは？ (UはMAXで決定) */
		{
			s32_DD_MTR_ctPpMIN_10mA = s32_DD_MTR_iWPp_10mA;					/* MINはWに決定 */
			a_en_minUVW = ENUM_IMIN_PHASE_W;															/* MinはWと記憶 */
		}
		else
		{
			/* MISRA-C 2004 14.10 対応 */
		}
	}
	else
	{
		s32_DD_MTR_ctPpMAX_10mA = s32_DD_MTR_iVPp_10mA;						/* MAXはV（仮）*/
		s32_DD_MTR_ctPpMIN_10mA = s32_DD_MTR_iUPp_10mA;						/* MINはU（仮）*/
		a_en_minUVW = ENUM_IMIN_PHASE_U;																/* MinはUと記憶（仮）*/
		if ( s32_DD_MTR_iVPp_10mA < s32_DD_MTR_iWPp_10mA )					/* VとWで比較MAXは？ */
		{
			s32_DD_MTR_ctPpMAX_10mA = s32_DD_MTR_iWPp_10mA;					/* WならMAXはWに決定、MINはUに決定 */
		}
		else if ( s32_DD_MTR_iUPp_10mA > s32_DD_MTR_iWPp_10mA )				/* UとWで比較MINは？ (VはMAXで決定) */
		{
			s32_DD_MTR_ctPpMIN_10mA = s32_DD_MTR_iWPp_10mA;					/* MINはWに決定 */
			a_en_minUVW = ENUM_IMIN_PHASE_W;															/* MinはWと記憶 */
		}
		else
		{
			/* MISRA-C 2004 14.10 対応 */
		}
	}

	/* 最大値ー最小値 で差がなければ正常、差が大きければ異常？*/
	if( (st_DD_MTR_dq32.s32_iqRef_10mA >= (S32_MTR_CT_PP_DIFF_JUDGE_IQREF_10MA)) && (s16_DD_MTR_revolution_rpm > (S16_MTR_CT_PP_DIFF_JUDGE_MOTREV_RPM)) )	/* 指示モータ電流且つロータ回転数のチェック */
	{
		if( s32_DD_MTR_ctPpMAX_10mA > (s32_DD_MTR_ctPpMIN_10mA * (s32)3) )	/* 最小値が最大値の1/3未満ならば */
		{
			if( u16_DD_MTR_wireOpenJudgeCount_62p5us < (U16_MTR_WIRE_OPEN_JUDGE_COUNT) )
			{
				u16_DD_MTR_wireOpenJudgeCount_62p5us ++;
			}
			en_DD_MTR_iMinPhase = a_en_minUVW;															/* 最新の最小値を覚えておく */
		}
		else
		{
			if( u16_DD_MTR_wireOpenJudgeCount_62p5us > (u16)0 )
			{
				u16_DD_MTR_wireOpenJudgeCount_62p5us --;
			}
		}
	}
	else	/* 指令値５Ａ付近で出力されたときのことを考えて、一気にカウント０にするのでなく、少しずつ減らす */
	{
		if( u16_DD_MTR_wireOpenJudgeCount_62p5us > (u16)0 )
		{
			u16_DD_MTR_wireOpenJudgeCount_62p5us --;
		}
	}

	switch (en_DD_MTR_wireOpenEachStatus)
	{
	case ENUM_WIRE_OPEN_NORMAL:
		if( u16_DD_MTR_wireOpenJudgeCount_62p5us >= (U16_MTR_WIRE_OPEN_JUDGE_COUNT) )					/* 640msでモータ断線確定 */
		{
			switch (en_DD_MTR_iMinPhase)
			{
			case ENUM_IMIN_PHASE_U:
				V_DD_EX_ForcedGateOff();																/* エキサイタ強制ゲートOFFを要求する */
				uni_DD_MTR_abnormalFlg.bit.b_abnormalUPhaseWireOpen = TRUE;								/* U相断線異常セット */
				break;

			case ENUM_IMIN_PHASE_V:
				V_DD_EX_ForcedGateOff();																/* エキサイタ強制ゲートOFFを要求する */
				uni_DD_MTR_abnormalFlg.bit.b_abnormalVPhaseWireOpen = TRUE;								/* V相断線異常セット */
				break;

			case ENUM_IMIN_PHASE_W:
				V_DD_EX_ForcedGateOff();																/* エキサイタ強制ゲートOFFを要求する */
				uni_DD_MTR_abnormalFlg.bit.b_abnormalWPhaseWireOpen = TRUE;								/* W相断線異常セット */
				break;

			default:
				break;
			}
			en_DD_MTR_wireOpenEachStatus = ENUM_WIRE_OPEN_ABNORMAL;										/* モータ断線確定 */
		}
		break;

	case ENUM_WIRE_OPEN_ABNORMAL:
		break;

	default:
		/* 想定外の分岐 */
		break;
	}
}

/**
 * @brief DA出力設定
 * @param none
 * @return none
 */
static void	v_DD_MTR_DaOutput(void)
{
/* 相電流 */
//	u16	a_u16_iU = (u16)((st_DD_MTR_phaseCurrent.s32_iU_10mA + (s32)75000)/ (s32)38);
//	u16	a_u16_iV = (u16)((st_DD_MTR_phaseCurrent.s32_iV_10mA + (s32)75000)/ (s32)38);
//	u16	a_u16_iW = (u16)((st_DD_MTR_phaseCurrent.s32_iW_10mA + (s32)75000)/ (s32)38);
//	R_Config_DA0_Set_ConversionValue( a_u16_iU );
//	if( UNI_GL_testFlg.bit.b_da1utputWphaseCurrent == TRUE )
//	{
//		R_Config_DA1_Set_ConversionValue( a_u16_iW );
//	}
//	else
//	{
//		R_Config_DA1_Set_ConversionValue( a_u16_iV );
//	}
//
//	s16_DD_MTR_iU_100mA = (s16)(st_DD_MTR_phaseCurrent.s32_iU_10mA / (s32)10);
//	s16_DD_MTR_iV_100mA = (s16)(st_DD_MTR_phaseCurrent.s32_iV_10mA / (s32)10);
//	s16_DD_MTR_iW_100mA = (s16)(st_DD_MTR_phaseCurrent.s32_iW_10mA / (s32)10);

/* eangle */
	u16	a_u16_eangle = u16_DD_MTR_eangle_0p03125deg >> (u16)2;														/* 電気角推定(for DEBUG) */
	R_Config_DA1_Set_ConversionValue( a_u16_eangle );

}

/**
 * @brief モータ回転数提供
 * @param none
 * @return s16_DD_MTR_revolution_rpm
 */
s16	S16_DD_MTR_GetRevolution_rpm(void)
{
	return	s16_DD_MTR_revolution_rpm;
}

/**
 * @brief 6移動平均モータ回転数提供
 * @param none
 * @return s16_DD_MTR_revolutionAve6_rpm
 */
s16	S16_DD_MTR_GetRevolutionAve6_rpm(void)
{
	return	s16_DD_MTR_revolutionAve6_rpm;
}

/**
 * @brief ゲートON/FF状態提供
 * @param none
 * @return b_DD_MTR_isGateOn
 */
bool	B_DD_MTR_IsGateOn(void)
{
	return b_DD_MTR_isGateOn;
}

/**
 * @brief モータ駆動を許可する
 * @param none
 * @return none
 */
void	V_DD_MTR_Permit(void)
{
	b_DD_MTR_isPermitted = TRUE;
}

/**
 * @brief モータ駆動を禁止する
 * @param none
 * @return none
 */
void	V_DD_MTR_Forbid(void)
{
	b_DD_MTR_isPermitted = FALSE;
}

/**
 * @brief モータ駆動を要求する
 * @param none
 * @return none
 */
void	V_DD_MTR_StartControl(void)
{
	b_DD_MTR_isStartRequested = TRUE;
}

/**
 * @brief ゲートOFFを要求する
 * @param none
 * @return none
 */
void	V_DD_MTR_StopControl(void)
{
	b_DD_MTR_isStartRequested = FALSE;
}

/**
 * @brief エンコーダパターン異常判断
 * @param none
 * @return TRUE or FALSE
 */
static bool	b_DD_MTR_DetectEncPattern(void)

{
	u8_DD_MTR_encPatternLast = u8_DD_MTR_encPattern;													/* 前回のパタンを保存する */

/*---- エンコーダ値を取り込む -----*/
	u8_DD_MTR_encPattern = U8_DD_MTR_GetEncPattern();

/*-------エンコーダパターン異常判断-------*/
	bool a_b_result = TRUE;

	if( (u8_DD_MTR_encPattern == (U8_MTR_NG_ENC_PATTERN1)) ||
		(u8_DD_MTR_encPattern == (U8_MTR_NG_ENC_PATTERN2)) )
	{																									/* エンコーダパターンが異常の場合 */
		a_b_result = FALSE;																				/* 異常値をセット */
	}
	return a_b_result;
}

/**
 * @brief エンコーダパターン提供
 * @param none
 * @return a_u8_EncPort
 */
u8	U8_DD_MTR_GetEncPattern(void)
{
	u8 a_u8_EncPort = (u8)(U8_ENC_VALUE());																/* ポートの値を読み込み */
	return a_u8_EncPort;
}

/**
 * @brief ロータ振動状態提供
 * @param none
 * @return b_DD_MTR_isRotorVibration
 */
bool	B_DD_MTR_IsRotorVibrated(void)
{
	return b_DD_MTR_isRotorVibration;
}

/**
 * @brief モータ単相断線異常を無効化する
 * @param none
 * @return none
 */
void	V_DD_MTR_DetectWireOpenInactivate(void)
{
	b_DD_MTR_detectWireOpenActivate = FALSE;
}

/**
 * @brief モータ単相断線異常を有効化する
 * @param none
 * @return none
 */
void	V_DD_MTR_DetectWireOpenActivate(void)
{
	b_DD_MTR_detectWireOpenActivate = TRUE;
}

/**
 * @brief エンコーダ異常を無効化する
 * @param none
 * @return none
 */
void	V_DD_MTR_DetectEncoderInactivate(void)
{
	b_DD_MTR_detectEncoderActivate = FALSE;
}

/**
 * @brief エンコーダ異常を有効化する
 * @param none
 * @return none
 */
void	V_DD_MTR_DetectEncoderActivate(void)
{
	b_DD_MTR_detectEncoderActivate = TRUE;
}

/**
 * @brief 電気角オフセット値設定
 * @param a_u16_eangleOfs_deg
 * @return none
 */
void	V_DD_MTR_SetEangleOffset_deg(u16 a_u16_eangleOfs_deg)
{
	u16_DD_MTR_eangleMapOffset_0p03125deg = (u16)(a_u16_eangleOfs_deg << (u16)5);						/* ここで1deg → 0.03125degに変換 */
}

/**
 * @brief AD0データ提供
 */
ST_ADC_ADVALUE0	St_DD_MTR_GetAdValue0(void)
{
	return	*pst_DD_MTR_adValue0;
}


/**
 * @brief バッテリ電圧ＡＤ提供(生)
 * @param none
 * @return st_DD_MTR_adValue0.u16_adValueBattV
 */
u16 U16_DD_MTR_GetAdValueBattVolt(void)
{
	return st_DD_MTR_adValue0.u16_adValueBattV;															/* 生のAD値 */
}

/**
 * @brief インバータ電圧ＡＤ提供(生)
 * @param none
 * @return st_DD_MTR_adValue0.u16_adValueInvV
 */
u16 U16_DD_MTR_GetAdValueInvVolt(void)
{
	return st_DD_MTR_adValue0.u16_adValueInvV;															/* 生のAD値 */
}

/**
 * @brief バッテリ電圧提供(フィルタなし)
 * @param none
 * @return u16_DD_MTR_battVoltRaw_100mV
 */
u16 U16_DD_MTR_GetBattVolt_10mV(void)
{
	return u16_DD_MTR_battVoltRaw_10mV;
}

/**
 * @brief DQ軸データ提供
 */
ST_ADC_DQ32	St_DD_MTR_GetDq32(void)
{
	return	*pst_DD_MTR_dq32;
}

/**
 * @brief D軸電流提供[10mA]
 * @param none
 * @return st_DD_MTR_dq32.s32_idFb_10mA
 */
s32	S32_DD_MTR_GetIdFb_10mA(void)
{
	return st_DD_MTR_dq32.s32_idFb_10mA;																/* d軸電流FB[10mA] */
}

/**
 * @brief Q軸電流提供[10mA]
 * @param none
 * @return st_DD_MTR_dq32.s32_iqFb_10mA
 */
s32	S32_DD_MTR_GetIqFb_10mA(void)
{
	return st_DD_MTR_dq32.s32_iqFb_10mA;																/* q軸電流FB[10mA] */
}

/**
 * @brief Id、Iq指令値セット[10mA]
 * @param a_s32_idRef
 * @param a_s32_iqRef
 * @return none
 */
void	V_DD_MTR_SetIdqRef_10mA(s32 a_s32_idRef, s32 a_s32_iqRef)
{
	st_DD_MTR_dq32.s32_idRef_10mA = a_s32_idRef;
	st_DD_MTR_dq32.s32_iqRef_10mA = a_s32_iqRef;
}

/**
 * @brief Id指令値提供[10mA]
 * @param none
 * @return st_DD_MTR_dq32.s32_idRef_10mA
 */
s32	S32_DD_MTR_GetIdRef_10mA(void)
{
	return	st_DD_MTR_dq32.s32_idRef_10mA;
}

/**
 * @brief Iq指令値提供[10mA]
 * @param none
 * @return st_DD_MTR_dq32.s32_iqRef_10mA
 */
s32	S32_DD_MTR_GetIqRef_10mA(void)
{
	return	st_DD_MTR_dq32.s32_iqRef_10mA;
}

/**
 * @brief 最終Iq指令値提供[10mA]
 * @param none
 * @return st_DD_MTR_dq32.s32_iqRef_10mA
 */
s32	S32_DD_MTR_GetIqRefFinal_10mA(void)
{
	return	s32_DD_MTR_iqRefFinal_10mA;
}

/**
 * @brief モータ逆転状態提供
 * @param none
 * @return b_DD_MTR_isMotorReverseRotate
 */
bool	B_DD_MTR_IsMotorReverseRotate(void)
{
	return b_DD_MTR_isMotorReverseRotate;
}

/**
 * @brief 電流制御 & AD取り込み処理用変数の初期化処理
 * @param none
 * @return none
 */
void V_DD_MTR_Initialize(void)
{
	struct ST_ENC_PATTERN_TABLE const *a_pu16;

	u8_DD_MTR_encPattern = U8_DD_MTR_GetEncPattern();													/* エンコーダパターン初期化 */

	u8_DD_MTR_encPatternLast = u8_DD_MTR_encPattern;													/* 前回のエンコーダ値 */

	a_pu16 = u16_DD_MTR_encPatternTable + (u16)u8_DD_MTR_encPattern;									/* エンコーダマップへのポインタ */
	u16_DD_MTR_eangleBase_deg = a_pu16 -> u16_eangleBase;												/* 現在のベース電気角度を読込 */
}

/**
 * @brief エンコーダパターンチェッカー初期化
 * @param none
 * @return none
 */
void	V_DD_MTR_ClearEncPatternChecker(void)
{
	V_CMF_Memset(u8_DD_MTR_encPatternChecker, (u8)0, (u8)6);											/* 回転数計算用パターンチェッカー初期化 */
}

/**
 * @brief エンコーダパターンチェッカー提供
 * @param none
 * @return none
 */
u8	U8_DD_MTR_GetEncPatternChecker(u8 idx)
{
	return	u8_DD_MTR_encPatternChecker[idx];
}

///**
// * @brief 相電流P-P最小値提供
// * @param none
// * @return none
// */
//s16	S16_DD_MTR_GetCtPpMIN_100mA(void)
//{
//	return	s16_DD_MTR_ctPpMIN_100mA;
//}

/**
 * @brief エンコーダ異常状態提供
 * @param none
 * @return uni_DD_MTR_abnormalFlg.bit.b_abnormalEncoder
 */
bool	B_DD_MTR_IsAbnormanEncoder(void)
{
	return uni_DD_MTR_abnormalFlg.bit.b_abnormalEncoder;
}

/**
 * @brief 過電流異常状態提供
 * @param none
 * @return uni_DD_MTR_abnormalFlg.bit.b_abnormalOverCurrent
 */
bool	B_DD_MTR_IsAbnormalOverCurrent(void)
{
	return uni_DD_MTR_abnormalFlg.bit.b_abnormalOverCurrent;
}

/**
 * @brief U相モータ断線異常状態提供
 * @param none
 * @return uni_DD_MTR_abnormalFlg.bit.b_abnormalUPhaseWireOpen
 */
bool	B_DD_MTR_IsAbnormalUPhaseWireOpen(void)
{
	return uni_DD_MTR_abnormalFlg.bit.b_abnormalUPhaseWireOpen;
}

/**
 * @brief V相モータ断線異常状態提供
 * @param none
 * @return uni_DD_MTR_abnormalFlg.bit.b_abnormalVPhaseWireOpen
 */
bool	B_DD_MTR_IsAbnormalVPhaseWireOpen(void)
{
	return uni_DD_MTR_abnormalFlg.bit.b_abnormalVPhaseWireOpen;
}

/**
 * @brief W相モータ断線異常状態提供
 * @param none
 * @return uni_DD_MTR_abnormalFlg.bit.b_abnormalWPhaseWireOpen
 */
bool	B_DD_MTR_IsAbnormalWPhaseWireOpen(void)
{
	return uni_DD_MTR_abnormalFlg.bit.b_abnormalWPhaseWireOpen;
}

/**
 * @brief d軸積分バッファ提供[10mV]
 * @param none
 * @return s32_DD_MTR_integralVd_10mV
 */
s32	S32_DD_MTR_GetIntegralVd_10mV(void)
{
	return	s32_DD_MTR_integralVd_10mV;
}

/**
 * @brief q軸積分バッファ提供[10mV]
 * @param none
 * @return s32_DD_MTR_integralVq_10mV
 */
s32	S32_DD_MTR_GetIntegralVq_10mV(void)
{
	return	s32_DD_MTR_integralVq_10mV;
}


/**
 * @brief D軸電圧Vd最大値提供[10mV]
 * @param none
 * @return s16_DD_MTR_vdMax_10mV
 */
s16	S16_DD_MTR_GetVdMax_10mV(void)
{
	return	s16_DD_MTR_vdMax_10mV;
}

/**
 * @brief Q軸電圧Vq最大値提供[10mV]
 * @param none
 * @return s16_DD_MTR_vqMax_10mV
 */
s16	S16_DD_MTR_GetVqMax_10mV(void)
{
	return	s16_DD_MTR_vqMax_10mV;
}

/**
 * @brief テスト用電気角オフセット提供[deg]
 * @param none
 * @return u16_DD_MTR_eangleOffsetForTest_deg
 */
u16	U16_DD_MTR_GetEangleOffsetForTest_deg(void)
{
	return	u16_DD_MTR_eangleOffsetForTest_deg;
}


/**
* @brief 電流制御 & AD取り込み処理用変数の初期化処理(再起動用)
 * @param none
 * @return none
 */
void V_DD_MTR_InitializeForRestart(void)
{
	b_DD_MTR_isRotorVibration = FALSE;																	/* ロータ振動フラグをクリア */
	u8_DD_MTR_encAbnormalCount = (u8)0;																	/* エンコーダ異常カウンタ初期化 */

	b_DD_MTR_isEncReverseRotate = FALSE;																/* 現サイクル回転方向	0:正転	1:逆転 */
	b_DD_MTR_isEncReverseRotateLast = FALSE;															/* 前サイクル回転方向	0:正転	1:逆転 */

	u16_DD_MTR_estimateEangle_0p03125deg = (u16)0;														/* 補正電気角 */
	u16_DD_MTR_estimateEangleDelta = (u16)0;															/* 単位補正電気角 */

	u16_DD_MTR_eangleOffset_0p03125deg = (U16_MTR_EANGLE_OFFSET_0P03125DEG);							/* 電気角オフセット補正値 */

	b_DD_MTR_isStartRequested = FALSE;
	b_DD_MTR_overRegenerativeVoltage = FALSE;															/* 回生電圧Over 1：Over */

	uni_DD_MTR_abnormalFlg.byte = (u8)0;																/* 異常フラグ初期化 */

	u16_DD_MTR_dAxisProportionalGain = (U16_MTR_DAXIS_KP);												/* d軸比例ゲイン */
	u16_DD_MTR_qAxisProportionalGain = (U16_MTR_QAXIS_KP);												/* q軸比例ゲイン */
	u16_DD_MTR_dAxisIntegralGain = (U16_MTR_DAXIS_KI);													/* d軸積分ゲイン */
	u16_DD_MTR_qAxisIntegralGain = (U16_MTR_QAXIS_KI);													/* q軸積分ゲイン */

	s16_DD_MTR_vdMax_10mV = (U16_MTR_VD_MAX);															/* D軸電圧Vd最大値 物理量[0,1V] */

	st_DD_MTR_phaseCurrent.s32_iU_10mA = (s32)0;														/* U相電流 */
	st_DD_MTR_phaseCurrent.s32_iW_10mA = (s32)0;														/* V相電流 */
	st_DD_MTR_phaseCurrent.s32_iV_10mA = (s32)0;														/* W相電流 */

	st_DD_MTR_dq32.s32_idRef_10mA = (s32)0;																/* Id指令値[0.1A] */
	st_DD_MTR_dq32.s32_iqRef_10mA = (s32)0;																/* Iq指令値[0.1A] */

	s32_DD_MTR_integralVd_10mV = (s32)0;																/* d軸積分項上位16bit */
	s32_DD_MTR_integralVq_10mV = (s32)0;																/* q軸積分項上位16bit */

	u8_DD_MTR_overCurrentJudgeCount		= (u8)0;														/* 過電流異常検出タイマ初期化 */

	u16_DD_MTR_wireOpenJudgeCount_62p5us = (u16)0;														/* モータ断線異常検出タイマ初期化 */
}
