/**
 * @file ADC.c
 * @brief AD変換結果のフィルタ処理
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "Config_S12AD0.h"
#include "Config_S12AD1.h"
#include "Config_S12AD2.h"

#include "MCU.h"
#include "DEFINE_SYSTEM.h"			/*定数定義*/
#include "DEFINE_MOTOR.h"			/*定数定義*/
#include "DEFINE_THERMISTOR.h"
#include "COMMON_FUNCTION.h"		/* 汎用関数群 */

/* DDL */
#include "DD_MOTOR.h"
#include "DD_EXCITER.h"

#include "CAN.h"
#include "MOTOR.h"
#include "ADC.h"

/* for DEBUG */
#include "ramdef.h"

/* Private define ------------------------------------------------------------*/
#define U8_ADC_AD2_MAX_WAITING_COUNT	(u8)120						/* AD2ループAD変換待機カウンタ最大数 */

#define	U8_ADC_TIME_CONST_FACTOR		(u8)2						/* AD変換後のLPF時定数係数 LPFの時定数2msに相当（ 2^2 = 4：4×0.5ms = 2msec ）*/

#define	U16_ADC_ACCEL_OFFSET				(u16)20					/* アクセルオフセット（遊び） */
#define	U16_ACCEL_ADC_MARGIN_LOWER		(u16)((u32)10 * (u32)(U16_SYS_AD_INPUT_MAX) / (u32)(U16_SYS_AD_MAX_100MV))	/* ACCEL 下側遊びAD閾値 1[V] */
#define	U16_ACCEL_ADC_MARGIN_UPPER		(u16)((u32)40 * (u32)(U16_SYS_AD_INPUT_MAX) / (u32)(U16_SYS_AD_MAX_100MV))	/* ACCEL 上側遊びAD閾値 4[V] */
/*================================================================*/
/*	バッテリ電流変換マップ定義( ROMデータとして保存する）*/
/*================================================================*/
#ifdef	DEBUG_WATCH_ACTIVITY

bool	b_ADC_isStartAD2Convert = FALSE;											/* AD2変換開始フラグ */
bool	b_ADC_isDelayAD2Convert = FALSE;											/* AD変換遅延状態フラグ */
u8	u8_ADC_ad2FinishedCounterMax = (u8)0;										/* AD2チャンネルAD変換終了カウンタ最大値 */

ST_CMF_RESULT_IIRF_U16	st_ADC_resultThermMtr;									/* ThermMtr	: モータサーミスタ0温度ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_resultThermFet;									/* ThermFet: FETサーミスタ温度ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_resultThermCapa;									/* ThermCapa: コンデンササーミスタ温度ＡＤ */

ST_CMF_RESULT_IIRF_U16	st_ADC_resultCtU;										/* IU	: Ｕ相電流ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_resultCtV;										/* IV	: Ｖ相電流ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_resultBattV;										/* VB	: バッテリ電圧ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_resultInvV;										/* VINV	: インバータ電圧ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_result12V;										/* 12V	: 12V監視電圧ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_resultAccel;										/* ACCEL: アクセル電圧ＡＤ */

ST_CMF_RESULT_IIRF_U16	st_ADC_resultVu;										/* Vu	: U相電圧ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_resultVv;										/* Vv	: V相電圧ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_resultVw;										/* Vw	: W相電圧ＡＤ */
ST_CMF_RESULT_IIRF_U16	st_ADC_resultIF;										/* IF	: エキサイタ電流ＡＤ */

ST_CMF_RESULT_IIRF_S32	st_ADC_resultIdRef_10mA;								/* IdRef: Ｄ軸電流指令値[10mA] */
ST_CMF_RESULT_IIRF_S32	st_ADC_resultIqRef_10mA;								/* IqRef: Ｑ軸電流指令値[10mA] */
ST_CMF_RESULT_IIRF_S32	st_ADC_resultIdFb_10mA;									/* IdFb	: Ｄ軸電流[10mA] */
ST_CMF_RESULT_IIRF_S32	st_ADC_resultIqFb_10mA;									/* IqFb	: Ｑ軸電流[10mA] */
ST_CMF_RESULT_IIRF_S32	st_ADC_resultVd_10mV;									/* Vd	: Ｄ軸電圧[10mV] */
ST_CMF_RESULT_IIRF_S32	st_ADC_resultVq_10mV;									/* Vq	: Ｑ軸電圧[10mV] */

s16	s16_ADC_motorTemperatureAve_0p1degC		= (s16)0;							/* モータ温度平均値[0.1degC] */
s16	s16_ADC_fetTemperatureAve_0p1degC		= (s16)0;							/* FET温度平均値[0.1degC] */
s16	s16_ADC_capTemperatureAve_0p1degC		= (s16)0;							/* CAP温度平均値[0.1degC] */

s16	s16_ADC_maxTemperature_0p1degC			= (s16)0;							/* 最大温度[0.1degC] */
s16	s16_ADC_minTemperature_0p1degC			= (s16)0;							/* 最低温度[0.1degC] */
volatile u16	u16_ADC_accelVoltageAve_10mV	= (u16)0;							/* アクセル電圧平均値[10mV] */

volatile u16	u16_ADC_adValueAccel = (u16)0;
volatile u16	u16_ADC_accelRate = (u16)0;											/* アクセル開度率：[100% = 1024] */

ST_ADC_ADVALUE2	st_ADC_adValue2		= {(u16)0,(u16)0,(u16)0,(u16)2048,(u16)0,(u16)2048,(u16)2048,(u16)0};	/* AD値生データ */

ST_ADC_ADVALUE0_AVE	st_ADC_adValue0Ave	= {(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_REFERENCE_BATT_ADVALUE),(U16_REFERENCE_BATT_ADVALUE)};	/* AD値平均化データ */
ST_ADC_ADVALUE0_AVE*	pst_ADC_adValue0Ave	= &st_ADC_adValue0Ave;					/* AD値平均化データの参照先 */

ST_ADC_ADVALUE1_AVE	st_ADC_adValue1Ave	= {(u16)0,(u16)0,(u16)0,(u16)0};		/* AD値平均化データ */
ST_ADC_ADVALUE1_AVE*	pst_ADC_adValue1Ave	= &st_ADC_adValue1Ave;					/* AD値平均化データの参照先 */

ST_ADC_ADVALUE2_AVE	st_ADC_adValue2Ave	= {(u16)0,(u16)0,(u16)0,(u16)2048,(u16)0,(u16)2048,(u16)2048,(u16)0};	/* AD値平均化データ */
ST_ADC_ADVALUE2_AVE*	pst_ADC_adValue2Ave	= &st_ADC_adValue2Ave;					/* AD値平均化データの参照先 */

ST_ADC_ADVALUE_AVE	st_ADC_adValueAve	= {(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_EX_IF_OFFSET_STANDARD_ADVALUE)};	/* AD値平均化データ */
ST_ADC_ADVALUE_AVE*	pst_ADC_adValueAve	= &st_ADC_adValueAve;					/* AD値平均化データの参照先 */

ST_ADC_DQ32	st_ADC_dq32Ave	= {(s32)0,(s32)0,(s32)0,(s32)0};					/* DQ軸平均化データ */
ST_ADC_DQ32*	pst_ADC_dq32Ave	= &st_ADC_dq32Ave;									/* DQ軸平均化データの参照先 */

ST_ADC_DQ	st_ADC_dqAve	= {(s16)0,(s16)0,(s16)0,(s16)0};					/* DQ軸平均化データ */
ST_ADC_DQ*	pst_ADC_dqAve	= &st_ADC_dqAve;									/* DQ軸平均化データの参照先 */

#else

static bool	b_ADC_isStartAD2Convert = FALSE;											/* AD2変換開始フラグ */
static bool	b_ADC_isDelayAD2Convert = FALSE;											/* AD変換遅延状態フラグ */
static u8	u8_ADC_ad2FinishedCounterMax = (u8)0;										/* AD2チャンネルAD変換終了カウンタ最大値 */

static ST_CMF_RESULT_IIRF_U16	st_ADC_resultThermMtr;									/* ThermMtr	: モータサーミスタ0温度ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_resultThermFet;									/* ThermFet: FETサーミスタ温度ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_resultThermCapa;									/* ThermCapa: コンデンササーミスタ温度ＡＤ */

static ST_CMF_RESULT_IIRF_U16	st_ADC_resultCtU;										/* IU	: Ｕ相電流ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_resultCtV;										/* IV	: Ｖ相電流ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_resultBattV;										/* VB	: バッテリ電圧ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_resultInvV;										/* VINV	: インバータ電圧ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_result12V;										/* 12V	: 12V監視電圧ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_resultAccel;										/* ACCEL: アクセル電圧ＡＤ */

static ST_CMF_RESULT_IIRF_U16	st_ADC_resultVu;										/* Vu	: U相電圧ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_resultVv;										/* Vv	: V相電圧ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_resultVw;										/* Vw	: W相電圧ＡＤ */
static ST_CMF_RESULT_IIRF_U16	st_ADC_resultIF;										/* IF	: エキサイタ電流ＡＤ */

static ST_CMF_RESULT_IIRF_S32	st_ADC_resultIdRef_10mA;								/* IdRef: Ｄ軸電流指令値[10mA] */
static ST_CMF_RESULT_IIRF_S32	st_ADC_resultIqRef_10mA;								/* IqRef: Ｑ軸電流指令値[10mA] */
static ST_CMF_RESULT_IIRF_S32	st_ADC_resultIdFb_10mA;									/* IdFb	: Ｄ軸電流[10mA] */
static ST_CMF_RESULT_IIRF_S32	st_ADC_resultIqFb_10mA;									/* IqFb	: Ｑ軸電流[10mA] */
static ST_CMF_RESULT_IIRF_S32	st_ADC_resultVd_10mV;									/* Vd	: Ｄ軸電圧[10mV] */
static ST_CMF_RESULT_IIRF_S32	st_ADC_resultVq_10mV;									/* Vq	: Ｑ軸電圧[10mV] */

static s16	s16_ADC_motorTemperatureAve_0p1degC		= (s16)0;							/* モータ温度平均値[0.1degC] */
static s16	s16_ADC_fetTemperatureAve_0p1degC		= (s16)0;							/* FET温度平均値[0.1degC] */
static s16	s16_ADC_capTemperatureAve_0p1degC		= (s16)0;							/* CAP温度平均値[0.1degC] */

static s16	s16_ADC_maxTemperature_0p1degC			= (s16)0;							/* 最大温度[0.1degC] */
static s16	s16_ADC_minTemperature_0p1degC			= (s16)0;							/* 最低温度[0.1degC] */
static volatile u16	u16_ADC_accelVoltageAve_10mV	= (u16)0;							/* アクセル電圧平均値[10mV] */

static volatile u16	u16_ADC_adValueAccel = (u16)0;
static volatile u16	u16_ADC_accelRate = (u16)0;											/* アクセル開度率：[100% = 1024] */

static ST_ADC_ADVALUE2	st_ADC_adValue2		= {(u16)0,(u16)0,(u16)0,(u16)2048,(u16)0,(u16)2048,(u16)2048,(u16)0};	/* AD値生データ */

static ST_ADC_ADVALUE0_AVE	st_ADC_adValue0Ave	= {(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_REFERENCE_BATT_ADVALUE),(U16_REFERENCE_BATT_ADVALUE)};	/* AD値平均化データ */
static ST_ADC_ADVALUE0_AVE*	pst_ADC_adValue0Ave	= &st_ADC_adValue0Ave;					/* AD値平均化データの参照先 */

static ST_ADC_ADVALUE1_AVE	st_ADC_adValue1Ave	= {(u16)0,(u16)0,(u16)0,(u16)0};		/* AD値平均化データ */
static ST_ADC_ADVALUE1_AVE*	pst_ADC_adValue1Ave	= &st_ADC_adValue1Ave;					/* AD値平均化データの参照先 */

static ST_ADC_ADVALUE2_AVE	st_ADC_adValue2Ave	= {(u16)0,(u16)0,(u16)0,(u16)2048,(u16)0,(u16)2048,(u16)2048,(u16)0};	/* AD値平均化データ */
static ST_ADC_ADVALUE2_AVE*	pst_ADC_adValue2Ave	= &st_ADC_adValue2Ave;					/* AD値平均化データの参照先 */

static ST_ADC_ADVALUE_AVE	st_ADC_adValueAve	= {(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_EX_IF_OFFSET_STANDARD_ADVALUE)};	/* AD値平均化データ */
static ST_ADC_ADVALUE_AVE*	pst_ADC_adValueAve	= &st_ADC_adValueAve;					/* AD値平均化データの参照先 */

static ST_ADC_DQ32	st_ADC_dq32Ave	= {(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0};		/* DQ軸平均化データ */
static ST_ADC_DQ32*	pst_ADC_dq32Ave	= &st_ADC_dq32Ave;									/* DQ軸平均化データの参照先 */

static ST_ADC_DQ	st_ADC_dqAve	= {(s16)0,(s16)0,(s16)0,(s16)0};					/* DQ軸平均化データ */
static ST_ADC_DQ*	pst_ADC_dqAve	= &st_ADC_dqAve;									/* DQ軸平均化データの参照先 */

#endif
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief AD変換後処理0.5msecループ
 * @detail AD取得値の平均演算処理
 * @param none
 * @return none
 */
void	V_ADC_Loop500us(void)
{
	if( (S12AD2.ADCSR.BIT.ADST) == TRUE )
	{
		b_ADC_isStartAD2Convert = TRUE;
	}

	ST_ADC_DQ32		a_st_dq32		= St_DD_MTR_GetDq32();										/* DQ軸データ */
	ST_ADC_ADVALUE0	a_st_adValue0	= St_DD_MTR_GetAdValue0();									/* AD0データ取得 */
	ST_ADC_ADVALUE1	a_st_adValue1	= St_DD_EX_GetAdValue1();									/* AD1データ取得 */

/*----- 電流センサAD値の平均値を算出 ------*/
	st_ADC_resultCtU	= St_CMF_CalcIIRFilterU16ByShift( a_st_adValue0.u16_adValueCtU,	st_ADC_resultCtU.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultCtU.u32_sum );				/* Ｕ相電流ＡＤ値更新 */
	st_ADC_resultCtV	= St_CMF_CalcIIRFilterU16ByShift( a_st_adValue0.u16_adValueCtV,	st_ADC_resultCtV.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultCtV.u32_sum );				/* Ｖ相電流ＡＤ値更新 */

	st_ADC_resultBattV	= St_CMF_CalcIIRFilterU16ByShift( a_st_adValue0.u16_adValueBattV,	st_ADC_resultBattV.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultBattV.u32_sum );		/* バッテリ電圧ＡＤ値更新 */
	st_ADC_resultInvV	= St_CMF_CalcIIRFilterU16ByShift( a_st_adValue0.u16_adValueInvV,	st_ADC_resultInvV.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultInvV.u32_sum );		/* インバータ電圧ＡＤ値更新 */

	st_ADC_resultVu		= St_CMF_CalcIIRFilterU16ByShift( a_st_adValue1.u16_adValueVu,		st_ADC_resultVu.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultVu.u32_sum );			/* U相電圧ＡＤ値更新 */
	st_ADC_resultVv		= St_CMF_CalcIIRFilterU16ByShift( a_st_adValue1.u16_adValueVv,		st_ADC_resultVv.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultVu.u32_sum );			/* V相電圧ＡＤ値更新 */
	st_ADC_resultVw		= St_CMF_CalcIIRFilterU16ByShift( a_st_adValue1.u16_adValueVw,		st_ADC_resultVw.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultVu.u32_sum );			/* W相電圧ＡＤ値更新 */
	st_ADC_resultIF		= St_CMF_CalcIIRFilterU16ByShift( a_st_adValue1.u16_adValueIf,		st_ADC_resultIF.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultCtU.u32_sum );			/* エキサイタ電流ＡＤ値更新 */


/*----- DQ軸電流指令値の平均値を算出 ------*/
	st_ADC_resultIdRef_10mA	= St_CMF_CalcIIRFilterS32ByShift( a_st_dq32.s32_idRef_10mA,st_ADC_resultIdRef_10mA.s32_ave,(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultIdRef_10mA.s64_sum );	/* IdRef値更新 */
	st_ADC_resultIqRef_10mA	= St_CMF_CalcIIRFilterS32ByShift( a_st_dq32.s32_iqRef_10mA,st_ADC_resultIqRef_10mA.s32_ave,(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultIqRef_10mA.s64_sum );	/* IqRef値更新 */
/*----- DQ軸電流の平均値を算出 ------*/
	st_ADC_resultIdFb_10mA	= St_CMF_CalcIIRFilterS32ByShift( a_st_dq32.s32_idFb_10mA,st_ADC_resultIdFb_10mA.s32_ave,(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultIdFb_10mA.s64_sum );	/* IdFb値更新 */
	st_ADC_resultIqFb_10mA	= St_CMF_CalcIIRFilterS32ByShift( a_st_dq32.s32_iqFb_10mA,st_ADC_resultIqFb_10mA.s32_ave,(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultIqFb_10mA.s64_sum );	/* IqFb値更新 */
/*----- DQ軸電圧の平均値を算出 ------*/
	st_ADC_resultVd_10mV	= St_CMF_CalcIIRFilterS32ByShift( a_st_dq32.s32_vd_10mV,	st_ADC_resultVd_10mV.s32_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultVd_10mV.s64_sum );		/* Ｖｄ値更新 */
	st_ADC_resultVq_10mV	= St_CMF_CalcIIRFilterS32ByShift( a_st_dq32.s32_vq_10mV,	st_ADC_resultVq_10mV.s32_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultVq_10mV.s64_sum );		/* Ｖｑ値更新 */


/*----- フィルタ後の値を構造体に纏める ------*/
	st_ADC_dq32Ave.s32_idRef_10mA	= st_ADC_resultIdRef_10mA.s32_ave;						/* d軸電流Ref[10mA] */
	st_ADC_dq32Ave.s32_iqRef_10mA	= st_ADC_resultIqRef_10mA.s32_ave;						/* q軸電流Ref[10mA] */
	st_ADC_dq32Ave.s32_idFb_10mA	= st_ADC_resultIdFb_10mA.s32_ave;						/* d軸電流FB[10mA] */
	st_ADC_dq32Ave.s32_iqFb_10mA	= st_ADC_resultIqFb_10mA.s32_ave;						/* q軸電流FB[10mA] */
	st_ADC_dq32Ave.s32_vd_10mV		= st_ADC_resultVd_10mV.s32_ave;							/* d軸電圧[10mV] */
	st_ADC_dq32Ave.s32_vq_10mV		= st_ADC_resultVq_10mV.s32_ave;							/* q軸電圧[10mV] */

	st_ADC_adValue0Ave.u16_adValueAveCtU	= st_ADC_resultCtU.u16_ave;						/* Ｕ相電流ＡＤ */
	st_ADC_adValue0Ave.u16_adValueAveCtV	= st_ADC_resultCtV.u16_ave;						/* Ｖ相電流ＡＤ */
	st_ADC_adValue0Ave.u16_adValueAveBattV	= st_ADC_resultBattV.u16_ave;					/* バッテリ電圧ＡＤ */
	st_ADC_adValue0Ave.u16_adValueAveInvV	= st_ADC_resultInvV.u16_ave;					/* インバータ電圧ＡＤ */

	/* オフセット取得用 */
	st_ADC_adValueAve.u16_adValueAveCtU	= st_ADC_resultCtU.u16_ave;							/* Ｕ相電流ＡＤ */
	st_ADC_adValueAve.u16_adValueAveCtV	= st_ADC_resultCtV.u16_ave;							/* Ｖ相電流ＡＤ */
	st_ADC_adValueAve.u16_adValueAveIf	= st_ADC_resultIF.u16_ave;							/* 界磁電流ＡＤ */

	/* for CuW */
	st_ADC_dqAve.s16_idFb_100mA	= (s16)(st_ADC_dq32Ave.s32_idFb_10mA / (s32)10);			/* d軸電流FB[10mA] */
	st_ADC_dqAve.s16_iqFb_100mA	= (s16)(st_ADC_dq32Ave.s32_iqFb_10mA / (s32)10);			/* q軸電流FB[10mA] */
	st_ADC_dqAve.s16_vd_100mV	= (s16)(st_ADC_dq32Ave.s32_vd_10mV / (s32)10);				/* d軸電圧[10mV] */
	st_ADC_dqAve.s16_vq_100mV	= (s16)(st_ADC_dq32Ave.s32_vq_10mV / (s32)10);				/* q軸電圧[10mV] */

/*-----AD2変換結果取り込み処理-----*/
	u8	a_u8_adCounter = (u8)0;
	while ( (S12AD2.ADCSR.BIT.ADST) == TRUE )
	{																							/* AD1ループスキャン完了してない */
		if( a_u8_adCounter < (U8_ADC_AD2_MAX_WAITING_COUNT) )
		{
			a_u8_adCounter++;																	/* 待ち */
			/* for DEBUG */
			u8_ADC_ad2FinishedCounterMax = (u8)CMF_GETMAX(a_u8_adCounter,u8_ADC_ad2FinishedCounterMax);
		}
		else
		{																						/* 最大待ち時間が超えると */
			b_ADC_isDelayAD2Convert = TRUE;														/* AD変換遅延状態フラグON */
			break;
		}
	}

	if( (b_ADC_isDelayAD2Convert == FALSE) && (b_ADC_isStartAD2Convert == TRUE) )
	{
		/*----AD2取り込み----*/
		R_Config_S12AD2_Get_ValueResult(ADCHANNEL0,&st_ADC_adValue2.u16_adValue12V);			/* 12V監視電圧 ＡＤ(生) */
		R_Config_S12AD2_Get_ValueResult(ADCHANNEL1,&st_ADC_adValue2.u16_adValueGp1);			/* 汎用1 ＡＤ(生) */
		R_Config_S12AD2_Get_ValueResult(ADCHANNEL2,&st_ADC_adValue2.u16_adValueExt5V);			/* 外部5V監視電圧 ＡＤ(生) */
		R_Config_S12AD2_Get_ValueResult(ADCHANNEL4,&st_ADC_adValue2.u16_adValueThermMtr);		/* サーミスタモータ ＡＤ(生) */
		R_Config_S12AD2_Get_ValueResult(ADCHANNEL5,&st_ADC_adValue2.u16_adValueGp2);			/* 汎用2 ＡＤ(生) */
		R_Config_S12AD2_Get_ValueResult(ADCHANNEL6,&st_ADC_adValue2.u16_adValueThermFet);		/* サーミスタＦＥＴ ＡＤ(生) */
		R_Config_S12AD2_Get_ValueResult(ADCHANNEL7,&st_ADC_adValue2.u16_adValueThermCapa);		/* サーミスタコンデンサ基板 ＡＤ(生) */
		R_Config_S12AD2_Get_ValueResult(ADCHANNEL9,&st_ADC_adValue2.u16_adValueAccel);			/* アクセル電圧 ＡＤ(生) */
		b_ADC_isStartAD2Convert = FALSE;
	}

/*----- サーミスタ電圧ADの平均値を算出 ------*/
	st_ADC_resultThermMtr		= St_CMF_CalcIIRFilterU16ByShift( st_ADC_adValue2.u16_adValueThermMtr, st_ADC_resultThermMtr.u16_ave, (u8)(U8_ADC_TIME_CONST_FACTOR), st_ADC_resultThermMtr.u32_sum );		/* サーミスタ0(Temp)ＡＤ値更新 */
	s16_ADC_motorTemperatureAve_0p1degC = S16_CMF_2DMapCal( (s16)st_ADC_resultThermMtr.u16_ave, (U8_MOTOR_TEMP_POINT_NUM), (s16*)s16_ADC_motorTempXarray_AdValue, (s16*)s16_ADC_motorTempYarray_0p1degC );	/* 2Dマップよりモータ温度平均値( パワー部1 )を算出*/

	st_ADC_resultThermFet		= St_CMF_CalcIIRFilterU16ByShift( st_ADC_adValue2.u16_adValueThermFet, st_ADC_resultThermFet.u16_ave, (u8)(U8_ADC_TIME_CONST_FACTOR), st_ADC_resultThermFet.u32_sum );		/* サーミスタ1(Temp)ＡＤ値更新 */
	s16_ADC_fetTemperatureAve_0p1degC = S16_CMF_2DMapCal( (s16)st_ADC_resultThermFet.u16_ave, (U8_INV_TEMP_POINT_NUM), (s16*)s16_ADC_invTempXarray_AdValue, (s16*)s16_ADC_invTempYarray_0p1degC );			/* 2DマップよりFET温度平均値( パワー部1 )を算出*/

	st_ADC_resultThermCapa		= St_CMF_CalcIIRFilterU16ByShift( st_ADC_adValue2.u16_adValueThermCapa, st_ADC_resultThermCapa.u16_ave, (u8)(U8_ADC_TIME_CONST_FACTOR), st_ADC_resultThermCapa.u32_sum );		/* サーミスタ1(Temp)ＡＤ値更新 */
	s16_ADC_capTemperatureAve_0p1degC = S16_CMF_2DMapCal( (s16)st_ADC_resultThermCapa.u16_ave, (U8_INV_TEMP_POINT_NUM), (s16*)s16_ADC_invTempXarray_AdValue, (s16*)s16_ADC_invTempYarray_0p1degC );			/* 2DマップよりFET温度平均値( パワー部1 )を算出*/

	s16_ADC_maxTemperature_0p1degC = (s16)CMF_GETMAX(s16_ADC_fetTemperatureAve_0p1degC,s16_ADC_capTemperatureAve_0p1degC);																					/* 最大温度更新[0.1degC] */
	s16_ADC_minTemperature_0p1degC = (s16)CMF_GETMIN(s16_ADC_fetTemperatureAve_0p1degC,s16_ADC_capTemperatureAve_0p1degC);																					/* 最大温度更新[0.1degC] */

/*----- ADユニット2 AD値の平均値を算出 ------*/
	st_ADC_result12V	= St_CMF_CalcIIRFilterU16ByShift( st_ADC_adValue2.u16_adValue12V,	st_ADC_result12V.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_result12V.u32_sum );							/* 12V監視電圧ＡＤ値更新 */
	st_ADC_resultAccel	= St_CMF_CalcIIRFilterU16ByShift( st_ADC_adValue2.u16_adValueAccel,	st_ADC_resultAccel.u16_ave,	(u8)(U8_ADC_TIME_CONST_FACTOR),	st_ADC_resultAccel.u32_sum );						/* アクセル電圧ＡＤ値更新 */

	st_ADC_adValue2Ave.u16_adValueAve12V	= st_ADC_result12V.u16_ave;						/* 12V監視電圧ＡＤ */
	st_ADC_adValue2Ave.u16_adValueAveAccel	= st_ADC_resultAccel.u16_ave;						/* アクセル電圧ＡＤ */

	/* アクセル電圧 */
	u16_ADC_accelVoltageAve_10mV = (u16)(u32)st_ADC_adValue2Ave.u16_adValueAveAccel * (u32)(U16_SYS_AD_MAX_10MV) / (u32)(U16_SYS_AD_INPUT_MAX);			/* アクセル電圧算出 */
	/* アクセル開度率 */
	s32 a_s32_accelAdValue = st_ADC_adValue2Ave.u16_adValueAveAccel;
	a_s32_accelAdValue = S32_CMF_GETMAX( (a_s32_accelAdValue - (s32)(U16_ACCEL_ADC_MARGIN_LOWER)), (s32)0);
	a_s32_accelAdValue = a_s32_accelAdValue * (s32)(U16_SYS_AD_INPUT_MAX) / (s32)((U16_ACCEL_ADC_MARGIN_UPPER) - (U16_ACCEL_ADC_MARGIN_LOWER));
	u16 a_u16_accelAdValue = (u16)S32_CMF_GETMIN( a_s32_accelAdValue, (s32)(U16_SYS_AD_INPUT_MAX));
	u16_ADC_adValueAccel = (u16)(a_u16_accelAdValue >> 2);										/* MAX 4096 -> 1024 */
	u16_ADC_accelRate = (u16)0;
	if( u16_ADC_adValueAccel > (U16_ADC_ACCEL_OFFSET) )
	{
		u16_ADC_accelRate = (u16)((u32)(u16_ADC_adValueAccel - (U16_ADC_ACCEL_OFFSET)) * (u32)(U16_SYS_ACCEL_RATE_MAX) / (u32)((U16_SYS_ACCEL_RATE_MAX) - (U16_ADC_ACCEL_OFFSET)));
	}
}


/**
 * @brief AD変換変数初期化処理(再起動用)
 * @param none
 * @return none
 */
void	V_ADC_InitializeForRestart(void)
{
	s16_ADC_motorTemperatureAve_0p1degC	= (s16)0;		/* モータ温度平均値[0.1degC] */
	s16_ADC_fetTemperatureAve_0p1degC	= (s16)0;		/* FET温度平均値[0.1degC] */
	s16_ADC_capTemperatureAve_0p1degC	= (s16)0;		/* CAP温度平均値[0.1degC] */
	s16_ADC_maxTemperature_0p1degC		= (s16)0;		/* 最大温度[0.1degC] */
	s16_ADC_minTemperature_0p1degC		= (s16)0;		/* 最低温度[0.1degC] */

}

/**
 * @brief モータサーミスタ ＡＤ値提供(フィルタ値)
 * @param none
 * @return st_ADC_resultThermMtr.u16_ave
 */
u16	U16_ADC_GetAdValueAveMotThermistor(void)
{
	return	st_ADC_resultThermMtr.u16_ave;					/* フィルタ後のAD値 */
}

/**
 * @brief FETサーミスタ ＡＤ値提供(フィルタ値)
 * @param none
 * @return st_ADC_resultThermFet.u16_ave
 */
u16 U16_ADC_GetAdValueAveFetThermistor(void)
{
	return	st_ADC_resultThermFet.u16_ave;					/* フィルタ後のAD値 */
}

/**
 * @brief コンデンササーミスタ ＡＤ値提供(フィルタ値)
 * @param none
 * @return st_ADC_resultThermCapa.u16_ave
 */
u16 U16_ADC_GetAdValueAveCapThermistor(void)
{
	return	st_ADC_resultThermCapa.u16_ave;					/* フィルタ後のAD値 */
}

/**
 * @brief U相CT ＡＤ値提供(フィルタ値)
 * @param none
 * @return st_ADC_adValueAve.u16_adValueAveCtU
 */
u16	U16_ADC_GetAdValueAveCtUPhase(void)
{
	return	st_ADC_adValue0Ave.u16_adValueAveCtU;			/* フィルタ後のAD値 */
}

/**
 * @brief W相CT ＡＤ値提供(フィルタ値)
 * @param none
 * @return st_ADC_adValueAve.u16_adValueAveCtV
 */
u16	U16_ADC_GetAdValueAveCtVPhase(void)
{
	return	st_ADC_adValue0Ave.u16_adValueAveCtV;			/* フィルタ後のAD値 */
}

/**
 * @brief アクセル ＡＤ値提供(フィルタ値)
 * @param none
 * @return st_ADC_adValueAve.u16_adValueAveAccel
 */
u16	U16_ADC_GetAdValueAveAccel(void)
{
	return	st_ADC_adValue2Ave.u16_adValueAveAccel;			/* フィルタ後のAD値 */
}

/**
 * @brief 12V監視電圧 ＡＤ値提供(フィルタ値)
 * @param none
 * @return st_ADC_adValue2Ave.u16_adValueAve12V
 */
u16	U16_ADC_GetAdValueAve12V(void)
{
	return	st_ADC_adValue2Ave.u16_adValueAve12V;				/* フィルタ後のAD値 */
}

/**
 * @brief モータ温度提供(フィルタ値)
 * @param none
 * @return s16_ADC_motorTemperatureAve_0p1degC
 */
s16	S16_ADC_GetMotorTemperature_0p1degC(void)
{
	return	s16_ADC_motorTemperatureAve_0p1degC;			/* 加重平均された温度 */
}

/**
 * @brief FET温度提供(フィルタ値)
 * @param none
 * @return s16_ADC_fetTemperatureAve_0p1degC
 */
s16	S16_ADC_GetFetTemperature_0p1degC(void)
{
	return	s16_ADC_fetTemperatureAve_0p1degC;			/* 加重平均された温度 */
}

/**
* @brief コンデンサ温度提供(フィルタ値)
 * @param none
 * @return s16_ADC_capTemperatureAve_0p1degC
 */
s16	S16_ADC_GetCapTemperature_0p1degC(void)
{
	return	s16_ADC_capTemperatureAve_0p1degC;			/* 加重平均された温度 */
}

/**
 * @brief 最大温度提供
 * @param none
 * @return s16_ADC_maxTemperature_0p1degC
 */
s16	S16_ADC_GetMaxTemperature_0p1degC(void)
{
	return	s16_ADC_maxTemperature_0p1degC;					/* 最大温度 */
}

/**
 * @brief 最低温度提供
 * @param none
 * @return s16_ADC_minTemperature_0p1degC
 */
s16	S16_ADC_GetMinTemperature_0p1degC(void)
{
	return	s16_ADC_minTemperature_0p1degC;					/* 最大温度 */
}

/**
 * @brief アクセル開度率提供
 * @param none
 * @return u16_ADC_accelRate
 */
u16	U16_ADC_GetAccelRate(void)
{
	return	u16_ADC_accelRate;
}

/**
 * @brief アクセル電圧提供
 * @param none
 * @return u16_ADC_accelVoltageAve_10mV
 */
u16	U16_ADC_GetAccelVoltageAve_10mV(void)
{
	return	u16_ADC_accelVoltageAve_10mV;
}

/**
 * @brief DQ軸平均化データ提供
 */
ST_ADC_DQ32	St_ADC_GetDq32Ave(void)
{
	return	*pst_ADC_dq32Ave;
}

/**
 * @brief AD値平均化データ提供
 */
ST_ADC_ADVALUE0_AVE	St_ADC_GetAdValue0Ave(void)
{
	return	*pst_ADC_adValue0Ave;
}

/**
 * @brief AD値平均化データ提供
 */
ST_ADC_ADVALUE1_AVE	St_ADC_GetAdValue1Ave(void)
{
	return	*pst_ADC_adValue1Ave;
}

/**
 * @brief AD値平均化データ提供
 */
ST_ADC_ADVALUE2_AVE	St_ADC_GetAdValue2Ave(void)
{
	return	*pst_ADC_adValue2Ave;
}

/**
 * @brief AD値平均化データ提供
 */
ST_ADC_ADVALUE_AVE	St_ADC_GetAdValueAve(void)
{
	return	*pst_ADC_adValueAve;
}

