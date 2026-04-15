/**
 * @file BATTERY.c
 * @brief バッテリ関連処理
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "BATTERY.h"
#include "Config_DA.h"

#include "DEFINE_SETTING.h"						/* IFDEF設定 */
#include "DEFINE_SYSTEM.h"						/*定数定義*/
#include "DEFINE_MOTOR.h"						/*定数定義*/
#include "TYPEDEF_ADC.h"
#include "TYPEDEF_ABN.h"
#include "COMMON_FUNCTION.h"					/* 汎用関数群 */
#include "COMMON_MACRO.h"
#include "DETECT_BATTERY.h"
#include "DETECT_MOTOR.h"
#include "MOTOR.h"
#include "ADC.h"
#include "MOTOR.h"
#include "CAN.h"
#include "MODE_CONDUCTOR.h"
#include "DD_EXCITER.h"

/* for DEBUG */
#include "ramdef.h"

/* Private define ------------------------------------------------------------*/
#define	U8_BAT_TIME_CONST_FACTOR			(u8)2					/* AD変換後のLPF時定数係数 LPFの時定数40msに相当（ 2^2 = 4：4×10ms）*/
#define	U8_BAT_DCDC_CONSUME_LIMIT_100MA		(u8)50					/* 通信ミスマッチで下げる電流量を制限(暫定) */

/* Private variables ---------------------------------------------------------*/
#define U8_BAT_IB_COEF_POINT_NUM		(u8)9
const s16	s16_BAT_ib1CoefXarray_rpm[(U8_BAT_IB_COEF_POINT_NUM)]
	= {(s16)(-10000),(s16)0,	(s16)1000,	(s16)2000,	(s16)3000,	(s16)4000,	(s16)5000,	(s16)6000,(s16)10000};		/* モータ回転数[rpm] */
const s16	s16_BAT_ib1CoefYarray[(U8_BAT_IB_COEF_POINT_NUM)]
	= {(s16)873,	(s16)873,	(s16)873,	(s16)844,	(s16)852,	(s16)853,	(s16)853,	(s16)850,	(s16)850};		/* 効率係数 3/4 / 効率 * 1024 */


const s16	s16_BAT_ib2CoefXarray_rpm[(U8_BAT_IB_COEF_POINT_NUM)]
	= {(s16)(-10000),(s16)0,	(s16)1000,	(s16)2000,	(s16)3000,	(s16)4000,	(s16)5000,	(s16)6000,(s16)10000};		/* モータ回転数[rpm] */
const s16	s16_BAT_ib2CoefYarray[(U8_BAT_IB_COEF_POINT_NUM)]
	= {(s16)900,	(s16)900,	(s16)823,	(s16)745,	(s16)682,	(s16)619,	(s16)596,	(s16)573,	(s16)573};		/* 効率係数 3/4 / 効率 * 1024 */


const s16	s16_BAT_ib3CoefXarray_rpm[(U8_BAT_IB_COEF_POINT_NUM)]
	= {(s16)(-10000),(s16)(-6000),	(s16)(-5000),(s16)(-4000),	(s16)(-3000),(s16)(-2000),	(s16)(-1000),(s16)0,	(s16)10000};	/* モータ回転数[rpm] */
const s16	s16_BAT_ib3CoefYarray[(U8_BAT_IB_COEF_POINT_NUM)]
	= {(s16)895,	(s16)895,		(s16)877,	(s16)859,		(s16)860,	(s16)860,		(s16)860,	(s16)860,	(s16)860};		/* 効率係数 3/4 / 効率 * 1024 */


const s16	s16_BAT_ib4CoefXarray_rpm[(U8_BAT_IB_COEF_POINT_NUM)]
	= {(s16)(-10000),(s16)(-6000),	(s16)(-5000),(s16)(-4000),	(s16)(-3000),(s16)(-2000),	(s16)(-1000),(s16)0,	(s16)10000};	/* モータ回転数[rpm] */
const s16	s16_BAT_ib4CoefYarray[(U8_BAT_IB_COEF_POINT_NUM)]
	= {(s16)565,	(s16)565,		(s16)589,	(s16)612,		(s16)681,	(s16)749,		(s16)825,	(s16)900,	(s16)900};		/* 効率係数 3/4 / 効率 * 1024 */


#define U8_BAT_IF_COEF_POINT_NUM		(u8)6
const s16	s16_BAT_ifCoefXarray_100mA[(U8_BAT_IF_COEF_POINT_NUM)]
	= {(s16)(-1000),(s16)0,		(s16)50,	(s16)120,	(s16)1500,	(s16)1000};		/* If指令値[100mA] */
const s16	s16_BAT_ifCoefYarray[(U8_BAT_IF_COEF_POINT_NUM)]
	= {(s16)80,		(s16)80,	(s16)80,	(s16)94,	(s16)94,	(s16)94};		/* 効率[×100] */

const s16	s16_BAT_ifCoefXarray_10mA[(U8_BAT_IF_COEF_POINT_NUM)]
	= {(s16)(-10000),(s16)0,		(s16)500,	(s16)1200,	(s16)15000,	(s16)10000};	/* If指令値[10mA] */
//const s16	s16_BAT_ifCoefYarray[(U8_BAT_IF_COEF_POINT_NUM)]
//	= {(s16)80,		(s16)80,	(s16)80,	(s16)94,	(s16)94,	(s16)94};			/* 効率[×100] */

#ifdef	DEBUG_WATCH_ACTIVITY

u16	u16_BAT_battVoltAve_10mV					= (u16)0;									/* 電源電圧平均値 */
volatile u16	u16_BAT_battVoltRaw_10mV		= (u16)0;									/* 電源電圧生値 */
u16	u16_BAT_invVoltAve_10mV						= (u16)0;									/* インバータ電圧平均値 */

s16	s16_BAT_battCurrent_10mA					= (s16)0;									/* 推定バッテリ電流[0.1A] */
s16	s16_BAT_battCurrentEx_10mA					= (s16)0;									/* 界磁側推定バッテリ電流[0.1A] */
s16	s16_BAT_totalBattCurrent_10mA				= (s16)0;									/* 合計推定バッテリ電流[0.1A] */
s16	s16_BAT_battCurrentAve_10mA					= (s16)0;									/* 推定平均バッテリ電流[0.1A] */

ST_CMF_RESULT_IIRF_S16	st_BAT_resultIb_10mA;												/* Ib	: 推定バッテリ電流[10mA] */

#else

static u16	u16_BAT_battVoltAve_10mV					= (u16)0;									/* 電源電圧平均値 */
static volatile u16	u16_BAT_battVoltRaw_10mV			= (u16)0;									/* 電源電圧生値 */
static u16	u16_BAT_invVoltAve_10mV						= (u16)0;									/* インバータ電圧平均値 */

static s16	s16_BAT_battCurrent_10mA					= (s16)0;									/* 推定バッテリ電流[0.1A] */
static s16	s16_BAT_battCurrentEx_10mA					= (s16)0;									/* 界磁側推定バッテリ電流[0.1A] */
static s16	s16_BAT_totalBattCurrent_10mA				= (s16)0;									/* 合計推定バッテリ電流[0.1A] */
static s16	s16_BAT_battCurrentAve_10mA					= (s16)0;									/* 推定平均バッテリ電流[0.1A] */

static ST_CMF_RESULT_IIRF_S16	st_BAT_resultIb_10mA;												/* Ib	: 推定バッテリ電流[10mA] */

#endif

/* Private function prototypes -----------------------------------------------*/
static s16	s16_BAT_CalcEstimatedBattCurrentAve_10mA(void);											/* バッテリ電流推定値平均化[10mA] */


/**
 * @brief バッテリ関連変数初期化処理(再起動用)
 * @param none
 * @return none
 */
void	V_BAT_InitializeForRestart(void)
{
	u16_BAT_battVoltAve_10mV		= (u16)0;														/* 電源電圧平均値 */
	u16_BAT_invVoltAve_10mV			= (u16)0;														/* インバータ電圧平均値 */
	s16_BAT_battCurrent_10mA		= (s16)0;														/* 推定バッテリ電流[10mA] */
	s16_BAT_battCurrentAve_10mA		= (s16)0;														/* 推定平均バッテリ電流[10mA] */
}

/**
 * @brief バッテリ関連10msecループ処理
 * @param none
 * @return none
 */
void	V_BAT_Loop10ms(void)
{
	ST_ADC_ADVALUE0_AVE	a_st_adValue0Ave	= St_ADC_GetAdValue0Ave();										/* 電流AD値平均データ取得 */

/*----- バッテリ、インバータ電圧の平均値を算出 ------*/
	/* 補正値反映バッテリ電圧単位変換[100mV] */
	u16_BAT_battVoltAve_10mV = (u16)((u32)a_st_adValue0Ave.u16_adValueAveBattV * (u32)(U16_SYS_BAT_VDC_MAX_10MV) >> (u32)12);
	u16_BAT_invVoltAve_10mV = (u16)((u32)a_st_adValue0Ave.u16_adValueAveInvV * (u32)(U16_SYS_BAT_VDC_MAX_10MV) >> (u32)12);

	u16_BAT_battVoltRaw_10mV = (u16)(u32)((u32)U16_MTR_GetAdValueBattVolt() * (u32)(U16_SYS_BAT_VDC_MAX_10MV) >> (u32)12);			/* バッテリ電圧単位変換[10mV] */
/*-------バッテリ電流推定-------*/

	s16_BAT_battCurrentAve_10mA = s16_BAT_CalcEstimatedBattCurrentAve_10mA();								/* バッテリ電流推定(平均値)[10mA] */
}

/**
 * @brief	バッテリ電流推定値平均化[0.1A]
 * @param	none
 * @return	none
 * @note	Ib = ( Vd * Id + Vq *Iq ) * 3/2 * 1/√3 / 効率 / Vb
 * @note	Ib = ( Vd * Id + Vq *Iq ) * 3/2 * 1/√3 / (効率 * Vb)
 * @note
 * @note	相対変換係数		：3/2
 * @note	線電流相電流変換係数：1/√3
 */
static s16	s16_BAT_CalcEstimatedBattCurrentAve_10mA(void)
{
	ST_ADC_DQ32	a_st_dq32Ave	= {(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0};								/* DQ軸平均値データ */
	a_st_dq32Ave	= St_ADC_GetDq32Ave();																	/* DQ軸平均データ取得 */
	s32	a_s32_vdid = a_st_dq32Ave.s32_vd_10mV * a_st_dq32Ave.s32_idFb_10mA;
	s32	a_s32_vqiq = a_st_dq32Ave.s32_vq_10mV * a_st_dq32Ave.s32_iqFb_10mA;
	u16	a_u16_VbAve_10mV = u16_BAT_battVoltAve_10mV;
	s16 a_s16_motRev_rpm = S16_MTR_GetRevolutionAve_rpm();
	u8	a_u8_quadrant = U8_MTR_GetQuadrant();																/* 象限取得 */
	u16	a_u16_ibCoef = (u16)0;
	switch( (u8)a_u8_quadrant )
	{
	case 1:
		a_u16_ibCoef = (u16)S16_CMF_2DMapCal( a_s16_motRev_rpm, (U8_BAT_IB_COEF_POINT_NUM), (s16*)s16_BAT_ib1CoefXarray_rpm, (s16*)s16_BAT_ib1CoefYarray );	/* 効率を算出 */
		break;

	case 2:
		a_u16_ibCoef = (u16)S16_CMF_2DMapCal( a_s16_motRev_rpm, (U8_BAT_IB_COEF_POINT_NUM), (s16*)s16_BAT_ib2CoefXarray_rpm, (s16*)s16_BAT_ib2CoefYarray );	/* 効率を算出 */
		break;

	case 3:
		a_u16_ibCoef = (u16)S16_CMF_2DMapCal( a_s16_motRev_rpm, (U8_BAT_IB_COEF_POINT_NUM), (s16*)s16_BAT_ib3CoefXarray_rpm, (s16*)s16_BAT_ib3CoefYarray );	/* 効率を算出 */
		break;

	case 4:
		a_u16_ibCoef = (u16)S16_CMF_2DMapCal( a_s16_motRev_rpm, (U8_BAT_IB_COEF_POINT_NUM), (s16*)s16_BAT_ib4CoefXarray_rpm, (s16*)s16_BAT_ib4CoefYarray );	/* 効率を算出 */
		break;

	default:
		a_u16_ibCoef = (u16)S16_CMF_2DMapCal( a_s16_motRev_rpm, (U8_BAT_IB_COEF_POINT_NUM), (s16*)s16_BAT_ib1CoefXarray_rpm, (s16*)s16_BAT_ib1CoefYarray );	/* 効率を算出 */
		break;
	}
	s16_BAT_battCurrent_10mA = (s16)((a_s32_vdid + a_s32_vqiq) * (s32)a_u16_ibCoef / (s32)a_u16_VbAve_10mV >> (s32)10);

	/* エキサイタ側のDC電流 */
	s16	a_s16_ifRef_10mA = S16_DD_EX_GetIfRef_10mA();
	s16	a_s16_ifFb_10mA = S16_DD_EX_GetIfFb_10mA();
	s16	a_s16_vf_10mV = S16_DD_EX_GetVf_10mV();
	s16	a_s16_ifCoef = S16_CMF_2DMapCal( a_s16_ifRef_10mA, (U8_BAT_IF_COEF_POINT_NUM), (s16*)s16_BAT_ifCoefXarray_10mA, (s16*)s16_BAT_ifCoefYarray );	/* 効率を算出 */
	s16_BAT_battCurrentEx_10mA = (s16)((s32)a_s16_ifFb_10mA * (s32)a_s16_vf_10mV / ((s32)a_u16_VbAve_10mV * (s32)a_s16_ifCoef));						/* 10mA単位 */
	/* 合算 */
	s16_BAT_totalBattCurrent_10mA = s16_BAT_battCurrent_10mA + s16_BAT_battCurrentEx_10mA;

/*----- トータルDC電流推定値の平均値を算出 ------*/
	st_BAT_resultIb_10mA	= St_CMF_CalcIIRFilterS16ByShift( s16_BAT_totalBattCurrent_10mA,	st_BAT_resultIb_10mA.s16_ave,	(u8)(U8_BAT_TIME_CONST_FACTOR),	st_BAT_resultIb_10mA.s32_sum );	/* 推定Ib値更新 */
	return st_BAT_resultIb_10mA.s16_ave;
}

/**
 * @brief バッテリ電圧提供(フィルタ値:補正反映)
 * @param none
 * @return u16_BAT_battVoltAve_100mV
 */
u16 U16_BAT_GetBattVoltAve_100mV(void)
{
	return u16_BAT_battVoltAve_10mV / (u16)10;																		/* フィルタ後の電圧(補正反映) */
}

/**
 * @brief インバータ電圧提供(フィルタ値:補正反映)
 * @param none
 * @return u16_BAT_invVoltAve_100mV
 */
u16 U16_BAT_GetInvVoltAve_100mV(void)
{
	return u16_BAT_invVoltAve_10mV / (u16)10;																/* フィルタ後の電圧(補正反映) */
}

/**
 * @brief バッテリ電圧提供(フィルタ値:補正反映)
 * @param none
 * @return u16_BAT_battVoltAve_10mV
 */
u16 U16_BAT_GetBattVoltAve_10mV(void)
{
	return u16_BAT_battVoltAve_10mV;																		/* フィルタ後の電圧(補正反映) */
}

/**
 * @brief インバータ電圧提供(フィルタ値:補正反映)
 * @param none
 * @return u16_BAT_invVoltAve_10mV
 */
u16 U16_BAT_GetInvVoltAve_10mV(void)
{
	return u16_BAT_invVoltAve_10mV;																		/* フィルタ後の電圧(補正反映) */
}

/**
 * @brief バッテリ電流値提供
 * @param none
 * @return s16_BAT_battCurrent_10mA
 */
s16	S16_BAT_GetBattCurrent_10mA(void)
{
	return s16_BAT_battCurrent_10mA;
}

/**
 * @brief バッテリ電流平均値提供
 * @param none
 * @return s16_BAT_battCurrentAve_10mA
 */
s16	S16_BAT_GetBattCurrentAve_10mA(void)
{
	return s16_BAT_battCurrentAve_10mA;
}

/**
 * @brief バッテリ電流平均値提供
 * @param none
 * @return s16_BAT_battCurrentAve_10mA / (s16)10
 */
s16	S16_BAT_GetBattCurrentAve_100mA(void)
{
	return s16_BAT_battCurrentAve_10mA / (s16)10;
}
