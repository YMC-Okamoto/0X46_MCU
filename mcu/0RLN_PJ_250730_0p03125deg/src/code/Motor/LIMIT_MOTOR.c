/**
 * @file LIMIT_MOTOR.c
 * @brief Iq指令値制限処理
 * @note 10ms周期
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "DEFINE_SYSTEM.h"		/*定数定義 */
#include "DEFINE_SETTING.h"
#include "DEFINE_MOTOR.h"
#include "COMMON_FUNCTION.h"	/* 汎用関数群 */

#include "Config_DA.h"

/* DDL */
#include "DD_MOTOR.h"
#include "DD_EXCITER.h"

#include "DETECT_MOTOR.h"
#include "TYPEDEF_ABN.h"
#include "MODE_CONDUCTOR.h"
#include "BATTERY.h"
#include "ADC.h"
#include "MOTOR.h"
#include "LIMIT_MOTOR.h"
#include "CURRENT_MAP.h"
#include "IMLIMIT_TABLE.h"
#include "MCU.h"
#include "CAN.h"
#include "PREVENT_CHATTERING.h"		/* チャタリング防止 */
/* for DEBUG */
#include "ramdef.h"

/* Private define ------------------------------------------------------------ */
#define	U16_MTR_TRQREF_INC_HIGH_100MNM						(u16)110					/* 漸増1ステップ分[100mNm] */
#define	U16_MTR_TRQREF_INC_LOW_100MNM						(u16)50						/* 漸増1ステップ分[100mNm] */

#define	U16_MTR_TRQREF_DEC_HIGH_100MNM						(u16)1100					/* 漸減1ステップ分[100mNm] */
#define	U16_MTR_TRQREF_DEC_LOW_100MNM						(u16)50						/* 漸減1ステップ分[100mNm] */

#define	U16_ACCEL_RATE_STEP_RESPONSE_ON_THRESHOLD			(u16)300					/* ステップ応答ON閾値 */
#define	U16_ACCEL_RATE_STEP_RESPONSE_OFF_THRESHOLD			(u16)250					/* ステップ応答OFF閾値 */
#define	U16_ACCEL_RATE_STEP_RESPONSE_READY_ON_THRESHOLD		(u16)150					/* ステップ応答準備ON閾値 */
#define	U16_ACCEL_RATE_STEP_RESPONSE_READY_OFF_THRESHOLD	(u16)100					/* ステップ応答準備OFF閾値 */
#define	S16_MTR_STEP_RESPONSE_READY_TORQUE_100MNM			(s16)10						/* ステップ応答準備中トルク[100mNm] */

#define	S16_MTR_STEP_RESPONSE_TORQUE_100MNM					(s16)300					/* ステップ応答時トルク初期値[100mNm] */

/* Private variables --------------------------------------------------------- */
#define U8_MAX_TORQUE_MAP_POINT_NUM		(u8)9

const s16 s16_LIM_MTR_maxTorqueMapXarray_rpm[(U8_MAX_TORQUE_MAP_POINT_NUM)]
	= {(s16)0,(s16)500,(s16)1000,(s16)2000,(s16)3000,(s16)4000,(s16)5000,(s16)6000,(s16)10000};				/* モータ回転数[rpm] */

const s16 s16_LIM_MTR_maxTorqueMapYarray_mNm[(U8_MAX_TORQUE_MAP_POINT_NUM)]
	= {(s16)0,(s16)600,(s16)600,(s16)500,(s16)400,(s16)300,(s16)250,(s16)200,(s16)200};						/* 最大トルク[mNm] */

typedef union
{
	u8			byte;
	struct
	{
		u8	b_stepResponseIsReady			:1;			/* b0:ステップ応答準備中 */
		u8	b_stepResponseIsExecute			:1;			/* b1:ステップ応答実行中 */
		u8									:6;			/* b2-7:未使用 */
	}bit;
}UNI_LIM_MTR_STEPRESPONSEFLG;


typedef enum{
	STEP_RESPONSE_MODE0 = 0,
	STEP_RESPONSE_MODE1,
	STEP_RESPONSE_MODE2,
	STEP_RESPONSE_MODE3,
}EN_LIM_MTR_STEP_RESPONSE_MODE;

#ifdef	DEBUG_WATCH_ACTIVITY

UNI_LIM_MTR_STEPRESPONSEFLG	uni_LIM_MTR_stepResponseFlg;									/* ステップ応答フラグ */

EN_LIM_MTR_STEP_RESPONSE_MODE	en_LIM_MTR_stepResponseMode = STEP_RESPONSE_MODE0;		/* ステップ応答モード */

//s16	s16_LIM_MTR_iqRef_100mA = (s16)0;												/* 最終Iq制限値 */
//s16	s16_LIM_MTR_idRef_100mA = (s16)0;												/* 最終Id制限値 */
s32	s32_LIM_MTR_iqRef_10mA = (s32)0;													/* Iq制限値 */
s32	s32_LIM_MTR_idRef_10mA = (s32)0;													/* Id制限値 */
s16	s16_LIM_MTR_ifRef_10mA = (s16)0;													/* If制限値 */
s32	s32_LIM_MTR_idRefTemp_10mA = (s32)0;												/* Id制限値 */

volatile s16	s16_LIM_MTR_revolution_rpm = (s16)0;										/* モータ回転数[rpm] */
s16	s16_LIM_MTR_revolutionAve_rpm = (s16)0;												/* 平均モータ回転数[rpm] */
s16	s16_LIM_MTR_revolutionLast_rpm = (s16)0;											/* トルクマップ参照用最終モータ回転数[rpm] */
s16	s16_LIM_MTR_torqueRef_100mNm = (s16)0;												/* トルク指令値[0.1Nm] */
s16	s16_LIM_MTR_torqueRefTemp1_100mNm = (s16)0;											/* 仮トルク指令値[0.1Nm] */
s16	s16_LIM_MTR_torqueRefLast_100mNm = (s16)0;											/* トルク指令前回値[0.1Nm] */
s16	s16_LIM_MTR_torqueRefTemp_100mNm = (s16)0;											/* トルク指令値[0.1Nm] */
s16	s16_LIM_MTR_torqueRefMax_100mNm = (U16_MTR_TORQUE_MAX_100MNM);						/* 最大トルク指令値[0.1Nm] */
s16	s16_LIM_MTR_stepTorqueRefTemp_100mNm = (S16_MTR_STEP_RESPONSE_TORQUE_100MNM);		/* 仮ステップ応答トルク指令値[0.1Nm] */
s16	s16_LIM_MTR_stepResponseTorqueRef_100mNm = (s16)0;									/* ステップ応答トルク指令値[0.1Nm] */
u16	u16_LIM_MTR_accelRate = (u16)0;														/* アクセル開度率 */
u8	u8_LIM_MTR_quadrant = (u8)0;														/* 象限 */
u16	u16_LIM_MTR_trqMode = (u16)0;

#else

static UNI_LIM_MTR_STEPRESPONSEFLG	uni_LIM_MTR_stepResponseFlg;								/* ステップ応答フラグ */

static EN_LIM_MTR_STEP_RESPONSE_MODE	en_LIM_MTR_stepResponseMode = STEP_RESPONSE_MODE0;		/* ステップ応答モード */

//static s16	s16_LIM_MTR_iqRef_100mA = (s16)0;													/* 最終Iq制限値 */
//static s16	s16_LIM_MTR_idRef_100mA = (s16)0;													/* 最終Id制限値 */
static s32	s32_LIM_MTR_iqRef_10mA = (s32)0;													/* Iq制限値 */
static s32	s32_LIM_MTR_idRef_10mA = (s32)0;													/* Id制限値 */
static s16	s16_LIM_MTR_ifRef_10mA = (s16)0;													/* If制限値 */
static s32	s32_LIM_MTR_idRefTemp_10mA = (s32)0;												/* Id制限値 */

static volatile s16	s16_LIM_MTR_revolution_rpm = (s16)0;										/* モータ回転数[rpm] */
static s16	s16_LIM_MTR_revolutionAve_rpm = (s16)0;												/* 平均モータ回転数[rpm] */
static s16	s16_LIM_MTR_revolutionLast_rpm = (s16)0;											/* トルクマップ参照用最終モータ回転数[rpm] */
static s16	s16_LIM_MTR_torqueRef_100mNm = (s16)0;												/* トルク指令値[0.1Nm] */
static s16	s16_LIM_MTR_torqueRefTemp1_100mNm = (s16)0;											/* 仮トルク指令値[0.1Nm] */
static s16	s16_LIM_MTR_torqueRefLast_100mNm = (s16)0;											/* トルク指令前回値[0.1Nm] */
static s16	s16_LIM_MTR_torqueRefTemp_100mNm = (s16)0;											/* トルク指令値[0.1Nm] */
static s16	s16_LIM_MTR_torqueRefMax_100mNm = (U16_MTR_TORQUE_MAX_100MNM);						/* 最大トルク指令値[0.1Nm] */
static s16	s16_LIM_MTR_stepTorqueRefTemp_100mNm = (S16_MTR_STEP_RESPONSE_TORQUE_100MNM);		/* 仮ステップ応答トルク指令値[0.1Nm] */
static s16	s16_LIM_MTR_stepResponseTorqueRef_100mNm = (s16)0;									/* ステップ応答トルク指令値[0.1Nm] */
static u16	u16_LIM_MTR_accelRate = (u16)0;														/* アクセル開度率 */
static u8	u8_LIM_MTR_quadrant = (u8)0;														/* 象限 */
static u16	u16_LIM_MTR_trqMode = (u16)0;

#endif
/* Private function prototypes -----------------------------------------------*/
static void	v_LIM_MTR_SetIdIqIfRef( s16 a_s16_rev_rpm );											/* 電流指令値セット */
#ifdef	TRQ_GRADUALLY_ACTIVITY
static s16	s16_LIM_MTR_CalcTorquRefGradually(s16 a_s16_trqRef_100mNm);								/* トルク指令値漸増漸減処理 */
#endif
static s16	s16_LIM_MTR_CalcIqLimByDischrgStopDec_100mA(u16 a_u16_vb, u16 a_s16_iqLim);				/* 放電停止によるIq制限値算出 */

static void v_LIM_MTR_stepResponse( u16 a_u16_accelRate );											/* ステップ応答 */
static void v_LIM_MTR_stepResponseMode0( u16 a_u16_accelRate );										/* ステップ応答 モード0：初期状態 */
static void v_LIM_MTR_stepResponseMode1( u16 a_u16_accelRate );										/* ステップ応答 モード1：準備状態 */
static void v_LIM_MTR_stepResponseMode2( u16 a_u16_accelRate );										/* ステップ応答 モード2：ステップ応答 */
static void v_LIM_MTR_stepResponseMode3( u16 a_u16_accelRate );										/* ステップ応答 モード3：停止状態 */

/**
 * @brief Iq制限値算出
 * @param none
 * @return none
 * @note 10msec周期
 */
void V_LIM_MTR_Loop10ms(void)
{
	uni_LIM_MTR_stepResponseFlg.byte = (u8)0;

	s16 a_s16_iqLim = (S16_MTR_IQ_MAX_100MA);
	s32 a_s32_iqLim = (S32_MTR_IQ_MAX_10MA);
	u16 a_u16_vb = U16_BAT_GetBattVoltAve_10mV() / (u16)10;																	/* バッテリ電圧取得 */
	s16_LIM_MTR_revolution_rpm = S16_DD_MTR_GetRevolution_rpm();															/* モータ回転数取得[rpm] */
	s16_LIM_MTR_revolutionAve_rpm = S16_MTR_GetRevolutionAve_rpm();															/* 平均モータ回転数取得[rpm] */
	s16	a_s16_rev_rpm = s16_LIM_MTR_revolutionAve_rpm;
	u16 a_u16_tmp = U16_BAT_GetBattVoltAve_100mV();
	u16 a_u16_vbAve_100mV = (u16)S32_CMF_GETMIN( (s32)a_u16_tmp, (s32)(U16_SYS_BAT_VDC_MAX_100MV) );
	s16_LIM_MTR_torqueRefLast_100mNm = s16_LIM_MTR_torqueRef_100mNm;														/* トルク指令前回値保存 */
	s16	a_s16_trqRef_100mNm = (s16)0;
	s16	a_s16_trqRefTemp_100mNm = (s16)0;
	s16	a_s16_torqueRef = S16_CAN_GetTorqueOrder_100mNm();																	/* トルク指令値取得 */
	u16_LIM_MTR_accelRate = U16_ADC_GetAccelRate();																			/* アクセル開度率取得 */
	u16	a_u16_accelRate = u16_LIM_MTR_accelRate;

	/*** ステップ応答 ***/
	if( UNI_GL_testFlg.bit.b_forBenchtest == TRUE )
	{
		if( UNI_GL_testFlg.bit.b_stepResponseActivity == TRUE )
		{
			v_LIM_MTR_stepResponse( a_u16_accelRate );																		/* ステップ応答 */

			a_s16_trqRefTemp_100mNm = s16_LIM_MTR_stepResponseTorqueRef_100mNm;												/* ステップ応答トルク指令値を仮トルク指令値にセット */
		}
		else
		{
			a_s16_trqRefTemp_100mNm = (s16)((u32)u16_LIM_MTR_accelRate * (u32)(U16_MTR_TORQUE_MAX_100MNM) / (u32)(U16_SYS_ACCEL_RATE_MAX));	/* トルク指令値算出 */
			en_LIM_MTR_stepResponseMode = STEP_RESPONSE_MODE0;																/* ステップ応答モード初期化 */
			s16_LIM_MTR_torqueRefTemp_100mNm = a_s16_trqRefTemp_100mNm;
		}
	}
	else
	{
		/*** 車両用(VCUあり) ***/
		a_s16_trqRefTemp_100mNm = (s16)S32_CMF_GETLIMIT( (s32)a_s16_torqueRef, (s32)(-(s16)(U16_MTR_TORQUE_MAX_100MNM)), (s32)(U16_MTR_TORQUE_MAX_100MNM) );
		en_LIM_MTR_stepResponseMode = STEP_RESPONSE_MODE0;																	/* ステップ応答モード初期化 */
	}

	if( UNI_GL_testFlg.bit.b_iqrefByPotentiometer == TRUE )
	{
		a_s32_iqLim = (s16)((s32)a_s32_iqLim * (s32)u16_LIM_MTR_accelRate / (s32)1024);
		s32_LIM_MTR_iqRef_10mA = a_s32_iqLim;																				/* 最終のIq要求指令値設定 */
		s32_LIM_MTR_idRef_10mA = s32_LIM_MTR_idRefTemp_10mA;																/* 最終のIq要求指令値設定 */

		return;
	}


/*======= 車両制限モードによるモータ電流制限値算出部 ======*/

	s16	a_s16_trqRefTemp1_100mNm = (s16)0;

	/** トルク指令漸減条件判定 **/
	if( B_MTR_PermittedByVehicleState() == FALSE )																			/* 上記3点のいずれかに該当しない */
	{
		a_s16_trqRefTemp_100mNm = (s16)0;																					/* トルク指令値を0にする */
	}

	/* for TEST */
	if( UNI_GL_testFlg.bit.b_revolutionConstant == TRUE )
	{
		switch( u16_LIM_MTR_trqMode )
		{
		case 0:
			if( a_s16_trqRefTemp_100mNm >= (s16)35 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)50;
				u16_LIM_MTR_trqMode = (u16)50;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)0;
			}
			break;

		case 50:
			if( a_s16_trqRefTemp_100mNm >= (s16)85 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)100;
				u16_LIM_MTR_trqMode = (u16)100;
			}
			else if( a_s16_trqRefTemp_100mNm < (s16)25 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)0;
				u16_LIM_MTR_trqMode = (u16)0;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)50;
				u16_LIM_MTR_trqMode = (u16)50;
			}
			break;

		case 100:
			if( a_s16_trqRefTemp_100mNm >= (s16)135 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)150;
				u16_LIM_MTR_trqMode = (u16)150;
			}
			else if( a_s16_trqRefTemp_100mNm <= (s16)65 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)50;
				u16_LIM_MTR_trqMode = (u16)50;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)100;
				u16_LIM_MTR_trqMode = (u16)100;
			}
			break;

		case 150:
			if( a_s16_trqRefTemp_100mNm >= (s16)185 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)200;
				u16_LIM_MTR_trqMode = (u16)200;
			}
			else if( a_s16_trqRefTemp_100mNm <= (s16)115 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)100;
				u16_LIM_MTR_trqMode = (u16)100;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)150;
				u16_LIM_MTR_trqMode = (u16)150;
			}
			break;

		case 200:
			if( a_s16_trqRefTemp_100mNm >= (s16)235 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)250;
				u16_LIM_MTR_trqMode = (u16)250;
			}
			else if( a_s16_trqRefTemp_100mNm <= (s16)165 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)150;
				u16_LIM_MTR_trqMode = (u16)150;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)200;
				u16_LIM_MTR_trqMode = (u16)200;
			}
			break;

		case 250:
			if( a_s16_trqRefTemp_100mNm >= (s16)285 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)300;
				u16_LIM_MTR_trqMode = (u16)300;
			}
			else if( a_s16_trqRefTemp_100mNm <= (s16)215 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)200;
				u16_LIM_MTR_trqMode = (u16)200;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)250;
				u16_LIM_MTR_trqMode = (u16)250;
			}
			break;

		case 300:
			if( a_s16_trqRefTemp_100mNm >= (s16)375 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)400;
				u16_LIM_MTR_trqMode = (u16)400;
			}
			else if( a_s16_trqRefTemp_100mNm <= (s16)265 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)250;
				u16_LIM_MTR_trqMode = (u16)250;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)300;
				u16_LIM_MTR_trqMode = (u16)300;
			}
			break;

		case 400:
			if( a_s16_trqRefTemp_100mNm >= (s16)475 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)500;
				u16_LIM_MTR_trqMode = (u16)500;
			}
			else if( a_s16_trqRefTemp_100mNm <= (s16)315 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)300;
				u16_LIM_MTR_trqMode = (u16)300;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)400;
				u16_LIM_MTR_trqMode = (u16)400;
			}
			break;

		case 500:
			if(  a_s16_trqRefTemp_100mNm >= (s16)575 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)600;
				u16_LIM_MTR_trqMode = (u16)600;
			}
			else if( a_s16_trqRefTemp_100mNm <= (s16)415 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)400;
				u16_LIM_MTR_trqMode = (u16)400;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)500;
				u16_LIM_MTR_trqMode = (u16)500;
			}
			break;

		case 600:
			if( a_s16_trqRefTemp_100mNm <= (s16)515 )
			{
				a_s16_trqRefTemp1_100mNm = (s16)500;
				u16_LIM_MTR_trqMode = (u16)500;
			}
			else
			{
				a_s16_trqRefTemp1_100mNm = (s16)600;
				u16_LIM_MTR_trqMode = (u16)600;
			}
			break;

		default:
				u16_LIM_MTR_trqMode = (u16)0;
			break;
		}

		a_s16_trqRefTemp_100mNm = a_s16_trqRefTemp1_100mNm;



		if( (a_s16_rev_rpm >= (s16)350) && (a_s16_rev_rpm <= (s16)600) )
		{
			a_s16_rev_rpm = (s16)500;
		}
		else if( (a_s16_rev_rpm >= (s16)700) && (a_s16_rev_rpm <= (s16)1450) )
		{
			a_s16_rev_rpm = (s16)1000;
		}
		else if( (a_s16_rev_rpm >= (s16)1550) && (a_s16_rev_rpm <= (s16)2450) )
		{
			a_s16_rev_rpm = (s16)2000;
		}
		else if( (a_s16_rev_rpm >= (s16)2550) && (a_s16_rev_rpm <= (s16)3450) )
		{
			a_s16_rev_rpm = (s16)3000;
		}
		else if( (a_s16_rev_rpm >= (s16)3550) && (a_s16_rev_rpm <= (s16)4450) )
		{
			a_s16_rev_rpm = (s16)4000;
		}
		else if( (a_s16_rev_rpm >= (s16)4550) && (a_s16_rev_rpm <= (s16)5450) )
		{
			a_s16_rev_rpm = (s16)5000;
		}
		else if( (a_s16_rev_rpm >= (s16)5550) && (a_s16_rev_rpm <= (s16)6500) )
		{
			a_s16_rev_rpm = (s16)6000;
		}
		else
		{
			/* MISRA-C 2004 14.10 対応 */
		}
	}
	/* テスト用負トルク対応 */
	if( UNI_GL_testFlg.bit.b_regenerateOrder == TRUE )
	{
		a_s16_trqRef_100mNm = a_s16_trqRefTemp_100mNm * (s16)(-1);
	}
	else
	{
		a_s16_trqRef_100mNm = a_s16_trqRefTemp_100mNm;
	}

	s16_LIM_MTR_torqueRefTemp1_100mNm = a_s16_trqRef_100mNm;

#ifdef	TRQ_GRADUALLY_ACTIVITY
	/** トルク指令漸増漸減処理 **/
	s16_LIM_MTR_torqueRef_100mNm = s16_LIM_MTR_CalcTorquRefGradually( s16_LIM_MTR_torqueRefTemp1_100mNm );					/* トルク指令漸増漸減処理 */
#else
	s16_LIM_MTR_torqueRef_100mNm = s16_LIM_MTR_torqueRefTemp1_100mNm;
#endif

	/* for DEBUG */
//#ifdef	DA_EANGLE
//	R_Config_DA0_Set_ConversionValue( (u16)((s16)CMF_GETABS(s16_LIM_MTR_torqueRefTemp1_100mNm) * (s16)5) );					/* DA0:漸増漸減前トルク指令×5 : MAX=3000 */
//#else
//	R_Config_DA0_Set_ConversionValue( (u16)((s16)CMF_GETABS(s16_LIM_MTR_torqueRefTemp1_100mNm) * (s16)5) );					/* DA0:漸増漸減前トルク指令×5 : MAX=3000 */
//	R_Config_DA1_Set_ConversionValue( (u16)((s16)CMF_GETABS(s16_LIM_MTR_torqueRef_100mNm) * (s16)5) );						/* DA1:最終トルク指令×5 : MAX=3000 */
//#endif
/*======= 放電制限によるモータ電流制限値算出部 ======*/
	s16 a_s16_iqLimByDischgStop = s16_LIM_MTR_CalcIqLimByDischrgStopDec_100mA( (s16)a_u16_vb, a_s16_iqLim );				/* 放電制限 BMS通信断線時のみ有効 */

	a_s16_iqLim = (s16)S32_CMF_GETMIN( (s32)a_s16_iqLim, (s32)a_s16_iqLimByDischgStop);										/* 放電漸減と比較 */

/*======= モータ回転数，トルク指令値によるd軸電流値算出部 ======*/
	v_LIM_MTR_SetIdIqIfRef( a_s16_rev_rpm );							/* 電流指令値セット */

}

/**
 * @brief 電流指令値セット
 * @param a_s16_rev_rpm
 * @return none
 * @detail 
 */
static void	v_LIM_MTR_SetIdIqIfRef( s16 a_s16_rev_rpm )
{
	s16_LIM_MTR_revolutionLast_rpm = a_s16_rev_rpm;

	/* 回転数による最大トルク算出 */
	u16	a_u16_rev_rpm = (u16)U32_CMF_GETABS( s16_LIM_MTR_revolutionLast_rpm );
	s16_LIM_MTR_torqueRefMax_100mNm = S16_CMF_2DMapCal( (s16)a_u16_rev_rpm, (u16)(U8_MAX_TORQUE_MAP_POINT_NUM), (s16*)s16_LIM_MTR_maxTorqueMapXarray_rpm,  (s16*)s16_LIM_MTR_maxTorqueMapYarray_mNm );
	s16_LIM_MTR_torqueRef_100mNm = (s16)S32_CMF_GETLIMIT( (s32)s16_LIM_MTR_torqueRef_100mNm, (s32)(-s16_LIM_MTR_torqueRefMax_100mNm), (s32)s16_LIM_MTR_torqueRefMax_100mNm );

	/* Id、Iq指令値に関してs16だとオーバーフローするのでマップ上は1/2した値とし、マップ参照後に2倍する */
/*======= モータ回転数，トルク指令値によるd軸電流値算出部 ======*/
//	s32	a_s32_idRefTemp = (s32)S16_CMF_3DMapCal( (U8_CURRENT_MAP_T_POINT_NUM), (s32)s16_LIM_MTR_torqueRef_100mNm, a_s16_rev_rpm,Id_Tmap );
	s32	a_s32_idRefTemp = (s32)S16_CMF_3DMapCal( (U8_CURRENT_MAP_T_POINT_NUM), (s32)s16_LIM_MTR_torqueRef_100mNm, s16_LIM_MTR_revolutionLast_rpm,Id_Tmap );
	s32_LIM_MTR_idRef_10mA = a_s32_idRefTemp * (s32)2;
/*======= モータ回転数，トルク指令値によるq軸電流値算出部 ======*/
//	s32	a_s32_iqRefTemp = (s32)S16_CMF_3DMapCal( (U8_CURRENT_MAP_T_POINT_NUM), (s32)s16_LIM_MTR_torqueRef_100mNm, a_s16_rev_rpm,Iq_Tmap );
	s32	a_s32_iqRefTemp = (s32)S16_CMF_3DMapCal( (U8_CURRENT_MAP_T_POINT_NUM), (s32)s16_LIM_MTR_torqueRef_100mNm, s16_LIM_MTR_revolutionLast_rpm,Iq_Tmap );
	s32_LIM_MTR_iqRef_10mA = a_s32_iqRefTemp * (s32)2;
/*======= モータ回転数，トルク指令値によるエキサイタ電流値算出部 ======*/
//	s16_LIM_MTR_ifRef_10mA = S16_CMF_3DMapCal( (U8_CURRENT_MAP_T_POINT_NUM), (s32)s16_LIM_MTR_torqueRef_100mNm, a_s16_rev_rpm,If_Tmap );
	s16_LIM_MTR_ifRef_10mA = S16_CMF_3DMapCal( (U8_CURRENT_MAP_T_POINT_NUM), (s32)s16_LIM_MTR_torqueRef_100mNm, s16_LIM_MTR_revolutionLast_rpm,If_Tmap );

/*======= CANによる電気角オフセット値上下限クリップ部 ======*/
	s16	a_s16_offsetCan =  (s8)CMF_GETLIMIT( (s8)S8_CAN_GetEangleOffsetFromCan(), (s8)(-90), (s8)90 );

	/* マップによるオフセットを加算しない */
	s16	a_s16_EangleOffset = a_s16_offsetCan + (s16)(U16_MTR_EANGLE_OFFSET_DEG);											/* デフォルトのオフセットとCANからのオフセットを加算 */

	if( a_s16_EangleOffset < (s16)0 )
	{
		for( ; a_s16_EangleOffset < (s16)0; )
		{																													/* 電気角度がマイナスの場合 */
			a_s16_EangleOffset = ( a_s16_EangleOffset + (s16)(U16_ENC_EANGLE_MAX_DEG) );									/* 電気角度処理 */
		}
	}
	else
	{
		for( ;a_s16_EangleOffset >= (U16_ENC_EANGLE_MAX_DEG); )
		{																													/* 電気角度が60度以上の場合 */
			a_s16_EangleOffset = (u16)( a_s16_EangleOffset - (U16_ENC_EANGLE_MAX_DEG) );									/* 電気角度処理 */
		}
	}

	V_DD_MTR_SetEangleOffset_deg((u16)a_s16_EangleOffset);

	/* 象限の設定（第〇象限） */
	if( (a_s16_rev_rpm >= (s16)0) && (s16_LIM_MTR_torqueRef_100mNm >= (s16)0) )												/* 回転数：正、トルク：正 */
	{
		u8_LIM_MTR_quadrant = (u8)1;																						/* 第1象限 */
	}
	else if( (a_s16_rev_rpm >=(s16)0) && (s16_LIM_MTR_torqueRef_100mNm < (s16)0) )											/* 回転数：正、トルク：負 */
	{
		u8_LIM_MTR_quadrant = (u8)2;																						/* 第2象限 */
	}
	else if( (a_s16_rev_rpm < (s16)0) && (s16_LIM_MTR_torqueRef_100mNm < (s16)0) )											/* 回転数：負、トルク：負 */
	{
		u8_LIM_MTR_quadrant = (u8)3;																						/* 第3象限 */
	}
	else if( (a_s16_rev_rpm < (s16)0) && (s16_LIM_MTR_torqueRef_100mNm >= (s16)0) )											/* 回転数：負、トルク：正 */
	{
		u8_LIM_MTR_quadrant = (u8)4;																						/* 第4象限 */
	}
	else
	{
			/* MISRA-C 2004 14.10 対応 */
	}
}

#ifdef	TRQ_GRADUALLY_ACTIVITY
/**
 * @brief トルク指令値漸増漸減処理
 * @param a_s16_trqRef_100mNm
 * @return a_s16_trqRefTemp
 */
static s16	s16_LIM_MTR_CalcTorquRefGradually(s16 a_s16_trqRef_100mNm)
{
	s16	a_s16_trqRefTemp = (s16)0;
	s16	a_s16_trqRefLast = s16_LIM_MTR_torqueRefLast_100mNm;
	s16	a_s16_trqRefDiff = a_s16_trqRef_100mNm - a_s16_trqRefLast;

	if( a_s16_trqRefDiff < (s16)0 )
	{																			/** 指令値漸減 **/
		if( (s16)CMF_GETABS(a_s16_trqRefDiff) > (s16)(U16_MTR_TRQREF_DEC_HIGH_100MNM) )				/* 指令値の差がHIGHレベルより大きい場合 */
		{
			a_s16_trqRefTemp = a_s16_trqRefLast - (s16)(U16_MTR_TRQREF_DEC_HIGH_100MNM);			/* 漸減値を高くする */
		}
		else if( (s16)CMF_GETABS(a_s16_trqRefDiff) >(s16)(U16_MTR_TRQREF_DEC_LOW_100MNM))			/* 指令値の差がLOWレベルより大きい場合 */
		{
			a_s16_trqRefTemp = a_s16_trqRefLast - (s16)(U16_MTR_TRQREF_DEC_LOW_100MNM);				/* 漸減値を低に設定 */
		}
		else																						/* 指令値の差がLOWレベルより小さい場合 */
		{
			a_s16_trqRefTemp = a_s16_trqRef_100mNm;													/* 漸減処理しない */
		}
	}
	else
	{																			/** 指令値漸増 **/
		if( a_s16_trqRefDiff > (s16)(U16_MTR_TRQREF_INC_HIGH_100MNM) )								/* 指令値の差がHIGHレベルより大きい場合 */
		{
			a_s16_trqRefTemp = a_s16_trqRefLast + (s16)(U16_MTR_TRQREF_INC_HIGH_100MNM);			/* 漸増値を高くする */
		}
		else if( a_s16_trqRefDiff >(s16)(U16_MTR_TRQREF_INC_LOW_100MNM))							/* 指令値の差がLOWレベルより大きい場合 */
		{
			a_s16_trqRefTemp = a_s16_trqRefLast + (s16)(U16_MTR_TRQREF_INC_LOW_100MNM);				/* 漸増値を低に設定 */
		}
		else																						/* 指令値の差がLOWレベルより小さい場合 */
		{
			a_s16_trqRefTemp = a_s16_trqRef_100mNm;													/* 漸増処理しない */
		}
	}

	return	a_s16_trqRefTemp;
}
#endif


/**
 * @brief ステップ応答
 * @param none
 * @return none
 */

/*------ 動作モードの判定 ------*/

static void v_LIM_MTR_stepResponse( u16 a_u16_accelRate )
{

	switch( en_LIM_MTR_stepResponseMode )
	{
	case STEP_RESPONSE_MODE0:									/* mode 0 */
		v_LIM_MTR_stepResponseMode0( a_u16_accelRate );
		break;
	case STEP_RESPONSE_MODE1:									/* mode 1 */
		v_LIM_MTR_stepResponseMode1( a_u16_accelRate );
		break;
	case STEP_RESPONSE_MODE2:									/* mode 2 */
		v_LIM_MTR_stepResponseMode2( a_u16_accelRate );
		break;
	case STEP_RESPONSE_MODE3:									/* mode 3 */
	default:
		v_LIM_MTR_stepResponseMode3( a_u16_accelRate );
		break;
	}
}

/*--------------------------------------*/
/*	モード０	初期状態				*/
/*--------------------------------------*/
static void v_LIM_MTR_stepResponseMode0( u16 a_u16_accelRate )
{
	if( a_u16_accelRate > (U16_ACCEL_RATE_STEP_RESPONSE_READY_ON_THRESHOLD) )				/* アクセル開度を確認 */
	{
		en_LIM_MTR_stepResponseMode = STEP_RESPONSE_MODE1;									/* モード←１ */
		s16_LIM_MTR_stepResponseTorqueRef_100mNm = (S16_MTR_STEP_RESPONSE_READY_TORQUE_100MNM);
	}
	else
	{
		en_LIM_MTR_stepResponseMode = STEP_RESPONSE_MODE0;									/* モード←0 */
		s16_LIM_MTR_stepResponseTorqueRef_100mNm = (s16)0;									/* トルクゼロ */
	}

	if( UNI_GL_testFlg.bit.b_stepResponseIfGateOffCansel == TRUE )
	{
		V_DD_EX_CanselForcedGateOff();														/* 界磁強制ゲートオフ解除 */
	}

}

/*--------------------------------------*/
/*	モード１	準備状態				*/
/*--------------------------------------*/
static void v_LIM_MTR_stepResponseMode1( u16 a_u16_accelRate )
{
	if( u16_LIM_MTR_accelRate >= (U16_ACCEL_RATE_STEP_RESPONSE_ON_THRESHOLD) )				/* アクセル開度を確認 */
	{
		en_LIM_MTR_stepResponseMode = STEP_RESPONSE_MODE2;									/* モード←2 */
		s16_LIM_MTR_stepResponseTorqueRef_100mNm = s16_LIM_MTR_stepTorqueRefTemp_100mNm;	/* ステップ応答用トルク指令値セット */
	}
}

/*--------------------------------------*/
/*	モード２	ステップ応答			*/
/*--------------------------------------*/
static void v_LIM_MTR_stepResponseMode2( u16 a_u16_accelRate )
{
	if( u16_LIM_MTR_accelRate < (U16_ACCEL_RATE_STEP_RESPONSE_OFF_THRESHOLD) )				/* アクセル開度を確認 */
	{
		en_LIM_MTR_stepResponseMode = STEP_RESPONSE_MODE3;									/* モード←3 */
		s16_LIM_MTR_stepResponseTorqueRef_100mNm = (s16)0;									/* トルクゼロ */
	}
}

/*--------------------------------------*/
/*	モード３	停止状態				*/
/*--------------------------------------*/
static void v_LIM_MTR_stepResponseMode3( u16 a_u16_accelRate )
{
	if( UNI_GL_testFlg.bit.b_stepResponseIfGateOff == TRUE )
	{
		V_DD_EX_ForcedGateOff();															/* 界磁強制ゲートオフ */
	}
	else
	{
		V_DD_EX_CanselForcedGateOff();														/* 界磁強制ゲートオフ解除 */
	}

	if( u16_LIM_MTR_accelRate < (U16_ACCEL_RATE_STEP_RESPONSE_READY_OFF_THRESHOLD) )		/* アクセル開度を確認 */
	{
		en_LIM_MTR_stepResponseMode = STEP_RESPONSE_MODE0;									/* モード←0 */
		s16_LIM_MTR_stepResponseTorqueRef_100mNm = (s16)0;									/* トルクゼロ */
	}
}


/**
 * @brief 放電漸減によるIq制限値の設定
 * @param a_u16_vb
 * @param a_s16_iqLim
 * @return a_s16_iqLim
 * @detail メインBMSとの通信断線時のみ処理すること
 */
static s16 s16_LIM_MTR_CalcIqLimByDischrgStopDec_100mA(u16 a_u16_vb, u16 a_s16_iqLim)
{
	if( a_u16_vb <  (U16_DISCHRGSTOP_DEC_BATT_VOLT_100MV) )																	/* 放電漸減開始電圧 確認 */
	{
		if( a_u16_vb < (U16_UNDER_BATT_VOLT_100MV) )																		/* 放電終了電圧 確認 */
		{
			a_s16_iqLim = (s16)0;																							/* 放電終了電圧以上 → 回生指令 = 0 */
		}
		else
		{																													/* 放電終了電圧以下 → 線形補間：回生指令値×(終了電圧 - 電池電圧) / (終了電圧 - 漸減開始電圧) */
			a_s16_iqLim = (s16)((s32)a_s16_iqLim * (s32)( (s16)a_u16_vb - (s16)(U16_UNDER_BATT_VOLT_100MV) ) / (s32)( (s16)(U16_DISCHRGSTOP_DEC_BATT_VOLT_100MV) - (s16)(U16_UNDER_BATT_VOLT_100MV)));
			a_s16_iqLim = (s16)S32_CMF_GETMAX( (s32)a_s16_iqLim, (s32)0 );
		}
	}
	return	a_s16_iqLim;
}

/**
 * @brief マップによるIq指令値提供[10mA]
 * @param none
 * @return s32_LIM_MTR_iqRef_10mA
 */
s32	S32_LIM_MTR_GetIqRef_10mA(void)
{
//	return	(s32)s16_LIM_MTR_iqRef_100mA * (s32)10;
	return	s32_LIM_MTR_iqRef_10mA;
}

/**
 * @brief マップによるId制指令値提供[10mA]
 * @param none
 * @return s32_LIM_MTR_iqRef_10mA
 */
s32	S32_LIM_MTR_GetIdRef_10mA(void)
{
//	return	(s32)s16_LIM_MTR_idRef_100mA * (s32)10;
	return	s32_LIM_MTR_idRef_10mA;
}

/**
 * @brief マップによるIf指令値提供[0.1A]
 * @param none
 * @return s16_LIM_MTR_ifRef_10mA
 */
s16	S16_LIM_MTR_GetIfRef_10mA(void)
{
	return	s16_LIM_MTR_ifRef_10mA;
}

/**
 * @brief 最終トルク指令値提供[0.1Nm]
 * @param none
 * @return s16_LIM_MTR_torqueRef_100mNm
 */
s16	S16_LIM_MTR_GetTorqueRef_100mNm(void)
{
	return	s16_LIM_MTR_torqueRef_100mNm;
}

/**
 * @brief 象限提供
 * @param none
 * @return u8_LIM_MTR_quadrant
 */
u8	U8_LIM_MTR_GetQuadrant(void)
{
	return	u8_LIM_MTR_quadrant;
}

/**
 * @brief Iq指令値制限処理変数初期化
 * @param none
 * @return none
 */
void V_LIM_MTR_Initialize(void)
{
	s32_LIM_MTR_iqRef_10mA = (s32)0;																						/* 最終Iq制限値 */
}
