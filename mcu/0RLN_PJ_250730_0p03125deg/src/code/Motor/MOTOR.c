/**
 * @file MOTOR.c
 * @brief モータ電流指令値決定
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11.
 */
/* Includes ------------------------------------------------------------------*/
#include "MCU.h"
#include "DEFINE_SYSTEM.h"					/*定数定義*/
#include "DEFINE_MOTOR.h"
#include "DEFINE_SETTING.h"					/* IFDEF設定 */
#include "COMMON_FUNCTION.h"				/* 汎用関数群 */

/* DDL */
#include "DD_MOTOR.h"
#include "DD_EXCITER.h"

#include "DETECT_ABNORMAL.h"
#include "DETECT_MOTOR.h"
#include "DETECT_BATTERY.h"
#include "TYPEDEF_ABN.h"
#include "ADC.h"
#include "CAN.h"
#include "LIMIT_MOTOR.h"
#include "PROPOTIONAL_GAIN_MAP.h"
#include "ENC_OFFSET_MAP.h"
#include "MODE_CONDUCTOR.h"
#include "BATTERY.h"
#include "MOTOR.h"
#include "PREVENT_CHATTERING.h"						/* チャタリング防止 */

/* for DEUBG */
#include "ramdef.h"


/* Private define ------------------------------------------------------------*/
#define	U8_MTR_TIME_CONST_FACTOR	(u8)2					/* エキサイタ電流LPF時定数係数 LPFの時定数40msに相当（ 2^2 = 4：4×10ms）*/

/* Private variables ---------------------------------------------------------*/

#if 0
/**
 *		エンコーダ補正値マップ
 *		(正数を入れた分、電流が進む)
 *		電流センサ遅れ	修正	織り込み済み
 **/
#define	OFFSET											(s16)(U16_MTR_EANGLE_OFFSET_DEG)	/* 位相角オフセット(1度単位) 度 */
#define U8_MTR_ANGLE_OFFSET_BY_REVOLUTION_POINT_NUM		(u8)15								/* エンコーダ補正値マップ */
const s16	s16_MTR_angleOffsetByRevolutionXarray_rpm[(U8_MTR_ANGLE_OFFSET_BY_REVOLUTION_POINT_NUM)] =		/* モータ回転数[rpm] */
	{ (s16)(-10000),	(s16)(-6000), 	(s16)(-5000), 	(s16)(-4000), 	(s16)(-3000), 	(s16)(-2000),	(s16)(-1000),	(s16)0,			(s16)1000,		(s16)2000,		(s16)3000,		(s16)4000,		(s16)5000,		(s16)6000,		(s16)10000 };
const s16	s16_MTR_angleOffsetByRevolutionYarray_deg[(U8_MTR_ANGLE_OFFSET_BY_REVOLUTION_POINT_NUM)] =		/* エンコーダオフセット[1deg] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
#endif

#ifdef	DEBUG_WATCH_ACTIVITY

bool	b_MTR_isStartRequested = FALSE;													/* モータ駆動要求 */
bool	b_MTR_isPermitted = TRUE;														/* ゲートON許可 */
bool	b_MTR_isPermittedByVehicleState = FALSE;										/* 車両状態による通電許可フラグ */
bool	b_MTR_isPermittedByVcu = FALSE;													/* VCUからの通電許可フラグ */
bool	b_MTR_emergencyStop = FALSE;												/* 緊急停止指令 */

s16	s16_MTR_ifFb_10mA = (s16)0;														/* エキサイタ電流フィードバック値[10mA] */
s16	s16_MTR_ifFbAve_10mA = (s16)0;													/* エキサイタ電流フィードバック平均値[10mA] */
volatile u16	u16_MTR_imRef_100mA = (u16)0;											/* Im電流指令値[100mA] */

s16	s16_MTR_torqueRef_100mNm = (s16)0;												/* トルク指令値[0.1Nm] */

s16	s16_MTR_ifRef_10mA = (s16)0;													/* If指令値[10mA](MTRクラスでの最終値) */
s16	s16_MTR_idRef_100mA = (s16)0;													/* Id指令値[10mA](MTRクラスでの最終値) */
s16	s16_MTR_iqRef_100mA = (s16)0;													/* Iq指令値[10mA](MTRクラスでの最終値) */
u16	u16_MTR_imFbAve_100mA = (u16)0;													/* Im電流フィードバック平均値[10mA] */

s32	s32_MTR_idRef_10mA = (s32)0;													/* Id指令値[10mA](MTRクラスでの最終値) */
s32	s32_MTR_iqRef_10mA = (s32)0;													/* Iq指令値[10mA](MTRクラスでの最終値) */
u32	u32_MTR_imFbAve_10mA = (u32)0;													/* Im電流フィードバック平均値[10mA] */
s32	u32_MTR_imRms_10mA = (s32)0;													/* モータ電流実効値ImRms[10mA] */
volatile u32	u32_MTR_imRef_10mA = (u32)0;											/* Im電流指令値[10mA] */
volatile u32	u32_MTR_imRmsRef_10mA = (u32)0;											/* Im電流指令実効値[10mA] */

ST_CMF_RESULT_IIRF_S16	st_MTR_resultIf_10mA;										/* If : エキサイタ電流[10mA] */
ST_CMF_RESULT_IIRF_S16	st_MTR_resultREVOLUTION;									/* モータ回転数 */
s16	s16_MTR_revolutionAve_rpm	= (s16)0;											/* モータ回転数平均値[rpm] */
u16	u16_MTR_dPGainTemp = (U16_MTR_DAXIS_KP);										/* d軸比例ゲインtemp */
u16	u16_MTR_qPGainTemp = (U16_MTR_QAXIS_KP);										/* q軸比例ゲインtemp */
u8	u8_revTimeConstFactor = (U8_SYS_TIME_CONST_FACTOR);								/* モータ回転数フィルタ時定数 */

volatile u8	u8_MTR_voltageUtilizationRate_percent = (u16)0;							/* 電圧利用率[%] */

ST_ADC_ADVALUE_AVE	st_MTR_adValueAve	= {(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_EX_IF_OFFSET_STANDARD_ADVALUE)};	/* AD値平均化データ */
ST_ADC_ADVALUE_AVE*	pst_MTR_adValueAve	= &st_MTR_adValueAve;						/* AD値平均化データの参照先 */

UNI_VCU_COMMAND	uni_MTR_vcuCommand;													/* VCU Command */

bool	b_MTR_isMotorStopped = TRUE;													/* モータ停止状態 */

/* for TEST */
volatile bool	b_MTR_idInactivate	= FALSE;										/* Idなし */

s16	s16_MTR_ifRefOffset_100mA = (s16)0;												/* テスト用：If指令値オフセット[100mA](MTRクラスでの最終値に加減算) */
s16	s16_MTR_idRefOffset_100mA = (s16)0;												/* テスト用：Id指令値オフセット[10mA](MTRクラスでの最終値に加減算) */
s16	s16_MTR_iqRefOffset_100mA = (s16)0;												/* テスト用：Iq指令値オフセット[10mA](MTRクラスでの最終値に加減算) */

#else

static bool	b_MTR_isStartRequested = FALSE;													/* モータ駆動要求 */
static bool	b_MTR_isPermitted = TRUE;														/* ゲートON許可 */
static bool	b_MTR_isPermittedByVehicleState = FALSE;										/* 車両状態による通電許可フラグ */
static bool	b_MTR_isPermittedByVcu = FALSE;													/* VCUからの通電許可フラグ */

static s16	s16_MTR_ifFb_10mA = (s16)0;														/* エキサイタ電流フィードバック値[10mA] */
static s16	s16_MTR_ifFbAve_10mA = (s16)0;													/* エキサイタ電流フィードバック平均値[10mA] */
static volatile u16	u16_MTR_imRef_100mA = (u16)0;											/* Im電流指令値[100mA] */

static s16	s16_MTR_torqueRef_100mNm = (s16)0;												/* トルク指令値[0.1Nm] */

static s16	s16_MTR_idRef_100mA = (s16)0;													/* Id指令値[100mA](MTRクラスでの最終値) */
static s16	s16_MTR_iqRef_100mA = (s16)0;													/* Iq指令値[100mA](MTRクラスでの最終値) */
static u16	u16_MTR_imFbAve_100mA = (u16)0;													/* Im電流フィードバック平均値[10mA] */

static s16	s16_MTR_ifRef_10mA = (s16)0;													/* If指令値[10mA](MTRクラスでの最終値) */
static s32	s32_MTR_idRef_10mA = (s32)0;													/* Id指令値[10mA](MTRクラスでの最終値) */
static s32	s32_MTR_iqRef_10mA = (s32)0;													/* Iq指令値[10mA](MTRクラスでの最終値) */
static u32	u32_MTR_imFbAve_10mA = (u32)0;													/* Im電流フィードバック平均値[10mA] */
static u32	u32_MTR_imRms_10mA = (u32)0;													/* モータ電流実効値ImRms[10mA] */
static volatile u32	u32_MTR_imRef_10mA = (u32)0;											/* Im電流指令値[10mA] */
static volatile u32	u32_MTR_imRmsRef_10mA = (u32)0;											/* Im電流指令実効値[10mA] */

static ST_CMF_RESULT_IIRF_S16	st_MTR_resultIf_10mA;										/* If : エキサイタ電流[10mA] */
static ST_CMF_RESULT_IIRF_S16	st_MTR_resultREVOLUTION;									/* モータ回転数 */
static s16	s16_MTR_revolutionAve_rpm	= (s16)0;											/* モータ回転数平均値[rpm] */
static u16	u16_MTR_dPGainTemp = (U16_MTR_DAXIS_KP);										/* d軸比例ゲインtemp */
static u16	u16_MTR_qPGainTemp = (U16_MTR_QAXIS_KP);										/* q軸比例ゲインtemp */
static u8	u8_revTimeConstFactor = (U8_SYS_TIME_CONST_FACTOR);								/* モータ回転数フィルタ時定数 */

static volatile u8	u8_MTR_voltageUtilizationRate_percent = (u16)0;							/* 電圧利用率[%] */

static ST_ADC_ADVALUE_AVE	st_MTR_adValueAve	= {(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_MTR_CT_OFFSET_STANDARD_ADVALUE),(U16_EX_IF_OFFSET_STANDARD_ADVALUE)};	/* AD値平均化データ */
static ST_ADC_ADVALUE_AVE*	pst_MTR_adValueAve	= &st_MTR_adValueAve;						/* AD値平均化データの参照先 */

static UNI_VCU_COMMAND	uni_MTR_vcuCommand;													/* VCU Command */

static bool	b_MTR_isMotorStopped = TRUE;													/* モータ停止状態 */

/* for TEST */
static volatile bool	b_MTR_idInactivate	= FALSE;										/* Idなし */

static s16	s16_MTR_ifRefOffset_100mA = (s16)0;												/* テスト用：If指令値オフセット[100mA](MTRクラスでの最終値に加減算) */
static s16	s16_MTR_idRefOffset_100mA = (s16)0;												/* テスト用：Id指令値オフセット[10mA](MTRクラスでの最終値に加減算) */
static s16	s16_MTR_iqRefOffset_100mA = (s16)0;												/* テスト用：Iq指令値オフセット[10mA](MTRクラスでの最終値に加減算) */

#endif
/* Private function prototypes -----------------------------------------------*/
static void	v_MTR_AdjustmentCtOffset(void);													/* CTオフセット補正 */
#if 0
static void	v_MTR_UpdateEangleOffset_deg(void);												/* エンコーダ補正値演算 */
#endif

static u32	u32_MTR_CalcVmAve_10mV(void);													/* モータ印加電圧算出[100mV] */
static u32	u32_MTR_CalcImAve_10mA(void);													/* モータ電流Im算出[100mA] */
static u32	u32_MTR_CalcImRms_10mA(void);													/* モータ電流実効値ImRMS算出[100mA] */
static void	v_MTR_CalcImRef_10mA(void);														/* モータ電流指令値ImRef算出[100mA] */
static u32	u32_MTR_CalcImRmsRef_10mA(void);												/* モータ実効電流指令値ImRef算出[100mA] */
static u8	u8_MTR_CalcVoltageUtilizationRate_percent(u32 a_u32_vmAve_10mV, u16 a_u16_battVolt_10mV);	/* 電圧利用率算出[%] */

/**
 * @brief MOTORクラス周期処理(10msec)
 * @param none
 * @return none
 * @detail 走行可否判定
 */
void	V_MTR_Loop10ms(void)
{
	EN_MDC_MODE	a_en_mode	= En_MDC_GetMode();													/* 動作モード取得 */

	uni_MTR_vcuCommand.byte = U8_CAN_GetVcuCommand();

	V_LIM_MTR_Loop10ms();																		/* 電流指令値演算 */

	bool	a_b_isDetectActivity	= B_MDC_IsDetectActivity();
	bool	a_b_isSomethingAbnormal	= B_ABN_IsSomethingAbnormal();
	bool	a_b_isGateOn			= B_DD_MTR_IsGateOn();
	s16		a_s16_revolution_rpm	= S16_DD_MTR_GetRevolution_rpm();
	u16		a_u16_revolution_rpm	= (u16)(s16)CMF_GETABS(a_s16_revolution_rpm);

	b_MTR_isPermittedByVcu = !uni_MTR_vcuCommand.bit.b_motorStopCommand;						/* モータ停止要求/駆動許可状態取得 */

	s16_MTR_ifFb_10mA = S16_DD_EX_GetIfFb_10mA();
	st_MTR_resultIf_10mA = St_CMF_CalcIIRFilterS16ByShift( s16_MTR_ifFb_10mA,	st_MTR_resultIf_10mA.s16_ave,	(u8)(U8_MTR_TIME_CONST_FACTOR),	st_MTR_resultIf_10mA.s32_sum );	/* エキサイタ電流値更新 */
	s16_MTR_ifFbAve_10mA = st_MTR_resultIf_10mA.s16_ave;

	/* for TEST */
	if( UNI_GL_vcuTestFlg.bit.b_vcuDummyOrderActivity == TRUE )									/* テスト用VCUダミー要求要求有効? */
	{
		b_MTR_isPermittedByVcu = !UNI_GL_vcuTestFlg.bit.b_motorStopRequest;
	}

	/* トルク指令値取得 */
	s16_MTR_torqueRef_100mNm = S16_LIM_MTR_GetTorqueRef_100mNm();

	/* マップによる電流指令値取得 */
	s16_MTR_ifRef_10mA = S16_LIM_MTR_GetIfRef_10mA();											/* マップによるIf指令値取得 */
	s32_MTR_idRef_10mA = S32_LIM_MTR_GetIdRef_10mA();											/* マップによるId指令値取得 */
	s32_MTR_iqRef_10mA = S32_LIM_MTR_GetIqRef_10mA();											/* マップによるIq指令値取得 */

/*======= 動作モードにより選択 ======*/
	switch( a_en_mode )
	{
	case MODE_ABNORMAL:
		b_MTR_isPermitted = FALSE;																/* モータ駆動禁止 */
		b_MTR_isPermittedByVehicleState = FALSE;												/* 通電不可 */
		V_DD_MTR_Forbid();																		/* 強制ゲートオフ（今回特別） */
		break;

	case MODE_RUN:
		b_MTR_isPermittedByVehicleState = TRUE;													/* 通電許可 */
		V_MTR_Permit();
		break;

	case MODE_IDLE:
		b_MTR_isPermitted = TRUE;																/* モータ駆動許可 */
		b_MTR_isPermittedByVehicleState = FALSE;												/* 通電不可 */
		break;

	case MODE_STARTUP:
	case MODE_STANDBY:
	case MODE_SHUTDOWN_WAIT:
	case MODE_SHUTDOWN:
		b_MTR_isPermittedByVehicleState = FALSE;												/* 通電不可 */
		break;

	default:
		b_MTR_isPermittedByVehicleState = FALSE;												/* 通電不可 */
		break;
	}

/***** 緊急停止対応 *****/
	bool	a_b_emergencyStop = B_DD_EX_IsEmergencyStop();

	if( a_b_emergencyStop == TRUE )
	{
		s16_MTR_torqueRef_100mNm = (s16)0;
		s16_MTR_iqRef_100mA = (s16)0;
		V_DD_EX_SetIfRef_10mA( s16_MTR_ifRef_10mA );											/* ie指令値セット */
		V_DD_MTR_SetIdqRef_10mA( s32_MTR_idRef_10mA, s32_MTR_iqRef_10mA );						/* id、iq指令値セット */
		b_MTR_isPermitted = FALSE;																/* モータ駆動禁止 */
		b_MTR_isPermittedByVehicleState = FALSE;												/* 通電不可 */
		V_DD_MTR_Forbid();																		/* 強制ゲートオフ（今回特別） */
	}

	/********** Idq指令値の漸増減有無 **********/
	if( ((b_MTR_isPermittedByVehicleState == FALSE) ||											/* 車両状態で通電禁止 */
		(b_MTR_isPermittedByVcu == FALSE))			&&											/* VCUからの通電禁止 */
		(s16_MTR_torqueRef_100mNm == (s16)0) )													/* 最終トルク指令値 = 0? */
	{
		s32_MTR_iqRef_10mA = (s32)0;
		V_DD_EX_SetIfRef_10mA((s16)0);
		if( a_b_emergencyStop == TRUE )
		{
			b_MTR_isStartRequested = FALSE;
		}
		else if( (u32_MTR_imFbAve_10mA <= (u32)(U16_MTR_STOPPED_IQFB_10MA)) &&					/* Im平均値が停止と判断する電流値以下 */
				(a_u16_revolution_rpm <= (U16_MTR_STOPPED_REVOLUTION_RPM)) )					/* モータ規定回転数以下 */
		{
			b_MTR_isStartRequested = FALSE;
		}
		else
		{
			/* MISRA-C 2004 14.10 対応 */
		}
	}
	else
	{
		b_MTR_isStartRequested = TRUE;															/* ゲートON維持 */
	}

	/********** Idq指令値の更新 **********/

	if( UNI_GL_testFlg.bit.b_referenceByCuW == TRUE )
	{
		s16_MTR_ifRef_10mA = s16_MTR_ifRef_10mA + (s16_MTR_ifRefOffset_100mA * (s16)10);
		s32_MTR_idRef_10mA = s32_MTR_idRef_10mA + (s32)((s32)s16_MTR_idRefOffset_100mA * (s32)10);
		s32_MTR_iqRef_10mA = s32_MTR_iqRef_10mA + (s32)((s32)s16_MTR_iqRefOffset_100mA * (s32)10);
	}

	V_DD_MTR_SetIdqRef_10mA( s32_MTR_idRef_10mA, s32_MTR_iqRef_10mA );							/* id、iq指令値セット */
	if( B_CHT_IsAccelSwOn() == FALSE )
	{
		s16_MTR_ifRef_10mA = (s16)0;
	}
	V_DD_EX_SetIfRef_10mA( s16_MTR_ifRef_10mA );												/* ie指令値セット */

	u32_MTR_imFbAve_10mA = u32_MTR_CalcImAve_10mA();											/* モータ電流Im演算 */
	u32_MTR_imRms_10mA = u32_MTR_CalcImRms_10mA();												/* モータ電流実効値ImRMS算出 */
	v_MTR_CalcImRef_10mA();
	u32_MTR_imRmsRef_10mA = u32_MTR_CalcImRmsRef_10mA();										/* モータ電流指令実効値ImRMS算出 */

	/* for CuW */
	u16_MTR_imFbAve_100mA = (u16)(u32_MTR_imFbAve_10mA / (u32)10);
	s16_MTR_idRef_100mA = (s16)(s32_MTR_idRef_10mA / (s32)10);
	s16_MTR_iqRef_100mA = (s16)(s32_MTR_iqRef_10mA / (s32)10);


	/********** ゲートON/OFF実行部(StartRequestで判断) **********/
	if( b_MTR_isStartRequested == TRUE )														/* ゲートON許可 */
	{
		V_DD_EX_StartControl();																	/* ゲートON要求 */
		V_DD_MTR_StartControl();																/* ゲートON要求 */
#if 0
		v_MTR_UpdateEangleOffset_deg();															/* エンコーダ補正値演算	*/
#endif
		V_MDC_FrobidRelayOff();																	/* リレーOFF禁止 */
		s16	a_s16_ifRefForGain_10mA = S16_DD_EX_GetIfFb_10mA();
		s16	a_s16_imRefRms = (s16)(u32_MTR_imRmsRef_10mA >> (u32)1);
		u16_MTR_dPGainTemp = (u16)S16_CMF_3DMapCal( (U8_GAIN_MAP_IF_POINT_NUM), (s32)s16_MTR_ifRef_10mA, a_s16_imRefRms, d_Gainmap );	/* 3DマップにてIfRefとImRmsRefよりd軸KPを算出 */
		u16_MTR_qPGainTemp = (u16)S16_CMF_3DMapCal( (U8_GAIN_MAP_IF_POINT_NUM), (s32)s16_MTR_ifRef_10mA, a_s16_imRefRms, q_Gainmap );	/* 3DマップにてIfRefとImRmsRefよりq軸KPを算出 */
	}
	else																						/* ゲートON禁止 */
	{
		V_DD_EX_SetIfRef_10mA((s16)0);
		V_DD_EX_StopControl();																	/* 界磁停止指示 */
		V_DD_MTR_StopControl();																	/* ゲートOFF指示 */
		V_MDC_PermitRelayOff();																	/* リレーOFF許可 */
	}

	/* モータ停止状態確認 */
	if( (u32_MTR_imFbAve_10mA		<= (u32)(U16_MTR_STOPPED_IQFB_10MA)) ||						/* Im平均値が停止と判断する電流値以下 */
		(s16_MTR_ifFbAve_10mA		<= (s16)(U16_MTR_STOPPED_IFFB_10MA)) ||						/* If平均値が停止と判断する電流値以下 */
		(s16_MTR_revolutionAve_rpm 	<= (U16_MTR_STOPPED_REVOLUTION_RPM)) )						/* モータ規定回転数以下 */
	{
		b_MTR_isMotorStopped = TRUE;
	}
	else
	{
		b_MTR_isMotorStopped = FALSE;
	}

	/** CTオフセット補正実施判断 **/
	if( b_MTR_isStartRequested == FALSE )														/* ゲートOFF指令 */
	{
		if( (a_b_isSomethingAbnormal == FALSE) &&												/* 異常状態を確認 */
			(a_b_isGateOn == FALSE) &&															/* 現在ゲートオフ状態 */
			(a_u16_revolution_rpm <= (U16_MTR_STOPPED_REVOLUTION_RPM)) &&						/* モータ規定回転数以下 */
			(a_b_isDetectActivity == TRUE) )													/* 故障検出有効 */
		{																						/* 異常なしの場合 */
			v_MTR_AdjustmentCtOffset();															/* CTオフセット補正 */
		}
	}
	/* 電圧利用率演算[%] */
	u8_MTR_voltageUtilizationRate_percent = u8_MTR_CalcVoltageUtilizationRate_percent(u32_MTR_CalcVmAve_10mV(),U16_BAT_GetBattVoltAve_10mV());
}

/**
 * @brief CTオフセット補正
 * @param none
 * @return none
 * @detail ADCでの時定数10msecLPFの結果を条件が合致した時にGetする
 */
static void	v_MTR_AdjustmentCtOffset(void)
{
	st_MTR_adValueAve = St_ADC_GetAdValueAve();													/* 電流AD値平均データ取得 */
}


/**
* @brief モータ回転数平均化処理
 * @param none
 * @return none
 * @note 1msec周期
 */
void	V_MTR_UpdateRevolutionAve_rpm(void)
{
	u8_revTimeConstFactor = U8_CAN_GetRevTimeConstFactorFromCan();								/* フィルタ時定数取得 */
	/* モータ回転数平均化処理 */
	s16	a_s16_revolution6_rpm = S16_DD_MTR_GetRevolutionAve6_rpm();
	st_MTR_resultREVOLUTION	= St_CMF_CalcIIRFilterS16( a_s16_revolution6_rpm,st_MTR_resultREVOLUTION.s16_ave,u8_revTimeConstFactor,st_MTR_resultREVOLUTION.s32_sum );	/* モータ回転数平均値更新 */
	s16_MTR_revolutionAve_rpm = st_MTR_resultREVOLUTION.s16_ave;

#if 0
	v_MTR_UpdateEangleOffset_deg();																/* エンコーダ補正値演算	*/
#endif
}

#if 0
/**
 * @brief エンコーダ補正値更新
 * @param none
 * @return none
 */
static void	v_MTR_UpdateEangleOffset_deg(void)
{
	s16	a_s16_revolution_rpm = S16_DD_MTR_GetRevolution_rpm();

//	u16	a_u16_eangleOfs_deg = (u16)S16_CMF_2DMapCal( a_s16_revolution_rpm,(U8_MTR_ANGLE_OFFSET_BY_REVOLUTION_POINT_NUM),(s16*)s16_MTR_angleOffsetByRevolutionXarray_rpm,(s16*)s16_MTR_angleOffsetByRevolutionYarray_deg );	/* 2Dマップより補正値計算(速度起因) */

//	u16	a_u16_eangleOfs_deg = (u16)S16_CMF_3DMapCal((U8_ENC_OFFSET_MAP_IM_POINT_NUM),(s32)u16_MTR_imRef_100mA,a_s16_revolution_rpm,EncOffset_ImMap );
//	u16	a_u16_eangleOfs_deg = (u16)S16_CMF_3DMapCal((U8_ENC_OFFSET_MAP_IM_POINT_NUM),(s32)s16_MTR_iqRef_100mA,a_s16_revolution_rpm,EncOffset_ImMap );

//	V_DD_MTR_SetEangleOffset_deg(a_u16_eangleOfs_deg);											/* エンコーダ補正値結果 */
}
#endif

/**
 * @brief モータ印加電圧算出
 * @note √(Vd^2＋Vq^2)
 * @param none
 * @return a_u32_vmAve
 */
static u32	u32_MTR_CalcVmAve_10mV(void)
{
	ST_ADC_DQ32	a_st_dq32Ave = {(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0};							/* DQ軸平均値データ */
	a_st_dq32Ave = St_ADC_GetDq32Ave();																/* DQ軸平均データ取得 */

	s32	a_s32_vdAve	= a_st_dq32Ave.s32_vd_10mV;
	s32	a_s32_vqAve	= a_st_dq32Ave.s32_vq_10mV;
	u32	a_u32_vmSquared = (u32)(a_s32_vdAve * a_s32_vdAve + a_s32_vqAve * a_s32_vqAve);
	u32	a_u32_vmAve	= U32_CMF_Sqrt(a_u32_vmSquared);

	return	a_u32_vmAve;
}

/**
 * @brief モータ電流Im算出
 * @note √(Id^2＋Iq^2)
 * @param none
 * @return a_u32_imAve
 */
static u32	u32_MTR_CalcImAve_10mA(void)
{
	ST_ADC_DQ32	a_st_dq32Ave = {(s32)0,(s32)0,(s32)0,(s32)0};
	a_st_dq32Ave = St_ADC_GetDq32Ave();																/* DQ軸平均データ取得 */

	s32	a_s32_idAve	= a_st_dq32Ave.s32_idFb_10mA;
	s32	a_s32_iqAve	= a_st_dq32Ave.s32_iqFb_10mA;
	u32	a_u32_imSquared = (u32)(a_s32_idAve * a_s32_idAve + a_s32_iqAve * a_s32_iqAve);
	u32	a_u32_imAve	= U32_CMF_Sqrt(a_u32_imSquared);

	return	a_u32_imAve;
}

/**
 * @brief モータ電流実効値ImRMS算出
 * @note Im/√2 = Im/1.4142
 * @note 実際には相電流ピーク値/√2だが相対変換なのでImで代用する
 * @param none
 * @return a_u32_imRms
 */
static u32	u32_MTR_CalcImRms_10mA(void)
{
	u32	a_u32_imAve = (s32)u32_MTR_imFbAve_10mA;
	u32	a_u32_imRms = a_u32_imAve * (u32)10000 / (u32)14142;

	return	a_u32_imRms;
}

/**
 * @brief モータ電流指令値ImRef算出
 * @note √(IdRef^2＋IqRef^2)
 * @param none
 * @return none
 */
static void	v_MTR_CalcImRef_10mA(void)
{
	s32	a_s32_idRef	= s32_MTR_idRef_10mA;
	s32	a_s32_iqRef	= s32_MTR_iqRef_10mA;
	u32	a_u32_imRefSquared = (u32)(a_s32_idRef * a_s32_idRef + a_s32_iqRef * a_s32_iqRef);
	u32_MTR_imRef_10mA = U32_CMF_Sqrt(a_u32_imRefSquared);
}

/**
 * @brief モータ電流指令値ImRef算出
 * @note ImRef/√2 = Im/1.4142
 * @param none
 * @return u32_MTR_imRef_10mA * (u32)10000 / (u32)14142
 */
static u32	u32_MTR_CalcImRmsRef_10mA(void)
{
	return	u32_MTR_imRef_10mA * (u32)10000 / (u32)14142;
}

/**
 * @brief 電圧利用率算出
 * @note 2/√3：過変調 = 1.1547
 * @note 3/2：相対変換 = 1.5
 * @note √(Vd^2＋Vq^2)×100／(2/√3×3/2×Vdc)
 * @note 1/√3×√(Vd^2＋Vq^2)×100／Vdc
 * @note 1/√3×100 = 57.7 = 577／10
 * @param a_u32_vmAve_10mV
 * @param a_u16_battVolt_10mV
 * @return a_u8_voltageRate_percent
 */
static u8	u8_MTR_CalcVoltageUtilizationRate_percent(u32 a_u32_vmAve_10mV, u16 a_u16_battVolt_10mV)
{
	u32	a_u32_voltageRate = (u32)0;
	u8	a_u8_voltageRate_percent = (u8)0;

	if(a_u16_battVolt_10mV == (u16)0)
	{
		a_u8_voltageRate_percent = (u32)100;
	}
	else
	{
		/* Vm *  2/√3 * 100 / Vb */
		a_u32_voltageRate = (u32)a_u32_vmAve_10mV * (u32)577 / (u32)a_u16_battVolt_10mV / (u32)10;
		a_u8_voltageRate_percent = (u8)(u32)CMF_GETMIN(a_u32_voltageRate,(u32)100);
	}

	return	a_u8_voltageRate_percent;
}

/**
* @brief モータ電流指令値演算の変数初期化処理(再起動用)
 * @param none
 * @return none
 */
void	V_MTR_InitializeForRestart(void)
{
/*--- 通電禁止 ---*/
	b_MTR_isStartRequested = FALSE;																/* モータ制御停止を要求する */

	s32_MTR_idRef_10mA = (s32)0;																/* Id指令値10mA](MTRクラスでの最終値) */
	s32_MTR_iqRef_10mA = (s32)0;																/* Iq指令値[10mA](MTRクラスでの最終値) */

	st_MTR_adValueAve.u16_adValueAveCtU = (U16_MTR_CT_OFFSET_STANDARD_ADVALUE);					/* U相電流センサーのオフセットAD値 */
	st_MTR_adValueAve.u16_adValueAveCtV = (U16_MTR_CT_OFFSET_STANDARD_ADVALUE);					/* V相電流センサーのオフセットAD値 */
	st_MTR_adValueAve.u16_adValueAveIf	= (U16_EX_IF_OFFSET_STANDARD_ADVALUE);					/* 界磁電流(IF)オフセットAD値 */

	V_LIM_MTR_Initialize();																		/* Iq指令値制限処理変数初期化 */
}

/**
 * @brief 電流AD値平均化データ提供
 */
ST_ADC_ADVALUE_AVE	St_MTR_GetAdValueAve(void)
{
	return	*pst_MTR_adValueAve;
}

/**
 * @brief 界磁電流オフセット提供
 * @param none
 * @return u16_MTR_iBOffsetAve_adValue
 */
u16	U16_MTR_GetAdValueAveIfOffset(void)
{
	return	st_MTR_adValueAve.u16_adValueAveIf;
}

/**
 * @brief モータ駆動を許可する
 * @param none
 * @return none
 */
void	V_MTR_Permit(void)
{
	b_MTR_isPermitted = TRUE;
	V_DD_MTR_Permit();
	V_DD_EX_Permit();
}

/**
 * @brief モータ駆動を禁止する
 * @param none
 * @return none
 */
void	V_MTR_Forbid(void)
{
	b_MTR_isPermitted = FALSE;
}

/**
 * @brief ゲートON許可/禁止状態提供
 * @param none
 * @return b_MTR_isPermitted
 */
bool	B_MTR_IsGateOnPermit(void)
{
	return b_MTR_isPermitted;
}

/**
 * @brief モータ駆動を要求する
 * @param none
 * @return none
 */
void	V_MTR_StartControl(void)
{
	b_MTR_isStartRequested = TRUE;
}

/**
 * @brief ゲートOFFを要求する
 * @param none
 * @return none
 * @note ※実際にはどこからもコールされていない
 */
void	V_MTR_StopControl(void)
{
	b_MTR_isStartRequested = FALSE;
}

/**
 * @brief 車両状態による通電許可を要求する
 * @param none
 * @return none
 */
void	V_MTR_PermitByVehicleState(void)
{
	b_MTR_isPermittedByVehicleState = TRUE;
}

/**
 * @brief 車両状態による通電禁止を要求する
 * @param none
 * @return none
 */
void	V_MTR_ForbidByVehicleState(void)
{
	b_MTR_isPermittedByVehicleState = FALSE;
}

/**
 * @brief モータ制御開始要求かどうかを返す
 * @param none
 * @return b_MTR_isStartRequested
 */
bool	B_MTR_IsStartRequested(void)
{
	return b_MTR_isStartRequested;
}

/**
 * @brief 車両モードによるモータ駆動禁止状態提供
 * @param none
 * @return none
 */
bool	B_MTR_PermittedByVehicleState(void)
{
	return	b_MTR_isPermittedByVehicleState;
}

/**
 * @brief モータ回転数提供
 * @param none
 * @return u16_MTR_motorRevolution_rpm
 */
s16	S16_MTR_GetRevolution_rpm(void)
{
	return	S16_DD_MTR_GetRevolution_rpm();
}

/**
 * @brief モータ回転数平均値提供
 * @param none
 * @return s16_MTR_revolutionAve_rpm
 */
s16	S16_MTR_GetRevolutionAve_rpm(void)
{
	return	s16_MTR_revolutionAve_rpm;
}

/**
 * @brief 界磁電流によるd軸比例ゲイン提供
 * @param none
 * @return u16_MTR_dPGainTemp
 */
u16	U16_MTR_GetDAxisPropoGain(void)
{
	return	u16_MTR_dPGainTemp;
}

/**
 * @brief 界磁電流によるq軸比例ゲイン提供
 * @param none
 * @return u16_MTR_qPGainTemp
 */
u16	U16_MTR_GetQAxisPropoGain(void)
{
	return	u16_MTR_qPGainTemp;
}

/**
 * @brief 最終Id指令値提供[10mA]
 * @param none
 * @return S32_DD_MTR_GetIdRef_10mA()
 */
s32	S32_MTR_GetIdRef_10mA(void)
{
	return	S32_DD_MTR_GetIdRef_10mA();
}

/**
 * @brief 最終Iq指令値提供[10mA]
 * @param none
 * @return S32_DD_MTR_GetIqRef_10mA()
 */
s32	S32_MTR_GetIqRef_10mA(void)
{
	return	S32_DD_MTR_GetIqRef_10mA();
}

/**
 * @brief D軸電流値提供[10mA]
 * @param none
 * @return S32_DD_MTR_GetIdFb_10mA()
 */
s32	S32_MTR_GetIdFb_10mA(void)
{
	return	S32_DD_MTR_GetIdFb_10mA();
}

/**
 * @brief Q軸電流値提供[10mA]
 * @param none
 * @return S32_DD_MTR_GetIqFb_10mA()
 */
s32	S32_MTR_GetIqFb_10mA(void)
{
	return	S32_DD_MTR_GetIqFb_10mA();
}

/**
 * @brief モータ電流実効値提供[10mA]
 * @param none
 * @return u32_MTR_imRms_10mA
 */
u32	U32_MTR_GetImRms_10mA(void)
{
	return	u32_MTR_imRms_10mA;
}

/**
 * @brief Id指令値提供[100mA]
 * @param none
 * @return S32_DD_MTR_GetIdRef_10mA() / 10
 */
s16	S16_MTR_GetIdRef_100mA(void)
{
	return	(s16)(S32_DD_MTR_GetIdRef_10mA() / (s32)10);
}

/**
 * @brief Iq指令値提供[100mA]
 * @param none
 * @return S32_DD_MTR_GetIqRef_10mA() / 10
 */
s16	S16_MTR_GetIqRef_100mA(void)
{
	return	(s16)(S32_DD_MTR_GetIqRef_10mA() / (s32)10);
}

/**
 * @brief 最終Iq指令値提供[100mA]
 * @param none
 * @return S32_DD_MTR_GetIqRefFinal_10mA() / 10
 */
s16	S16_MTR_GetIqRefFinal_100mA(void)
{
	return	(s16)(S32_DD_MTR_GetIqRefFinal_10mA() / (s32)10);
}

/**
 * @brief D軸電流値提供[100mA]
 * @param none
 * @return S32_DD_MTR_GetIdFb_10mA() / 10
 */
s16	S16_MTR_GetIdFb_100mA(void)
{
	return	(s16)(S32_DD_MTR_GetIdFb_10mA() / (s32)10);
}

/**
 * @brief Q軸電流値提供[100mA]
 * @param none
 * @return S32_DD_MTR_GetIqFb_10mA() / 10
 */
s16	S16_MTR_GetIqFb_100mA(void)
{
	return	(s16)(S32_DD_MTR_GetIqFb_10mA() / (s32)10);
}

/**
 * @brief モータ電流実効値提供[100mA]
 * @param none
 * @return u32_MTR_imRms_10mA / 10
 */
u16	U16_MTR_GetImRms_100mA(void)
{
	return	(u16)(u32_MTR_imRms_10mA / (u32)10);
}

/**
 * @brief エキサイタ電流平均値提供[10mA]
 * @param none
 * @return s16_MTR_ifFbAve_10mA
 */
s16	S16_MTR_GetIfFbAve_10mA(void)
{
	return	s16_MTR_ifFbAve_10mA;
}

/**
 * @brief ゲートON/FF状態提供
 * @param none
 * @return a_b_isGateOn
 */
bool	B_MTR_IsGateOn(void)
{
	return	B_DD_MTR_IsGateOn();
}

/**
 * @brief モータ停止状態提供
 * @param none
 * @return b_MTR_isMotorStopped
 */
bool	B_MTR_IsMotorStopped(void)
{
	return	b_MTR_isMotorStopped;
}

/**
 * @brief モータ駆動許可状態提供
 * @param none
 * @return b_MTR_isPermitted | b_MTR_isPermittedByVehicleState | b_MTR_isPermittedByVcu

 */
bool	B_MTR_IsPermitted(void)
{
	return	(bool)(b_MTR_isPermitted | b_MTR_isPermittedByVehicleState | b_MTR_isPermittedByVcu);
}

/**
 * @brief バッテリ電圧ＡＤ提供(生)
 * @param none
 * @return st_DD_MTR_adValue0.u16_adValueBATTV
 */
u16 U16_MTR_GetAdValueBattVolt(void)
{
	return U16_DD_MTR_GetAdValueBattVolt();														/* 生のAD値 */
}

/**
 * @brief 象限提供
 * @param none
 * @return u8_LIM_MTR_quadrant
 */
u8	U8_MTR_GetQuadrant(void)
{
	return	U8_LIM_MTR_GetQuadrant();
}

/**
 * @brief 電圧利用率提供[%]
 * @param none
 * @return u8_MTR_voltageUtilizationRate_percent
 */
u8	U8_MTR_GetVoltageUtilizationRate_percent(void)
{
	return	u8_MTR_voltageUtilizationRate_percent;
}

/**
 * @brief d軸積分バッファ提供[10mV]
 * @param none
 * @return s32_DD_MTR_integralVd_10mV
 */
s32	S32_MTR_GetIntegralVd_10mV(void)
{
	return	S32_DD_MTR_GetIntegralVd_10mV();
}

/**
 * @brief q軸積分バッファ提供[10mV]
 * @param none
 * @return s32_DD_MTR_integralVq_10mV
 */
s32	S32_MTR_GetIntegralVq_10mV(void)
{
	return	S32_DD_MTR_GetIntegralVq_10mV();
}


/**
 * @brief D軸電圧Vd最大値提供[10mV]
 * @param none
 * @return s16_DD_MTR_vdMax_10mV
 */
s16	S16_MTR_GetVdMax_10mV(void)
{
	return	S16_DD_MTR_GetVdMax_10mV();
}

/**
 * @brief Q軸電圧Vq最大値提供[10mV]
 * @param none
 * @return s16_DD_MTR_vqMax_10mV
 */
s16	S16_MTR_GetVqMax_10mV(void)
{
	return	S16_DD_MTR_GetVqMax_10mV();
}
