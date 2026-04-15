/**
 * @file MODE_CONDUCTOR.c
 * @brief モード移行処理
 * @note 10ms周期
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "TYPEDEF_MODE.h"
#include "MCU.h"
#include "DEFINE_SYSTEM.h"		/* 定数定義 */
#include "DEFINE_SETTING.h"
#include "DEFINE_MOTOR.h"
#include "DEFINE_ABN.h"
#include "COMMON_FUNCTION.h"	/* 汎用関数群 */

/* DDL */
#include "DD_MOTOR.h"
#include "DD_EXCITER.h"

#include "DETECT_ABNORMAL.h"
#include "DETECT_BATTERY.h"
#include "DETECT_MOTOR.h"
#include "ADC.h"
#include "CAN.h"
#include "PREVENT_CHATTERING.h"		/* チャタリング防止 */
#include "DIAG_SET.h"
#include "MOTOR.h"
#include "BATTERY.h"
#include "MODE_CONDUCTOR.h"

/* for DEBUG */
#include "ramdef.h"

/* Private define -----------------------------------------------------------*/
#define	U16_MDC_SHUTDOWN_TIME_10MS				(u16)10						/* 電源遮断時間					100[ms] */
#define	U16_MDC_FORCED_SHUTDOWN_TIME_10MS		(u16)200					/* 強制電源遮断時間				2[s] */
#define	U16_MDC_DETECT_ACTIVITY_TIME_10MS		(u16)50						/* 異常検出有効判定時間			500[ms] */
#define	U8_MDC_STARTUP_TIME_10MS				(u16)1						/* 初期化待ち時間				10[ms](上記の”異常検出有効判定時間”が支配的) */
#define	U8_MDC_BATTVOLTWAIT_TIME_10MS			(u8)5						/* 電池電圧フィルタ待ち時間	50[ms] */

#define	U16_MDC_FORCED_SD_FROM_ABN_10MS			(u16)100					/* 異常モード時、キーSW OFFからの強制終了時間 1[s] */
#define	U16_MDC_FORCED_SD_FROM_INIT_TIME_10MS	(u16)2000					/* 初期化モード強制終了時間	20[s] */

/* Private variables ---------------------------------------------------------*/
#ifdef	DEBUG_WATCH_ACTIVITY

EN_MDC_MODE				en_MDC_mode				= MODE_STARTUP;				/* 現在モード		 */
EN_MDC_PRECHARGE_MODE	en_MDC_prechargeMode	= MD_PRECHARGE_INITIAL;		/* 現在プリチャージモード */
EN_MDC_DISCHARGE_MODE	en_MDC_dischargeMode	= MD_DISCHARGE_INITIAL;		/* 現在ディスチャージモード */

UNI_VCU_COMMAND	uni_MDC_vcuCommand;									/* VCU Command */

u16	u16_MDC_shutdownCounter				= (u16)0;					/* 電源遮断カウンタ */
u16	u16_MDC_detectActivityCounter		= (u16)0;					/* 異常検出有効判定カウンタ */
u8	u8_MDC_intializeCounter				= (u8)0;					/* 初期化完了待ちカウンタ */
u16	u16_MDC_prechargeWaitCounter		= (u16)0;					/* プリチャージ完了待ちカウンタ */
u16	u16_MDC_prechargeTimeoutCounter		= (u16)0;					/* プリチャージ異常検出タイマ */
u16	u16_MDC_dischargeTimeoutCounter		= (u16)0;					/* ディスチャージ異常検出タイマ */
u8	u8_MDC_mainVoltWaitTimeCounter		= (u8)0;					/* メイン電圧フィルタ待ちタイマ */

bool	b_MDC_isDetectActivity				= FALSE;					/* 異常検出有効フラグ */
u16	u16_MDC_battVoltAve_10mV			= (u16)0;					/* フィルタ後のバッテリ電圧(補正反映) */
u16	u16_MDC_invVoltAve_10mV				= (u16)0;					/* フィルタ後のインバータ電圧 */

volatile u8	u8_MDC_port8State			= (u8)0;					/* PORT8状態 */

bool	b_MDC_prechargeRequest			= FALSE;					/* プリチャージ要求フラグ */
bool	b_MDC_dischargeRequest			= FALSE;					/* ディスチャージ要求 */
bool	b_MDC_motorStopRequest			= FALSE;					/* モータ停止要求 */
bool	b_MDC_relayOffRequest			= FALSE;					/* メインリレーOFF要求 */
bool	b_MDC_permitRelayOff			= FALSE;					/* メインリレーOFF許可 */
bool	b_MDC_emergencyStop				= FALSE;					/* 緊急停止指令 */
bool	b_MDC_isMainRelayOn				= FALSE;

#else

static EN_MDC_MODE				en_MDC_mode				= MODE_STARTUP;				/* 現在モード		 */
static EN_MDC_PRECHARGE_MODE	en_MDC_prechargeMode	= MD_PRECHARGE_INITIAL;		/* 現在プリチャージモード */
static EN_MDC_DISCHARGE_MODE	en_MDC_dischargeMode	= MD_DISCHARGE_INITIAL;		/* 現在ディスチャージモード */

static UNI_VCU_COMMAND	uni_MDC_vcuCommand;									/* VCU Command */

static u16	u16_MDC_shutdownCounter				= (u16)0;					/* 電源遮断カウンタ */
static u16	u16_MDC_detectActivityCounter		= (u16)0;					/* 異常検出有効判定カウンタ */
static u8	u8_MDC_intializeCounter				= (u8)0;					/* 初期化完了待ちカウンタ */
static u16	u16_MDC_prechargeWaitCounter		= (u16)0;					/* プリチャージ完了待ちカウンタ */
static u16	u16_MDC_prechargeTimeoutCounter		= (u16)0;					/* プリチャージ異常検出タイマ */
static u16	u16_MDC_dischargeTimeoutCounter		= (u16)0;					/* ディスチャージ異常検出タイマ */
static u8	u8_MDC_mainVoltWaitTimeCounter		= (u8)0;					/* メイン電圧フィルタ待ちタイマ */

static bool	b_MDC_isDetectActivity				= FALSE;					/* 異常検出有効フラグ */
static u16	u16_MDC_battVoltAve_10mV			= (u16)0;					/* フィルタ後のバッテリ電圧(補正反映) */
static u16	u16_MDC_invVoltAve_10mV				= (u16)0;					/* フィルタ後のインバータ電圧 */

static volatile u8	u8_MDC_port8State			= (u8)0;					/* PORT8状態 */

static bool	b_MDC_prechargeRequest				= FALSE;					/* プリチャージ要求フラグ */
static bool	b_MDC_dischargeRequest				= FALSE;					/* ディスチャージ要求 */
static bool	b_MDC_motorStopRequest				= FALSE;					/* モータ停止要求 */
static bool	b_MDC_relayOffRequest				= FALSE;					/* メインリレーOFF要求 */
static bool	b_MDC_permitRelayOff				= FALSE;					/* メインリレーOFF許可 */
static bool	b_MDC_emergencyStop					= FALSE;					/* 緊急停止指令 */
static bool	b_MDC_isMainRelayOn					= FALSE;

#endif
/* Private function prototypes ------------------------------------------------*/
static void	v_MDC_ModeShutwait(void);										/* 電源遮断待機モード */
static void	v_MDC_ModeStartup(void);										/* 起動モード */
static void	v_MDC_ModeStandby(void);										/* 始動操作待ちモード */
static void	v_MDC_ModeRun(void);											/* 走行モード */
static void	v_MDC_ModeIdle(void);											/* IDLEモード */
static void	v_MDC_ModeAbnormal(void);										/* 異常モード */
static void	v_MDC_ModeShutdown(void);										/* 電源遮断モード */
static void	v_MDC_ModePrecharge(void);										/* プリチャージモード */
static void	v_MDC_PrechargeOFF(void);										/* プリチャージ回路OFF */
static void	v_MDC_PrechargeRequest(void);									/* プリチャージ実施判定 */
static void	v_MDC_ResetPrechardeMode(void);									/* プリチャージモードリセット */
static void	v_MDC_ModeDischarge(void);										/* ディスチャージモード */
static void	v_MDC_ResetDischardeMode(void);									/* ディスチャージモードリセット */

/**
 * @brief モード移行判定処理
 * @param none
 * @return none
 * @detail イベントモードを移行判定する
 */
void	V_MDC_Loop10ms(void)
{
	u16_MDC_invVoltAve_10mV = U16_BAT_GetInvVoltAve_10mV();								/* インバータ電圧取得 */
	u16_MDC_battVoltAve_10mV = U16_BAT_GetBattVoltAve_10mV();								/* バッテリ電圧取得 */

	uni_MDC_vcuCommand.byte = U8_CAN_GetVcuCommand();

	b_MDC_relayOffRequest	= uni_MDC_vcuCommand.bit.b_relayOpenCommand;
	b_MDC_dischargeRequest	= uni_MDC_vcuCommand.bit.b_dischargeCommand;
	b_MDC_motorStopRequest	= !B_MTR_IsPermitted();

	u8_MDC_port8State		= (PORT8_STATE);
	/* for TEST */
	if( UNI_GL_vcuTestFlg.bit.b_vcuDummyOrderActivity == TRUE )								/* テスト用VCUダミー要求要求有効? */
	{
		b_MDC_relayOffRequest = UNI_GL_vcuTestFlg.bit.b_relayOffRequest;
		b_MDC_dischargeRequest = UNI_GL_vcuTestFlg.bit.b_dischargeRequest;
	}

#ifdef	EMERGENCY_STOP_ACTIVITY
/***** 緊急停止対応 *****/
	if( B_DD_EX_IsEmergencyStop() == TRUE )
	{
		b_MDC_emergencyStop = TRUE;
//		b_MDC_relayOffRequest	= TRUE;
		b_MDC_motorStopRequest	= TRUE;
	}
#endif
/*======= モード移行関係判定 =====================================*/
//	en_MDC_modeLast = en_MDC_mode;															/* 1回前のmode保存 */
	switch( en_MDC_mode )
	{
/*======= 動作モードにより選択 ======*/
	case MODE_STARTUP:
		v_MDC_ModeStartup();																/* 起動モード */
		break;

	case MODE_STANDBY:
		v_MDC_ModeStandby();																/* 始動操作モード */
		break;

	case MODE_RUN:
		v_MDC_ModeRun();																	/* 走行モード */
		break;

	case MODE_IDLE:
		v_MDC_ModeIdle();																	/* IDLEモード */
		break;

	case MODE_ABNORMAL:
		v_MDC_ModeAbnormal();																/* 異常モード */
		break;

	case MODE_SHUTDOWN_WAIT:
		v_MDC_ModeShutwait();																/* 電源遮断待機モード */
		break;

	case MODE_SHUTDOWN:
		v_MDC_ModeShutdown();																/* 遮断モード */
		break;

	case MODE_NONE:
		break;

	default:
		break;
	}
}

/**
 * @brief プリチャージリレー処理
 * @param none
 * @return none
 */
void	v_MDC_PrechargeRequest(void)
{

	/*------- 過回転or異常状態でプリチャージは実施しない -------*/
	/*------- プリチャージの選択 -------*/
	u8_MDC_mainVoltWaitTimeCounter++;														/* フィルタ後のメイン電圧が安定するのを待つ */
	if( u8_MDC_mainVoltWaitTimeCounter >= (U8_MDC_BATTVOLTWAIT_TIME_10MS) )
	{
		u8_MDC_mainVoltWaitTimeCounter = (U8_MDC_BATTVOLTWAIT_TIME_10MS);					/* カウンタクリップ */
		/*----------------------------------------------
		 *	MISRA-C:2004 14.7
		 *	【逸脱】主要な処理が不要となる為。
		 *----------------------------------------------*/
		if( B_MTR_IsGateOnPermit() == FALSE )
		{
			v_MDC_PrechargeOFF();															/* 全リレーOFF */
			v_MDC_ResetPrechardeMode();														/* プリチャージモードリセット */
			return;
		}

		if( b_MDC_prechargeRequest == FALSE)												/* プリチャージを始めていない場合 */
		{
			if( u16_MDC_invVoltAve_10mV  < (u16_MDC_battVoltAve_10mV + (U16_SYS_VOLTAGE_DIFF_10MV)))	/* バッテリ電圧の方が高い場合 */
			{
				b_MDC_prechargeRequest = TRUE;												/* プリチャージリクエスト要求ON */
			}
		}
	}

	/*------- プリチャージの実行 -------*/
	if( b_MDC_prechargeRequest == TRUE )													/* プリチャージ要求 */
	{
		v_MDC_ModePrecharge();																/* プリチャージON判定 */
	}
}

/**
 * @brief 電源遮断待機モード処理
 * @param none
 * @return none
 */
static void	v_MDC_ModeShutwait(void)
{
	bool	a_b_isShutdownVoltage	= B_DET_BAT_IsShutdownVoltageDropped();

	u16_MDC_shutdownCounter++;																/* 電源遮断カウンタを+1 */
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( (a_b_isShutdownVoltage == TRUE) && 													/* 電圧低下？ */
		(u16_MDC_shutdownCounter >= (U16_MDC_FORCED_SHUTDOWN_TIME_10MS)) )					/* 電源遮断カウンタタイムアップ? */
	{	/*** 電源遮断時間経過の場合 ***/
		u16_MDC_shutdownCounter = (u16)0;													/* 電源遮断カウンタをリセット */
		en_MDC_mode = MODE_SHUTDOWN;														/* 電源遮断モードへ移行 */
	}
	else
	{
		u16_MDC_shutdownCounter = (u16)0;													/* 電源遮断カウンタをリセット */
		v_MDC_ResetDischardeMode();															/* ディスチャージモード初期化 */
		en_MDC_mode = MODE_IDLE;															/* IDLEモードへ移行 */
	}
}


/**
 * @brief 起動モード処理
 * @param none
 * @return none
 */
static void	v_MDC_ModeStartup(void)
{
	bool	a_b_isSomethingAbnormal	= B_ABN_IsSomethingAbnormal();
	bool	a_b_isShutdownVoltage	= B_DET_BAT_IsShutdownVoltageDropped();
	bool	a_b_isKeySwOn			= B_CHT_IsKeySwOn();

	/*------- 初期異常検出の遅延 ---*/
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( u16_MDC_detectActivityCounter < (U16_MDC_DETECT_ACTIVITY_TIME_10MS) )
	{
		u16_MDC_detectActivityCounter++;													/* 異常検出判定有効カウンタ +1 */
		return;																				/* 異常検出遅延が終了するまでモード移行はしない。暫定 */
	}

	u16_MDC_detectActivityCounter = (u16)CMF_GETMIN(u16_MDC_detectActivityCounter, (U16_MDC_DETECT_ACTIVITY_TIME_10MS));	/* カウンタを最大値に制限する */

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( u8_MDC_intializeCounter < (U8_MDC_STARTUP_TIME_10MS) )								/* 初期化完了待ちカウンタを確認 */
	{
		u8_MDC_intializeCounter++;															/* 初期化完了待ちカウンタを+1 */
		return;
	}

/*--------------別のモードへ遷移の判断----------------------------*/
	if( a_b_isKeySwOn == (SW_ON) )															/* キーSW ON？ */
	{
		b_MDC_isDetectActivity = TRUE;														/* 異常検出判定有効 */
		en_MDC_mode = MODE_STANDBY;															/* 始動操作待ちモードへ移行 */
	}
}


/**
 * @brief 始動待ちモード処理
 * @param none
 * @return none
 */
static void	v_MDC_ModeStandby(void)
{
	bool	a_b_isShutdownVoltage	= B_DET_BAT_IsShutdownVoltageDropped();
	bool	a_b_isSomethingAbnormal	= B_ABN_IsSomethingAbnormal();
	bool	a_b_isKeySwOn			= B_CHT_IsKeySwOn();

	/*----------------------------------------------
	 *	下記優先順位に従い状態を遷移させる
		1 シャットダウンフラグ（電圧低下）			→ 電源遮断待機
		2 異常検知									→ 異常
		3 キーSW OFF or 緊急停止					→ IDLE
		4 リレーOFF要求								→ IDLE
		5 モータ停止要求							→ IDLE
		6 車両状態Drive&プリチャージ完了			→ 走行

	※	リレーOFF要求はモード移行条件とするが
		STANDBYモード内ではリレーOFFは実施しない
	 *----------------------------------------------*/
	if( a_b_isShutdownVoltage == TRUE )														/* 1 電圧低下 */
	{
		en_MDC_mode = MODE_SHUTDOWN_WAIT;													/* 電源遮断待機 */
	}
	else if( a_b_isSomethingAbnormal == TRUE )												/* 2 異常状態を確認 */
	{
		v_MDC_ResetDischardeMode();															/* ディスチャージモード初期化 */
		en_MDC_mode = MODE_ABNORMAL;														/* 異常モードへ移行 */
	}
	else if( (a_b_isKeySwOn == (SW_OFF)) ||													/* 3 キーSW OFF？ */
			(b_MDC_emergencyStop == TRUE) )													/*   緊急停止 */
	{
		v_MDC_ResetDischardeMode();															/* ディスチャージモード初期化 */
		en_MDC_mode = MODE_IDLE;															/* IDLEモードへ */
	}
	else if( (MAIN_RELAY_STATE) == (ON_STATE) )
	{
		if( b_MDC_relayOffRequest == FALSE )												/* 4 VCUからのリレーOFF要求無し */
		{
#ifdef	DETECT_ACCEL_ACTIVITY
			if( (B_ABN_IsJudgedDetectAccelAtBoot() == TRUE) &&								/* 初期アクセル異常判定了 */
				 (b_MDC_motorStopRequest == FALSE) )										/* モータ停止要求なし？ */
			{
				en_MDC_mode = MODE_RUN;														/* 走行モードへ移行 */
			}
#else
			if( b_MDC_motorStopRequest == FALSE )											/* モータ停止要求なし？ */
			{
				en_MDC_mode = MODE_RUN;														/* 走行モードへ移行 */
			}
#endif
		}
	}
	else
	{																						/* その他の場合 */
		v_MDC_PrechargeRequest();															/* プリチャージ判定 */

#ifdef	DETECT_ACCEL_ACTIVITY
		if( (en_MDC_prechargeMode == MD_PRECHARGE_FINISH)	&&								/* プリチャージ完了 */
			(B_ABN_IsJudgedDetectAccelAtBoot()) 			&&								/* 初期アクセル異常判定了 */
			(b_MDC_motorStopRequest == FALSE)				&&								/* モータ停止要求なし？ */
			(b_MDC_relayOffRequest == FALSE) )												/* VCUからのリレーOFF要求無し */
#else
		if( (en_MDC_prechargeMode == MD_PRECHARGE_FINISH)	&&								/* プリチャージ完了 */
			(b_MDC_motorStopRequest == FALSE)				&&								/* モータ停止要求なし？ */
			(b_MDC_relayOffRequest == FALSE) )												/* VCUからのリレーOFF要求無し */
#endif
		{																					/* プリチャージ完了 且つ 初期アクセル判断完了 且つ 異常ではない */
			en_MDC_mode = MODE_RUN;															/* 走行モードへ移行 */
		}
	}
}


/**
 * @brief 走行モード処理
 * @param none
 * @return none
 */
static void	v_MDC_ModeRun(void)
{

	bool	a_b_isSomethingAbnormal	= B_ABN_IsSomethingAbnormal();
	bool	a_b_isUnderVoltage		= B_DET_BAT_IsWarningVoltageDropped();
	bool	a_b_isShutdownVoltage	= B_DET_BAT_IsShutdownVoltageDropped();
	bool	a_b_isKeySwOn			= B_CHT_IsKeySwOn();

	/*----------------------------------------------
	 *	下記優先順位に従い状態を遷移させる
		1 シャットダウンフラグ（電圧低下）			→ 電源遮断待機
		2 異常検知									→ 異常
		3 キーSW OFF								→ IDLE
		4 リレーOFF要求								→ IDLE
		5 モータ停止要求							→ IDLE
		5 緊急停止要求								→ IDLE
	 *----------------------------------------------*/
	if( a_b_isShutdownVoltage == TRUE )														/* 1 電圧低下 */
	{
		en_MDC_mode = MODE_SHUTDOWN_WAIT;													/* 電源遮断待機 */
	}
	else if( a_b_isSomethingAbnormal == TRUE )												/* 2 異常状態 */
	{
		en_MDC_mode = MODE_ABNORMAL;														/* 異常モード */
	}
	else if( (a_b_isKeySwOn == (SW_OFF))		||											/* 3 キーSW OFF？ */
			 (b_MDC_relayOffRequest == TRUE)	||											/* 4 リレーOFF要求？ */
			 (b_MDC_motorStopRequest == TRUE)	||											/* 5 モータ停止要求？ */
			 (b_MDC_emergencyStop == TRUE) )												/* 6 緊急停止要求？ */
	{
		v_MDC_ResetDischardeMode();															/* ディスチャージモード初期化 */
		en_MDC_mode = MODE_IDLE;															/* IDLEモードへ */
	}
	else																					/* その他の場合 */
	{																						/* 処理無し */
		/* MISRA-C 2004 14.10 対応 */
	}
}

/**
 * @brief IDLEモード処理
 * @param none
 * @return none
 */
static void	v_MDC_ModeIdle(void)
{

	bool	a_b_isSomethingAbnormal	= B_ABN_IsSomethingAbnormal();
	bool	a_b_isUnderVoltage		= B_DET_BAT_IsWarningVoltageDropped();
	bool	a_b_isShutdownVoltage	= B_DET_BAT_IsShutdownVoltageDropped();
	bool	a_b_isKeySwOn			= B_CHT_IsKeySwOn();

	/*----------------------------------------------
	 *	下記優先順位に従い状態を遷移させる
		1 シャットダウンフラグ（電圧低下）			→ 電源遮断待機
		2 異常検知									→ 異常
		3 リレーOFF要求無し &
		4 キーSW ON								→ スタンバイ
	 *----------------------------------------------*/
	if( a_b_isShutdownVoltage == TRUE )														/* 1 電圧低下 */
	{
		en_MDC_mode = MODE_SHUTDOWN_WAIT;													/* 電源遮断待機 */
	}
	else if( a_b_isSomethingAbnormal == TRUE )												/* 2 異常状態 */
	{
		en_MDC_mode = MODE_ABNORMAL;														/* 異常モード */
	}
	else																					/* その他の場合 */
	{																						/* 処理無し */
		/* MISRA-C 2004 14.10 対応 */
	}

	/* VCUからのディスチャージ要求対応 */
	if( (b_MDC_dischargeRequest == TRUE) &&
		(B_DD_EX_IsAbnormalOverCurrent() == FALSE) )
	{
		v_MDC_ModeDischarge();																/* ディスチャージ処理 */
	}

	/* VCUからのリレーOFF要求とキーSWの対応 */
	if( b_MDC_relayOffRequest == TRUE )														/* リレーOFF要求？ */
	{
		if( b_MDC_permitRelayOff == TRUE )													/* モータ的にリレーOFF可能 */
		{
			v_MDC_ResetPrechardeMode();														/* プリチャージ初期化処理 */
			PRECHA_FET_OFF();																/* プリチャージ抵抗FET OFF */
			MAIN_RELAY_OFF();																/* リレーOFF */
			b_MDC_isMainRelayOn = FALSE;
		}
	}

	if( (a_b_isKeySwOn	== (SW_ON)) &&														/* キーSW ON？ */
		(b_MDC_emergencyStop == FALSE) )													/* 緊急停止要求なし？ */
	{
		if( (MAIN_RELAY_STATE) == (OFF_STATE) )
		{
			PRECHA_FET_OFF();																/* プリチャージ抵抗FET OFF */
			v_MDC_ResetPrechardeMode();														/* プリチャージ初期化処理 */
		}
#ifdef	DETECT_ACCEL_ACTIVITY
		V_ABN_ReleaseJudgedDetectAccelAtBoot();												/* 初期アクセル異常判定済みフラグ解除 */
#endif
		en_MDC_mode = MODE_STANDBY;															/* スタンバイモードへ */
	}
	else																					/* その他の場合 */
	{																						/* 処理無し */
		/* MISRA-C 2004 14.10 対応 */
	}
}

/**
 * @brief 異常モード処理
 * @param none
 * @return none
 */
static void	v_MDC_ModeAbnormal(void)
{
	bool	a_b_isAbnormalCanCom	= B_ABN_IsAbnormalCanCommunication();
	bool	a_b_isShutdownVoltage	= B_DET_BAT_IsShutdownVoltageDropped();
	bool	a_b_isKeySwOn			= B_CHT_IsKeySwOn();
	bool	a_b_isSomethingAbnormal	= B_ABN_IsSomethingAbnormal();

	/*----------------------------------------------
	 *	下記優先順位に従い状態を遷移させる
		1 シャットダウンフラグ（電圧低下）			→ 電源遮断待機
		2 異常無し									→ IDLE
		3 緊急停止要求無し							→ IDLE
	 *----------------------------------------------*/

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( a_b_isShutdownVoltage == TRUE )														/* 1 電圧低下 */
	{
		en_MDC_mode = MODE_SHUTDOWN_WAIT;													/* 車両状態 ＝ 電源遮断待機 */
		return;
	}

	if( (a_b_isSomethingAbnormal == FALSE) &&												/* 2 異常無し状態 */
		(b_MDC_emergencyStop == FALSE) )													/* 3 緊急停止要求無し？ */
	{
		v_MDC_ResetDischardeMode();															/* ディスチャージモード初期化 */
		en_MDC_mode = MODE_IDLE;															/* IDLEモード */
	}

	/* VCUからのリレーOFF要求の対応 */
	if( b_MDC_relayOffRequest == TRUE )														/* リレーOFF要求？ */
	{
		if( b_MDC_permitRelayOff == TRUE )													/* モータ的にリレーOFF可能 */
		{
			v_MDC_ResetPrechardeMode();														/* プリチャージ初期化処理 */
			PRECHA_FET_OFF();																/* プリチャージ抵抗FET OFF */
			MAIN_RELAY_OFF();																/* リレーOFF */
			b_MDC_isMainRelayOn = FALSE;
		}
	}

	/* VCUからのディスチャージ要求対応 */
	if( (b_MDC_dischargeRequest == TRUE) &&
		(B_DD_EX_IsAbnormalOverCurrent() == FALSE) &&
		(b_MDC_emergencyStop == FALSE) )
	{
		v_MDC_ModeDischarge();																/* ディスチャージ処理 */
	}

}

/**
 * @brief 電源遮断モード処理
 * @param none
 * @return none
 */
static void	v_MDC_ModeShutdown(void)
{
	bool	a_b_isShutdownVoltDropped	= B_DET_BAT_IsShutdownVoltageDropped();

	v_MDC_PrechargeOFF();																	/* 全リレーOFF */
	v_MDC_ResetPrechardeMode();
	v_MDC_ResetDischardeMode();
}

/**
 * @brief プリチャージ回路OFF
 * @param none
 * @return none
 */
static void	v_MDC_PrechargeOFF(void)
{
	MAIN_RELAY_OFF();																		/* メインリレーFET OFF */
	PRECHA_FET_OFF();																		/* プリチャージ抵抗FET OFF just in case */
	b_MDC_isMainRelayOn = FALSE;
}

/**
 * @brief プリチャージモードリセット 
 * @param none
 * @return none
 */
static void	v_MDC_ResetPrechardeMode(void)
{
	if( en_MDC_prechargeMode != (MD_PRECHARGE_ABNORMAL))
	{
		b_MDC_prechargeRequest = FALSE;														/* プリチャージ要求フラグをクリア */
		en_MDC_prechargeMode = MD_PRECHARGE_INITIAL;										/* プリチャージ状態初期化 */
		u8_MDC_mainVoltWaitTimeCounter = (u8)0;												/* リレー切替でメイン電圧待ちタイマリセット */
	}
}

/**
 * @brief ディスチャージモードリセット 
 * @param none
 * @return none
 */
static void	v_MDC_ResetDischardeMode(void)
{
	if( en_MDC_dischargeMode != (MD_DISCHARGE_ABNORMAL))
	{
		en_MDC_dischargeMode = MD_DISCHARGE_INITIAL;										/* ディスチャージ状態初期化 */
	}
}

/**
 * @brief プリチャージ処理
 * @note 初期化からプリチャージ完了までの処理
 * @param none
 * @return none
 */
static void	v_MDC_ModePrecharge(void)
{
	s16	a_s16_invVoltForPrechgJdg = (s16)u16_MDC_battVoltAve_10mV - (s16)(U16_SYS_PRECHARGE_FIN_VOLTAGE_DIFF_10MV);

	switch( en_MDC_prechargeMode )
	{
	case MD_PRECHARGE_INITIAL:
		v_MDC_ResetDischardeMode();															/* ディスチャージモードリセット */
		PRECHA_FET_ON();																	/* プリチャージ抵抗FET ON */
		MAIN_RELAY_OFF();																	/* メインリレーFET OFF */
		b_MDC_isMainRelayOn = FALSE;

		en_MDC_prechargeMode = MD_PRECHARGE_RUN;
		break;

	case MD_PRECHARGE_RUN:
		if( ( u16_MDC_prechargeWaitCounter >= (u16)20 ) && ( a_s16_invVoltForPrechgJdg < (s16)u16_MDC_invVoltAve_10mV ) )	/* 200 ms待機 & インバータ電圧が溜まっている */
		{
			u16_MDC_prechargeWaitCounter = (u16)0;											/* プリチャージカウント0 */
			MAIN_RELAY_ON();																/* メインリレーFET ON */
			PRECHA_FET_OFF();																/* プリチャージ抵抗FET OFF */
			en_MDC_prechargeMode = MD_PRECHARGE_FINISH;										/* プリチャージ状態を終了判定 */
			b_MDC_isMainRelayOn = TRUE;
		}
		else																				/* 500 ms 経過していない */
		{
			u16_MDC_prechargeWaitCounter ++ ;												/* カウントアップ */
			u16_MDC_prechargeWaitCounter =(u16)CMF_GETMIN(u16_MDC_prechargeWaitCounter,(u16)20);
		}
		break;

	case MD_PRECHARGE_FINISH:
		break;

	case MD_PRECHARGE_OFF:
		v_MDC_PrechargeOFF();
		break;

	case MD_PRECHARGE_ABNORMAL:
		v_MDC_PrechargeOFF();
		break;

	default:
		break;
	}
}

/**
 * @brief プリチャージ異常判定
 * @note DETECT_ABNORMALでコール：10msec周期
 * @param none
 * @return none
 */
void	V_MDC_DetectPrecharge(void)
{
	u16	a_u16_invVolt_10mV = U16_BAT_GetInvVoltAve_10mV();									/* インバータ電圧取得 */

	u16	a_u16_invVoltForDetect_10mV = U16_BAT_GetBattVoltAve_10mV() >> (u16)1;				/* バッテリ電圧の1/2 */

	if( en_MDC_prechargeMode == MD_PRECHARGE_RUN )											/* プリチャージ中 */
	{
		u16_MDC_prechargeTimeoutCounter++;													/* プリチャージタイム計測 */
		if( u16_MDC_prechargeTimeoutCounter >= (u16)(U16_ABN_ABNORMAL_COUNT_10S) )			/* 10s以上経過でプリチャージが終了していない */
		{
			en_MDC_prechargeMode = MD_PRECHARGE_ABNORMAL;									/* プリチャージ異常と判断 */
			V_ABN_SetAbnormalPrechargeWaitOver();											/* プリチャージ異常セット */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_PRECHARGE_ERR) );						/* ダイアグコードの格納 */
			u16_MDC_prechargeTimeoutCounter = (u16)(U16_ABN_ABNORMAL_COUNT_10S);
		}
	}
	else
	{
		u16_MDC_prechargeTimeoutCounter = (u16)0;
	}
}

/**
 * @brief ディスチャージ処理
 * @note VCUからディスチャージ要求が入った場合のみ呼び出される
 * @param none
 * @return none
 */
static void	v_MDC_ModeDischarge(void)
{
	switch( en_MDC_dischargeMode )
	{
	case MD_DISCHARGE_INITIAL:
		PRECHA_FET_OFF();																	/* プリチャージ抵抗FET OFF */
		v_MDC_ResetPrechardeMode();															/* プリチャージモードリセット */
		if( (MAIN_RELAY_STATE) == (OFF_STATE) )												/* リレーOFF状態 ディスチャージ実行可能 */
		{
			en_MDC_dischargeMode = (MD_DISCHARGE_RUN);										/* ディスチャージ実行モード */
		}
		else																				/* リレーON状態 */
		{
			if( b_MDC_permitRelayOff == TRUE )												/* モータ的にリレーOFF可能 */
			{
				MAIN_RELAY_OFF();															/* リレーOFF */
				b_MDC_isMainRelayOn = FALSE;
			}
		}
		break;

	case MD_DISCHARGE_RUN:
		if( ((u16_MDC_invVoltAve_10mV <= (U16_SYS_DISCHARGE_FIN_VOLTAGE_10MV))		&&		/* インバータ電圧がディスチャージ終了電圧よりも低い */
			(u16_MDC_battVoltAve_10mV > (U16_LOWVOLTAGE_RELEASE_BATT_VOLT_10MV)))	||		/* バッテリ電圧が低電圧警告解除電圧よりも高い */
			(b_MDC_dischargeRequest == FALSE) )												/* ディスチャージ要求無し */
		{
			en_MDC_dischargeMode = (MD_DISCHARGE_PREPARING_FINISH);							/* ディスチャージ状態を終了準備判定 */
		}
		break;

	case MD_DISCHARGE_PREPARING_FINISH:
		if( S16_MTR_GetIfFbAve_10mA() <= (s16)(U16_MTR_STOPPED_IFFB_10MA) )					/* If平均値が停止と判断する電流値以下 */
		{
			en_MDC_dischargeMode = (MD_DISCHARGE_FINISH);									/* ディスチャージ状態を終了 */
		}
		break;

	case MD_DISCHARGE_FINISH:
		break;

	case MD_DISCHARGE_OFF:
		v_MDC_PrechargeOFF();
		break;

	case MD_DISCHARGE_ABNORMAL:
		v_MDC_PrechargeOFF();
		break;

	default:
		break;
	}
}

/**
 * @brief ディスチャージ異常判定
 * @note DETECT_ABNORMALでコール：10msec周期
 * @param none
 * @return none
 */
void	V_MDC_DetectDischarge(void)
{
	u16	a_u16_invVolt_10mV = U16_BAT_GetInvVoltAve_10mV();									/* インバータ電圧取得 */

	if( en_MDC_dischargeMode == MD_DISCHARGE_RUN )											/* ディスチャージ中 */
	{
		u16_MDC_dischargeTimeoutCounter++;													/* ディスチャージタイム計測 */
		/* for DEBUG */
		if( u16_MDC_dischargeTimeoutCounter >= (U16_ABN_ABNORMAL_COUNT_10S) )				/* 10s以上経過でディスチャージが終了していない */
		{
			en_MDC_dischargeMode = MD_DISCHARGE_ABNORMAL;									/* ディスチャージ異常と判断 */
			V_ABN_SetAbnormalDischargeWaitOver();											/* ディスチャージ異常セット */
			V_DIAG_SaveDiagCode( (u16)(U8_DIAG_NUM_DISCHARGE_ERR) );						/* ダイアグコードの格納 */
			u16_MDC_dischargeTimeoutCounter = (U16_ABN_ABNORMAL_COUNT_10S);
		}
	}
	else
	{
		u16_MDC_dischargeTimeoutCounter = (u16)0;
	}
}

/**
 * @brief 動作モード提供
 * @param none
 * @return en_MDC_mode
 */
EN_MDC_MODE	En_MDC_GetMode(void)
{
	return	en_MDC_mode;
}

/**
 * @brief 電源遮断カウンタセット
 * @param a_u16_count
 * @return none
 */
void	U16_MDC_SetShutdownCounter(u16 a_u16_count)
{
	u16_MDC_shutdownCounter = a_u16_count;
}

/**
 * @brief プリチャージモード提供
 * @param none
 * @return en_MDC_prechargeMode
 */
EN_MDC_PRECHARGE_MODE	En_MDC_GetPrechargeMode(void)
{
	return	en_MDC_prechargeMode;
}

/**
 * @brief ディスチャージモード提供
 * @param none
 * @return en_MDC_prechargeMode
 */
EN_MDC_DISCHARGE_MODE	En_MDC_GetDischargeMode(void)
{
	return	en_MDC_dischargeMode;
}

/**
 * @brief 異常検出許可状態提供
 * @param none
 * @return b_MDC_isDetectActivity
 */
bool	B_MDC_IsDetectActivity(void)
{
	return	b_MDC_isDetectActivity;
}

/**
 * @brief 異常検出許可
 * @param none
 * @return none
 */
void	V_MDC_SetDetectActivity(void)
{
	b_MDC_isDetectActivity = TRUE;
}

/**
 * @brief 異常検出禁止
 * @param none
 * @return none
 */
void	V_MDC_DetectInactivity(void)
{
	b_MDC_isDetectActivity = FALSE;
}

/**
 * @brief メインリレーOFF要求
 * @param none
 * @return none
 */
void	V_MDC_RelayOffRequest(void)
{
	b_MDC_relayOffRequest = TRUE;
}

/**
 * @brief メインリレーOFF要求解除
 * @param none
 * @return none
 */
void	V_MDC_CancelRelayOffRequest(void)
{
	b_MDC_relayOffRequest = FALSE;
}

/**
 * @brief メインリレーOFF許可
 * @param none
 * @return none
 */
void	V_MDC_PermitRelayOff(void)
{
	b_MDC_permitRelayOff = TRUE;
}

/**
 * @brief メインリレーOFF禁止
 * @param none
 * @return none
 */
void	V_MDC_FrobidRelayOff(void)
{
	b_MDC_permitRelayOff = FALSE;
}

/**
 * @brief 緊急停止要求提供
 * @param none
 * @return b_MDC_emergencyStop
 */
bool	B_MDC_IsEmergencyStop(void)
{
	return	b_MDC_emergencyStop;
}

/**
 * @brief モード移行初期化
 * @param none
 * @return none
 */
void	V_MDC_Initialize(void)
{
	/* 暫定 */
	b_MDC_isDetectActivity = FALSE;
	u16_MDC_detectActivityCounter = (u16)0;													/* 異常検出判定有効カウンタ初期化 */

	en_MDC_prechargeMode = MD_PRECHARGE_INITIAL;											/* プリチャージ未処理 */
	en_MDC_dischargeMode = MD_DISCHARGE_INITIAL;											/* ディスチャージ未処理 */
}

