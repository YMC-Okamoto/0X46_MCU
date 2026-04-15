/**
 * @file CAN.c
 * @brief CAN通信
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */

/* Includes ------------------------------------------------------------------*/
#include "can_api.h"

#include "TYPEDEF_MODE.h"
#include "DEFINE_SYSTEM.h"
#include "DEFINE_SETTING.h"
#include "DEFINE_MOTOR.h"		/*定数定義*/
#include "DEFINE_ABN.h"			/*定数定義*/
#include "DEFINE_CAN.h"			/*定数定義*/
#include "COMMON_FUNCTION.h"	/* 汎用関数群 */
#include "TRIGONOMETRIC_FUNCTION.h"

#include "DIAG_SET.h"
#include "MOTOR.h"
#include "BATTERY.h"
#include "DETECT_ABNORMAL.h"
#include "DETECT_BATTERY.h"
#include "DETECT_MOTOR.h"
#include "LIMIT_MOTOR.h"
#include "DD_EXCITER.h"
#include "DD_MOTOR.h"
#include "ADC.h"
#include "MODE_CONDUCTOR.h"
#include "CuW.h"
#include "CAN.h"
#include "PREVENT_CHATTERING.h"
#include "MCU.h"
#include "ramdef.h"

/* Private define -----------------------------------------------------------*/
#define U16_CAN_CYCLE_10MS			(u16)10
#define U16_CAN_CYCLE_500MS			(u16)500

#define U8_CAN0_RXFIFO_USEMAX		(u8)1								/* 現在使用する最大受信FIFO数 */

#define	U8_CAN_WARNING_COUNT_10MS	(u8)16								/* CAN未受信カウント 160msec */

/* Private variables --------------------------------------------------------*/
#pragma bit_order right
#pragma unpack

#ifdef	DEBUG_WATCH_ACTIVITY

union
{
	u16			word;
	struct
	{
		u16	b_canTxReqMucData1		:1;					/* b0:MCUデータ1 */
		u16	b_canTxReqMucData2		:1;					/* b1:MCUデータ2 */
		u16	b_canTxReqMucData3		:1;					/* b2:MCUデータ3 */
		u16	b_canTxReqMucData4		:1;					/* b3:MCUデータ4 */
		u16	b_canTxReqMucData5		:1;					/* b4:MCUデータ5 */
		u16	b_canTxReqMucData6		:1;					/* b5:MCUデータ6 */
		u16	b_canTxReqMucData7		:1;					/* b6:MCUデータ7 */
		u16	b_canTxReqMucData8		:1;					/* b7:MCUデータ8 */
		u16	b_canTxReqMucData9		:1;					/* b8:MCUデータ9 */
		u16	b_canTxReqMcuErrCode	:1;					/* b9:MCU異常コード */
		u16	b_canTxReqCuw			:1;					/* b10:ロギング */
		u16							:5;					/* b11-15:予備 */
	}bit;
}uni_CAN_txReqMcu;

ST_CAN_STD_FRAME	*pst_CAN_txBuff;										/* CAN送信データバッファのポインタ */
ST_CAN_STD_FRAME	*pst_CAN_rxBuff;										/* CAN受信データバッファのポインタ */
ST_CAN_STD_FRAME	st_CAN_txData;											/* CAN送信データバッファ */
ST_CAN_STD_FRAME	st_CAN_rxData;											/* CAN受信データバッファ */

UNI_VCU_COMMAND	uni_CAN_vcuCommand;										/* VCU Command */

bool b_CAN_firstTransmit = FALSE;

bool b_CAN_notYetReceived		= TRUE;									/* CAN未受信 From VCU */
u8	u8_CAN_warningCanVcuCounter	= (u8)0;								/* VCUからのCAN受信異常検出タイマ  */
bool	b_CAN_isReceivedFromVcu		= FALSE;								/* 受信フラグ */
bool	b_CAN_rceptionWarningFromVCU = FALSE;								/* CAN通信警告 VCU */

bool	b_CAN_detectCanActivate		= TRUE;									/* CAN通信異常検出有効(TRUE)/無効(FALSE) */
volatile u8	u8_CAN_abnormalCanCounter	= (u8)0;						/* CAN異常検出タイマ  */

volatile u16	u16_CAN_accelAdValue		= (u16)0;						/* アクセルAD値(max：4096) */
bool	b_CAN_keySwStatusVCU				= FALSE;						/* キーSW状態 */

s16	s16_CAN_torqueOrder_100mNm			= (s16)0;						/* トルク指令値[0.1Nm] */
u8	u8_CAN_buzzerPattern				= (u8)0;						/* ブザーパターン */
u8	u8_CAN_vcuErrorInfo					= (u8)0;						/* VCUエラー詳細 */
s8	s8_CAN_eangleOffsetFromCan			= (s8)0;						/* エンコーダオフセット */
u8	u8_CAN_revTimeConstFactorFromCan	= (U8_SYS_TIME_CONST_FACTOR);	/* モータ回転数フィルタ時定数(×1ms) */

u8 u8_CAN_rxVcuCommandData[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* VCU受信データ */

u8 u8_CAN_rxCanoeData1[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ1 */
u8 u8_CAN_rxCanoeData2[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ2 */
u8 u8_CAN_rxCanoeData3[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ3 */
u8 u8_CAN_rxCanoeData4[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ4 */
u8 u8_CAN_rxCanoeData5[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ5 */

volatile bool	b_CAN_isErrorPeripheral	= FALSE;						/* CAN周辺エラー */

volatile bool	b_CAN_isNewDataReceived	= FALSE;

u16	u16_CAN_cycle = (u16)0;												/* CAN送信間隔カウンタ */

#else

static union
{
	u16			word;
	struct
	{
		u16	b_canTxReqMucData1		:1;					/* b0:MCUデータ1 */
		u16	b_canTxReqMucData2		:1;					/* b1:MCUデータ2 */
		u16	b_canTxReqMucData3		:1;					/* b2:MCUデータ3 */
		u16	b_canTxReqMucData4		:1;					/* b3:MCUデータ4 */
		u16	b_canTxReqMucData5		:1;					/* b4:MCUデータ5 */
		u16	b_canTxReqMucData6		:1;					/* b5:MCUデータ6 */
		u16	b_canTxReqMucData7		:1;					/* b6:MCUデータ7 */
		u16	b_canTxReqMucData8		:1;					/* b7:MCUデータ8 */
		u16	b_canTxReqMucData9		:1;					/* b8:MCUデータ9 */
		u16	b_canTxReqMcuErrCode	:1;					/* b9:MCU異常コード */
		u16	b_canTxReqCuw			:1;					/* b10:ロギング */
		u16							:5;					/* b11-15:予備 */
	}bit;
}uni_CAN_txReqMcu;


static ST_CAN_STD_FRAME	*pst_CAN_txBuff;										/* CAN送信データバッファのポインタ */
static ST_CAN_STD_FRAME	*pst_CAN_rxBuff;										/* CAN受信データバッファのポインタ */
static ST_CAN_STD_FRAME	st_CAN_txData;											/* CAN送信データバッファ */
static ST_CAN_STD_FRAME	st_CAN_rxData;											/* CAN受信データバッファ */

static UNI_VCU_COMMAND	uni_CAN_vcuCommand;										/* VCU Command */

static bool b_CAN_firstTransmit = FALSE;

static bool b_CAN_notYetReceived		= TRUE;									/* CAN未受信 From VCU */
static u8	u8_CAN_warningCanVcuCounter	= (u8)0;								/* VCUからのCAN受信異常検出タイマ  */
static bool	b_CAN_isReceivedFromVcu		= FALSE;								/* 受信フラグ */
static bool	b_CAN_rceptionWarningFromVCU = FALSE;								/* CAN通信警告 VCU */

static bool	b_CAN_detectCanActivate		= TRUE;									/* CAN通信異常検出有効(TRUE)/無効(FALSE) */
static volatile u8	u8_CAN_abnormalCanCounter	= (u8)0;						/* CAN異常検出タイマ  */

static volatile u16	u16_CAN_accelAdValue		= (u16)0;						/* アクセルAD値(max：4096) */
static bool	b_CAN_keySwStatusVCU				= FALSE;						/* キーSW状態 */

static s16	s16_CAN_torqueOrder_100mNm			= (s16)0;						/* トルク指令値[0.1Nm] */
static u8	u8_CAN_buzzerPattern				= (u8)0;						/* ブザーパターン */
static u8	u8_CAN_vcuErrorInfo					= (u8)0;						/* VCUエラー詳細 */
static s8	s8_CAN_eangleOffsetFromCan			= (s8)0;						/* エンコーダオフセット */
static u8	u8_CAN_revTimeConstFactorFromCan	= (U8_SYS_TIME_CONST_FACTOR);	/* モータ回転数フィルタ時定数(×1ms) */

static u8 u8_CAN_rxVcuCommandData[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* VCU受信データ */

static u8 u8_CAN_rxCanoeData1[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ1 */
static u8 u8_CAN_rxCanoeData2[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ2 */
static u8 u8_CAN_rxCanoeData3[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ3 */
static u8 u8_CAN_rxCanoeData4[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ4 */
static u8 u8_CAN_rxCanoeData5[8] = {(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0,(u8)0};	/* CANoe受信データ5 */

static volatile bool	b_CAN_isErrorPeripheral	= FALSE;						/* CAN周辺エラー */

static volatile bool	b_CAN_isNewDataReceived	= FALSE;

static u16	u16_CAN_cycle = (u16)0;												/* CAN送信間隔カウンタ */

#endif
/* Private function prototypes ------------------------------------------------*/
static void	v_CAN_UpdateTxRequest(void);								/* CAN送信要求更新 */
static void	v_CAN_Transmit(u16 a_u16_canId);							/* CANデータ送信 */
static void	v_CAN_TransmitDataSet(u16 a_u16_canId);						/* CAN送信データの設定 */
static void	v_CAN_GetReceiveData(void);									/* CANデータ受信 */
static void	v_CAN_AllocationReceiveData(void);							/* CAN受信データの割り当て */
static void	v_CAN_AllocationReceiveData1FromCanoe(void);				/* CAN受信データの割り当て */
static void	v_CAN_AllocationReceiveData2FromCanoe(void);				/* CAN受信データの割り当て */
static void	v_CAN_DetectRxTimeout(void);								/* CAN受信タイムアウト（割り込みこない）*/
static void	v_CAN_SafetyAllocationAtWarning(void);						/* 通信無効判定時のSafetyデータ割り付け */


/**
 * @brief CAN初期化
 * @detail CANモードと通信ポート初期化
 * @param none
 * @return none
 */
void	V_CAN_Initialize(void)
{
	pst_CAN_txBuff = &st_CAN_txData;																		/* CAN送受信バッファポインタ設定 */
	pst_CAN_rxBuff = &st_CAN_rxData;
	R_CAN_PortSet( (U32_CAN_UNIT0), (u32)(ENABLE) );														/* CANモードよりポートの設定 */
	R_CAN_RxSet((U32_CAN_CH0),(U32_CAN_MBOX3),(CANID_VCU_COMMAND),(U32_CAN_DATA_FRAME));					/* 受信メールボクス定義 */
	R_CAN_RxSet((U32_CAN_CH0),(U32_CAN_MBOX4),(CANID_CUW_RX),(U32_CAN_DATA_FRAME));							/* 受信メールボクス定義 */
	uni_CAN_txReqMcu.word = (u16)0;																			/* 送信要求フラグ初期化 */
}

/**
 * @brief CAN関連1msecループ
 * @param none
 * @return none
 */
void	V_CAN_Loop1ms(void)																					/* CAN関連1msecループ */
{
	v_CAN_UpdateTxRequest();																				/* CAN送信要求更新 */
}

/**
 * @brief CAN関連10msecループ
 * @param none
 * @return none
 */
void	V_CAN_Loop10ms(void)																				/* CAN関連1msecループ */
{
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( UNI_GL_testFlg.bit.b_forBenchtest == TRUE )
	{
		uni_CAN_vcuCommand.bit.b_motorStopCommand = FALSE;													/* モータ停止要求解除 */
		uni_CAN_vcuCommand.bit.b_relayOpenCommand = FALSE;													/* リレー解放要求解除 */
		return;
	}

	if( b_CAN_rceptionWarningFromVCU == TRUE )
	{
		v_CAN_SafetyAllocationAtWarning();																	/* 通信無効判定時のSafetyデータ割り付け */
	}
}

/**
 * @brief CAN送信要求更新
 * @note 1msec周期
 * @param none
 * @return none
 * @note CAN送信要求を更新する
 */
static void	v_CAN_UpdateTxRequest(void)
{
	if( B_CUW_IsStoredTxData() == TRUE )
	{
		uni_CAN_txReqMcu.bit.b_canTxReqCuw = TRUE;															/* ロギング CuW 0x63D	送信要求セット */
	}

	if( (u16)( u16_CAN_cycle + (u16)9 ) % (U16_CAN_CYCLE_10MS) == (u16)0 )									/* 10ms周期で（1msからstart）送信 */
	{
		uni_CAN_txReqMcu.bit.b_canTxReqMucData1 = TRUE;														/* MCUデータ1 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData2 = TRUE;														/* MCUデータ2 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData3 = TRUE;														/* MCUデータ3 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData4 = TRUE;														/* MCUデータ4 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData5 = TRUE;														/* MCUデータ5 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData6 = TRUE;														/* MCUデータ6 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData7 = TRUE;														/* MCUデータ7 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData8 = TRUE;														/* MCUデータ8 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData9 = TRUE;														/* MCUデータ9 */
//		uni_CAN_txReqMcu.bit.b_canTxReqMcuErrCode = TRUE;													/* 異常コード */
	}

	if( (u16)( u16_CAN_cycle + (u16)496 ) % (U16_CAN_CYCLE_500MS) == (u16)0 )								/* 500ms周期で（4msからstart）送信 */
	{
		R_BSP_NOP();
	}

	u16_CAN_cycle++;
	if( u16_CAN_cycle >= (u16)500 )
	{
		u16_CAN_cycle = (u16)0;
	}
}

/**
 * @brief CANタイミング送信
 * @note メインループ
 * @param none
 * @return none
 * @note 送信タイミング毎にCAN送信する
 */
void	V_CAN_CyclicTransmit(void)
{
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( uni_CAN_txReqMcu.word == (u16)0 )
	{
		return;																								/* 送信要求なし */
	}

	if( uni_CAN_txReqMcu.bit.b_canTxReqCuw == TRUE )
	{
		v_CAN_Transmit( (CANID_CUW_TX) );																	/* ロギング CuW 0x63D	送信データセット済み時のみ実行 */
	}
	else if( uni_CAN_txReqMcu.bit.b_canTxReqMucData1 == TRUE )
	{
		v_CAN_Transmit( (CANID_MCU_DATA1) );																/* MCUデータ1 */
	}
	else if( uni_CAN_txReqMcu.bit.b_canTxReqMucData2 == TRUE )
	{
		v_CAN_Transmit( (CANID_MCU_DATA2) );																/* MCUデータ2 */
	}
	else if( uni_CAN_txReqMcu.bit.b_canTxReqMucData3 == TRUE )
	{
		v_CAN_Transmit( (CANID_MCU_DATA3) );																/* MCUデータ3 */
	}
	else if( uni_CAN_txReqMcu.bit.b_canTxReqMucData4 == TRUE )
	{
		v_CAN_Transmit( (CANID_MCU_DATA4) );																/* MCUデータ4 */
	}
	else if( uni_CAN_txReqMcu.bit.b_canTxReqMucData5 == TRUE )
	{
		v_CAN_Transmit( (CANID_MCU_DATA5) );																/* MCUデータ5 */
	}
	else if( uni_CAN_txReqMcu.bit.b_canTxReqMucData6 == TRUE )
	{
		v_CAN_Transmit( (CANID_MCU_DATA6) );																/* MCUデータ6 */
	}
	else if( uni_CAN_txReqMcu.bit.b_canTxReqMucData7 == TRUE )
	{
		v_CAN_Transmit( (CANID_MCU_DATA7) );																/* MCUデータ7 */
	}
	else if( uni_CAN_txReqMcu.bit.b_canTxReqMucData8 == TRUE )
	{
		v_CAN_Transmit( (CANID_MCU_DATA8) );																/* MCUデータ8 */
	}
	else if( uni_CAN_txReqMcu.bit.b_canTxReqMucData9 == TRUE )
	{
		v_CAN_Transmit( (CANID_MCU_DATA9) );																/* MCUデータ9 */
	}
//	else if( uni_CAN_txReqMcu.bit.b_canTxReqMcuErrCode == TRUE )
//	{
//		v_CAN_Transmit( (CANID_MCU_ERRCODE) );																/* 異常コード */
//	}
	else
	{
		/* MISRA-C 2004 14.10 対応 */
	}
}


/**
 * @brief CANデータ送信
 * @param a_u8_msgNum:メールボックス番号（0-31)
 * @return none
 * @note メールボックスのメセッジを送信
 */
static void	v_CAN_Transmit(u16 a_u16_canId)
{
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if(b_CAN_firstTransmit == TRUE)
	{
		if( R_CAN_TxCheck((U32_CAN_UNIT0), (u32)0) != (u32)0 )
		{
			return;
		}
	}
	else
	{
		b_CAN_firstTransmit = true;
	}
	v_CAN_TransmitDataSet(a_u16_canId);																		/* IDより送信データセット */
}


/**
 * @brief CAN送信データ設定
 * @param a_u16_canId:メールボックス番号（0-31)
 * @return none
 * @note 各送信メールボックスのメセッジデータ設定
 */
static void	v_CAN_TransmitDataSet(u16 a_u16_canId)
{
	EN_MDC_MODE	a_en_Mode = En_MDC_GetMode();																/* 動作モード取得 */

	UNI_BYTE_FLAG a_u8_txData ;
	a_u8_txData.byte = (u8)0;

	ST_ADC_DQ32	a_st_dq32Ave = {(s32)0,(s32)0,(s32)0,(s32)0,(s32)0,(s32)0};							/* DQ軸平均値データ */
	a_st_dq32Ave = St_ADC_GetDq32Ave();																/* DQ軸平均データ取得 */

	s16	a_s16_vdAve	= (s16)a_st_dq32Ave.s32_vd_10mV;
	s16	a_s16_vqAve	= (s16)a_st_dq32Ave.s32_vq_10mV;

	s32	a_s32_data	= (s32)0;
	s32	a_s32_data1	= (s32)0;

	u32	a_u32_data	= (u32)0;
	u32	a_u32_data1	= (u32)0;

	u16	a_u16_data	= (u16)0;
	u16	a_u16_data1	= (u16)0;
	u16	a_u16_data2	= (u16)0;
//	u16	a_u16_data3	= (u16)0;

	s16	a_s16_data	= (s16)0;
	s16	a_s16_data1	= (s16)0;
	s16	a_s16_data2	= (s16)0;
	s16	a_s16_data3	= (s16)0;

	u8	a_u8_data	= (s8)0;

	s8	a_s8_data	= (s8)0;
	s8	a_s8_data1	= (s8)0;

	switch (a_u16_canId)
	{
	case (CANID_MCU_DATA1):																					/* MCUデータ1 */ /* 10ms */
		a_u8_txData.bit.b0 = B_CHT_IsAccelSwOn();															/* アクセルSW取得 */
		a_u8_txData.bit.b1 = B_CHT_IsFwdSwOn();																/* 前進SW取得 */
		a_u8_txData.bit.b2 = B_CHT_IsBackSwOn();															/* 後進SW取得 */
		a_u8_txData.bit.b3 = B_CHT_IsKeySwOn();																/* キーSW取得 */
		a_u8_txData.bit.b4 = 0;																				/* チャージャ接続取得 */
		a_u8_txData.bit.b5 = PI_BMS_TOGGLE();																/* BMSウォッチドッグ信号取得 */
		a_u16_data = U16_ADC_GetAccelVoltageAve_10mV();														/* アクセル電圧取得[10mV] */
		a_s16_data = S16_MTR_GetRevolutionAve_rpm();														/* モータ回転数取得[rpm] */
		a_u8_data = U8_DIAG_GetDiagCode();																	/* ダイアグコード */
		a_s8_data = (s8)(S16_LIM_MTR_GetTorqueRef_100mNm() / (s16)10);										/* トルク指令[Nm] */

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = (u8)a_u8_txData.byte;
		pst_CAN_txBuff -> u8_canData[1] = a_u8_data;														/* ダイアグコード */
		pst_CAN_txBuff -> u8_canData[2] = U8_CMF_GETLOWER_BYTE( a_u16_data );								/* アクセル電圧[10mV] */
		pst_CAN_txBuff -> u8_canData[3] = U8_CMF_GETUPPER_BYTE( a_u16_data );
		pst_CAN_txBuff -> u8_canData[4] = U8_CMF_GETLOWER_BYTE( a_s16_data );								/* モータ回転数[rpm] */
		pst_CAN_txBuff -> u8_canData[5] = U8_CMF_GETUPPER_BYTE( a_s16_data );
		pst_CAN_txBuff -> u8_canData[6] = (u8)a_en_Mode;													/* 動作モード */
		pst_CAN_txBuff -> u8_canData[7] = a_s8_data;														/* トルク指令[Nm] */

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData1 = FALSE;
		break;

	case (CANID_MCU_DATA2):																					/* MCUデータ2 */ /* 10ms */
		a_u16_data	= U16_BAT_GetBattVoltAve_10mV();														/* バッテリ電圧 [10mV] */
		a_u16_data1	= U16_BAT_GetInvVoltAve_10mV();															/* インバータ電圧 [10mV] */
		a_u16_data2	= U16_GL_vqTable;																		/* Vqリミットテーブル参照値 */
//		a_u16_data3	= U16_MTR_GetImRms_100mA();																/* モータ電流実効値 */
		a_s16_data	= S16_BAT_GetBattCurrentAve_100mA();													/* バッテリ電流 */

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = U8_CMF_GETLOWER_BYTE( a_u16_data1 );								/* 電池電圧(インバータ電圧) [10mV] */
		pst_CAN_txBuff -> u8_canData[1] = U8_CMF_GETUPPER_BYTE( a_u16_data1 );
		pst_CAN_txBuff -> u8_canData[2] = U8_CMF_GETLOWER_BYTE( a_s16_data1 );								/* バッテリ電流 [100mA] */
		pst_CAN_txBuff -> u8_canData[3] = U8_CMF_GETUPPER_BYTE( a_s16_data1 );
		pst_CAN_txBuff -> u8_canData[4] = U8_CMF_GETLOWER_BYTE( a_u16_data2 );								/* モータ電流実効値 [100mA] */
		pst_CAN_txBuff -> u8_canData[5] = U8_CMF_GETUPPER_BYTE( a_u16_data2 );
		pst_CAN_txBuff -> u8_canData[6] = U8_CMF_GETLOWER_BYTE( a_u16_data );								/* KEY電圧(バッテリ電圧) [10mV] */
		pst_CAN_txBuff -> u8_canData[7] = U8_CMF_GETUPPER_BYTE( a_u16_data );

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData2 = FALSE;
		break;

	case (CANID_MCU_DATA3):																					/* MCUデータ3 */ /* 10ms */
#ifdef	MOTOR_THERMISTOR_ACTIVITY
		a_s16_data = S16_ADC_GetMotorTemperature_0p1degC();
#else
		a_s16_data = (s16)0;
#endif
		a_s8_data = (s8)(S16_ADC_GetFetTemperature_0p1degC() / (s16)10);
		a_s8_data1 = (s8)(S16_ADC_GetCapTemperature_0p1degC() / (s16)10);
		a_s16_data1	= S16_DD_EX_GetVfMax_10mV();															/* 界磁電圧最大値[10mV] */
		a_u8_data = U8_MTR_GetVoltageUtilizationRate_percent();												/* 電圧利用率[%] */

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = U8_CMF_GETLOWER_BYTE( a_s16_data );								/* モータ温度[0.1℃] */
		pst_CAN_txBuff -> u8_canData[1] = U8_CMF_GETUPPER_BYTE( a_s16_data );
		pst_CAN_txBuff -> u8_canData[2] = (u8)a_s8_data;													/* FET温度[℃] */
		pst_CAN_txBuff -> u8_canData[3] = a_u8_data;														/* 電圧利用率[%] */
		pst_CAN_txBuff -> u8_canData[4] = (u8)a_s8_data1;													/* コンデンサ温度[℃] */
		pst_CAN_txBuff -> u8_canData[5] = (u8)0;
		pst_CAN_txBuff -> u8_canData[6] = U8_CMF_GETLOWER_BYTE( a_s16_data1 );								/* 界磁電圧最大値[10mV] */
		pst_CAN_txBuff -> u8_canData[7] = U8_CMF_GETUPPER_BYTE( a_s16_data1 );

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData3 = FALSE;
		break;

	case (CANID_MCU_DATA4):																					/* MCUデータ4 */ /* 10ms */
		a_s16_data	= S16_MTR_GetIdRef_100mA();																/* Id指令値[100mA] */
		a_s16_data1	= S16_MTR_GetIdFb_100mA();																/* D軸電流[100mA] */
		a_s16_data2	= S16_MTR_GetIqRef_100mA();																/* Iq指令値[100mA] */
		a_s16_data3	= S16_MTR_GetIqFb_100mA();																/* Q軸電流[100mA] */

		if( UNI_GL_testFlg.bit.b_iqrefByPotentiometer == TRUE )
		{
			a_u16_data	= U16_DD_MTR_GetEangleOffsetForTest_deg();

			a_s16_data	= (s16)(((s32)(-a_s16_data2) * (s32)S16_TRIG_Sin( a_u16_data << (u16)5 )) >>(s32)14);							/* IdRef = Im * -Sin(θ) */
			a_s16_data1 = (s16)(((s32)(-a_s16_data3) * (s32)S16_TRIG_Sin( a_u16_data << (u16)5 )) >>(s32)14);							/* Id = Ia * -Sin(θ) */
			a_s16_data2 = (s16)(((s32)a_s16_data2 * (s32)S16_TRIG_Sin( ((a_u16_data << (u16)5) + U16_COSIGN_OFFSET_90) )) >>(s32)14);	/* IqRef = Im * Cos(θ) */
			a_s16_data3 = (s16)(((s32)a_s16_data3 * (s32)S16_TRIG_Sin( ((a_u16_data << (u16)5) + U16_COSIGN_OFFSET_90) )) >>(s32)14);	/* Iq = Ia * Cos(θ) */
			S16_GL_idRef_100mA = a_s16_data;
			S16_GL_iqRef_100mA = a_s16_data2;
		}

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = U8_CMF_GETLOWER_BYTE( a_s16_data );								/* Id指令値[100mA] */
		pst_CAN_txBuff -> u8_canData[1] = U8_CMF_GETUPPER_BYTE( a_s16_data );
		pst_CAN_txBuff -> u8_canData[2] = U8_CMF_GETLOWER_BYTE( a_s16_data1 );								/* D軸電流[100mA] */
		pst_CAN_txBuff -> u8_canData[3] = U8_CMF_GETUPPER_BYTE( a_s16_data1 );
		pst_CAN_txBuff -> u8_canData[4] = U8_CMF_GETLOWER_BYTE( a_s16_data2 );								/* Iq指令値[100mA] */
		pst_CAN_txBuff -> u8_canData[5] = U8_CMF_GETUPPER_BYTE( a_s16_data2 );
		pst_CAN_txBuff -> u8_canData[6] = U8_CMF_GETLOWER_BYTE( a_s16_data3 );								/* Q軸電流[100mA] */
		pst_CAN_txBuff -> u8_canData[7] = U8_CMF_GETUPPER_BYTE( a_s16_data3 );

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData4 = FALSE;
		break;

	case (CANID_MCU_DATA5):																					/* MCUデータ5 */ /* 10ms */
		a_s16_data	= S16_DD_EX_GetIfRef_10mA();															/* If指令値[10mA] */
		a_s16_data1	= S16_DD_EX_GetIfFb_10mA();																/* IfFB電流[10mA] */
		a_s32_data	= S32_DD_EX_GetIntegralVf_10mV();														/* 界磁積分バッファ[10mV] */

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = U8_CMF_GETLOWER_BYTE( a_s16_data );								/* If指令値[10mA] */
		pst_CAN_txBuff -> u8_canData[1] = U8_CMF_GETUPPER_BYTE( a_s16_data );
		pst_CAN_txBuff -> u8_canData[2] = U8_CMF_GETLOWER_BYTE( a_s16_data1 );								/* IfFB電流[10mA] */
		pst_CAN_txBuff -> u8_canData[3] = U8_CMF_GETUPPER_BYTE( a_s16_data1 );
		pst_CAN_txBuff -> u8_canData[4] = U8_CMF_GETFOURTH_BYTE( a_s32_data );								/* 界磁積分バッファ[10mV] */
		pst_CAN_txBuff -> u8_canData[5] = U8_CMF_GETTHIRD_BYTE( a_s32_data );
		pst_CAN_txBuff -> u8_canData[6] = U8_CMF_GETSECOND_BYTE( a_s32_data );
		pst_CAN_txBuff -> u8_canData[7] = U8_CMF_GETFIRST_BYTE( a_s32_data );

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData5 = FALSE;
		break;

	case (CANID_MCU_DATA6):																					/* MCUデータ6 */ /* 10ms */
		a_s32_data	= S32_MTR_GetIntegralVd_10mV();															/* d軸積分バッファ[10mV] */
		a_s32_data1	= S32_MTR_GetIntegralVq_10mV();															/* q軸積分バッファ[10mV] */

		a_s16_data = S16_GL_integVd;
		a_s16_data1 = S16_GL_propoVd;
		a_s16_data2 = S16_GL_integVq;
		a_s16_data3 = S16_GL_propoVq;

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = U8_CMF_GETLOWER_BYTE( a_s16_data );								/* d軸積分バッファ[10mV] */
		pst_CAN_txBuff -> u8_canData[1] = U8_CMF_GETUPPER_BYTE( a_s16_data );
		pst_CAN_txBuff -> u8_canData[2] = U8_CMF_GETLOWER_BYTE( a_s16_data1 );								/* d軸比例項[10mV] */
		pst_CAN_txBuff -> u8_canData[3] = U8_CMF_GETUPPER_BYTE( a_s16_data1 );
		pst_CAN_txBuff -> u8_canData[4] = U8_CMF_GETLOWER_BYTE( a_s16_data2 );								/* q軸積分バッファ[10mV] */
		pst_CAN_txBuff -> u8_canData[5] = U8_CMF_GETUPPER_BYTE( a_s16_data2 );
		pst_CAN_txBuff -> u8_canData[6] = U8_CMF_GETLOWER_BYTE( a_s16_data3 );								/* q軸比例項[10mV] */
		pst_CAN_txBuff -> u8_canData[7] = U8_CMF_GETUPPER_BYTE( a_s16_data3 );

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData6 = FALSE;
		break;

	case (CANID_MCU_DATA7):																					/* MCUデータ7 */ /* 10ms */
		a_s16_data	= S16_MTR_GetVdMax_10mV();																/* D軸電圧Vd最大値[10mV] */
		a_s16_data1	= S16_MTR_GetVqMax_10mV();																/* Q軸電圧Vq最大値[10mV] */

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = U8_CMF_GETLOWER_BYTE( a_s16_data );								/* D軸電圧Vd最大値[10mV] */
		pst_CAN_txBuff -> u8_canData[1] = U8_CMF_GETUPPER_BYTE( a_s16_data );
		pst_CAN_txBuff -> u8_canData[2] = U8_CMF_GETLOWER_BYTE( a_s16_data1 );								/* Q軸電圧Vq最大値[10mV] */
		pst_CAN_txBuff -> u8_canData[3] = U8_CMF_GETUPPER_BYTE( a_s16_data1 );
		pst_CAN_txBuff -> u8_canData[4] = U8_CMF_GETLOWER_BYTE( a_s16_vdAve );								/* Vd */
		pst_CAN_txBuff -> u8_canData[5] = U8_CMF_GETUPPER_BYTE( a_s16_vdAve );
		pst_CAN_txBuff -> u8_canData[6] = U8_CMF_GETLOWER_BYTE( a_s16_vqAve );								/* Vq */
		pst_CAN_txBuff -> u8_canData[7] = U8_CMF_GETUPPER_BYTE( a_s16_vqAve );

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData7 = FALSE;
		break;

	case (CANID_MCU_DATA8):																					/* MCUデータ8 */ /* 10ms */
		a_s32_data	= S32_GL_idDiff;																		/* Id指令値とFb差分[10mA] */
		a_s32_data1	= S32_GL_iqDiff;																		/* Iq指令値とFb差分[10mA] */

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = U8_CMF_GETFOURTH_BYTE( a_s32_data );								/* Id指令値とFb差分[10mA] */
		pst_CAN_txBuff -> u8_canData[1] = U8_CMF_GETTHIRD_BYTE( a_s32_data );
		pst_CAN_txBuff -> u8_canData[2] = U8_CMF_GETSECOND_BYTE( a_s32_data );
		pst_CAN_txBuff -> u8_canData[3] = U8_CMF_GETFIRST_BYTE( a_s32_data );
		pst_CAN_txBuff -> u8_canData[4] = U8_CMF_GETFOURTH_BYTE( a_s32_data1 );								/* Iq指令値とFb差分[10mA] */
		pst_CAN_txBuff -> u8_canData[5] = U8_CMF_GETTHIRD_BYTE( a_s32_data1 );
		pst_CAN_txBuff -> u8_canData[6] = U8_CMF_GETSECOND_BYTE( a_s32_data1 );
		pst_CAN_txBuff -> u8_canData[7] = U8_CMF_GETFIRST_BYTE( a_s32_data1 );

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData8 = FALSE;
		break;

	case (CANID_MCU_DATA9):																					/* MCUデータ8 */ /* 10ms */
		a_s16_data	= S16_MTR_GetIqRefFinal_100mA();														/* Iq指令最終値[100mA] */

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = U8_CMF_GETLOWER_BYTE( a_s16_data );								/* Iq指令最終値[100mA] */
		pst_CAN_txBuff -> u8_canData[1] = U8_CMF_GETUPPER_BYTE( a_s16_data );
		pst_CAN_txBuff -> u8_canData[2] = (u8)0;
		pst_CAN_txBuff -> u8_canData[3] = (u8)0;
		pst_CAN_txBuff -> u8_canData[4] = (u8)0;
		pst_CAN_txBuff -> u8_canData[5] = (u8)0;
		pst_CAN_txBuff -> u8_canData[6] = (u8)0;
		pst_CAN_txBuff -> u8_canData[7] = (u8)0;

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMucData9 = FALSE;
		break;

	case (CANID_MCU_ERRCODE):																				/* MCU異常コード */ /* 10ms */
		a_u32_data	= U32_ABN_GetWarningFlg();																/* MCU警告取得 */
		a_u32_data1	= U32_ABN_GetErrorFlg();																/* MCU異常取得 */

		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x08;
		pst_CAN_txBuff -> u8_canData[0] = U8_CMF_GETFOURTH_BYTE( a_u32_data );								/* MCU警告0-7 */
		pst_CAN_txBuff -> u8_canData[1] = U8_CMF_GETTHIRD_BYTE( a_u32_data );								/* MCU警告8-15 */
		pst_CAN_txBuff -> u8_canData[2] = U8_CMF_GETSECOND_BYTE( a_u32_data );								/* MCU警告16-23 */
		pst_CAN_txBuff -> u8_canData[3] = U8_CMF_GETFIRST_BYTE( a_u32_data );								/* MCU警告24-31 */
		pst_CAN_txBuff -> u8_canData[4] = U8_CMF_GETFOURTH_BYTE( a_u32_data1 );								/* MCU異常0-7 */
		pst_CAN_txBuff -> u8_canData[5] = U8_CMF_GETTHIRD_BYTE( a_u32_data1 );								/* MCU異常8-15 */
		pst_CAN_txBuff -> u8_canData[6] = U8_CMF_GETSECOND_BYTE( a_u32_data1 );								/* MCU異常16-23 */
		pst_CAN_txBuff -> u8_canData[7] = U8_CMF_GETFIRST_BYTE( a_u32_data1 );								/* MCU異常24-31 */

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE),(U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		uni_CAN_txReqMcu.bit.b_canTxReqMcuErrCode = FALSE;
		break;

	case (CANID_CUW_TX):																					/* ロギング CuW */ /* 0x63D */
		pst_CAN_txBuff -> u32_canId = (u32)a_u16_canId;
		pst_CAN_txBuff -> u8_canDlc = (u8)0x07;

		V_CUW_GetTxData( pst_CAN_txBuff -> u8_canData );

		R_CAN_TxSet( (U32_CAN_CH0), (NORMAL_MAILBOX_MODE), (U32_CAN_MBOX0), pst_CAN_txBuff, (U32_CAN_DATA_FRAME) );	/* 送信データ設定 */
		V_CUW_ClearStoredTxData();																			/* ロギング送信データ格納完了フラグ クリア */
		uni_CAN_txReqMcu.bit.b_canTxReqCuw = FALSE;
		break;

	default:
		break;
	}
}

/**
 * @brief CANデータ受信
 * @param none
 * @return none
 * @note 受信したID番号に応じたデータ受信
 */
static void	v_CAN_GetReceiveData(void)
{																											/* 受信データより各制御変数に値を付ける */
	u16 a_u16_rxID = (u16)pst_CAN_rxBuff -> u32_canId;

	switch(a_u16_rxID)
	{
	case (CANID_CUW_RX):			/*----CuWからのmessage 0x63C -----------*/
		V_CUW_StoreRxData(pst_CAN_rxBuff -> u8_canData);
		V_CUW_SetFinishedReceive();																			/* 受信完了フラグ セット */
		break;

	case (CANID_VCU_COMMAND):
		for ( u8 i = 0; i < 8 ; i++ )																		/* COMMON_FUNCTION.cのV_CMF_Copy()関数を使用したほうが良いか? */
		{
			u8_CAN_rxVcuCommandData[i] = pst_CAN_rxBuff -> u8_canData[i];
		}
		v_CAN_AllocationReceiveData();																		/* 受信データ割り付け */
		b_CAN_isReceivedFromVcu = TRUE;																		/* VCUからのCANデータフラグON */
		b_CAN_notYetReceived = FALSE;																		/* CAN未受信 From VCU クリア */
		break;

	case (CANID_CANOE_RX1):
		for ( u8 i = 0; i < 8 ; i++ )																		/* COMMON_FUNCTION.cのV_CMF_Copy()関数を使用したほうが良いか? */
		{
			u8_CAN_rxCanoeData1[i] = pst_CAN_rxBuff -> u8_canData[i];
		}
		v_CAN_AllocationReceiveData1FromCanoe();															/* 受信データ割り付け */
		break;

	case (CANID_CANOE_RX2):
		for ( u8 i = 0; i < 8 ; i++ )																		/* COMMON_FUNCTION.cのV_CMF_Copy()関数を使用したほうが良いか? */
		{
			u8_CAN_rxCanoeData2[i] = pst_CAN_rxBuff -> u8_canData[i];
		}
		v_CAN_AllocationReceiveData2FromCanoe();															/* 受信データ割り付け */
		break;
#if 0
	case (CANID_CANOE_RX3):
		for ( u8 i = 0; i < 8 ; i++ )																		/* COMMON_FUNCTION.cのV_CMF_Copy()関数を使用したほうが良いか? */
		{
			u8_CAN_rxCanoeData3[i] = pst_CAN_rxBuff -> u8_canData[i];
		}
//		v_CAN_AllocationReceiveData3FromCanoe();															/* 受信データ割り付け */
		break;


	case (CANID_CANOE_RX4):
		for ( u8 i = 0; i < 8 ; i++ )																		/* COMMON_FUNCTION.cのV_CMF_Copy()関数を使用したほうが良いか? */
		{
			u8_CAN_rxCanoeData4[i] = pst_CAN_rxBuff -> u8_canData[i];
		}
//		v_CAN_AllocationReceiveData4FromCanoe();															/* 受信データ割り付け */
		break;


	case (CANID_CANOE_RX5):
		for ( u8 i = 0; i < 8 ; i++ )																		/* COMMON_FUNCTION.cのV_CMF_Copy()関数を使用したほうが良いか? */
		{
			u8_CAN_rxCanoeData5[i] = pst_CAN_rxBuff -> u8_canData[i];
		}
//		v_CAN_AllocationReceiveData5FromCanoe();															/* 受信データ割り付け */
		break;
#endif
	default:																								/* ID異常情報用 */
		break;
	}
}

/**
 * @brief CAN受信データの割付
 * @param none
 * @return none
 * @note 受信したデータの割付
 */
static void	v_CAN_AllocationReceiveData(void)
{																											/* 受信データより各制御変数に値を付ける */
	UNI_VCU_COMMAND_RX_0	a_uni_vcuCmdRx0;
	a_uni_vcuCmdRx0.byte = (u8)0;
	UNI_VCU_COMMAND_RX_4	a_uni_vcuCmdRx4;
	a_uni_vcuCmdRx4.byte = (u8)0;

	a_uni_vcuCmdRx0.byte = u8_CAN_rxVcuCommandData[0];

	uni_CAN_vcuCommand.bit.b_motorStopCommand = a_uni_vcuCmdRx0.bit.b_motorStopCommand;						/* モータ停止要求/駆動許可 */
	uni_CAN_vcuCommand.bit.b_relayOpenCommand = a_uni_vcuCmdRx0.bit.b_relayOpenCommand;						/* リレー解放要求/接続許可 */

	u8_CAN_vcuErrorInfo = u8_CAN_rxVcuCommandData[1];														/* エラー情報 */

	s16_CAN_torqueOrder_100mNm = (s16)U16_CMF_MERGE_BYTE( u8_CAN_rxVcuCommandData[3], (u16)u8_CAN_rxVcuCommandData[2] );	/* トルク指令値[0.1Nm] */

	a_uni_vcuCmdRx4.byte = u8_CAN_rxVcuCommandData[4];
	uni_CAN_vcuCommand.bit.b_dischargeCommand = a_uni_vcuCmdRx4.bit.b_dischargeCommand;						/* ディスチャージ指示 */
	b_CAN_keySwStatusVCU = a_uni_vcuCmdRx4.bit.b_keySwStatusFromVcu;										/* VCUキーSW入力 */
	u8_CAN_buzzerPattern = (u8)a_uni_vcuCmdRx4.bit.u8_buzzerPattern & 0x0F;									/* ブザーパターン */

	s8_CAN_eangleOffsetFromCan = (s8)CMF_GETLIMIT( (s8)u8_CAN_rxVcuCommandData[5], (s8)(-90), (s8)90 );		/* エンコーダオフセット */
	u8_CAN_revTimeConstFactorFromCan = (u8)CMF_GETMAX( u8_CAN_rxVcuCommandData[6], (U8_SYS_TIME_CONST_FACTOR) );	/* モータ回転数フィルタ時定数 [×1msec] */
}

/**
 * @brief CAN受信データの割付
 * @param none
 * @return none
 * @note 受信したデータの割付
 */
static void	v_CAN_AllocationReceiveData1FromCanoe(void)
{																											/* 受信データより各制御変数に値を付ける */
	UNI_GL_testFlg.word = U16_CMF_MERGE_BYTE( u8_CAN_rxCanoeData1[1] , u8_CAN_rxCanoeData1[0] );			/* test_flg セット */
	U16_GL_pGainTemp = U16_CMF_MERGE_BYTE( u8_CAN_rxCanoeData1[3] , u8_CAN_rxCanoeData1[2] );				/* 界磁比例ゲイン セット */
	U16_GL_iGainTemp = U16_CMF_MERGE_BYTE( u8_CAN_rxCanoeData1[5] , u8_CAN_rxCanoeData1[4] );				/* 界磁積分ゲイン セット */
}


/**
 * @brief CAN受信データの割付
 * @param none
 * @return none
 * @note 受信したデータの割付
 */
static void	v_CAN_AllocationReceiveData2FromCanoe(void)
{																											/* 受信データより各制御変数に値を付ける */
	U16_GL_dPGainForTest = U16_CMF_MERGE_BYTE( u8_CAN_rxCanoeData2[1] , u8_CAN_rxCanoeData2[0] );			/* d軸比例ゲイン セット */
	U16_GL_dIGainForTest = U16_CMF_MERGE_BYTE( u8_CAN_rxCanoeData2[3] , u8_CAN_rxCanoeData2[2] );			/* d軸積分ゲイン セット */
	U16_GL_qPGainForTest = U16_CMF_MERGE_BYTE( u8_CAN_rxCanoeData2[5] , u8_CAN_rxCanoeData2[4] );			/* q軸比例ゲイン セット */
	U16_GL_qIGainForTest = U16_CMF_MERGE_BYTE( u8_CAN_rxCanoeData2[7] , u8_CAN_rxCanoeData2[6] );			/* q軸積分ゲイン セット */
}

/**
 * @brief 通信無効判定時のSafetyデータ割り付け
 * @param none
 * @return none
 * @note Safetyデータの割付
 */
static void	v_CAN_SafetyAllocationAtWarning(void)
{
	uni_CAN_vcuCommand.bit.b_motorStopCommand = TRUE;														/* モータ停止要求 */
	uni_CAN_vcuCommand.bit.b_relayOpenCommand = TRUE;														/* リレー解放要求 */
	uni_CAN_vcuCommand.bit.b_dischargeCommand = TRUE;														/* ディスチャージ要求 */
	s16_CAN_torqueOrder_100mNm = (s16)0;																	/* トルク指令値[0.1Nm] */
}

/**
 * @brief CAN0_RXM0_ISR
 * @param none
 * @return none
 * @note CAN0 Receive interrupt.
 * @detail Check which mailbox received data and process it.
 */
#if FAST_INTERRUPT_VECTOR == VECT_CAN0_RXM0
#pragma interrupt CAN0_RXM0_ISR(vect=VECT_CAN0_RXM0,fint)
#else
#pragma interrupt CAN0_RXM0_ISR(vect=VECT_CAN0_RXM0)
#endif

void CAN0_RXM0_ISR(void)																					/* CAN受信割り込み */
{
	volatile struct st_can __evenaccess * p_canBlock;

	p_canBlock = (struct st_can __evenaccess *) 0x90200;
	u32	a_u32_mboxNum = (u32)p_canBlock->MSSR.BIT.MBNST;		/*受信のメールボックス番号サーチ結果*/

	b_CAN_isNewDataReceived = TRUE;

	R_CAN_RxRead( (u32)0, a_u32_mboxNum, pst_CAN_rxBuff );
	v_CAN_GetReceiveData();																					/* データ受信 */

	for( u8 idx = 0; idx < 8; idx++ )
	{
		pst_CAN_rxBuff -> u8_canData[idx] = 0;							/*受信バファークリア*/
	}
}

/**
 * @brief CAN関連異常判定
 * @note V_ABN_Loop10ms()内でコール(10msec周期)
 * @param none
 * @return none
 */
void	V_CAN_DetectCan(void)
{
/*----- CAN通信異常 -----*/
	v_CAN_DetectRxTimeout();																				/* CAN受信タイムアウト異常 */
}

/**
 * @brief CAN受信タイムアウト
 * @note 10msec周期
 * @param none
 * @return none
 */
static void	v_CAN_DetectRxTimeout(void)
{
	bool	a_b_isDetectActivity = B_MDC_IsDetectActivity();
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
#ifdef	CAN_OPEN_NON_RETURN
	if( b_CAN_rceptionWarningFromVCU == TRUE )																/* 一度でも受信警告になったら非復帰とする */
	{
		return;
	}
#endif
	if( (a_b_isDetectActivity		== FALSE)	||															/* 異常検出禁止 */
		(b_CAN_detectCanActivate	== FALSE)	||
		(b_CAN_notYetReceived		== TRUE) )																/* VCUから未受信 */
	{
		u8_CAN_warningCanVcuCounter = (u8)0;																/* カウンタゼロに設定 */
		V_ABN_ClearAbnormalCanCommunication();																/* CAN通信異常解除 */
		return;
	}

/* VCUデータの受信カウンタ判断部 */
	if ( b_CAN_isReceivedFromVcu == TRUE )
	{																										/* CAN受信割り込みある */
		u8_CAN_warningCanVcuCounter = (u8)0;																/* 警告カウンタを0に設定 */
		b_CAN_isReceivedFromVcu = FALSE;																	/* CAN受信データフラグOFF */
	}
	else
	{																										/* 受信データが更新されない場合 */
		u8_CAN_warningCanVcuCounter++;																		/* ID警告カウンタをインクリメント */
		u8_CAN_warningCanVcuCounter = (u8)CMF_GETMIN( u8_CAN_warningCanVcuCounter,  (U8_CAN_WARNING_COUNT_10MS) );
	}

/* CAN受信警告フラグ判断 異常値以内の場合のみ，警告を解除 */	
	if( u8_CAN_warningCanVcuCounter < (U8_CAN_WARNING_COUNT_10MS) )
	{
		b_CAN_rceptionWarningFromVCU = FALSE;																/* VCU CAN受信警告フラグクリア */
	}
	else
	{																										/* どちらか一方でもタイムアウトの場合 */
		b_CAN_rceptionWarningFromVCU = TRUE;																/* VCU CAN受信警告フラグセット */
	}
}

/**
 * @brief CAN変数初期化(再起動用)
 * @detail CANモードと通信ポート初期化
 * @param none
 * @return none
 */
void V_CAN_InitializeForRestart(void)
{
	u16_CAN_accelAdValue					= (u16)0;					/* アクセルAD値(max：4096) */
	b_CAN_isErrorPeripheral					= FALSE;					/* CAN周辺エラー */
	u8_CAN_abnormalCanCounter				= (u8)0;					/* CAN異常検出タイマ  */

	b_CAN_isNewDataReceived					= FALSE;
}

/**
 * @brief キーSW状態提供
 * @param none
 * @return b_CAN_keySwStatusVCU
 */
bool	B_CAN_GetKeySwStatus(void)
{
	return	b_CAN_keySwStatusVCU;
}

/**
 * @brief トルク指令値提供
 * @param none
 * @return s16_CAN_torqueOrder_100mNm
 */
s16	S16_CAN_GetTorqueOrder_100mNm(void)
{
	return	s16_CAN_torqueOrder_100mNm;
}

/**
 * @brief ブザーパターン提供
 * @param none
 * @return u8_CAN_buzzerPattern
 */
u8	U8_CAN_GetBuzzerPattern(void)
{
	return	u8_CAN_buzzerPattern;
}

/**
 * @brief VCU異常状態提供
 * @param none
 * @return a_b_isAbn
 */
bool	B_CAN_IsVcuAbnormal(void)
{
	bool	a_b_isAbn = FALSE;
	if( u8_CAN_vcuErrorInfo != (u8)0 )
	{
		a_b_isAbn = TRUE;
	}
	return	a_b_isAbn;
}

/**
 * @brief エンコーダオフセット提供
 * @param none
 * @return s8_CAN_eangleOffsetFromCan
 */
s8	S8_CAN_GetEangleOffsetFromCan(void)
{
	return	s8_CAN_eangleOffsetFromCan;
}

/**
 * @brief モータ回転数フィルタ時定数提供
 * @param none
 * @return u8_CAN_revTimeConstFactorFromCan
 */
u8	U8_CAN_GetRevTimeConstFactorFromCan(void)
{
	return	u8_CAN_revTimeConstFactorFromCan;
}

/**
 * @brief CAN通信異常異常を無効化する
 * @param none
 * @return none
 */
void	V_CAN_DetectCanInactivate(void)
{
	b_CAN_detectCanActivate = FALSE;
}

/**
 * @brief CAN通信異常異常を有効化する
 * @param none
 * @return none
 */
void	V_CAN_DetectCanActivate(void)
{
	b_CAN_detectCanActivate = TRUE;
}

/**
 * @brief VCUコマンド提供
 * @param none
 * @return uni_CAN_vcuCommand
 */
u8	U8_CAN_GetVcuCommand(void)
{
	return	(u8)uni_CAN_vcuCommand.byte;
}
