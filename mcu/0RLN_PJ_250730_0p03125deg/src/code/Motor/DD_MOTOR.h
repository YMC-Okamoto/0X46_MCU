/*==========================================================================*/
/**
 * @file	DD_MOTOR.h
 * @brief	モータ制御処理ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __DD_MOTOR_H
#define __DD_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"
#include "TYPEDEF_ADC.h"
#include "DEFINE_SETTING.h"						/* IFDEF設定 */

/* Private define -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

#pragma bit_order right
#pragma unpack

struct ST_ENC_PATTERN_TABLE
{
	const u16 u16_encPatternNormalRotate;		/*正転時、前回のエンコーダパターン*/
	const u16 u16_encPatternReverseRotate;		/*逆転時、前回のエンコーダパターン*/
	const u16 u16_eangleBase;					/*U相ベース基準電気角*/
};

/* U相立下がり(V、WがU相に対して120deg、240deg遅れ位相) */
/* 注：PORTB_3:U相、PORTB_2:V相 PORTB_1:W相    */
/*                                             */
/* PORTB_3  U |_____|￣￣￣|______|￣￣￣|__   */
/* PORTB_2  V ￣￣|______|￣￣￣|______|￣￣   */
/* PORTB_1  W _|￣￣￣|______|￣￣￣|_______   */
/*           0     180    360                  */
/*     U相のベースで電気角度ベースを決める	   */
/*     U相立下がりを0degとする                 */
static struct ST_ENC_PATTERN_TABLE const u16_DD_MTR_encPatternTable[8] =
{						/* 正転で前回の検出位相1／逆転で前回の検出位相2／U相ベース電気角(単位：度) */
	{ (u16)7, (u16)7, (u16)0	},
	{ (u16)3, (u16)5, (u16)120	},
	{ (u16)6, (u16)3, (u16)0	},
	{ (u16)2, (u16)1, (u16)60	},
	{ (u16)5, (u16)6, (u16)240	},
	{ (u16)1, (u16)4, (u16)180	},
	{ (u16)4, (u16)2, (u16)300	},
	{ (u16)0, (u16)0, (u16)0	},
};

#if 0
/* U相立上がり(V、WがU相に対して120deg、240deg遅れ位相) */
/* 注：PORTB_3:U相、PORTB_2:V相 PORTB_1:W相   */
/*                                            */
/* PORTB_3 U _|￣￣￣|______|￣￣￣|_______   */
/* PORTB_2 V ______|￣￣￣|______|￣￣￣|__   */
/* PORTB_1 W ￣￣|______|￣￣￣|______|￣￣   */
/*            0      180    360               */
/*     U相のベースで電気角度ベースを決める	  */
/*     U相立上がりを0degとする                */
struct ST_ENC_PATTERN_TABLE const u16_DD_MTR_encPatternTable[8] =
{						/* 正転で前回の検出位相1／逆転で前回の検出位相2／U相ベース電気角(単位：度) */
	{ (u16)7, (u16)7, (u16)0	},
	{ (u16)5, (u16)3, (u16)60	},
	{ (u16)3, (u16)6, (u16)180	},
	{ (u16)1, (u16)2, (u16)120	},
	{ (u16)6, (u16)5, (u16)300	},
	{ (u16)4, (u16)1, (u16)0	},
	{ (u16)2, (u16)4, (u16)240	},
	{ (u16)0, (u16)0, (u16)0	},
};
#endif

/* Grobal function prototypes ------------------------------------------------*/
/* Interrupt */
extern void INTRPT_DD_MTR_MotorControlValley(void);						/* モータ制御:電気角推定、AD取り込み、モータ電流単位変換、過電流判定、モータ単相断線判定 */
extern void INTRPT_DD_MTR_MotorControlPeak(void);						/* モータ制御:ゲートON/OFF判断、ベクトル演算、モータ回転数演算、バッテリ電圧単位変換 */

extern void	V_DD_MTR_Initialize(void);									/* 電流制御 & AD取り込み処理用変数の初期化処理 */
extern void	V_DD_MTR_InitializeForRestart(void);						/* 電流制御 & AD取り込み処理用変数の初期化処理(再起動用) */
extern s16	S16_DD_MTR_GetRevolution_rpm(void);							/* モータ回転数提供 */
extern s16	S16_DD_MTR_GetRevolutionAve6_rpm(void);						/* 6移動平均モータ回転数提供 */
extern u8	U8_DD_MTR_GetEncPattern(void);								/* 現在のエンコーダパターン提供 */
extern bool	B_DD_MTR_IsGateOn(void);									/* ゲートON/FF状態提供 */
extern void	V_DD_MTR_Permit(void);										/* モータ駆動を許可する */
extern void	V_DD_MTR_Forbid(void);										/* モータ駆動を禁止する */
extern void	V_DD_MTR_StartControl(void);								/* モータ制御開始を要求する */
extern void	V_DD_MTR_StopControl(void);									/* モータ制御停止を要求する */
extern void	V_DD_MTR_ExecuteGateOff(void);								/* モータGateOffを実施する */
extern bool	B_DD_MTR_IsRotorVibrated(void);								/* ロータ振動状態提供 */
extern void	V_DD_MTR_ClearEncPatternChecker(void);						/* エンコーダパターンチェッカー初期化 */
extern u8	U8_DD_MTR_GetEncPatternChecker(u8 idx);						/* エンコーダパターンチェッカー提供 */
//extern s16	S16_DD_MTR_GetCtPpMIN_100mA(void);							/* 相電流P-P最小値提供 */
//extern s32	S32_DD_MTR_GetCtPpMIN_10mA(void);							/* 相電流P-P最小値提供 */

extern void	V_DD_MTR_SetEangleOffset_deg(u16 a_u16_eangleOfs_deg);		/* 電気角オフセット値設定 */
extern ST_ADC_ADVALUE0	St_DD_MTR_GetAdValue0(void);					/* AD0データ提供 */
extern ST_ADC_DQ32		St_DD_MTR_GetDq32(void);						/* DQ軸データ提供 */
extern u16	U16_DD_MTR_GetBattVolt_10mV(void);							/* バッテリ電圧提供(フィルタなし) */
extern u16	U16_DD_MTR_GetAdValueBattVolt(void);						/* バッテリ電圧ＡＤ提供(生) */
extern u16	U16_DD_MTR_GetAdValueInvVolt(void);							/* インバータ電圧ＡＤ提供(生) */
extern s32	S32_DD_MTR_GetIdFb_10mA(void);								/* D軸電流提供[10mA] */
extern s32	S32_DD_MTR_GetIqFb_10mA(void);								/* Q軸電流提供[10mA] */
extern s32	S32_DD_MTR_GetIdRef_10mA(void);								/* Id指令値提供[10mA] */
extern s32	S32_DD_MTR_GetIqRef_10mA(void);								/* Iq指令値提供[10mA] */
extern s32	S32_DD_MTR_GetIqRefFinal_10mA(void);						/* 最終Iq指令値提供[10mA] */

extern bool	B_DD_MTR_IsMotorReverseRotate(void);						/* モータ逆転状態提供 */

extern void	V_DD_MTR_SetIdqRef_10mA(s32 a_s32_idRef, s32 a_s32_iqRef);	/* Id、Iq指令値セット[10mA] */

extern bool	B_DD_MTR_IsAbnormanEncoder(void);							/* エンコーダ異常状態提供 */
extern bool	B_DD_MTR_IsAbnormalOverCurrent(void);						/* 過電流異常状態提供 */
extern bool	B_DD_MTR_IsAbnormalUPhaseWireOpen(void);					/* U相モータ断線異常状態提供 */
extern bool	B_DD_MTR_IsAbnormalVPhaseWireOpen(void);					/* V相モータ断線異常状態提供 */
extern bool	B_DD_MTR_IsAbnormalWPhaseWireOpen(void);					/* W相モータ断線異常状態提供 */
extern void	V_DD_MTR_DetectWireOpenInactivate(void);					/* モータ単相断線異常を無効化する */
extern void	V_DD_MTR_DetectWireOpenActivate(void);						/* モータ単相断線異常を有効化する */
extern void	V_DD_MTR_DetectEncoderInactivate(void);						/* エンコーダ異常を無効化する */
extern void	V_DD_MTR_DetectEncoderActivate(void);						/* エンコーダ異常を有効化する */
extern bool	B_DD_MTR_IsUPhaseConstEangle(void);							/* U相電気角固定状態提供 */
extern bool	B_DD_MTR_IsVPhaseConstEangle(void);							/* V相電気角固定状態提供 */

extern s32	S32_DD_MTR_GetIntegralVd_10mV(void);						/* d軸積分バッファ提供[10mV] */
extern s32	S32_DD_MTR_GetIntegralVq_10mV(void);						/* q軸積分バッファ提供[10mV] */
extern s16	S16_DD_MTR_GetVdMax_10mV(void);								/* D軸電圧Vd最大値提供[10mV] */
extern s16	S16_DD_MTR_GetVqMax_10mV(void);								/* Q軸電圧Vq最大値提供[10mV] */
extern u16	U16_DD_MTR_GetEangleOffsetForTest_deg(void);				/* テスト用電気角オフセット提供[deg] */

#endif /* __DD_MOTOR_H */
