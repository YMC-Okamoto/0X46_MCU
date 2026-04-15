/*==========================================================================*/
/**
 * @file	DD_EXCITER.h
 * @brief	界磁制御処理ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __DD_EXCITER_H
#define __DD_EXCITER_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"
#include "TYPEDEF_ADC.h"
#include "DEFINE_SETTING.h"						/* IFDEF設定 */

/* Private define -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
/* Interrupt */
extern void INTRPT_DD_EX_FieldControlValley(void);						/* 界磁制御:電気角推定、AD取り込み、モータ電流単位変換、過電流判定、モータ単相断線判定 */
extern void INTRPT_DD_EX_FieldControlPeak(void);						/* 界磁制御:ゲートON/OFF判断、ベクトル演算、モータ回転数演算、バッテリ電圧単位変換 */

extern void	V_DD_EX_InitializeForRestart(void);							/* 電流制御 & AD取り込み処理用変数の初期化処理(再起動用) */
extern bool	B_DD_EX_IsGateOn(void);										/* ゲートON/FF状態提供 */
extern void	V_DD_EX_Permit(void);										/* 界磁駆動を許可する */
extern void	V_DD_EX_Forbid(void);										/* 界磁駆動を禁止する */
extern void	V_DD_EX_StartControl(void);									/* 界磁制御開始を要求する */
extern void	V_DD_EX_StopControl(void);									/* 界磁制御停止を要求する */
extern void	V_DD_EX_ForcedGateOff(void);								/* エキサイタ強制ゲートOFFを要求する */
extern void	V_DD_EX_CanselForcedGateOff(void);							/* エキサイタ強制ゲートOFFを解除する */
//extern bool	B_DD_EX_IsStartRequested(void);							/* 界磁制御開始要求かどうかを返す */
extern void	V_DD_EX_ExecuteGateOff(void);								/* モータGateOffを実施する */
extern bool	B_DD_EX_IsAbnormalOverCurrent(void);						/* エキサイタ過電流異常状態提供 */

extern ST_ADC_ADVALUE1	St_DD_EX_GetAdValue1(void);						/* AD1データ提供 */
extern s16	S16_DD_EX_GetIfRef_10mA(void);								/* 最終If指令値提供[10mA] */
extern s16	S16_DD_EX_GetIfFb_10mA(void);								/* Ifフィードバック値提供[10mA] */
extern s16	S16_DD_EX_GetVf_10mV(void);									/* 界磁電圧提供[10mV] */
extern bool	B_DD_EX_IsEmergencyStop(void);								/* 緊急停止要求提供 */

extern void	V_DD_EX_SetIfRef_10mA(s16 a_s16_ifRef);						/* If指令値セット[10mA] */

extern s32	S32_DD_EX_GetIntegralVf_10mV(void);							/* 界磁積分バッファ提供[10mV] */
extern s16	S16_DD_EX_GetVfMax_10mV(void);								/* 界磁電圧最大値提供[10mV] */

#endif /* __DD_EXCITER_H */
