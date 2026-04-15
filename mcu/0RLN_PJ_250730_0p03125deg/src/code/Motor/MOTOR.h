/*==========================================================================*/
/**
 * @file	MOTOR.h
 * @brief	d,q軸電流指令値演算ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"
#include "TYPEDEF_ADC.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void	V_MTR_Loop10ms(void);										/* MOTORクラス周期処理:10msec周期 */
#if 0
extern void V_MTR_Initialize(void);										/* モータ電流指令値演算初期化 */
#endif
extern void V_MTR_InitializeForRestart(void);							/* モータ電流指令値演算初期化(再起動用) */
extern void V_MTR_UpdateRevolutionAve_rpm(void);						/* モータ回転数平均化処理(1msec毎) */
extern ST_ADC_ADVALUE_AVE	St_MTR_GetAdValueAve(void);					/* 電流AD値平均化データ提供 */
extern u16	U16_MTR_GetAdValueAveIfOffset(void);						/* 界磁電流オフセット提供 */
extern void	V_MTR_Permit(void);											/* モータ駆動を許可する */
extern void	V_MTR_Forbid(void);											/* モータ駆動を禁止する */
extern bool	B_MTR_IsGateOnPermit(void);									/* ゲートON許可/禁止状態提供 */
extern void	V_MTR_StartControl(void);									/* モータ駆動を要求する */
extern void	V_MTR_StopControl(void);									/* ゲートOFFを要求する */
extern void	V_MTR_PermitByVehicleState(void);							/* 車両状態による通電許可を要求する */
extern void	V_MTR_ForbidByVehicleState(void);							/* 車両状態による通電禁止を要求する */
extern bool	B_MTR_IsStartRequested(void);								/* モータ制御開始要求かどうかを返す */
extern bool	B_MTR_PermittedByVehicleState(void);						/* 車両モードによるモータ駆動禁止状態提供 */
extern s16	S16_MTR_GetRevolution_rpm(void);							/* モータ回転数提供 */
extern s16	S16_MTR_GetRevolutionAve_rpm(void);							/* モータ回転数平均値提供 */
extern u16	U16_MTR_GetDAxisPropoGain(void);							/* 界磁電流によるd軸比例ゲイン提供 */
extern u16	U16_MTR_GetQAxisPropoGain(void);							/* 界磁電流によるq軸比例ゲイン提供 */

extern s16	S16_MTR_GetIdRef_100mA(void);								/* Id指令値提供[0.1A] */
extern s16	S16_MTR_GetIqRef_100mA(void);								/* Iq指令値提供[0.1A] */
extern s16	S16_MTR_GetIqRefFinal_100mA(void);							/* 最終Iq指令値提供[0.1A] */
extern s16	S16_MTR_GetIdFb_100mA(void);								/* D軸電流値提供[0.1A] */
extern s16	S16_MTR_GetIqFb_100mA(void);								/* Q軸電流値提供[0.1A] */
extern s16	S16_MTR_GetIfFbAve_10mA(void);								/* エキサイタ電流平均値提供[10mA] */
extern u16	U16_MTR_GetImRms_100mA(void);								/* モータ電流実効値提供[100mA] */
extern bool	B_MTR_IsGateOn(void);										/* ゲートON/FF状態提供 */
extern bool	B_MTR_IsMotorStopped(void);									/* モータ停止状態提供 */
extern u16 U16_MTR_GetAdValueBattVolt(void);							/* バッテリ電圧ＡＤ提供(生) */
extern u8	U8_MTR_GetQuadrant(void);									/* 象限提供 */
extern bool	B_MTR_IsPermitted(void);									/* モータ駆動許可状態提供 */

extern u8	U8_MTR_GetVoltageUtilizationRate_percent(void);				/* 電圧利用率提供[%] */

extern s32	S32_MTR_GetIntegralVd_10mV(void);							/* d軸積分バッファ提供[10mV] */
extern s32	S32_MTR_GetIntegralVq_10mV(void);							/* q軸積分バッファ提供[10mV] */
extern s16	S16_MTR_GetVdMax_10mV(void);								/* D軸電圧Vd最大値提供[10mV] */
extern s16	S16_MTR_GetVqMax_10mV(void);								/* Q軸電圧Vq最大値提供[10mV] */

#endif /* __MOTOR_H */
