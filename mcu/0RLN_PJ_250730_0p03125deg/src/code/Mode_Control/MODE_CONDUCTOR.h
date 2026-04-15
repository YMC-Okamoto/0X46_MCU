/*==========================================================================*/
/**
 * @file	MODE_CONDUCTOR.h
 * @brief	モード移行演算処理ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __MODE_CONDUCTOR_H
#define __MODE_CONDUCTOR_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"
#include "TYPEDEF_MODE.h"				/* 型定義 */

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void	V_MDC_Loop10ms(void);										/* モード移行処理 */
extern void	V_MDC_Initialize(void);										/* モード移行処理初期化 */
extern EN_MDC_MODE	En_MDC_GetMode(void);								/* 動作モード提供 */
extern void	U16_MDC_SetShutdownCounter(u16 a_u16_count);				/* 電源遮断カウンタセット */
extern void	V_MDC_DetectPrecharge(void);								/* プリチャージ異常判定 */
extern void	V_MDC_DetectDischarge(void);								/* ディスチャージ異常判定 */
extern bool	B_MDC_IsDetectActivity(void);								/* 異常検出許可状態提供 */
extern void	V_MDC_SetDetectActivity(void);								/* 異常検出許可 */
extern void	V_MDC_DetectInactivity(void);								/* 異常検出禁止 */
extern void	V_MDC_RelayOffRequest(void);								/* メインリレーOFF要求 */
extern void	V_MDC_CancelRelayOffRequest(void);							/* メインリレーOFF要求解除 */
extern void	V_MDC_PermitRelayOff(void);									/* メインリレーOFF許可 */
extern void	V_MDC_FrobidRelayOff(void);									/* メインリレーOFF禁止 */
extern bool	B_MDC_IsEmergencyStop(void);								/* 緊急停止要求提供 */

extern EN_MDC_PRECHARGE_MODE	En_MDC_GetPrechargeMode(void);			/* プリチャージモード提供 */
extern EN_MDC_DISCHARGE_MODE	En_MDC_GetDischargeMode(void);			/* ディスチャージモード提供 */

#endif /* __MODE_CONDUCTOR_H */
