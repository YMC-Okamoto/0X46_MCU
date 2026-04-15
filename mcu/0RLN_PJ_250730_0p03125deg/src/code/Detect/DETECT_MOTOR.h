/*==========================================================================*/
/**
 * @file	DETECT_MOTOR.h
 * @brief	モータ系異常検出処理ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2019.08
 */
/*==========================================================================*/
#ifndef __DETECT_MOTOR_H
#define __DETECT_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void V_DET_MTR_Loop10ms(void);							/* モータ系異常検出 */
extern void V_DET_MTR_Initialize(void);							/* モータ系異常検出変数初期化 */

extern bool	B_DET_MTR_IsAbnormanEncoder(void);					/* エンコーダ異常状態提供 */
extern void	V_DET_MTR_SetAbnormalEncoder(void);					/* エンコーダ異常セット */
extern bool	B_DET_MTR_IsAbnormalOverCurrent(void);				/* 過電流異常状態取得 */
extern void	V_DET_MTR_SetAbnormalOverCurrent(void);				/* 過電流異常セット */
extern void	V_DET_MTR_SetAbnormalExOverCurrent(void);			/* エキサイタ過電流異常セット */
extern void	V_DET_MTR_SetAbnormalUPhaseWireOpen(void);			/* U相モータ断線異常セット */
extern void	V_DET_MTR_SetAbnormalVPhaseWireOpen(void);			/* V相モータ断線異常セット */
extern void	V_DET_MTR_SetAbnormalWPhaseWireOpen(void);			/* W相モータ断線異常セット */
extern void	V_DET_MTR_ClearAbnormalFlg(void);					/* (DET_MTRクラス)異常フラグクリア */
extern void	V_DET_MTR_ClearAbnormalFlg2(void);					/* (DET_MTRクラス)異常フラグ2クリア */
extern void	V_DET_MTR_ClearWarningFlg(void);					/* (DET_MTRクラス)警告フラグクリア */
extern u8	U8_DET_MTR_IsAbnormalFlg(void);						/* (DET_MTRクラス)異常フラグ提供 */
extern u8	U8_DET_MTR_IsAbnormalFlg2(void);					/* (DET_MTRクラス)異常フラグ2提供 */
extern u32	U32_DET_MTR_GetErrorFlg(void);						/* (DET_MTRクラス)異常フラグ提供 */
extern u32	U32_DET_MTR_GetWarningFlg(void);					/* (DET_MTRクラス)警告フラグ提供 */
extern bool	B_DET_MTR_ClearOverCurrent(void);					/* 過電流異常クリア要求 */

#endif /* __DETECT_MOTOR_H */
