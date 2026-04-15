/*==========================================================================*/
/**
 * @file	DETECT_BATTERY.h
 * @brief	異常検出処理ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __DETECT_BATTERY_H
#define __DETECT_BATTERY_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void V_DET_BAT_Loop10ms(void);							/*異常検出*/
extern void V_DET_BAT_Initialize(void);							/* バッテリ異常関連の変数初期化処理 */

extern bool	B_DET_BAT_IsWarningVoltageDropped(void);			/* 低電圧警告状態提供 */
extern bool	B_DET_BAT_IsShutdownVoltageDropped(void);			/* 低電圧シャットダウン状態提供 */
extern void	V_DET_BAT_ClearAbnormalFlg(void);					/* (DET_BATクラス)異常フラグクリア */
extern void	V_DET_BAT_ClearWarningFlg(void);					/* (DET_BATクラス)警告フラグクリア */
extern u8	U8_DET_BAT_IsAbnormalFlg(void);						/* (DET_BATクラス)異常フラグ提供 */
extern void	V_DET_BAT_ClearBattWarningFlg(void);				/* バッテリ警告フラグクリア */
extern u8	U8_DET_BAT_GetBattWarningFlg(void);					/* バッテリ1警告フラグ提供 */
extern u32	U32_DET_BAT_GetErrorFlg(void);						/* (DET_BATクラス)異常フラグ提供 */
extern u32	U32_DET_BAT_GetWarningFlg(void);					/* (DET_BATクラス)警告フラグ提供 */

#endif /* __DETECT_BATTERY_H */
