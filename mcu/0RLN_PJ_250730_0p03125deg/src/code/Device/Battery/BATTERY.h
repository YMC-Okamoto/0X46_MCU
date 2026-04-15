/*==========================================================================*/
/**
 * @file	BATTERY.h
 * @brief	バッテリ電流制御処理ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __BATTERY_H
#define __BATTERY_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void V_BAT_Loop10ms(void);											/* バッテリ関連周期処理 */
extern void	V_BAT_InitializeForRestart(void);								/* バッテリ関連変数初期化処理(再起動用) */
extern u16	U16_BAT_GetBattVoltAve_100mV(void);								/* バッテリ電圧提供(フィルタ値:補正反映) */
extern u16	U16_BAT_GetInvVoltAve_100mV(void);								/* インバータ電圧提供(フィルタ値:補正反映) */

//extern s16	S16_BAT_GetBattCurrent_10mA(void);								/* バッテリ電流値提供 */
extern s16	S16_BAT_GetBattCurrentAve_10mA(void);							/* バッテリ電流平均値提供 */
extern s16	S16_BAT_GetBattCurrentAve_100mA(void);							/* バッテリ電流平均値提供 */
extern u16	U16_BAT_GetBattVoltAve_10mV(void);								/* バッテリ電圧提供(フィルタ値:補正反映) */
extern u16	U16_BAT_GetInvVoltAve_10mV(void);								/* インバータ電圧提供(フィルタ値:補正反映) */

#endif /* __BATTERY_H */
