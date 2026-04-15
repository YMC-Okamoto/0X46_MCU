/*==========================================================================*/
/**
 * @file	PREVENT_CHATTERING.h
 * @brief	SW入力チャタリング防止ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __PREVENT_CHATTERING_H
#define __PREVENT_CHATTERING_H
/* Includes ------------------------------------------------------------------*/
#include "r_cg_macrodriver.h"
#include "ctypedef.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void V_CHT_Loop10ms(void);					/* SW入力チャタリング処理 */
#if 0
extern void V_CHT_Initialize(void);					/* SW入力チャタリング処理初期化	*/
#endif
extern bool	B_CHT_IsKeySwOn(void);					/* キーSW状態提供 */
extern bool	B_CHT_IsAccelSwOn(void);				/* アクセル SW状態提供 */
extern bool	B_CHT_IsBackSwOn(void);					/* BACK SW状態提供 */
extern bool	B_CHT_IsFwdSwOn(void);					/* FWD SW状態提供 */
extern bool	B_CHT_IsBrOn(void);						/* BR状態提供 */
extern bool	B_CHT_IsChgOn(void);					/* Charge状態提供 */

#endif /* __PREVENT_CHATTERING_H */
