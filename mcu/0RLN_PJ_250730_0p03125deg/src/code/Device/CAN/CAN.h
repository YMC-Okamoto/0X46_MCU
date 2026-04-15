/*==========================================================================*/
/**
 * @file	CAN.h
 * @brief	CAN通信ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __CAN_H
#define __CAN_H
/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
/* Interrupt */
extern void CAN0_RXM0_ISR(void);								/* CAN受信割り込み */

extern void V_CAN_CyclicTransmit(void);							/* CAN周期毎に送信処理 */
extern void V_CAN_Initialize(void);								/* CAN通信初期化処理	*/
extern void	V_CAN_Loop1ms(void);								/* CAN関連1msecループ */
extern void	V_CAN_Loop10ms(void);								/* CAN関連10msecループ */
extern void	V_CAN_DetectCan(void);								/* CAN関連異常判定 */
extern bool	B_CAN_GetKeySwStatus(void);							/* キーSW状態提供 */
extern s16	S16_CAN_GetTorqueOrder_100mNm(void);				/* トルク指令値[0.1Nm]提供 */
extern u8	U8_CAN_GetBuzzerPattern(void);						/* ブザーパターン提供 */
extern bool	B_CAN_IsVcuAbnormal(void);							/* VCU異常状態提供 */
extern s8	S8_CAN_GetEangleOffsetFromCan(void);				/* エンコーダオフセット提供 */
extern u8	U8_CAN_GetRevTimeConstFactorFromCan(void);			/* モータ回転数フィルタ時定数提供 */
extern void V_CAN_InitializeForRestart(void);					/* CAN変数初期化(再起動用) */
extern void	V_CAN_DetectCanInactivate(void);					/* CAN通信異常異常を無効化する */
extern void	V_CAN_DetectCanActivate(void);						/* CAN通信異常異常を有効化する */
extern u8	U8_CAN_GetVcuCommand(void);							/* VCUコマンド提供 */

#endif /* __CAN_H */
