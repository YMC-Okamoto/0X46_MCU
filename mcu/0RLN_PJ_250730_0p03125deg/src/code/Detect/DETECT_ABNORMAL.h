/*==========================================================================*/
/**
 * @file	DETECT_ABNORMAL.h
 * @brief	異常検出処理ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __DETECT_ABNNORMAL_H
#define __DETECT_ABNNORMAL_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void	V_ABN_Loop10ms(void);							/*異常検出メイン 10ms周期*/
extern void	V_ABN_Initialize(void);							/*異常検出変数初期化*/
extern bool	B_ABN_GetResultAbnormal( u8 *a_pu8_count, s16 a_s16_cmpItem, u8 a_u8_cmpType, s16 a_s16_threshold, u16 a_u16_countMax );
extern void	V_ABN_ClearAbnormalFlg(void);					/* (ABNクラス)異常フラグクリア */
extern u8	U8_ABN_IsAbnormalFlg(void);						/* (ABNクラス)異常フラグ確認 */
#ifdef	DETECT_ACCEL_ACTIVITY
extern bool	B_ABN_IsJudgedDetectAccelAtBoot(void);			/* 初期アクセル異常判定状態提供 */
extern void	V_ABN_ReleaseJudgedDetectAccelAtBoot(void);		/* 初期アクセル異常判定済みフラグ解除 */
#endif
extern void	V_ABN_ClearAbnormalCanCommunication(void);		/* CAN通信異常解除 */
extern void	V_ABN_SetAbnormalCanCommunication(void);		/* CAN通信異常セット */
extern bool	B_ABN_IsAbnormalCanCommunication(void);			/* CAN通信異常状態提供 */
extern void	V_ABN_SetAbnormalPrechargeWaitOver(void);		/* プリチャージ異常セット */
extern bool	B_ABN_IsAbnormalPrechargeWaitOver(void);		/* プリチャージ異常状態提供 */
extern void	V_ABN_SetAbnormalDischargeWaitOver(void);		/* ディスチャージ異常セット */
extern bool	B_ABN_IsAbnormalDischargeWaitOver(void);		/* ディスチャージ異常状態提供 */
extern bool	B_ABN_IsAbnormalMosThermistorOpen(void);        /* FETサーミスタ異常状態提供 */
extern bool	B_ABN_IsAbnormalCapThermistorOpen(void);        /* コンデンササーミスタ異常状態提供 */
extern bool	B_ABN_IsSomethingAbnormal(void);				/* 異常状態提供 */
extern u32	U32_ABN_GetErrorFlg(void);						/* 異常フラグ集約＆提供 */
extern u32	U32_ABN_GetWarningFlg(void);					/* 警告フラグ集約＆提供 */


#endif /* __DETECT_ABNNORMAL_H */
