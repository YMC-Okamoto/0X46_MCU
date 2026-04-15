/******************************************************************/
/*!
 * @file	CUW.h
 * @brief	CuW通信 ヘッダ
 * @author	鈴木 秀彰
 * @date	2018/10/15
 */
/*******************************************************************/
#ifndef __CUW_H
#define __CUW_H						/* 二重定義回避 */

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"
//#include "DEFINE_SETTING.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void	V_CUW_Loop10ms(void);								/* CuW通信10msecループ */
extern void	V_CUW_Initialize(void);								/* ロギング通信関連 初期化 */
extern bool	B_CUW_IsFinishedReceive(void);						/* ロギング受信完了フラグ 提供 */
extern void	V_CUW_SetFinishedReceive(void);						/* ロギング受信完了フラグ セット */
extern void	V_CUW_SetRxErr(void);								/* ロギング通信異常フラグ セット */
extern void	V_CUW_ClearStoredTxData(void);						/* ロギング送信データ格納完了フラグ クリア */
extern bool	B_CUW_IsStoredTxData(void);							/* ロギング送信データ格納完了フラグ 提供 */
extern void	V_CUW_GetTxData(u8 *a_pu8_data);					/* 送信データ提供 */
extern void	V_CUW_StoreRxData(u8 *a_pu8_data);					/* 受信データ格納 */

#endif /* __CUW_H */
