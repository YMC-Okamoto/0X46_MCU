/*==========================================================================*/
/**
 * @file	DIAG_SET.c
 * @brief	ダイアグモードの処理
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
/* Includes ------------------------------------------------------------------*/
#include "DEFINE_SYSTEM.h"		/* 定数定義 */

#include "DIAG_SET.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static u16	u16_DIAG_diagCodeRequest = (u16)0;												/* ダイアグ格納要求コード */
static u16	u16_DIAG_diagCode[5] = {(u16)0, (u16)0,(u16)0,(u16)0,(u16)0};					/* ダイアグコード集合 */
static u16	*pu16_DIAG_diagCode = &u16_DIAG_diagCode[0];									/* ダイアグコード ポインタの初期化 */

/**
 * @brief	ダイアグコード格納
 * @param a_u16_diag:		要求ダイアグコード
 * @return none
 */
void V_DIAG_SaveDiagCode( u16 a_u16_diag )
{
	u16_DIAG_diagCodeRequest = a_u16_diag;													/* ダイアグコード要求 */

	for( u8 idx = (u8)4; idx > (u8)0; idx-- )
	{																						/* ダイアグデータ保存 */
		u16_DIAG_diagCode[idx] = u16_DIAG_diagCode[idx - (u8)1];
	}
	u16_DIAG_diagCode[0] = u16_DIAG_diagCodeRequest;										/* ダイアグ格納要求最新コード格納 */
}

/**
 * @brief ダイアグコードポインタ提供
 * @param none
 * @return none
 */

u16 *Pu16_DIAG_GetDiagCode(void)
{
	return pu16_DIAG_diagCode;
}

/**
 * @brief ダイアグコード提供
 * @param none
 * @return none
 */

u8 U8_DIAG_GetDiagCode(void)
{
	return (u8)u16_DIAG_diagCodeRequest;
}
