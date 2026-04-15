/*==========================================================================*/
/**
 * @file	DIAG_SET.h
 * @brief	ダイアグセットヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __DIAG_SET_H
#define __DIAG_SET_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void V_DIAG_SaveDiagCode( u16 a_u16_diag );				/* ダイアグコードを格納 */

extern u16 *Pu16_DIAG_GetDiagCode(void);						/* ダイアグコード ポインタ提供 */
extern u8 U8_DIAG_GetDiagCode(void);							/* ダイアグコード提供 */



#endif /* __DIAG_SET_H */
