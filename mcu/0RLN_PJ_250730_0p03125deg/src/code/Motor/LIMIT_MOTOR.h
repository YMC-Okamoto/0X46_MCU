/*==========================================================================*/
/**
 * @file	LIMIT_MOTOR.h
 * @brief	Iq制限値算出処理ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __LIMIT_MOTOR_H
#define __LIMIT_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void	V_LIM_MTR_Initialize(void);								/* Iq指令値処理変数初期化 */
extern void	V_LIM_MTR_Loop10ms(void);								/* マップによる電流指令値算出 */
extern s32	S32_LIM_MTR_GetIdRef_10mA(void);						/* マップによるId指令値提供[10mA] */
extern s32	S32_LIM_MTR_GetIqRef_10mA(void);						/* マップによるIq指令値提供[10mA] */
extern s16	S16_LIM_MTR_GetIfRef_10mA(void);						/* マップによるIf指令値提供[10mA] */
extern s16	S16_LIM_MTR_GetTorqueRef_100mNm(void);					/* 最終トルク指令値提供[0.1Nm] */
extern u8	U8_LIM_MTR_GetQuadrant(void);							/* 象限提供 */

#endif /* __LIMIT_MOTOR_H */
