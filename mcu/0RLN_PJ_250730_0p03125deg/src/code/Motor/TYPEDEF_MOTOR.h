/**
 * @file TYPEDEF_MOTOR.h
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
#ifndef __TYPEDEF_MOTOR_H
#define __TYPEDEF_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"					/* 型宣言 */

/* External define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/
/* AD変換結果 */
#pragma bit_order right
#pragma unpack

typedef struct{
	s32 s32_iU_10mA;							/* U相電流[10mA] */
	s32 s32_iV_10mA;							/* V相電流[10mA] */
	s32 s32_iW_10mA;							/* W相電流[10mA] */
}ST_MTR_PHASE_CURRENT;

typedef struct{
	s16	s16_vd;									/* D軸電圧Vd（タイマ値） */
	s16	s16_vq;									/* Q軸電圧Vq（タイマ値） */
	s16	s16_vu;									/* U相電圧指令（タイマ値） */
	s16	s16_vv;									/* V相電圧指令（タイマ値） */
	s16	s16_vw;									/* W相電圧指令（タイマ値） */
	s16	s16_uCommand;							/* U相上アーム用指令値 */
	s16	s16_vCommand;							/* V相上アーム用指令値 */
	s16	s16_wCommand;							/* W相上アーム用指令値 */
	s16	s16_uCommandTemp;						/* U相上アーム用指令値(仮) */
	s16	s16_vCommandTemp;						/* V相上アーム用指令値(仮) */
	s16	s16_wCommandTemp;						/* W相上アーム用指令値(仮) */
}ST_MTR_PWM_TIMER;

typedef struct{
	s32	s32_iUPp_10mA;							/* U相電流PP(TOP-BOTTOM)値 */
	s32	s32_iVPp_10mA;							/* V相電流PP(TOP-BOTTOM)値 */
	s32	s32_iWPp_10mA;							/* W相電流PP(TOP-BOTTOM)値 */
	s32	s32_iUTopBuff_10mA;						/* U相電流TOP仮値 */
	s32	s32_iVTopBuff_10mA;						/* V相電流TOP仮値 */
	s32	s32_iWTopBuff_10mA;						/* W相電流TOP仮値 */
	s32	s32_iUBtmBuff_10mA;						/* U相電流BOT仮値 */
	s32	s32_iVBtmBuff_10mA;						/* V相電流BOT仮値 */
	s32	s32_iWBtmBuff_10mA;						/* W相電流BOT仮値 */
	s32	s32_ctPpMAX_10mA;						/* U,V,Wモータ電流の振幅の最大値 単位10mA */
	s32	s32_ctPpMIN_10mA;						/* U,V,Wモータ電流の振幅の最小値 単位10mA */
}ST_MTR_CURRENT_PP;




/* external variables --------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/


#endif /* __TYPEDEF_MOTOR_H */
