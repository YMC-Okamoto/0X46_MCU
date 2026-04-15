/*==========================================================================*/
/**
 * @file	TRIGONOMETRIC_FUNCTION.h
 * @brief	三角関数演算群
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2025.01
 */
/*==========================================================================*/
#ifndef __TRIGONOMETRIC_FUNCTION_H
#define __TRIGONOMETRIC_FUNCTION_H

/* Includes ------------------------------------------------------------------*/
#include "TRIGONOMETRIC_FUNCTION.h"
#include "DEFINE_MOTOR.h"

/* External define ------------------------------------------------------------*/
#define U16_2SLASH_SQRT3_16384		(u16)(18919)									/* 2/√3 ×16384 */
#define U16_THETA_OFFSET_120		(u16)((u16)120 << (u16)5)
#define U16_COSIGN_OFFSET_90		(u16)((u16)90 << (u16)5)

#define	U16_ENC_90DEG_EXP5			(u16)((u16)90 << (u16)5)						/* 90deg：分解能0.03215 */
#define	U16_ENC_180DEG_EXP5			(u16)((U16_ENC_90DEG_EXP5) * (u16)2)			/* 180deg：分解能0.03215 */
#define	U16_ENC_270DEG_EXP5			(u16)((U16_ENC_90DEG_EXP5) * (u16)3)			/* 180deg：分解能0.03215 */
#define	U16_ENC_360DEG_EXP5			(u16)((U16_ENC_90DEG_EXP5) * (u16)4)			/* 180deg：分解能0.03215 */
#define	U16_ENC_FIRST_QUADRANT_NUM	(u16)((U16_ENC_90DEG_EXP5) + (u16)1)			/* SINテーブル ポイント数 */

#define TRIG_CALC_DQ_TRANSFORM_A(a_u16_eangle_0p03125deg)	(-((s32)(U16_2SLASH_SQRT3_16384) * (s32)S16_TRIG_Sin(a_u16_eangle_0p03125deg + (U16_THETA_OFFSET_120))) >> (s32)14)								/* -2/√3×sin(θ+120) */
#define TRIG_CALC_DQ_TRANSFORM_B(a_u16_eangle_0p03125deg)	(((s32)(U16_2SLASH_SQRT3_16384) * (s32)S16_TRIG_Sin(a_u16_eangle_0p03125deg)) >> (s32)14)														/* 2/√3×sin(θ) */
#define TRIG_CALC_DQ_TRANSFORM_C(a_u16_eangle_0p03125deg)	(-((s32)(U16_2SLASH_SQRT3_16384) * (s32)S16_TRIG_Sin(a_u16_eangle_0p03125deg + (U16_THETA_OFFSET_120) + (U16_COSIGN_OFFSET_90))) >> (s32)14)	/* -2/√3×cos(θ+120) */
#define TRIG_CALC_DQ_TRANSFORM_D(a_u16_eangle_0p03125deg)	(((s32)(U16_2SLASH_SQRT3_16384) * (s32)S16_TRIG_Sin(a_u16_eangle_0p03125deg + (U16_COSIGN_OFFSET_90))) >> (s32)14)								/* 2/√3×cos(θ) */

#define TRIG_CALC_UVW_TRANSFORM_A(a_u16_eangle_0p03125deg)	(-S16_TRIG_Sin(a_u16_eangle_0p03125deg + (U16_COSIGN_OFFSET_90)))							/* -cosθ */
#define TRIG_CALC_UVW_TRANSFORM_B(a_u16_eangle_0p03125deg)	(S16_TRIG_Sin(a_u16_eangle_0p03125deg))														/* sinθ */
#define TRIG_CALC_UVW_TRANSFORM_C(a_u16_eangle_0p03125deg)	(-S16_TRIG_Sin(a_u16_eangle_0p03125deg + (U16_THETA_OFFSET_120) + (U16_COSIGN_OFFSET_90)))	/* -cos(θ+120) */
#define TRIG_CALC_UVW_TRANSFORM_D(a_u16_eangle_0p03125deg)	(S16_TRIG_Sin(a_u16_eangle_0p03125deg + (U16_THETA_OFFSET_120)))							/* sin(θ+120) */

/* Grobal function prototypes ------------------------------------------------*/
extern s16	S16_TRIG_Sin(u16 a_u16_eangle_0p03125deg);								/* SINテーブル参照 */


#endif /* __COMMON_FUNCTION_H */
