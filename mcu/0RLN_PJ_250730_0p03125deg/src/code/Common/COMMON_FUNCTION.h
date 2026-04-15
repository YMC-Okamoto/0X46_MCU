/*==========================================================================*/
/**
 * @file	COMMON_FUNCTION.h
 * @brief	汎用関数群
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2019.08
 */
/*==========================================================================*/
#ifndef __COMMON_FUNCTION_H
#define __COMMON_FUNCTION_H

/* Includes ------------------------------------------------------------------*/
#include "COMMON_MACRO.h"

/* External define ------------------------------------------------------------*/
/* External typedef ----------------------------------------------------------*/
#pragma bit_order right
#pragma unpack

typedef struct {			/* U16フィルタリング結果を格納する構造体 */
	u32	u32_sum;
	u16	u16_ave;
} ST_CMF_RESULT_IIRF_U16;

typedef struct {			/* S16フィルタリング結果を格納する構造体 */
	s32	s32_sum;
	s16	s16_ave;
} ST_CMF_RESULT_IIRF_S16;

typedef struct {			/* S32フィルタリング結果を格納する構造体 */
	s64	s64_sum;
	s32	s32_ave;
} ST_CMF_RESULT_IIRF_S32;

/*3Dマップ構造体定義*/
struct ST_CMF_3D_MAP{
	const s32	s32_keyData;	/* キーデータ→小さいから大きい順に並べる */
	const s16	* s16_x_p;		/* xデータ行列アドレス */
	const s16	* s16_y_p;		/* yデータ行列アドレス */
	const s16	s16_tabSize;	/* 2Dのテーブルサイズ */
};

/* External macro ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Grobal function prototypes ------------------------------------------------*/
extern ST_CMF_RESULT_IIRF_S32	St_CMF_CalcIIRFilterS32ByShift(s32 a_s32_newValue, s32 a_s32_ave, u8 a_u8_fltFactorByShiftBits, s64 a_s64_sum);	/* 符号付き32bitIIRフィルタ(ビットシフト版) */
extern ST_CMF_RESULT_IIRF_U16	St_CMF_CalcIIRFilterU16ByShift(u16 a_u16_newValue, u16 a_u16_ave, u8 a_u8_fltFactorByShiftBits, u32 a_u32_sum);	/* 符号なし16bitIIRフィルタ(ビットシフト版) */
extern ST_CMF_RESULT_IIRF_S16	St_CMF_CalcIIRFilterS16ByShift(s16 a_s16_newValue, s16 a_s16_ave, u8 a_u8_fltFactorByShiftBits, s32 a_s32_sum);	/* 符号付き16bitIIRフィルタ(ビットシフト版) */
extern ST_CMF_RESULT_IIRF_U16	St_CMF_CalcIIRFilterU16(u16 a_u16_newValue, u16 a_u16_ave, u8 a_u8_fltFactor, u32 a_u32_sum);					/* 符号なし16bitIIRフィルタ */
extern ST_CMF_RESULT_IIRF_S16	St_CMF_CalcIIRFilterS16(s16 a_s16_newValue, s16 a_s16_ave, u8 a_u8_fltFactor, s32 a_s32_sum);					/* 符号付き16bitIIRフィルタ */

extern u16						U16_CMF_CalcUnsignedMovingAve4( u16 a_u16_xDat, u16 *a_u16_xBuf );												/* 4移動平均処理符号無し	*/
extern s16						S16_CMF_CalcSignedMovingAve4( s16 a_s16_xDat, s16 *a_s16_xBuf );												/* 4移動平均処理符号付き	*/
extern u16						U16_CMF_CalcUnsignedMovingAve8( u16 a_u16_xDat, u16 *a_u16_xBuf );												/* 8移動平均処理符号無し	*/
extern u32						U32_CMF_Sqrt(u32 a_u32_input);																					/* 平方根 */
extern void						V_CMF_Copy(u8 a_u8_dstArray[], u8 a_u8_srcArray[], u32 a_u32_length);											/* 配列コピー(配列対配列) */
extern void						V_CMF_Memset16(u16 a_u16_array[], u16 a_u16_value, u32 a_u32_length);											/* 16bit配列コピー(配列へ値をコピー) */

extern void						V_CMF_Memset(u8 a_u8_array[], u8 a_u8_value, u32 a_u32_length);													/* 配列コピー(配列へ値をコピー) */
extern s16						S16_CMF_2DMapCal( s16 a_s16_inputData, u16 a_u16_tabSize, const s16 a_s16_xData[], const s16 a_s16_yData[] );	/* 2次元マップ補間 */
extern s16						S16_CMF_3DMapCal( u8 keydat_size, s32 inputKeydat,  s16 inputXdata, const struct ST_CMF_3D_MAP *p );			/* 3次元マップ補間 */
extern u8						U8_CMF_CalcCheckSum(u8 a_u8_array[], u16 a_u16_length);															/* チェックサム計算 */

extern s32						S32_CMF_GETMAX( s32 a_s32_value1, s32 a_s32_value2 );															/* 大きい方を返す */
extern s32						S32_CMF_GETMIN( s32 a_s32_value1, s32 a_s32_value2 );															/* 小さい方を返す */
extern s32						S32_CMF_GETLIMIT( s32 a_s32_value, s32 a_s32_min, s32 a_s32_max );												/* 最大値と最小値に制限させる値を返す */
extern u32						U32_CMF_GETABS( s32 a_s32_value );																				/* 絶対値を返す */

extern s32						S32_CMF_QADD(s32 a_s32_value1, s32 a_s32_value2);																/* 飽和加算 */
extern s32						S32_CMF_QSUB(s32 a_s32_value1, s32 a_s32_value2);																/* 飽和減算 */

#endif /* __COMMON_FUNCTION_H */
