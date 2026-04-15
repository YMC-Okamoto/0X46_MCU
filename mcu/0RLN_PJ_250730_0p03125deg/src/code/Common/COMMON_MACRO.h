/*==========================================================================*/
/**
 * @file	COMMON_MACRO.h
 * @brief	汎用関数マクロ群
 * @author	Power Conversion System Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __COMMON_MACRO_H
#define __COMMON_MACRO_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"

/* External define ------------------------------------------------------------*/
/* 【ビット操作】bitNumber：対象ビット、value：対象変数 */
#define CMF_GETBIT(value,bitNum)			(((u32)((u32)(value) >> (u32)(bitNum))) & (u32)1 )					/* ＧＥＴビット */ /* 出力はu32 */
#define CMF_BITNOT(value,bitNum)			(( (CMF_BITTEST((value),(bitNum))) ) ? (CMF_BITCLR((value),(bitNum))) : (CMF_BITSET((value),(bitNum))))	/* ビットＮＯＴ */
#define CMF_BITTEST(value,bitNum)			(u32)(s32)(((value) & ((u32)1 << (bitNum))) != (u32)0 )				/* ビットＴＥＳＴ（１：true） */ /* 出力はboolean */
#define CMF_BITSET(value,bitNum)			((value) |=  ( (u32)1 << (bitNum) ))								/* ビットＳＥＴ */
#define CMF_BITCLR(value,bitNum)			((value) &= ~( (u32)1 << (bitNum) ))								/* ビットＣＬＲ */

#define U32_CMF_BITSET(value,bitNum)		((value) |  ( (u32)1 << (bitNum) ))									/* MISRA-C10.1対応ビットＳＥＴ */
#define U32_CMF_BITCLR(value,bitNum)		((value) & ~( (u32)1 << (bitNum) ))									/* MISRA-C10.1対応ビットＣＬＲ */

#define CMF_BITXOR(value1,value2,bitNum)	(CMF_GETBIT(((value1)^(value2)),(bitNum)))							/* ビット排他（違う：1） */ /* 出力はu32 */
#define CMF_BITCOMP(value1,value2,bitNum)	((CMF_GETBIT(((value1)^(value2)),(bitNum))) == (u32)0 )				/* ビット比較（同じ：true） */ /* 出力はboolean */

#define CMF_CLIP_MAX( a,max )				((( a ) < ( max )) ? ( a ) : ( max ))								/* 最大値でクリップ */
#define CMF_GETMAX( a, b )					(( a ) > ( b ) ? ( a ) : ( b ))										/* 大きい方を返す*/
#define CMF_GETMIN( a, b )					(( a ) < ( b ) ? ( a ) : ( b ))										/* 小さい方を返す*/
#define CMF_GETABS( a )						(( a ) < ( 0 ) ? ( -a ) : ( a ))									/* 絶対値を返す */
#define CMF_GETLIMIT( a, min, max )			(( a ) < ( min ) ? ( min ) : (( a ) > ( max ) ? ( max ) : ( a )))	/* 最大値と最小値に制限させる値を返す */
#define CMF_GETROUNDRSHIFT( a,b )			((( a ) + ( 1 << (( b )(-1) ))) >> ( b ))							/* 値a を bだけ右シフトしつつ、溢れる分の1ビットをLSBに加算して、四捨五入する */ 
																												/* 注意：a＜0 の時は、絶対値が小さい方に丸められる */

#define U8_CMF_GETUPPER_BYTE( a )			(u8)(( (u16)a & 0xFF00 ) >> (u16)8 )								/* １６ビット中の上位８ビット取得 */
#define U8_CMF_GETLOWER_BYTE( a ) 			(u8)( (u16)a & 0x00FF )												/* １６ビット中の下位８ビット取得 */
#define U16_CMF_MERGE_BYTE( a , b )			(u16)( ((u16)a << (u16)8 ) | (u16)b )								/* 8ビットと8ビットを16ビットに結合 */

#define U8_CMF_GETFIRST_BYTE( a )			(u8)(( (u32)a & 0xFF000000 ) >> (u32)24 )							/* ３２ビット中の最上位８ビット取得 */
#define U8_CMF_GETSECOND_BYTE( a )			(u8)(( (u32)a & 0x00FF0000 ) >> (u32)16 )							/* ３２ビット中の上位から２番目８ビット取得 */
#define U8_CMF_GETTHIRD_BYTE( a )			(u8)(( (u32)a & 0x0000FF00 ) >> (u32)8 )							/* ３２ビット中の上位から３番目８ビット取得 */
#define U8_CMF_GETFOURTH_BYTE( a )			(u8)( (u32)a & 0x000000FF )											/* ３２ビット中の最下位８ビット取得 */

/* External typedef ----------------------------------------------------------*/

/* External macro ------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/


#endif /* __COMMON_MACRO_H */
