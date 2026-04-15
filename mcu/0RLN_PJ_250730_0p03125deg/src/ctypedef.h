/*==========================================================================*/
/**
 * @file	CTYPEDEF.h
 * @brief	型宣言
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11 Atsumi Takayuki
 */
/*==========================================================================*/

/*
ヘッダファイルには、以下の順序で記述する。
(1)  ファイル名を大文字、ピリオドを下線にしたマクロ名の定義（例：iostd.h→__IOSTD_H)・・・重複インクルードを防止するため。
(2)  コメント
(3)  システムヘッダの取り込み
(4)  ユーザ作成ヘッダの取り込み
(5)  #define マクロ
(6)  #define 関数マクロ
(7)  typedef 定義（int やchar といった基本型に対する型定義）
(8)  enum タグ定義（typedef を同時に行う）
(9)  struct/union タグ定義（typedef を同時に行う）
(10) extern 変数宣言
(11) 関数プロトタイプ宣言
*/

#ifndef __CTYPEDEF_H
#define	__CTYPEDEF_H

#include <stdbool.h>
#include "r_cg_macrodriver.h"

#define NDEBUG
#ifdef NDEBUG
#define STATIC
#endif

#ifdef	DEBUG
#define STATIC static
#endif

/* typedef -----------------------------------------------------------*/
typedef uint8_t		u8;
typedef int8_t		s8;
typedef uint16_t	u16;
typedef int16_t		s16;
typedef uint32_t	u32;
typedef int32_t		s32;
typedef uint64_t	u64;
typedef int64_t 	s64;
/* external define -----------------------------------------------------------*/
/*
#define	BOOL	bool;

	bool、true、falseに関してはC99では"stdbool.h"をインクルードすれば使用可能
*/

/* 真偽値の再定義 */
#define TRUE	(true)					/* 真 */
#define FALSE	(false)					/* 偽 */

#define HIGH	(TRUE)					/* High */
#define LOW		(FALSE)					/* Low */

#define	SW_ON	(TRUE)
#define	SW_OFF	(FALSE)

#pragma bit_order right
#pragma unpack

typedef union
{
	u8		byte;
	struct
	{
		u8	b0:1;
		u8	b1:1;
		u8	b2:1;
		u8	b3:1;
		u8	b4:1;
		u8	b5:1;
		u8	b6:1;
		u8	b7:1;
	}bit;
}UNI_BYTE_FLAG;

typedef union
{
	u16		word;
	struct
	{
		u16	b0:1;
		u16	b1:1;
		u16	b2:1;
		u16	b3:1;
		u16	b4:1;
		u16	b5:1;
		u16	b6:1;
		u16	b7:1;
		u16	b8:1;
		u16	b9:1;
		u16	b10:1;
		u16	b11:1;
		u16	b12:1;
		u16	b13:1;
		u16	b14:1;
		u16	b15:1;
	}bit;
}UNI_WORD_FLAG;

typedef union
{
	u32		ulong;
	struct
	{
		u32	b0:1;
		u32	b1:1;
		u32	b2:1;
		u32	b3:1;
		u32	b4:1;
		u32	b5:1;
		u32	b6:1;
		u32	b7:1;
		u32	b8:1;
		u32	b9:1;
		u32	b10:1;
		u32	b11:1;
		u32	b12:1;
		u32	b13:1;
		u32	b14:1;
		u32	b15:1;
		u32	b16:1;
		u32	b17:1;
		u32	b18:1;
		u32	b19:1;
		u32	b20:1;
		u32	b21:1;
		u32	b22:1;
		u32	b23:1;
		u32	b24:1;
		u32	b25:1;
		u32	b26:1;
		u32	b27:1;
		u32	b28:1;
		u32	b29:1;
		u32	b30:1;
		u32	b31:1;
	}bit;
}UNI_LONG_FLAG;

/*--------------------------------------------------------------------------*/
/*	各型の最大値，最小値、オフセットの定義									*/
/*--------------------------------------------------------------------------*/
#define S8_MIN		(s8)(-127 - 1)						/**<  8bit 整数型 最小値	0x80 */
#define S16_MIN		(s16)(-32767 - 1)					/**< 16bit 整数型 最小値	0x8000 */
#define S32_MIN		(s32)(-2147483647 - 1)				/**< 32bit 整数型 最小値	0x8000 0000 */
#define S64_MIN		(s64)(-9223372036854775807LL - 1)	/**< 64bit 整数型 最小値	0x8000 0000 0000 0000 */

#define S8_MAX		(s8)(127)							/**<  8bit 整数型 最大値	0x7F */
#define S16_MAX		(s16)(32767)						/**< 16bit 整数型 最大値	0x7FFF */
#define S32_MAX		(s32)(2147483647)					/**< 32bit 整数型 最大値	0x7FFF FFFF */
#define S64_MAX		(s64)(9223372036854775807LL)		/**< 64bit 整数型 最大値	0x7FFF FFFF FFFF FFFF */

#define U8_MAX		(u8)(255U)							/**<  8bit 整数型 (非負整数) 最大値	0xFF */
#define U16_MAX		(u16)(65535U)						/**< 16bit 整数型 (非負整数) 最大値	0xFFFF */
#define U32_MAX		(u32)(4294967295U)					/**< 32bit 整数型 (非負整数) 最大値	0xFFFF FFFF */
#define U64_MAX		(u64)(18446744073709551615ULL)		/**< 64bit 整数型 (非負整数) 最大値	0xFFFF FFFF FFFF FFFF */

#define	U8_OFFSET	(u8)((u8)(S8_MAX) + (u8)1)			/* 符号なしで負の値を表現する為のオフセット	128:0x80 */
#define	U16_OFFSET	(u16)((u16)(S16_MAX) + (u16)1)		/* 符号なしで負の値を表現する為のオフセット	32768:0x8000 */

#endif	/* end of __CTYPEDEF_H */
