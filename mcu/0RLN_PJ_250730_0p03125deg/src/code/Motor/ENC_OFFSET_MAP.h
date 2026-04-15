/**
 * @file	ENC_OFFSET_MAP.h
 * @brief	エンコーダオフセットマップ定義
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2024.4
 * @note	正転、逆転、力行、回生 合体バージョン
 */
#include "ctypedef.h"					/* Cタイプ変数定義 */
#include "DEFINE_MOTOR.h"

/**
 *		エンコーダ補正値マップ
 *		(正数を入れた分、電流が進む)
 *		電流センサ遅れ	修正	織り込み済み
 **/
#define	OFFSET		(s16)(U16_MTR_EANGLE_OFFSET_DEG)													/* 位相角オフセット(1度単位) 度 */


#define U8_ENC_OFFSET_MAP_POINT_NUM							(u8)15										/* Iqマップ */

const s16 s16_LIM_MTR_encOffsetMapByImXarray_rpm[(U8_ENC_OFFSET_MAP_POINT_NUM)];						/* モータ回転数[rpm] */

const s16 s16_LIM_MTR_encOffsetMap1ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap2ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap3ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap4ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap5ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap6ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap7ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap8ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap9ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap10ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap11ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap12ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap13ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap14ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap15ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap16ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap17ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap18ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap19ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap20ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap21ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap22ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap23ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap24ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */
const s16 s16_LIM_MTR_encOffsetMap25ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)]; 						/* ENC OFFSET */

/*	ENC OFFSET 3D map */
#define U8_ENC_OFFSET_MAP_IM_POINT_NUM						(u8)13										/* エンコーダオフセットマップ トルク軸数 */
const struct ST_CMF_3D_MAP	EncOffset_ImMap[(U8_ENC_OFFSET_MAP_IM_POINT_NUM)] = {
		{0,			s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap1ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{500,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap2ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{1000,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap3ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{1500,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap4ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{2000,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap5ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{2500,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap6ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{3000,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap7ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{3500,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap8ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{4000,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap9ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{4500,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap10ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{5000,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap11ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{5500,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap12ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
		{10000,		s16_LIM_MTR_encOffsetMapByImXarray_rpm, s16_LIM_MTR_encOffsetMap13ByImYarray_deg, (U8_ENC_OFFSET_MAP_POINT_NUM) },
	};

const s16 s16_LIM_MTR_encOffsetMapByImXarray_rpm[(U8_ENC_OFFSET_MAP_POINT_NUM)] =
	{ (s16)(-10000),	(s16)(-6000), 	(s16)(-5000), 	(s16)(-4000), 	(s16)(-3000), 	(s16)(-2000),	(s16)(-1000),	(s16)0,			(s16)1000,		(s16)2000,		(s16)3000,		(s16)4000,		(s16)5000,		(s16)6000,		(s16)10000 };

const s16 s16_LIM_MTR_encOffsetMap1ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:0[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap2ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:500[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap3ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:1000[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap4ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:1500[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap5ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:2000[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap6ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:2500[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap7ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:3000[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap8ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:3500[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap9ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:4000[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap10ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:4500[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap11ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:5000[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap12ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:5500[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};
const s16 s16_LIM_MTR_encOffsetMap13ByImYarray_deg[(U8_ENC_OFFSET_MAP_POINT_NUM)] =		/* ENC OFFSET:10000[100mA] */
	{ (OFFSET)+(s16)0,	(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0,(OFFSET)+(s16)0	};


/* eof */
