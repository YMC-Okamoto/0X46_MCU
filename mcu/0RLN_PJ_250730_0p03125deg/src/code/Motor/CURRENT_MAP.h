/**
 * @file	CURRENT_MAP.h
 * @brief	IdRef、IqRef、IfRefマップ定義
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2025.6
 * @note	正転、逆転、力行、回生 合体バージョン
 */
#include "ctypedef.h"					/* Cタイプ変数定義 */

	/*
		s16なので32767以内に抑えること
		Id、Iq指令値に関してs16だとオーバーフローするのでマップ上は1/2した値とし、マップ参照後に2倍する
	*/

#define S16_CURRENT_MAP_POINT_NUM							(s16)17									/* Iqマップ */

const s16 s16_LIM_MTR_currentMapByTorqueXarray_rpm[(S16_CURRENT_MAP_POINT_NUM)];					/* モータ回転数[rpm] */

const s16 s16_LIM_MTR_idMap1ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap2ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap3ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap4ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap5ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap6ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap7ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap8ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap9ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap10ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap11ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap12ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap13ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap14ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap15ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap16ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap17ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap18ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap19ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap20ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */
const s16 s16_LIM_MTR_idMap21ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Id指令値 */

const s16 s16_LIM_MTR_iqMap1ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap2ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap3ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap4ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap5ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap6ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap7ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap8ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap9ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap10ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap11ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap12ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap13ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap14ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap15ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap16ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap17ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap18ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap19ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap20ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */
const s16 s16_LIM_MTR_iqMap21ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* Iq指令値 */

const s16 s16_LIM_MTR_ifMap1ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap2ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap3ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap4ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap5ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap6ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap7ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap8ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap9ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap10ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap11ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap12ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap13ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap14ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap15ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap16ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap17ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap18ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap19ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap20ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */
const s16 s16_LIM_MTR_ifMap21ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]; 						/* If指令値 */


#define U8_CURRENT_MAP_T_POINT_NUM						(u8)21										/* 電流マップ トルク軸数 */

/*	Id 3D map */
const struct ST_CMF_3D_MAP	Id_Tmap[(U8_CURRENT_MAP_T_POINT_NUM)] = {
		{(-1000),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap1ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-600),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap2ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-500),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap3ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-400),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap4ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-300),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap5ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-250),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap6ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-200),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap7ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-150),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap8ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-100),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap9ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-50),		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap10ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{0,			s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap11ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{50,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap12ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{100,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap13ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{150,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap14ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{200,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap15ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{250,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap16ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{300,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap17ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{400,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap18ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{500,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap19ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{600,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap20ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{1000,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_idMap21ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
	};

const s16 s16_LIM_MTR_currentMapByTorqueXarray_rpm[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-10000),(s16)(-6000),(s16)(-5000),(s16)(-4000),(s16)(-3000),(s16)(-2000),(s16)(-1000),(s16)(-500),(s16)0,(s16)500,(s16)1000,(s16)2000,(s16)3000,(s16)4000,(s16)5000,(s16)6000,(s16)10000};				/* モータ回転数[rpm] */

const s16 s16_LIM_MTR_idMap1ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)] 
	= {(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1135,(s16)1135,(s16)1135,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578};				/* Id指令値 [10mA]:-1000[mNm] */
const s16 s16_LIM_MTR_idMap2ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1135,(s16)1135,(s16)1135,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578};				/* Id指令値 [10mA]:-600[mNm] */
const s16 s16_LIM_MTR_idMap3ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1325,(s16)1635,(s16)1635,(s16)1635,(s16)1325,(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1658};				/* Id指令値 [10mA]:-500[mNm] */
const s16 s16_LIM_MTR_idMap4ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)1072,(s16)1665,(s16)1920,(s16)1920,(s16)1920,(s16)1665,(s16)1072,(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)(-7652)};				/* Id指令値 [10mA]:-400[mNm] */
const s16 s16_LIM_MTR_idMap5ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-11908),(s16)(-11908),(s16)(-11908),(s16)(-11908),(s16)(-3792),(s16)1278,(s16)1576,(s16)1461,(s16)1461,(s16)1461,(s16)1576,(s16)1278,(s16)(-3792),(s16)(-11908),(s16)(-11908),(s16)(-11908),(s16)(-11908)};			/* Id指令値 [10mA]:-300[mNm] */
const s16 s16_LIM_MTR_idMap6ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-14131),(s16)(-14131),(s16)(-14131),(s16)(-8295),(s16)(-973),(s16)1293,(s16)1295,(s16)1446,(s16)1446,(s16)1446,(s16)1295,(s16)1293,(s16)(-973),(s16)(-8295),(s16)(-14131),(s16)(-14131),(s16)(-14131)};			/* Id指令値 [10mA]:-250[mNm] */
const s16 s16_LIM_MTR_idMap7ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-15041),(s16)(-15041),(s16)(-10232),(s16)(-4848),(s16)638,(s16)1576,(s16)1567,(s16)1526,(s16)1526,(s16)1526,(s16)1567,(s16)1576,(s16)638,(s16)(-4848),(s16)(-10232),(s16)(-15041),(s16)(-15041)};			/* Id指令値 [10mA]:-200[mNm] */
const s16 s16_LIM_MTR_idMap8ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-10600),(s16)(-10600),(s16)(-6978),(s16)(-2276),(s16)1547,(s16)1540,(s16)1278,(s16)1616,(s16)1616,(s16)1616,(s16)1278,(s16)1540,(s16)1547,(s16)(-2276),(s16)(-6978),(s16)(-10600),(s16)(-10600)};			/* Id指令値 [10mA]:-150[mNm] */
const s16 s16_LIM_MTR_idMap9ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-5194),(s16)(-5194),(s16)(-3010),(s16)(-98),(s16)1059,(s16)1166,(s16)965,(s16)1901,(s16)1901,(s16)1901,(s16)965,(s16)1166,(s16)1059,(s16)(-98),(s16)(-3010),(s16)(-5194),(s16)(-5194)};			/* Id指令値 [10mA]:-100[mNm] */
const s16 s16_LIM_MTR_idMap10ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-1037),(s16)(-1037),(s16)284,(s16)906,(s16)769,(s16)710,(s16)757,(s16)866,(s16)866,(s16)866,(s16)757,(s16)710,(s16)769,(s16)906,(s16)284,(s16)(-1037),(s16)(-1037)};			/* Id指令値 [10mA]:-50[mNm] */
const s16 s16_LIM_MTR_idMap11ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0,(s16)0};			/* Id指令値 [10mA]:0[mNm] */
const s16 s16_LIM_MTR_idMap12ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-1037),(s16)(-1037),(s16)284,(s16)906,(s16)769,(s16)710,(s16)757,(s16)866,(s16)866,(s16)866,(s16)757,(s16)710,(s16)769,(s16)906,(s16)284,(s16)(-1037),(s16)(-1037)};			/* Id指令値 [10mA]:50[mNm] */
const s16 s16_LIM_MTR_idMap13ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-5194),(s16)(-5194),(s16)(-3010),(s16)(-98),(s16)1059,(s16)1166,(s16)965,(s16)1901,(s16)1901,(s16)1901,(s16)965,(s16)1166,(s16)1059,(s16)(-98),(s16)(-3010),(s16)(-5194),(s16)(-5194)};			/* Id指令値 [10mA]:100[mNm] */
const s16 s16_LIM_MTR_idMap14ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-10600),(s16)(-10600),(s16)(-6978),(s16)(-2276),(s16)1547,(s16)1540,(s16)1278,(s16)1616,(s16)1616,(s16)1616,(s16)1278,(s16)1540,(s16)1547,(s16)(-2276),(s16)(-6978),(s16)(-10600),(s16)(-10600)};			/* Id指令値 [10mA]:150[mNm] */
const s16 s16_LIM_MTR_idMap15ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-15041),(s16)(-15041),(s16)(-10232),(s16)(-4848),(s16)638,(s16)1576,(s16)1567,(s16)1526,(s16)1526,(s16)1526,(s16)1567,(s16)1576,(s16)638,(s16)(-4848),(s16)(-10232),(s16)(-15041),(s16)(-15041)};			/* Id指令値 [10mA]:200[mNm] */
const s16 s16_LIM_MTR_idMap16ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-14131),(s16)(-14131),(s16)(-14131),(s16)(-8295),(s16)(-973),(s16)1293,(s16)1295,(s16)1446,(s16)1446,(s16)1446,(s16)1295,(s16)1293,(s16)(-973),(s16)(-8295),(s16)(-14131),(s16)(-14131),(s16)(-14131)};			/* Id指令値 [10mA]:250[mNm] */
const s16 s16_LIM_MTR_idMap17ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-11908),(s16)(-11908),(s16)(-11908),(s16)(-11908),(s16)(-3792),(s16)1278,(s16)1576,(s16)1461,(s16)1461,(s16)1461,(s16)1576,(s16)1278,(s16)(-3792),(s16)(-11908),(s16)(-11908),(s16)(-11908),(s16)(-11908)};			/* Id指令値 [10mA]:300[mNm] */
const s16 s16_LIM_MTR_idMap18ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)1072,(s16)1665,(s16)1920,(s16)1920,(s16)1920,(s16)1665,(s16)1072,(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)(-7652),(s16)(-7652)};			/* Id指令値 [10mA]:400[mNm] */
const s16 s16_LIM_MTR_idMap19ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1325,(s16)1635,(s16)1635,(s16)1635,(s16)1325,(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1658,(s16)1658};			/* Id指令値 [10mA]:500[mNm] */
const s16 s16_LIM_MTR_idMap20ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1135,(s16)1135,(s16)1135,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578};			/* Id指令値 [10mA]:600[mNm] */
const s16 s16_LIM_MTR_idMap21ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1135,(s16)1135,(s16)1135,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578,(s16)1578};			/* Id指令値 [10mA]:1000[mNm] */


/* Iq 3D map */
const struct ST_CMF_3D_MAP	Iq_Tmap[(U8_CURRENT_MAP_T_POINT_NUM)] = {
		{(-1000),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap1ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-600),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap2ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-500),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap3ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-400),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap4ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-300),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap5ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-250),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap6ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-200),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap7ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-150),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap8ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-100),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap9ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-50),		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap10ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{0,			s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap11ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{50,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap12ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{100,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap13ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{150,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap14ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{200,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap15ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{250,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap16ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{300,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap17ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{400,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap18ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{500,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap19ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{600,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap20ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{1000,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_iqMap21ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
	};

const s16 s16_LIM_MTR_iqMap1ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)] 
	= {(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-22562),(s16)(-21649),(s16)(-21649),(s16)(-21649),(s16)(-22562),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018)};			/* Iq指令値 [10mA]:-1000[mNm] */
const s16 s16_LIM_MTR_iqMap2ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-22562),(s16)(-21649),(s16)(-21649),(s16)(-21649),(s16)(-22562),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018)};			/* Iq指令値 [10mA]:-600[mNm] */
const s16 s16_LIM_MTR_iqMap3ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-18947),(s16)(-18941),(s16)(-18682),(s16)(-18682),(s16)(-18682),(s16)(-18941),(s16)(-18947),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018)};			/* Iq指令値 [10mA]:-500[mNm] */
const s16 s16_LIM_MTR_iqMap4ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15330),(s16)(-15835),(s16)(-15639),(s16)(-15639),(s16)(-15639),(s16)(-15835),(s16)(-15330),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018),(s16)(-15018)};			/* Iq指令値 [10mA]:-400[mNm] */
const s16 s16_LIM_MTR_iqMap5ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-11908),(s16)(-11908),(s16)(-11908),(s16)(-11908),(s16)(-12404),(s16)(-12155),(s16)(-12832),(s16)(-11900),(s16)(-11900),(s16)(-11900),(s16)(-12832),(s16)(-12155),(s16)(-12404),(s16)(-11908),(s16)(-11908),(s16)(-11908),(s16)(-11908)};			/* Iq指令値 [10mA]:-300[mNm] */
const s16 s16_LIM_MTR_iqMap6ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-10267),(s16)(-10267),(s16)(-10267),(s16)(-11417),(s16)(-11126),(s16)(-10525),(s16)(-10545),(s16)(-10285),(s16)(-10285),(s16)(-10285),(s16)(-10545),(s16)(-10525),(s16)(-11126),(s16)(-11417),(s16)(-10267),(s16)(-10267),(s16)(-10267)};			/* Iq指令値 [10mA]:-250[mNm] */
const s16 s16_LIM_MTR_iqMap7ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-7997),(s16)(-7997),(s16)(-9541),(s16)(-9939),(s16)(-9122),(s16)(-8938),(s16)(-8888),(s16)(-8652),(s16)(-8652),(s16)(-8652),(s16)(-8888),(s16)(-8938),(s16)(-9122),(s16)(-9939),(s16)(-9541),(s16)(-7997),(s16)(-7997)};			/* Iq指令値 [10mA]:-200[mNm] */
const s16 s16_LIM_MTR_iqMap8ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-6624),(s16)(-6624),(s16)(-7483),(s16)(-8493),(s16)(-7276),(s16)(-7245),(s16)(-7248),(s16)(-6999),(s16)(-6999),(s16)(-6999),(s16)(-7248),(s16)(-7245),(s16)(-7276),(s16)(-8493),(s16)(-7483),(s16)(-6624),(s16)(-6624)};			/* Iq指令値 [10mA]:-150[mNm] */
const s16 s16_LIM_MTR_iqMap9ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-6190),(s16)(-6190),(s16)(-6455),(s16)(-5617),(s16)(-5445),(s16)(-5483),(s16)(-5472),(s16)(-5850),(s16)(-5850),(s16)(-5850),(s16)(-5472),(s16)(-5483),(s16)(-5445),(s16)(-5617),(s16)(-6455),(s16)(-6190),(s16)(-6190)};			/* Iq指令値 [10mA]:-100[mNm] */
const s16 s16_LIM_MTR_iqMap10ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)(-4492),(s16)(-4492),(s16)(-4052),(s16)(-3922),(s16)(-3955),(s16)(-4026),(s16)(-3894),(s16)(-3750),(s16)(-3750),(s16)(-3750),(s16)(-3894),(s16)(-4026),(s16)(-3955),(s16)(-3922),(s16)(-4052),(s16)(-4492),(s16)(-4492)};			/* Iq指令値 [10mA]:-50[mNm] */
const s16 s16_LIM_MTR_iqMap11ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0};	/* Iq指令値 [10mA]:0[mNm] */
const s16 s16_LIM_MTR_iqMap12ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)4492,(s16)4492,(s16)4052,(s16)3922,(s16)3955,(s16)4026,(s16)3894,(s16)3750,(s16)3750,(s16)3750,(s16)3894,(s16)4026,(s16)3955,(s16)3922,(s16)4052,(s16)4492,(s16)4492};			/* Iq指令値 [10mA]:50[mNm] */
const s16 s16_LIM_MTR_iqMap13ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)6190,(s16)6190,(s16)6455,(s16)5617,(s16)5445,(s16)5483,(s16)5472,(s16)5850,(s16)5850,(s16)5850,(s16)5472,(s16)5483,(s16)5445,(s16)5617,(s16)6455,(s16)6190,(s16)6190};			/* Iq指令値 [10mA]:100[mNm] */
const s16 s16_LIM_MTR_iqMap14ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)6624,(s16)6624,(s16)7483,(s16)8493,(s16)7276,(s16)7245,(s16)7248,(s16)6999,(s16)6999,(s16)6999,(s16)7248,(s16)7245,(s16)7276,(s16)8493,(s16)7483,(s16)6624,(s16)6624};			/* Iq指令値 [10mA]:150[mNm] */
const s16 s16_LIM_MTR_iqMap15ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)7997,(s16)7997,(s16)9541,(s16)9939,(s16)9122,(s16)8938,(s16)8888,(s16)8652,(s16)8652,(s16)8652,(s16)8888,(s16)8938,(s16)9122,(s16)9939,(s16)9541,(s16)7997,(s16)7997};			/* Iq指令値 [10mA]:200[mNm] */
const s16 s16_LIM_MTR_iqMap16ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)10267,(s16)10267,(s16)10267,(s16)11417,(s16)11126,(s16)10525,(s16)10545,(s16)10285,(s16)10285,(s16)10285,(s16)10545,(s16)10525,(s16)11126,(s16)11417,(s16)10267,(s16)10267,(s16)10267};			/* Iq指令値 [10mA]:250[mNm] */
const s16 s16_LIM_MTR_iqMap17ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)11908,(s16)11908,(s16)11908,(s16)11908,(s16)12404,(s16)12155,(s16)12832,(s16)11900,(s16)11900,(s16)11900,(s16)12832,(s16)12155,(s16)12404,(s16)11908,(s16)11908,(s16)11908,(s16)11908};			/* Iq指令値 [10mA]:300[mNm] */
const s16 s16_LIM_MTR_iqMap18ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15330,(s16)15835,(s16)15639,(s16)15639,(s16)15639,(s16)15835,(s16)15330,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018};			/* Iq指令値 [10mA]:400[mNm] */
const s16 s16_LIM_MTR_iqMap19ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)18947,(s16)18941,(s16)18682,(s16)18682,(s16)18682,(s16)18941,(s16)18947,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018};			/* Iq指令値 [10mA]:500[mNm] */
const s16 s16_LIM_MTR_iqMap20ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)22562,(s16)21649,(s16)21649,(s16)21649,(s16)22562,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018};			/* Iq指令値 [10mA]:600[mNm] */
const s16 s16_LIM_MTR_iqMap21ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)22562,(s16)21649,(s16)21649,(s16)21649,(s16)22562,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018,(s16)15018};			/* Iq指令値 [10mA]:1000[mNm] */


/* If 3D map */
const struct ST_CMF_3D_MAP	If_Tmap[(U8_CURRENT_MAP_T_POINT_NUM)] = {
		{(-1000),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap1ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-600),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap2ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-500),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap3ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-400),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap4ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-300),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap5ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-250),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap6ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-200),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap7ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-150),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap8ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-100),	s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap9ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{(-50),		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap10ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{0,			s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap11ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{50,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap12ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{100,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap13ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{150,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap14ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{200,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap15ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{250,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap16ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{300,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap17ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{400,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap18ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{500,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap19ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{600,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap20ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
		{1000,		s16_LIM_MTR_currentMapByTorqueXarray_rpm, s16_LIM_MTR_ifMap21ByTorqueYarray_10mA, (S16_CURRENT_MAP_POINT_NUM) },
	};

const s16 s16_LIM_MTR_ifMap1ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)] 
	= {(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1400,(s16)1500,(s16)1500,(s16)1500,(s16)1400,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300};	/* If指令値 [10mA]:-1000[mNm] */
const s16 s16_LIM_MTR_ifMap2ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1400,(s16)1500,(s16)1500,(s16)1500,(s16)1400,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300};	/* If指令値 [10mA]:-600[mNm] */
const s16 s16_LIM_MTR_ifMap3ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300};	/* If指令値 [10mA]:-500[mNm] */
const s16 s16_LIM_MTR_ifMap4ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1400,(s16)1400,(s16)1400,(s16)1400,(s16)1400,(s16)1200,(s16)1100,(s16)1100,(s16)1100,(s16)1100,(s16)1100,(s16)1200,(s16)1400,(s16)1400,(s16)1400,(s16)1400,(s16)1400};	/* If指令値 [10mA]:-400[mNm] */
const s16 s16_LIM_MTR_ifMap5ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1400,(s16)1400,(s16)1400,(s16)1400,(s16)1100,(s16)1000,(s16)900,(s16)1000,(s16)1000,(s16)1000,(s16)900,(s16)1000,(s16)1100,(s16)1400,(s16)1400,(s16)1400,(s16)1400};	/* If指令値 [10mA]:-300[mNm] */
const s16 s16_LIM_MTR_ifMap6ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1400,(s16)1400,(s16)1400,(s16)1100,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)1100,(s16)1400,(s16)1400,(s16)1400};	/* If指令値 [10mA]:-250[mNm] */
const s16 s16_LIM_MTR_ifMap7ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1400,(s16)1400,(s16)1100,(s16)900,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)900,(s16)1100,(s16)1400,(s16)1400};	/* If指令値 [10mA]:-200[mNm] */
const s16 s16_LIM_MTR_ifMap8ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1100,(s16)1100,(s16)900,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)900,(s16)1100,(s16)1100};	/* If指令値 [10mA]:-150[mNm] */
const s16 s16_LIM_MTR_ifMap9ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)700,(s16)700,(s16)600,(s16)600,(s16)600,(s16)600,(s16)600,(s16)500,(s16)500,(s16)500,(s16)600,(s16)600,(s16)600,(s16)600,(s16)600,(s16)700,(s16)700};	/* If指令値 [10mA]:-100[mNm] */
const s16 s16_LIM_MTR_ifMap10ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400};	/* If指令値 [10mA]:-50[mNm] */
const s16 s16_LIM_MTR_ifMap11ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0,	(s16)0};	/* If指令値 [10mA]:0[mNm] */
const s16 s16_LIM_MTR_ifMap12ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400,(s16)400};	/* If指令値 [10mA]:50[mNm] */
const s16 s16_LIM_MTR_ifMap13ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)700,(s16)700,(s16)600,(s16)600,(s16)600,(s16)600,(s16)600,(s16)500,(s16)500,(s16)500,(s16)600,(s16)600,(s16)600,(s16)600,(s16)600,(s16)700,(s16)700};	/* If指令値 [10mA]:100[mNm] */
const s16 s16_LIM_MTR_ifMap14ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1100,(s16)1100,(s16)900,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)700,(s16)900,(s16)1100,(s16)1100};	/* If指令値 [10mA]:150[mNm] */
const s16 s16_LIM_MTR_ifMap15ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1400,(s16)1400,(s16)1100,(s16)900,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)800,(s16)900,(s16)1100,(s16)1400,(s16)1400};	/* If指令値 [10mA]:200[mNm] */
const s16 s16_LIM_MTR_ifMap16ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1400,(s16)1400,(s16)1400,(s16)1100,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)900,(s16)1100,(s16)1400,(s16)1400,(s16)1400};	/* If指令値 [10mA]:250[mNm] */
const s16 s16_LIM_MTR_ifMap17ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1400,(s16)1400,(s16)1400,(s16)1400,(s16)1100,(s16)1000,(s16)900,(s16)1000,(s16)1000,(s16)1000,(s16)900,(s16)1000,(s16)1100,(s16)1400,(s16)1400,(s16)1400,(s16)1400};	/* If指令値 [10mA]:300[mNm] */
const s16 s16_LIM_MTR_ifMap18ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1400,(s16)1400,(s16)1400,(s16)1400,(s16)1400,(s16)1200,(s16)1100,(s16)1100,(s16)1100,(s16)1100,(s16)1100,(s16)1200,(s16)1400,(s16)1400,(s16)1400,(s16)1400,(s16)1400};	/* If指令値 [10mA]:400[mNm] */
const s16 s16_LIM_MTR_ifMap19ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300};	/* If指令値 [10mA]:500[mNm] */
const s16 s16_LIM_MTR_ifMap20ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1400,(s16)1500,(s16)1500,(s16)1500,(s16)1400,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300};	/* If指令値 [10mA]:600[mNm] */
const s16 s16_LIM_MTR_ifMap21ByTorqueYarray_10mA[(S16_CURRENT_MAP_POINT_NUM)]
	= {(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1400,(s16)1500,(s16)1500,(s16)1500,(s16)1400,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300,(s16)1300};	/* If指令値 [10mA]:1000[mNm] */


/* eof */
