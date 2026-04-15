/**
 * @file	PROPOTIONAL_GAIN_MAP.h
 * @brief	dqé≤â¬ïœî‰ó·ÉQÉCÉìÉ}ÉbÉvíËã`
 * @author	MCU GroupÅFAtsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2025.7
 */
#include "ctypedef.h"					/* CÉ^ÉCÉvïœêîíËã` */

	/*
		s16Ç»ÇÃÇ≈65535à»ì‡Ç…ó}Ç¶ÇÈÇ±Ç∆
		ImÇ…ä÷ÇµÇƒs16ÇæÇ∆ÉIÅ[ÉoÅ[ÉtÉçÅ[Ç∑ÇÈÇÃÇ≈É}ÉbÉvè„ÇÕ1/2ÇµÇΩílÇ∆Ç∑ÇÈ
	*/

#define S16_GAIN_MAP_POINT_NUM							(s16)6							/* ÉQÉCÉìÉ}ÉbÉv */

const s16 s16_MTR_gainMapByIfXarray_10mA[(S16_GAIN_MAP_POINT_NUM)];						/* Im[10mA] */

const s16 s16_MTR_dGainMap1ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* dé≤ÉQÉCÉì */
const s16 s16_MTR_dGainMap2ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* dé≤ÉQÉCÉì */
const s16 s16_MTR_dGainMap3ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* dé≤ÉQÉCÉì */
const s16 s16_MTR_dGainMap4ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* dé≤ÉQÉCÉì */
const s16 s16_MTR_dGainMap5ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* dé≤ÉQÉCÉì */
const s16 s16_MTR_dGainMap6ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* dé≤ÉQÉCÉì */
const s16 s16_MTR_dGainMap7ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* dé≤ÉQÉCÉì */
const s16 s16_MTR_dGainMap8ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* dé≤ÉQÉCÉì */

const s16 s16_MTR_qGainMap1ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* qé≤ÉQÉCÉì */
const s16 s16_MTR_qGainMap2ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* qé≤ÉQÉCÉì */
const s16 s16_MTR_qGainMap3ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* qé≤ÉQÉCÉì */
const s16 s16_MTR_qGainMap4ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* qé≤ÉQÉCÉì */
const s16 s16_MTR_qGainMap5ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* qé≤ÉQÉCÉì */
const s16 s16_MTR_qGainMap6ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* qé≤ÉQÉCÉì */
const s16 s16_MTR_qGainMap7ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* qé≤ÉQÉCÉì */
const s16 s16_MTR_qGainMap8ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]; 						/* qé≤ÉQÉCÉì */




#define U8_GAIN_MAP_IF_POINT_NUM						(u8)8							/* ÉQÉCÉìÉ}ÉbÉv Ifé≤êî */

/*	dé≤ÉQÉCÉì 3D map */
const struct ST_CMF_3D_MAP	d_Gainmap[(U8_GAIN_MAP_IF_POINT_NUM)] = {
		{0,		s16_MTR_gainMapByIfXarray_10mA, s16_MTR_dGainMap1ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{300,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_dGainMap2ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{600,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_dGainMap3ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{900,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_dGainMap4ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{1200,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_dGainMap5ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{1500,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_dGainMap6ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{1800,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_dGainMap7ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{10000,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_dGainMap8ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
	};

const s16 s16_MTR_gainMapByIfXarray_10mA[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)0,(s16)6250,(s16)12500,(s16)18500,(s16)24750,(s16)32767};				/* Im[10mA] */

const s16 s16_MTR_dGainMap1ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)40,(s16)49,(s16)44,(s16)78,(s16)34,(s16)34};							/* dé≤ÉQÉCÉì:If=0[10mA] */
const s16 s16_MTR_dGainMap2ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)40,(s16)48,(s16)44,(s16)77,(s16)33,(s16)33};							/* dé≤ÉQÉCÉì:If=300[10mA] */
const s16 s16_MTR_dGainMap3ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)40,(s16)43,(s16)39,(s16)36,(s16)32,(s16)32};							/* dé≤ÉQÉCÉì:If=600[10mA] */
const s16 s16_MTR_dGainMap4ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)40,(s16)33,(s16)33,(s16)32,(s16)30,(s16)30};							/* dé≤ÉQÉCÉì:If=900[10mA] */
const s16 s16_MTR_dGainMap5ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)40,(s16)21,(s16)24,(s16)26,(s16)26,(s16)26};							/* dé≤ÉQÉCÉì:If=1200[10mA] */
const s16 s16_MTR_dGainMap6ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)40,(s16)16,(s16)18,(s16)21,(s16)22,(s16)22};							/* dé≤ÉQÉCÉì:If=1500[10mA] */
const s16 s16_MTR_dGainMap7ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)40,(s16)14,(s16)14,(s16)16,(s16)18,(s16)18};							/* dé≤ÉQÉCÉì:If=1800[10mA] */
const s16 s16_MTR_dGainMap8ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)40,(s16)14,(s16)14,(s16)16,(s16)18,(s16)18};							/* dé≤ÉQÉCÉì:If=10000[10mA] */


/* qé≤ÉQÉCÉì 3D map */
const struct ST_CMF_3D_MAP	q_Gainmap[(U8_GAIN_MAP_IF_POINT_NUM)] = {
		{0,		s16_MTR_gainMapByIfXarray_10mA, s16_MTR_qGainMap1ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{300,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_qGainMap2ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{600,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_qGainMap3ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{900,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_qGainMap4ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{1200,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_qGainMap5ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{1500,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_qGainMap6ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{1800,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_qGainMap7ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
		{10000,	s16_MTR_gainMapByIfXarray_10mA, s16_MTR_qGainMap8ByIfYarray, (S16_GAIN_MAP_POINT_NUM) },
	};

const s16 s16_MTR_qGainMap1ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)30,(s16)27,(s16)24,(s16)19,(s16)17,(s16)17};							/* qé≤ÉQÉCÉì:If=00[10mA] */
const s16 s16_MTR_qGainMap2ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)30,(s16)26,(s16)23,(s16)20,(s16)17,(s16)17};							/* qé≤ÉQÉCÉì:If=300[10mA] */
const s16 s16_MTR_qGainMap3ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)30,(s16)24,(s16)22,(s16)20,(s16)17,(s16)17};							/* qé≤ÉQÉCÉì:If=600[10mA] */
const s16 s16_MTR_qGainMap4ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)30,(s16)21,(s16)20,(s16)19,(s16)17,(s16)17};							/* qé≤ÉQÉCÉì:If=900[10mA] */
const s16 s16_MTR_qGainMap5ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)30,(s16)18,(s16)18,(s16)18,(s16)17,(s16)17};							/* qé≤ÉQÉCÉì:If=1200[10mA] */
const s16 s16_MTR_qGainMap6ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)30,(s16)15,(s16)16,(s16)16,(s16)16,(s16)16};							/* qé≤ÉQÉCÉì:If=1500[10mA] */
const s16 s16_MTR_qGainMap7ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)30,(s16)14,(s16)14,(s16)15,(s16)10,(s16)10};							/* qé≤ÉQÉCÉì:If=1800[10mA] */
const s16 s16_MTR_qGainMap8ByIfYarray[(S16_GAIN_MAP_POINT_NUM)]
	= {(s16)30,(s16)14,(s16)14,(s16)15,(s16)10,(s16)10};							/* qé≤ÉQÉCÉì:If=10000[10mA] */





/* eof */
