/**
 * @file TYPEDEF_ADC.h
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
#ifndef __TYPEDEF_ADC_H
#define __TYPEDEF_ADC_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"					/* 型宣言 */

/* External define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/
/* AD変換結果 */
#pragma bit_order right
#pragma unpack

typedef struct{
	u16 u16_adValueCtU;							/* Ｕ相電流ＡＤ(生) */
	u16 u16_adValueCtV;							/* Ｖ相電流ＡＤ(生) */
	u16 u16_adValueBattV;						/* バッテリ電圧ＡＤ(生) */
	u16 u16_adValueInvV;						/* インバータ電圧ＡＤ(生) */
}ST_ADC_ADVALUE0;

typedef struct{
	u16 u16_adValueVu;							/* Ｕ相電圧ＡＤ(生) */
	u16 u16_adValueVv;							/* Ｖ相電圧ＡＤ(生) */
	u16 u16_adValueVw;							/* Ｗ相電圧ＡＤ(生) */
	u16 u16_adValueIf;							/* 界磁電流ＡＤ(生) */ /* 扱いにくいので独立させる */
}ST_ADC_ADVALUE1;

typedef struct{
	u16 u16_adValue12V;							/* 12V監視電圧 ＡＤ(生) */
	u16 u16_adValueGp1;							/* 汎用1 ＡＤ(生) */
	u16 u16_adValueExt5V;						/* 外部5V監視電圧 ＡＤ(生) */
	u16 u16_adValueThermMtr;					/* サーミスタモータ ＡＤ(生) */
	u16 u16_adValueGp2;							/* 汎用2 ＡＤ(生) */
	u16 u16_adValueThermFet;					/* サーミスタＦＥＴ ＡＤ(生) */
	u16 u16_adValueThermCapa;					/* サーミスタコンデンサ基板 ＡＤ(生) */
	u16 u16_adValueAccel;						/* アクセル電圧 ＡＤ(生) */
}ST_ADC_ADVALUE2;

typedef struct{
	s16 s16_idFb_100mA;							/* d軸電流FB[100mA] */
	s16 s16_iqFb_100mA;							/* q軸電流FB[100mA] */
	s16 s16_vd_100mV;							/* d軸電圧[100mV] */
	s16 s16_vq_100mV;							/* q軸電圧[100mV] */
}ST_ADC_DQ;

typedef struct{
	s32 s32_idRef_10mA;							/* d軸電流指令値[10mA] */
	s32 s32_iqRef_10mA;							/* q軸電流指令値[10mA] */
	s32 s32_idFb_10mA;							/* d軸電流FB[10mA] */
	s32 s32_iqFb_10mA;							/* q軸電流FB[10mA] */
	s32 s32_vd_10mV;							/* d軸電圧[10mV] */
	s32 s32_vq_10mV;							/* q軸電圧[10mV] */
}ST_ADC_DQ32;

typedef struct{
	u16 u16_adValueAveCtU;						/* Ｕ相電流ＡＤ(フィルタ後) */
	u16 u16_adValueAveCtV;						/* Ｖ相電流ＡＤ(フィルタ後) */
	u16 u16_adValueAveBattV;					/* バッテリ電圧ＡＤ(フィルタ後) */
	u16 u16_adValueAveInvV;						/* インバータ電圧ＡＤ(フィルタ後) */
}ST_ADC_ADVALUE0_AVE;

typedef struct{
	u16 u16_adValueAveVU;						/* Ｕ相電圧ＡＤ(フィルタ後) */
	u16 u16_adValueAveVV;						/* Ｖ相電圧ＡＤ(フィルタ後) */
	u16 u16_adValueAveVW;						/* Ｗ相電圧ＡＤ(フィルタ後) */
	u16 u16_adValueIf;							/* 界磁電流ＡＤ(生) */
}ST_ADC_ADVALUE1_AVE;

typedef struct{
	u16 u16_adValueAve12V;						/* 12V監視電圧 ＡＤ(フィルタ後) */
	u16 u16_adValueAveGp1;						/* 汎用1 ＡＤ(フィルタ後) */
	u16 u16_adValueAveExt5V;					/* 外部5V監視電圧 ＡＤ(フィルタ後) */
	u16 u16_adValueAveThermMtr;					/* サーミスタモータ ＡＤ(フィルタ後) */
	u16 u16_adValueAveGp2;						/* 汎用2 ＡＤ(フィルタ後) */
	u16 u16_adValueAveThermFet;					/* サーミスタＦＥＴ ＡＤ(フィルタ後) */
	u16 u16_adValueAveThermCapa;					/* サーミスタコンデンサ基板 ＡＤ(フィルタ後) */
	u16 u16_adValueAveAccel;					/* アクセル電圧 ＡＤ(フィルタ後) */
}ST_ADC_ADVALUE2_AVE;

typedef struct{
	u16 u16_adValueAveCtU;						/* Ｕ相電流ＡＤ(フィルタ後) */
	u16 u16_adValueAveCtV;						/* Ｖ相電流ＡＤ(フィルタ後) */
	u16 u16_adValueAveIf;						/* 界磁電流ＡＤ(フィルタ後) */
}ST_ADC_ADVALUE_AVE;


/* external variables --------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/


#endif /* __TYPEDEF_ADC_H */
