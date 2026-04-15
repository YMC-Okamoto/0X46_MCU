/*==========================================================================*/
/**
 * @file	ADC.h
 * @brief	AD取得値の平均演算処理ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __ADC_H
#define __ADC_H
/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"
#include "TYPEDEF_ADC.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void V_ADC_Loop500us(void);							/* AD取得値の平均演算処理 呼び出し周期：0.5msewc */
extern void	V_ADC_InitializeForRestart(void);				/* AD変換変数初期化処理(再起動用) */
extern u16	U16_ADC_GetAdValueAveMotThermistor(void);		/* モータサーミスタ ＡＤ値提供(フィルタ値) */
extern u16	U16_ADC_GetAdValueAveFetThermistor(void);		/* FETサーミスタ ＡＤ値提供(フィルタ値) */
extern u16	U16_ADC_GetAdValueAveCapThermistor(void);		/* コンデンササーミスタ ＡＤ値提供(フィルタ値) */
extern u16	U16_ADC_GetAdValueAveCtUPhase(void);			/* U相CT ＡＤ値提供(フィルタ値) */
extern u16	U16_ADC_GetAdValueAveCtVPhase(void);			/* V相CT ＡＤ値提供(フィルタ値) */
extern u16	U16_ADC_GetAdValueAveAccel(void);				/* アクセル電圧 ＡＤ値提供(フィルタ値) */
extern u16	U16_ADC_GetAdValueAve12V(void);					/* 12V監視電圧 ＡＤ値提供(フィルタ値) */
extern s16	S16_ADC_GetMotorTemperature_0p1degC(void);		/* モータ温度提供(フィルタ値) */
extern s16	S16_ADC_GetFetTemperature_0p1degC(void);		/* FET温度提供(フィルタ値) */
extern s16	S16_ADC_GetCapTemperature_0p1degC(void);		/* コンデンサ温度提供(フィルタ値) */
extern s16	S16_ADC_GetMaxTemperature_0p1degC(void);		/* 最大温度提供 */
extern s16	S16_ADC_GetMinTemperature_0p1degC(void);		/* 最低温度提供 */
extern u16	U16_ADC_GetAccelRate(void);						/* アクセル開度率提供 */
extern u16	U16_ADC_GetAccelVoltageAve_10mV(void);			/* アクセル電圧提供 */
extern ST_ADC_DQ			St_ADC_GetDqAve(void);			/* DQ軸平均データ提供 */
extern ST_ADC_DQ32			St_ADC_GetDq32Ave(void);		/* DQ軸平均データ提供 */
extern ST_ADC_ADVALUE0_AVE	St_ADC_GetAdValue0Ave(void);	/* AD値平均化データ提供 */
extern ST_ADC_ADVALUE1_AVE	St_ADC_GetAdValue1Ave(void);	/* AD値平均化データ提供 */
extern ST_ADC_ADVALUE2_AVE	St_ADC_GetAdValue2Ave(void);	/* AD値平均化データ提供 */
extern ST_ADC_ADVALUE_AVE	St_ADC_GetAdValueAve(void);		/* AD値平均化データ提供 */

#endif /* __ADC_H */
