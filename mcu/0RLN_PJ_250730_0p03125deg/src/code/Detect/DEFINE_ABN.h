#ifndef __DEFINE_ABN_H
#define __DEFINE_ABN_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"					/* 型宣言 */
#include "DEFINE_SETTING.h"

/* Global define ------------------------------------------------------------*/
#define	U16_THERM_ADC_RANGE_MIN		  			(u16)64																			/* サーミスタAD断線下限閾値	0.084[V] = 84[10mV]相当 */
#define	U16_THERM_ADC_RANGE_MAX					(u16)4000																		/* サーミスタAD断線上限閾値	4.9[V] = 4900[10mV]相当 */
#define	U16_THERM_ADC_RELEASE_RANGE_MIN			(u16)112																		/* サーミスタAD断線解除下限閾値	*/
#define	U16_THERM_ADC_RELEASE_RANGE_MAX			(u16)3968																		/* サーミスタAD断線解除上限閾値	*/
#define	U16_MTR_THERM_ADC_RELEASE_RANGE_MIN		(u16)37																			/* モータサーミスタAD断線解除下限閾値 */
#define	U16_MTR_THERM_ADC_RELEASE_RANGE_MAX 	(u16)3849																		/* モータサーミスタAD断線解除上限閾値 */
#define	U16_MTR_THERM_ADC_RANGE_MIN				(u16)((u32)21 * (u32)(U16_SYS_AD_INPUT_MAX) / (u32)(U16_SYS_AD_MAX_10MV))		/* モータサーミスタAD断線下限閾値	0.21[V] = 21[10mV]相当 */
#define	U16_MTR_THERM_ADC_RANGE_MAX				(u16)((u32)495 * (u32)(U16_SYS_AD_INPUT_MAX) / (u32)(U16_SYS_AD_MAX_10MV))		/* モータサーミスタAD断線上限閾値	4.95[V] = 495[10mV]相当 */

/* アクセル異常閾値 */
#define	U16_ACCEL_ADC_RANGE_AT_BOOT				(u16)((u32)100 * (u32)(U16_SYS_AD_INPUT_MAX) / (u32)(U16_SYS_AD_MAX_10MV))		/* アクセル 下貼り付きAD閾値 1[V] */
#define	U16_ACCEL_ADC_RANGE_LOWER				(u16)((u32)50 * (u32)(U16_SYS_AD_INPUT_MAX) / (u32)(U16_SYS_AD_MAX_10MV))		/* アクセル 下貼り付きAD閾値 0.5[V] */
#define	U16_ACCEL_ADC_RANGE_UPPER				(u16)((u32)450 * (u32)(U16_SYS_AD_INPUT_MAX) / (u32)(U16_SYS_AD_MAX_10MV))		/* アクセル 上貼り付きAD閾値	4.5[V] */

/*** CTオフセット異常 ***/
#define	U16_MTR_CT_OFFSET_MARGIN				(u16)61																			/* 電流センサオフセットAD値マージン */
#define	U16_DET_MTR_CT_OFFSET_UPPER				(u16)((U16_MTR_CT_OFFSET_STANDARD_ADVALUE) + (u16)(U16_MTR_CT_OFFSET_MARGIN))	/* 電流センサオフセット閾AD値（上限）+0.07V相当暫定 */
#define	U16_DET_MTR_CT_OFFSET_LOWER				(u16)((U16_MTR_CT_OFFSET_STANDARD_ADVALUE) - (u16)(U16_MTR_CT_OFFSET_MARGIN))	/* 電流センサオフセット閾AD値（下限）-0.07V相当暫定 */

/*** リレー動作異常 ***/
#define	U16_ABN_BATT_VOLT_DIFF_10MV				(u16)500																		/* リレー動作異常 電圧差閾値 */

/* 異常判定時間 */
#define	U8_ABN_UPPER_LIM_COMPARISON				(u8)0					/* 異常閾値を超えると異常になる */
#define U8_ABN_LOWER_LIM_COMPARISON				(u8)1					/* 異常閾値を下回ると異常になる */

#define	U16_ABN_ABNORMAL_COUNT_30MS				(u16)3					/* 警告検出連続時間	10×3=30ms */
#define	U16_ABN_ABNORMAL_COUNT_50MS				(u16)5					/* 警告検出連続時間	10×5=50ms */
#define	U16_ABN_ABNORMAL_COUNT_100MS			(u16)10					/* 警告検出連続時間	10×10=100ms */
#define	U16_ABN_ABNORMAL_COUNT_250MS			(u16)25					/* 異常検出連続時間	25×10=250ms */
#define	U16_ABN_ABNORMAL_COUNT_270MS			(u16)27					/* 異常検出連続時間	27×10=270ms */
#define	U16_ABN_ABNORMAL_COUNT_500MS			(u16)50					/* 異常検出連続時間	50×10=500ms */
#define	U16_ABN_ABNORMAL_COUNT_750MS			(u16)75					/* 異常検出連続時間	75×10=750ms */
#define	U16_ABN_ABNORMAL_COUNT_1S				(u16)100				/* 異常検出連続時間	100×10=1s */
#define	U16_ABN_ABNORMAL_COUNT_2S				(u16)200				/* 異常検出連続時間	200×10=2s */
#define	U16_ABN_ABNORMAL_COUNT_3S				(u16)300				/* 異常検出連続時間	300×10=3s */
#define	U16_ABN_ABNORMAL_COUNT_4S				(u16)400				/* 異常検出連続時間	400×10=4s */
#define	U16_ABN_ABNORMAL_COUNT_10S				(u16)1000				/* 異常検出連続時間	1000×10=10s */
#define	U16_ABN_ABNORMAL_COUNT_60S				(u16)6000				/* 異常検出連続時間	6000×10=60s */

/*--- ダイアグNo定義 ---*/
#define	U8_DIAG_NUM_MTR_OVER_CUR				(u8)2					/* モータ過電流異常 */
#define	U8_DIAG_NUM_ENC_ABN						(u8)3					/* エンコーダ異常 */
#define	U8_DIAG_NUM_CT_UOFFSET					(u8)4					/* U相電流 オフセット異常ダイアグコード */
#define	U8_DIAG_NUM_CT_VOFFSET					(u8)5					/* V相電流 オフセット異常ダイアグコード */

#define	U8_DIAG_NUM_PRECHARGE_ERR				(u8)6					/* プリチャージ異常ダイアグコード */

#define	U8_DIAG_NUM_FET_THERMISTOR_OPEN			(u8)7					/* FETサーミスタ断線異常ダイアグコード */
#define	U8_DIAG_NUM_CAP_THERMISTOR_OPEN			(u8)8					/* キャパシタサーミスタ断線異常ダイアグコード */

//#ifdef	MOTOR_THERMISTOR_ACTIVITY
//#define	U8_DIAG_NUM_MOT_THERMISTOR_OPEN			(u8)1					/* モータサーミスタ断線異常ダイアグコード */
//#endif

#define	U8_DIAG_NUM_RELAY_ERR					(u8)1					/* メインリレー駆動異常ダイアグコード */

#define	U8_DIAG_NUM_ACCEL_AT_BOOT				(u8)9					/* アクセル電圧逸脱異常ダイアグコード */
#define	U8_DIAG_NUM_ACCEL						(u8)10					/* アクセル電圧逸脱異常ダイアグコード */

#define	U8_DIAG_NUM_OVER_VOLT					(u8)11					/* 過電圧異常 */
#define	U8_DIAG_NUM_ABN_BATT_VOLT				(u8)12					/* 電池電圧差異常 */

#define	U8_DIAG_NUM_OVERHEAT_INV				(u8)13					/* インバータ基板過熱異常 */
#define	U8_DIAG_NUM_OVERHEAT_CAP				(u8)14					/* コンデンサ基板過熱異常 */

#ifdef	MOTOR_THERMISTOR_ACTIVITY
#define	U8_DIAG_NUM_OVERHEAT_MTR				(u8)15					/* モータ過熱異常 */
#endif

#define	U8_DIAG_NUM_LOW_TEMP_CAP				(u8)16					/* コンデンサ基板低温異常 */

#define	U8_DIAG_NUM_U_PHASE_WIRE_OPEN			(u8)17					/* U相モータ断線異常ダイアグコード */
#define	U8_DIAG_NUM_V_PHASE_WIRE_OPEN			(u8)18					/* V相モータ断線異常ダイアグコード */
#define	U8_DIAG_NUM_W_PHASE_WIRE_OPEN			(u8)19					/* W相モータ断線異常ダイアグコード */
#define	U8_DIAG_NUM_EX_OVER_CUR					(u8)20					/* エキサイタ過電流異常 */

#define	U8_DIAG_NUM_DISCHARGE_ERR				(u8)21					/* ディスチャージ異常ダイアグコード */


#ifdef	DETECT_OVERREVOLUTION_ACTIVITY
#define	U8_DIAG_NUM_OVER_REVOLUTION				(u8)22					/* 過回転異常ダイアグコード */
#endif

#endif		/* __DEFINE_ABN_H */
