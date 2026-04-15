#ifndef __DEFINE_SYSTEM_H
#define __DEFINE_SYSTEM_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"							/* 型宣言 */
#include "DEFINE_SETTING.h"						/* IFDEF設定 */

/* Global define ------------------------------------------------------------*/

/* ポート定義 */
/* MCU.hへ統合 */

/*----------SWのポート定義-----------------*/

/***********************************************************/
/*====== 常用マクロの定義 ======*/
/***********************************************************/
#define U16_CPU_CLK0_MHZ			(u16)160					/*システムクロック定義（初期設定より設定）*/
#define U16_CPU_MTU0_CLK_SCALE		(u16)256					/*タイマプリスケーラ（初期設定より設定）速度計算用*/

/*-------------------------------------------------------*/
/*	電圧仕様*/
/*-------------------------------------------------------*/
#define	U16_SYS_AD_INPUT_MAX						(u16)4096						/* AD変換最大値(12bit) */
#define	U16_SYS_AD_MAX_100MV						(u16)50							/* AD変換最大電圧値[100mV] */
#define	U16_SYS_AD_MAX_10MV							(u16)500						/* AD変換最大電圧値[10mV] */
#define	U16_SYS_BAT_VDC_MAX_100MV					(u16)950						/* バッテリ最大電圧[100mV] */
#define	U16_SYS_BAT_VDC_MAX_10MV					(u16)9500						/* バッテリ最大電圧[10mV] */
#define	U16_SYS_12V_MAX_10MV						(u16)1563						/* 12V監視最大電圧 15.625V[10mV] */
#define	U16_SYS_ACCEL_VOLT_MAX_10MV					(u16)500						/* アクセル最大電圧[10mV] */
#define	U16_SYS_12V_AD_STANDARD						(u16)681						/* 12V監視基準値(AD値) 14.3V * 1024 / 21.5V */

/*** 電圧閾値の設定 ***/
#define	U16_OVER_BATT_RELEASE_VOLT_100MV			(u16)550						/* 過電圧異常解除値*0.1[V] */
#define	U16_DISCHRGSTOP_DEC_BATT_VOLT_100MV			(u16)450						/* 放電制限開始電圧*0.1[V] */
#define	U16_UNDER_BATT_VOLT_100MV					(u16)430						/* 低電圧警告検出値*0.1[V] */

//#define	U16_ADC_REFERENCE_BATT_VOLT_10MV			(u16)4920						/* デフォルト電圧定義 */
#define	U16_ADC_REFERENCE_BATT_VOLT_10MV			(u16)6000						/* デフォルト電圧定義 */
#define	U16_DETECT_BATT_VOLT_DIFF_10MV				(u16)500						/* 電池電圧差異常検出値*10[mV] */
#define	U16_OVER_BATT_VOLT_10MV						(u16)6300						/* 過電圧異常検出値*10[mV] */
#define	U16_OVER_BATT_RELEASE_VOLT_10MV				(u16)5500						/* 過電圧異常解除値*10[mV] */
#define	U16_DISCHRGSTOP_DEC_BATT_VOLT_10MV			(u16)4500						/* 放電制限開始電圧*10[mV] */
#define	U16_UNDER_BATT_VOLT_10MV					(u16)4300						/* 低電圧警告検出値*10[mV] */
#define	U16_LOWVOLTAGE_RELEASE_BATT_VOLT_10MV		(u16)3700						/* 低電圧シャットダウン解除値[10mV] */
#define	U16_LOWVOLTAGE_SHUTDOWN_BATT_VOLT_10MV		(u16)3500						/* 低電圧シャットダウン検出値[10mV] */

/*** 電圧閾値の設定(AD値) ***/
#define	U16_REFERENCE_BATT_ADVALUE					(u16)((u32)((U16_ADC_REFERENCE_BATT_VOLT_10MV) * (u32)(U16_SYS_AD_INPUT_MAX) * (u32)10 + (u32)5) / (u32)((U16_SYS_BAT_VDC_MAX_10MV) * (u16)10))			/* デフォルト電圧定義(AD値) */

//#define	U16_OVER_BATT_ADVALUE						(u16)((u32)((U16_OVER_BATT_VOLT_100MV) * (u32)(U16_SYS_AD_INPUT_MAX) * (u32)10 + (u32)5) / (u32)((U16_SYS_BAT_VDC_MAX_100MV) * (u16)10))					/* 過電圧異常検出値(AD値) */
//#define	U16_DISCHRGSTOP_DEC_BATT_ADVALUE			(u16)((u32)((U16_DISCHRGSTOP_DEC_BATT_VOLT_100MV) * (u32)(U16_SYS_AD_INPUT_MAX) * (u32)10 + (u32)5) / (u32)((U16_SYS_BAT_VDC_MAX_100MV) * (u16)10))			/* 放電制限開始電圧(AD値) */
//#define	U16_UNDER_BATT_ADVALUE						(u16)((u32)((U16_UNDER_BATT_VOLT_100MV) * (u32)(U16_SYS_AD_INPUT_MAX) * (u32)10 + (u32)5) / (u32)((U16_SYS_BAT_VDC_MAX_100MV) * (u16)10))					/* 低電圧警告検出値(AD値) */
//#define	U16_LOWVOLTAGE_SHUTDOWN_BATT_ADVALUE		(u16)((u32)((U16_LOWVOLTAGE_SHUTDOWN_BATT_VOLT_100MV) * (u32)(U16_SYS_AD_INPUT_MAX) * (u32)10 + (u32)5) / (u32)((U16_SYS_BAT_VDC_MAX_100MV) * (u16)10))		/* 低電圧シャットダウン検出値(AD値) */

#define U16_IS_ON_VOLTAGE_100MV						(u16)350						/* SW ON 電圧閾値 */
#define U16_IS_ON_VOLTAGE_10MV						(u16)3500						/* SW ON 電圧閾値 */
#define U16_VOLTAGE_MIN_FOR_DQWAT					(u16)370						/* ワットマップ計算する最低電圧 */
#define U16_VOLTAGE_MIN_FOR_DQWAT_10MV				(u16)3700						/* ワットマップ計算する最低電圧 */

#define U16_SYS_ACCEL_RATE_MAX						(u16)1024						/* アクセル開度率 */
/*** プリチャージ関連 ***/
#define	U16_SYS_PRECHARGE_FIN_VOLTAGE_DIFF_100MV	(u16)30							/* プリチャージ完了電位差		70[100mV] */
#define	U16_SYS_VOLTAGE_DIFF_100MV					(u16)10							/* インバータ、バッテリ電位差	10[100mV] */
#define	U16_SYS_PRECHARGE_ON_VOLTAGE_DIFF_100MV		(u16)20							/* プリチャージON/OFFチャタリング防止差分電圧 20[100mV] */
#define	S16_SYS_PRECHARGE_ON_REV_DIFF_RPM			(s16)20							/* プリチャージON/OFFチャタリング防止差分回転数 20[rpm] */
#define	U16_SYS_DISCHARGE_FIN_VOLTAGE_100MV			(u16)50							/* ディスチャージ完了電圧[100mV] */

#define	U16_SYS_PRECHARGE_FIN_VOLTAGE_DIFF_10MV		(u16)300						/* プリチャージ完了電位差		300[10mV] */
#define	U16_SYS_VOLTAGE_DIFF_10MV					(u16)100						/* インバータ、バッテリ電位差	100[10mV] */
#define	U16_SYS_PRECHARGE_ON_VOLTAGE_DIFF_10MV		(u16)200						/* プリチャージON/OFFチャタリング防止差分電圧 200[10mV] */
#define	U16_SYS_DISCHARGE_FIN_VOLTAGE_10MV			(u16)500						/* ディスチャージ完了電圧[10mV] */

#define	U8_SYS_TIME_CONST_FACTOR					(u8)4							/* モータ回転数のLPF時定数係数 LPFの時定数4msに相当（ 4：4×1ms）*/

#endif		/* __DEFINE_SYSTEM_H */
