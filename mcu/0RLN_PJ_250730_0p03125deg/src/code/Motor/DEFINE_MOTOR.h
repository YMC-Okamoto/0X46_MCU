#ifndef __DEFINE_MOTOR_H
#define	__DEFINE_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"							/* 型宣言 */
#include "DEFINE_SETTING.h"						/* IFDEF設定 */

/* Private define ------------------------------------------------------------*/

/*--------------------------------------------------------------*/
/*	PWMタイマ設定関連 システム初期化とタイマ割込両方のため共通化	*/
/*--------------------------------------------------------------*/
/*           PWM_NP              PWM_NP					*/		/*電圧振幅*/
/*0-----------------PWM_NP----------------TIMER_MAX		*/		/*レジスタセット値*/
/*						：上ゲート	下ゲート	*/
/*	0					：ベタＨ	ベタＬ		*/
/*	1			DT		：ＳＷ		ベタＬ		*/
/*	DT			Max-DT	：ＳＷ		ＳＷ		*/
/*	Max-DT+1	Max-1	：ベタＬ	ＳＷ		*/
/*	Max					：ベタＬ	ベタＨ		*/
/*  以上、DTは設定デッドタイマ値、MaxはPWMタイマ最大値*/

#define U16_MTU3_PWM_CLK_MHZ					(u16)(U16_CPU_CLK0_MHZ) / (u16)4					/* MTU3,4のクロック周波数は40MHz */
#define U16_MTR_PWM_FREQ_KHZ					(u16)16												/* PWM周波数：16KHz */
#define	U16_MTR_DEAD_TIME						(u16)(u16)( (u16)15 * (U16_MTU3_PWM_CLK_MHZ) / (u16)10 + (u16)1 ) /* デッドタイム定義 1.5us μsとMHzで10^-6と10^6で相殺 */
#define	U16_MTR_PWM_CYCLE						(u16)( (u16)((U16_MTU3_PWM_CLK_MHZ) * (u16)1000 * (u16)10 / (u16)(U16_MTR_PWM_FREQ_KHZ) + (u16)5) / (u16)10 )			/* キャリア周波数は16KHz 四捨五入の為(×10+5)/10 */
#define	U16_MTR_PWM_HALF						(u16)( ((u16)(U16_MTR_PWM_CYCLE) - (u16)1) / (u16)2 )	/* PWM周期の半分タイマ値 */

#define U16_MTR_PWM_TIMER_MAX					(u16)(U16_MTR_PWM_HALF + U16_MTR_DEAD_TIME)	/* PWMタイマ設定の最大値(duty比0％) */
#define S16_MTR_PWM_TIMER_MAX					(s16)(U16_MTR_PWM_TIMER_MAX)			/* PWMタイマ設定の最大値(duty比0％) */
#define U16_MTR_PWM_TIMER_MIN					(u16)0									/* PWMタイマ設定の最大値(duty比100％) */
#define	U16_MTR_PWM_NEUTRAL_POINT				(u16)( ((U16_MTR_PWM_TIMER_MAX) - (U16_MTR_PWM_TIMER_MIN) + (u16)1) / (u16)2 )		/*タイマのカウンタ設定値 四捨五入の為(÷2の前に+1) */

#define	U16_MTR_PWM_VM_100PER					(u16)(U16_MTR_PWM_NEUTRAL_POINT)										/* 電圧基準振幅 */
#define	S16_MTR_PWM_VM_100PER_NEG				(s16)(-((U16_MTR_PWM_TIMER_MAX) - (U16_MTR_PWM_NEUTRAL_POINT)) )		/* 電圧基準振幅 */

#define	U16_MTR_OVER_MODULATION_COEF			(u16)9459												/* 過変調係数 2/√3 * 8192 (後で13bit右シフトする) */
/* Vd最大電圧 U16_MTR_PWM_VM_100PER×2/√3＝U16_MTR_PWM_VM_100PER×1.1547（基準電圧 × 115.47%) 四捨五入の為(÷10000の前に+5000) */
#define	U16_MTR_VD_MAX							(u16)(u32)( ((u32)(U16_MTR_PWM_VM_100PER) * (u32)11547 + (u32)5000) / (u32)10000 )
#define U16_MTR_VQ_MAX							(u16)(u32)((u32)(U16_MTR_VD_MAX) * (u32)105 / (u32)100)

/*************	界磁制御用	*************/
#define U16_MTU6_PWM_CLK_MHZ					(u16)(U16_CPU_CLK0_MHZ) / (u16)4				/* MTU6,7のクロック周波数は40MHz */
#define U16_EX_PWM_FREQ_KHZ						(u16)20											/* PWM周波数：20KHz */
#define	U16_EX_PWM_CYCLE						(u16)( (u16)((U16_MTU6_PWM_CLK_MHZ) * (u16)1000 * (u16)10 / (u16)(U16_EX_PWM_FREQ_KHZ) + (u16)5) / (u16)10 )			/* キャリア周波数は20KHzである 四捨五入の為(×10+5)/10 */
#define	U16_EX_PWM_HALF							(u16)( ((u16)(U16_EX_PWM_CYCLE) - (u16)1) / (u16)2 )	/* PWM周期の半分タイマ値 */

#define U16_EX_PWM_TIMER_MAX					(u16)(U16_EX_PWM_HALF)					/* PWMタイマ設定の最大値(duty比0％) */
#define S16_EX_PWM_TIMER_MAX					(s16)(U16_EX_PWM_TIMER_MAX)				/* PWMタイマ設定の最大値(duty比0％) */
#define U16_EX_PWM_TIMER_MIN					(u16)0									/* PWMタイマ設定の最大値(duty比100％) */

#define	U16_EX_PWM_VM_100PER					(u16)( ((U16_EX_PWM_TIMER_MAX) - (U16_EX_PWM_TIMER_MIN) + (u16)1) / (u16)2 )		/*タイマのカウンタ設定値 四捨五入の為(÷2の前に+1) */

/* Vf最大電圧 */
#define	U16_EX_VF_MAX							(u16)(U16_EX_PWM_VM_100PER)

#define	S16_EX_IF_RANGE_MAX_10MA				(s16)2500					/* If用シャント最大レンジ電流値[10mA] */
/* for DEBUG */
#if defined		(DEBUG_MAX_TORQUE_5NM)
#define	S16_EX_IF_MAX_10MA						(s16)500					/* If指令最大値[10mA] */
#elif defined	(DEBUG_MAX_TORQUE_10NM)
#define	S16_EX_IF_MAX_10MA						(s16)700					/* If指令最大値[10mA] */
#else
#define	S16_EX_IF_MAX_10MA						(s16)1500					/* If指令最大値[10mA] */
#endif
#define	S16_EX_IF_MAX2_10MA						(s16)1000					/* If指令最大値[10mA] */

#define	U8_MTR_NUM_ENCODER_PATTERNS				(u8)6						/* エンコーダパターン数 */

#define	U16_MTR_POLEPAIRS						(u16)4						/* 極対数 */
/**
 *	位相角オフセット(1度単位)
 *	(正数を入れた分、電流が進む)
 **/
#define	U16_MTR_EANGLE_OFFSET_0P03125DEG		(u16)((U16_MTR_EANGLE_OFFSET_DEG) << (u16)5)	/* 位相角オフセット(0.03125度単位) */
#define	U16_MTR_EANGLE_OFFSET_DEG				(u16)(360-9-3)				/* 位相角オフセット(1度単位)：-9-3[deg] */
//#define	U16_MTR_EANGLE_OFFSET_DEG				(u16)351					/* 位相角オフセット(1度単位)：-9[deg] */
#define	U16_MTR_EANGLE_LOWSPD_OFFSET_0P03125DEG	(u16)0						/* 位相角オフセット(0.03125度単位) 0deg */
#define	U16_MTR_EANGLE_LOWSPD_OFFSET_DEG		(u16)0						/* 位相角オフセット(1度単位) 0deg */

/*--- 漸増漸減(10msec周期用) ----*/
#define	S16_MTR_GATEON_IQREF_100MA				(s16)20						/* ゲートオン電流指令値 [100mA] 暫定値 */
#define	U16_MTR_IQREF_INC_HIGH_100MA			(u16)200					/* 漸増漸減1ステップ分[100mA] */
#define	U16_MTR_IQREF_INC_LOW_100MA				(u16)100					/* 漸増漸減1ステップ分[100mA] */

#define	U16_MTR_IQREF_DEC_HIGH_100MA			(u16)400					/* 漸増漸減1ステップ分[100mA] */
#define	U16_MTR_IQREF_DEC_MIDDLE_100MA			(u16)60						/* 漸増漸減1ステップ分[100mA] */
#define	U16_MTR_IQREF_DEC_LOW_100MA				(u16)40						/* 漸増漸減1ステップ分[100mA] */

#define	U16_MTR_STOPPED_IQFB_10MA				(u16)100					/* 停止と判定するIqFb	1A */
#define	U16_MTR_STOPPED_IQFB_100MA				(u16)10						/* 停止と判定するIqFb	1A */
#define	U16_MTR_STOPPED_IFFB_10MA				(u16)50						/* 停止と判定するIfFb	0.5A */

#define	S16_MTR_GATEON_IQREF_10MA				(s16)200					/* ゲートオン電流指令値 [10mA] 暫定値 */
#define	U16_MTR_IQREF_INC_HIGH_10MA				(u16)2000					/* 漸増漸減1ステップ分[10mA] */
#define	U16_MTR_IQREF_INC_LOW_10MA				(u16)1000					/* 漸増漸減1ステップ分[10mA] */

#define	U16_MTR_IQREF_DEC_HIGH_10MA				(u16)4000					/* 漸増漸減1ステップ分[10mA] */
#define	U16_MTR_IQREF_DEC_MIDDLE_10MA			(u16)600					/* 漸増漸減1ステップ分[10mA] */
#define	U16_MTR_IQREF_DEC_LOW_10MA				(u16)400					/* 漸増漸減1ステップ分[10mA] */

/*
	W = Vq * Iq * (3/2) * (√3/2) * (1/√3)	※ｄ軸も同様
	  = Vq * Iq * 3/4

	相対変換係数			：3/2
	線電流相電流変換係数	：1/√3
	過変調分				：√3/2
	効率					：0.9（仮）
	最終係数				：3/4 * 0.9 * 1024 ≒ 691 (後で/1024：実際には10bit右シフト)
*/
#define	U16_MTR_WATT_CALC_COEF					(u16)853					/* W制限用係数 3/4 / 0.9	= 0.833..  → 3/4 * 1024 / 0.9[inv効率]		≒ 853	(1024を掛けて最後に10bit右シフトする) */

#define	U16_MTR_LOW_SPD_REVOLUTION_RPM			(u16)55						/* 低速と判定するモータ回転数 */
#define	U16_MTR_STOPPED_REVOLUTION_RPM			(u16)10						/* 停止と判定するモータ回転数 */

#define	U16_MTR_OVERMODULATION_COEF				(u16)3547					/* 2/√3:過変調 * 3/2:相対変換係数 * 2048 = 3547 (後で11ビット右シフトする) */
//#define	U16_MTR_OVERMODULATION_COEF				(u16)2365					/* 2/√3:過変調 * 2048 = 2365 (後で11ビット右シフトする) */

#define	S16_MTR_IQ_MAX_100MA					(s16)5000					/* Iq指令最大値[100mA] */

#define	S16_MTR_CORRECT_IQ_MAX_100MA			(s16)3000					/* Iq指令最大値[100mA] */

#define	S16_MTR_IM_MAX_100MA					(S16_MTR_IQ_MAX_100MA)		/* Im指令最大値[100mA] */

#define	S16_MTR_IQ_0A_100MA						(s16)0						/* Iq力行指令最小値0[100mA] */

#if defined		(DEBUG_MAX_TORQUE_5NM)
#define	S32_MTR_IQ_MAX_10MA						(s32)1000					/* Iq指令最大値[10mA] */
#elif defined	(DEBUG_MAX_TORQUE_10NM)
#define	S32_MTR_IQ_MAX_10MA						(s32)5000					/* Iq指令最大値[10mA] */
#else
#define	S32_MTR_IQ_MAX_10MA						(s32)50000					/* Iq指令最大値[10mA] */
#endif
#define	S32_MTR_IQ_MAX2_10MA					(s32)10000					/* Iq指令最大値[10mA] */

#define	S32_MTR_CORRECT_IQ_MAX_10MA				(s32)30000					/* Iq指令最大値[10mA] */

#define	S32_MTR_IM_MAX_10MA						(S32_MTR_IQ_MAX_10MA)		/* Im指令最大値[10mA] */

#define	S32_MTR_IQ_0A_10MA						(s32)0						/* Iq力行指令最小値0[10mA] */


#define	U16_MTR_ACCEL_RATE_MIN					(u16)0						/* アクセル開度率の最小値(0%) */
#define	U16_MTR_ACCEL_RATE_MAX					(u16)1024					/* アクセル開度率の最大値(100%=1024) */

/*--- Id指令値 ---*/
#define	S16_MTR_ID_MAX_100MA					(s16)0						/* Id指令最大値0[100mA] */
#define	S16_MTR_ID_MIN_100MA					(s16)0						/* Id指令最小値0[100mA] */

#define	S32_MTR_ID_MAX_10MA						(s32)0						/* Id指令最大値0[10mA] */
#define	S32_MTR_ID_MIN_10MA						(s32)0						/* Id指令最小値0[10mA] */

#define	U16_MTR_CT_RANGE_MAX_100MA				(u16)7500					/* CT最大レンジ電流[100mA] */
#define	U32_MTR_CT_RANGE_MAX_10MA				(u32)75000					/* CT最大レンジ電流[10mA] */
#define	U16_MTR_CT_OFFSET_STANDARD_ADVALUE		(u16)2048					/* モータ電流オフセット基準値（AD値）(2.5V⇒2048LSB) */

#define	U16_EX_IF_OFFSET_STANDARD_ADVALUE		(u16)2048					/* 2.5V = 2048 界磁電流オフセット基準値（AD値) */

/*--- 回生指令値 ---*/
#define	S16_MTR_NO_REGENE_IQ_100MA				(s16)0						/* 回生指令最大値 0[100mA] */

//#define	U16_MTR_REGENE_END_BATT_VOLT_100MV		(u16)580					/* 回生充電終了電圧[100mV] */
//#define	U16_MTR_REGENE_DEC_BATT_VOLT_100MV		(u16)570					/* 回生制限開始電圧[100mV] */

#if defined		(DEBUG_MAX_TORQUE_5NM)
#define	U16_MTR_TORQUE_MAX_100MNM				(u16)50						/* デバッグ時最大トルク指令値[100mNm] */
#elif defined	(DEBUG_MAX_TORQUE_10NM)
#define	U16_MTR_TORQUE_MAX_100MNM				(u16)100					/* デバッグ時最大トルク指令値[100mNm] */
#else
#define	U16_MTR_TORQUE_MAX_100MNM				(u16)600					/* 最大トルク指令値[100mNm] */
#endif
/*--------------------------------------------------------------*/
/*	エンコーダより速度や電気補正角度を求める関連常数設定	*/
/*--------------------------------------------------------------*/
#define	U16_MTR_REVOLUTION_MAX_RPM				(u16)10000					/* 最大速度に対応する回転数 単位:rpm */
#define	S16_MTR_REVOLUTION_MAX_RPM				(s16)(U16_MTR_REVOLUTION_MAX_RPM)	/* 最大速度に対応する回転数 単位:rpm */
/* 360deg */
#define	U16_ENC_CYCLE_MIN_TIMER_VALUE			(u16)((u32)(U16_CPU_CLK0_MHZ) * (u32)1000000 / (u32)((u32)(U16_MTR_REVOLUTION_MAX_RPM) * (u32)(U16_MTR_POLEPAIRS) * (u32)(U16_CPU_MTU0_CLK_SCALE) / (u32)60))	/* 最大速度に対応するU相エンコーダの周期カウンタ値 */

#define	U16_ENC_EANGLE_MAX_0P03125DEG			(u16)((U16_ENC_EANGLE_MAX_DEG) << (u16)5)	/* エンコーダ最大電気角 */
#define	U16_ENC_EANGLE60_MAX_0P03125DEG			(u16)((U16_ENC_EANGLE60_MAX_DEG) << (u16)5)	/* エンコーダ最大電気角 */
#define	U16_ENC_EANGLE_MAX_DEG					(u16)360							/* エンコーダ最大電気角 */
#define	U16_ENC_EANGLE60_MAX_DEG				(u16)60								/* エンコーダ最大電気角 */
#define	U32_ENC_EANGLE360_MAX					(u32)((u32)360 << (u32)8)			/* エンコーダ最大電気角(×256) */
#define	U32_ENC_EANGLE60_MAX					(u32)((u32)60 << (u32)8)			/* エンコーダ最大電気角(×256) */
#define	U32_ENC_TIMER_RESOLUTON_HZ				(u32)((u32)(U16_CPU_CLK0_MHZ) * (u32)1000000  * (u32)2 / (u32)(U16_CPU_MTU0_CLK_SCALE))		/* エンコーダタイマ周波数 160[MHz] / 1024 = 156.25[kHz] = 156250[Hz] */
#define	U32_ENC_TIMER_RESOLUTON_60HZ			(u32)((u32)(U32_ENC_TIMER_RESOLUTON_HZ) * (u32)60)									/* エンコーダタイマ分解能 60[Hz] */
#define	U32_ENC_EANGLE360_TIMER_VALUE			(u32)((U32_ENC_EANGLE360_MAX) * (u32)1000 / (u32)(U16_MTR_PWM_FREQ_KHZ) * (u32)(U16_CPU_CLK0_MHZ) / (u32)(U16_CPU_MTU0_CLK_SCALE))	/* PWM周期(62.5us)のサンプリングタイムで電気角度360度に相当するタイマ値 360*256*62.5us*0.625us */
#define	U32_ENC_EANGLE60_TIMER_VALUE			(u32)((U32_ENC_EANGLE60_MAX) * (u32)1000 * (u32)2 / (u32)(U16_MTR_PWM_FREQ_KHZ) * (u32)(U16_CPU_CLK0_MHZ) / (u32)(U16_CPU_MTU0_CLK_SCALE))	/* PWM周期(62.5us)のサンプリングタイムで電気角度360度に相当するタイマ値 60*256*2*62.5us*0.625us *2は両エッジだから */

#define	U8_ENC_TIMEROVERFLOW_COUNT_MAX			(u8)5			/* ENCタイマオーバーフロー最大回数 */
/* 360deg */
#define	U16_ENC_EANGLE_ESTIMATE_UNIT_MAX_DEG	(u16)((U32_ENC_EANGLE360_TIMER_VALUE) / (U16_ENC_CYCLE_MIN_TIMER_VALUE))	/*最大単位補正電気角*/
#define	U32_ENC_MAX_TIMER_VALUE					(u32)(((u32)(U16_MAX) + (u32)1) * (u32)(U8_ENC_TIMEROVERFLOW_COUNT_MAX))		/*エンコーダタイマ最大数 ≒0.21s  0.625MHzの16bitタイマのオーバーフロー4回 */

#define	U16_MTR_OVER_REVOLUTION_RPM				(u16)200000																		/* モータ過回転判定値 [rpm] */
#define	U16_MTR_OVER_REVOLUTION_RELEASE_RPM		(u16)100000																		/* モータ過回転復帰値 [rpm] */
#define	S16_MTR_OVER_BACK_REVOLUTION_RPM		(s16)(-300)							/* 逆転時の異常回転数 */
#define	S16_MTR_OVER_BACK_REVOLUTION_RELEASE_RPM	(s16)(-250)						/* 逆転時の復帰回転数 */

/*
	PWMタイマ最大値：5rpmでのENC60度間の12kHzでのカウント数
	キャリア周波数[12] / 極対数[21] / ENCパターン数[6] / 5rpm回転数[5] * 60 * 1000 = 1142
*/
#define	U16_MTR_MAX_PWM_TIMER_VALUE				(u16)((u32)(U16_MTR_PWM_FREQ_KHZ) * (u32)60 * (u32)1000 / ((u32)(U16_MTR_POLEPAIRS) * (u32)10 * (u32)(U8_MTR_NUM_ENCODER_PATTERNS)))

//#define		U16_MTR_OVER_REVOLUTION_RPM				(u16)(100 * (EQ_MAX_MOT_REV/EQ_MAX_SPEED_KMPH))	/* 回転数超過判定値	[km/h] naka */

/*-------------------------------------------------------*/
/*	制御ゲイン	*/
/*-------------------------------------------------------*/
/* 旧モータ */
//#define	U16_MTR_DAXIS_KP						(u16)4									/* 電流FB比例定数 */
//#define	U16_MTR_DAXIS_KI						(u16)15									/* 電流FB積分定数 */
//#define	U16_MTR_QAXIS_KP						(u16)7									/* 電流FB比例定数 */
//#define	U16_MTR_QAXIS_KI						(u16)15									/* 電流FB積分定数 */
//#define	U16_EX_KP								(u16)2398								/* 電流FB比例定数 実測値で計算 3Ω 150mH */
//#define	U16_EX_KI								(u16)614								/* 電流FB積分定数 実測値で計算 3Ω 150mH */

/* 以下、2024モータ */
/* R：3.1256mΩ、Ld：0.85mH、Lq：0.69mH */
/* R：1.7Ω、L：115mH */
/* 新ゲイン（2025.7.2以降) */
#define	U16_MTR_DAXIS_KP						(u16)40							/* 電流FB比例定数 (×256) */
#define	U16_MTR_DAXIS_KI						(u16)50							/* 電流FB積分定数 (×65536) */
#define	U16_MTR_QAXIS_KP						(u16)30							/* 電流FB比例定数 (×256) */
#define	U16_MTR_QAXIS_KI						(u16)50							/* 電流FB積分定数 (×65536) */
#define	U16_EX_KP								(u16)655						/* 電流FB比例定数 (×256) */
#define	U16_EX_KI								(u16)140						/* 電流FB積分定数 (×65536) */

/* 旧ゲイン（2025.6.25) */
#define	U16_MTR_DAXIS_KP2						(u16)150						/* 電流FB比例定数 (×256) */
#define	U16_MTR_DAXIS_KI2						(u16)200						/* 電流FB積分定数 (×65536) */
#define	U16_MTR_QAXIS_KP2						(u16)85							/* 電流FB比例定数 (×256) */
#define	U16_MTR_QAXIS_KI2						(u16)200						/* 電流FB積分定数 (×65536) */
#define	U16_EX_KP2								(u16)200						/* 電流FB比例定数 (×256)	R,L 1.7Ω 100mH */
#define	U16_EX_KI2								(u16)100						/* 電流FB積分定数 (×65536)	R,L 1.7Ω 100mH */


#endif		/* __DEFINE_MOTOR_H */
