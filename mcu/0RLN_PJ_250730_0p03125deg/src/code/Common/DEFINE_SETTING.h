#ifndef __DEFINE_SETTING_H
#define __DEFINE_SETTING_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"							/* 型宣言 */
/* for DEBUG */
//	#define		EMERGENCY_STOP_ACTIVITY

//	#define		UNTI_WINDUP_ACTIVITY

	#define		DEBUG_WATCH_ACTIVITY

	#define		PI_GAIN_TYPE1						/* PIゲインタイプ1（計算値）

//	#define		DEBUG_MAX_TORQUE_5NM
//	#define		DEBUG_MAX_TORQUE_10NM

//	#define		VQ_CLIP_INACTIVITY					/* VQ CLIP無効化 */
//	#define		IQREF_BY_POTENTIOMETER				/* ポテンショによるIq指令値決定(Id=0) */

	#define		APS_GROUND_FAULT_INVALID			/* 天絡異常が有効、地絡異常は無効 */
	#define		APS_FAULT_INVALID					/* 天絡地絡異常無効 */
	#define		DETECT_ACCEL_ACTIVITY				/* アクセル異常有効 */
//	#define		MOTOR_THERMISTOR_ACTIVITY			/* モータサーミスタ有効 */
//	#define		DETECT_OVERREVOLUTION_ACTIVITY		/* モータ過回転異常有効 */
/* for Exciter DEBUG */
	#define		DA_EANGLE
	#define		CUW_ACTIVITY						/* Cuw有効 */
//	#define		VCU_ERROR_ACTIVITY					/* VCU異常有効 */
//	#define		TRQ_GRADUALLY_ACTIVITY				/* 漸増漸減有効 */
//	#define		CAN_OPEN_NON_RETURN					/* CAN途絶非復帰 */

#endif		/* __DEFINE_SETTING_H */
