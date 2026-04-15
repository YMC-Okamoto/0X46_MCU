/**
 * @file TYPEDEF_MODE.h
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */

#ifndef __TYPEDEF_MODE_H
#define __TYPEDEF_MODE_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"					/* 型宣言 */

/* External define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/
typedef enum{
	MODE_STARTUP = 0,
	MODE_STANDBY,
	MODE_RUN,
	MODE_ABNORMAL,
	MODE_IDLE,
	MODE_SHUTDOWN_WAIT,
	MODE_SHUTDOWN,
	MODE_NONE,
}EN_MDC_MODE;

typedef enum{
	MD_PRECHARGE_INITIAL = 0,			/* プリチャージ未処理 */
	MD_PRECHARGE_RUN,					/* プリチャージ処理中 */
	MD_PRECHARGE_FINISH,				/* プリチャージ終了 */	
	MD_PRECHARGE_ABNORMAL,				/* プリチャージ異常 */
	MD_PRECHARGE_OFF,					/* プリチャージ停止 */
}EN_MDC_PRECHARGE_MODE;

typedef enum{
	MD_DISCHARGE_INITIAL = 0,			/* ディスチャージ未処理 */
	MD_DISCHARGE_RUN,					/* ディスチャージ処理中 */
	MD_DISCHARGE_PREPARING_FINISH,		/* ディスチャージ終了準備 */
	MD_DISCHARGE_FINISH,				/* ディスチャージ終了 */
	MD_DISCHARGE_ABNORMAL,				/* ディスチャージ異常 */
	MD_DISCHARGE_OFF,					/* ディスチャージ停止 */
}EN_MDC_DISCHARGE_MODE;

typedef union
{
	u8			byte;
	struct
	{
		u8	b_motorStopCommand		:1;			/* b0:モータ停止指令 */
		u8	b_relayOpenCommand		:1;			/* b1:リレー開放指令 */
		u8	b_dischargeCommand		:1;			/* b2:ディスチャージ指示 */
		u8							:5;			/* b3-7:未使用 */
	}bit;
}UNI_VCU_COMMAND;

typedef union
{
	u8			byte;
	struct
	{
		u8	b_motorStopCommand		:1;			/* b0:モータ停止指令 */
		u8	b_relayOpenCommand		:1;			/* b1:リレー開放指令 */
		u8							:6;			/* b2-7:未使用 */
	}bit;
}UNI_VCU_COMMAND_RX_0;

typedef union
{
	u8			byte;
	struct
	{
		u8	u8_buzzerPattern		:4;			/* b0-3:ブザー鳴動指示 */
		u8	b_dischargeCommand		:1;			/* b4:ディスチャージ指示 */
		u8	b_keySwStatusFromVcu	:1;			/* b5:キーSW入力 */
		u8							:2;			/* b6-7:未使用 */
	}bit;
}UNI_VCU_COMMAND_RX_4;

/* external variables --------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/


#endif /* __TYPEDEF_MODE_H */

