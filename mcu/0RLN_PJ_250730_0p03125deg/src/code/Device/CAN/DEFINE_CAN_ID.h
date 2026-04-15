/*==========================================================================*/
/**
 * @file	DEFINE_CAN_ID.h
 * @brief	CAN通信ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @date	2017.04
 *
 * Copyright&copy; 2011-2012 Yamaha Motor Co.,Ltd. All rights reserved.
 */
/*==========================================================================*/
#ifndef __DEFINE_CAN_ID_H
#define __DEFINE_CAN_ID_H

/* Includes ------------------------------------------------------------------*/
#include "r_cg_macrodriver.h"
#include "ctypedef.h"					/* 型定義 */
#include "DEFINE_CAN_ID.h"				/* CAN ID定義 */
#include "DEFINE_SETTING.h"				/* 定義 */

/* Private define -----------------------------------------------------------*/
#define U32_CAN_DATA_FRAME				DATA_FRAME						/* データフレーム */
#define U32_CAN_RMT_FRAME				REMOTE_FRAME					/* リモートフレーム */

/***** 受信CAN ID *****/
/* CAN0のチャンネル0で、MCUの全て受信バッファに対応するID集合 */

/***** 送信CAN ID *****/
/*-------------MCU 送信ID--------------*/
#define CANID_MCU_DATA1				(u16)0x131		/* 10ms		MCUデータ1 */
#define CANID_MCU_DATA2				(u16)0x132		/* 10ms		MCUデータ2 */
#define CANID_MCU_DATA3				(u16)0x133		/* 10ms		MCUデータ3 */
#define CANID_MCU_DATA4				(u16)0x134		/* 10ms		MCUデータ4 */
#define CANID_MCU_DATA5				(u16)0x135		/* 10ms		MCUデータ5 */
#define CANID_MCU_DATA6				(u16)0x136		/* 10ms		MCUデータ6 */
#define CANID_MCU_DATA7				(u16)0x137		/* 10ms		MCUデータ7 */
#define CANID_MCU_DATA8				(u16)0x138		/* 10ms		MCUデータ8 */
#define CANID_MCU_DATA9				(u16)0x139		/* 10ms		MCUデータ9 */
#define CANID_MCU_ERRCODE			(u16)0x141		/* 10ms		MCU異常コード */
/*-------------MCU 受信ID--------------*/
#define CANID_VCU_COMMAND			(u16)0x101		/* 10ms		MCUデータ1 */

/*** CANoe ***/
/* 受信 */
#define	CANID_CANOE_RX1				(u16)0x611		/* CANoe受信1 */
#define	CANID_CANOE_RX2				(u16)0x612		/* CANoe受信2 */
#define	CANID_CANOE_RX3				(u16)0x613		/* CANoe受信3 */
#define	CANID_CANOE_RX4				(u16)0x614		/* CANoe受信4 */
#define	CANID_CANOE_RX5				(u16)0x615		/* CANoe受信5 */


/*** CuW ***/
/* 受信 */
#define	CANID_CUW_RX				(u16)0x63C		/* ロギング */
/*-- 送信 --*/
#define CANID_CUW_TX				(u16)0x63D		/* 10ms		ロギング */

#define CANID_DUMMY			 		(u16)0x999		/* CAN ID ダミー */

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/


#endif /* __DEFINE_CAN_H */
