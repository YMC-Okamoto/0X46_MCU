/*==========================================================================*/
/**
 * @file	DEFINE_CAN.h
 * @brief	CAN通信ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @date	2017.04
 *
 * Copyright&copy; 2011-2012 Yamaha Motor Co.,Ltd. All rights reserved.
 */
/*==========================================================================*/
#ifndef __DEFINE_CAN_H
#define __DEFINE_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "r_cg_macrodriver.h"
#include "ctypedef.h"					/* 型定義 */
#include "DEFINE_CAN_ID.h"				/* CAN ID定義 */

/* Private define -----------------------------------------------------------*/
#define U32_CAN_UNIT0					(u32)0							/*Unit0 RCFDC0*/
#define U32_CAN_UNIT1					(u32)1							/*Unit0 RCFDC0*/
#define U32_CAN_MBOX0					(u32)0							/*CANメールBOX0(TX)*/
#define U32_CAN_MBOX3					(u32)3							/*CANメールBOX4(RX)*/
#define U32_CAN_MBOX4					(u32)4							/*CANメールBOX4(RX)*/
#define U32_CAN_CH0						(u32)0							/*CANチャネル0*/
#define U32_CAN_CH1						(u32)1							/*CANチャネル1*/
#define U32_CAN_CH2						(u32)2							/*CANチャネル1*/
#define U32_CAN0_CHANNEL_MAX			(u32)3							/*64Pin CAN0 Unit channel can0～can2*/

/*** CAN API return values *********************/
#define	U32_R_CAN_OK					(u32)0x00000000
#define	U32_R_CAN_NOT_OK				(u32)0x00000001
//available
#define	U32_R_CAN_MSGLOST				(u32)0x00000004
#define	U32_R_CAN_NO_SENDDATA			(u32)0x00000008
#define	U32_R_CAN_RXPOLL_TMO			(u32)0x00000010
#define	U32_R_CAN_BAD_CH_NUM 			(u32)0x00000020
#define	U32_R_CAN_SW_BAD_MBOX			(u32)0x00000040
#define	U32_R_CAN_BAD_ACTION_TYPE		(u32)0x00000080
/* CAN peripheral timeout reasons. */
#define	U32_R_CAN_SW_OPERATE_ERR		(u32)0x00000100
#define	U32_R_CAN_SW_STOP_ERR			(u32)0x00000200
#define	U32_R_CAN_SW_TEST_ERR			(u32)0x00000400
#define	U32_R_CAN_SW_RST_ERR			(u32)0x00000800
#define	U32_R_CAN_SW_TSRC_ERR			(u32)0x00001000
#define	U32_R_CAN_SW_SET_TX_TMO			(u32)0x00002000
#define	U32_R_CAN_SW_SET_RX_TMO			(u32)0x00004000
#define	U32_R_CAN_SW_ABORT_ERR			(u32)0x00008000
/* CAN STATE CODES */
#define	U32_R_CAN_STATUS_ERROR_ACTIVE   (u32)0x00000001
#define	U32_R_CAN_STATUS_ERROR_PASSIVE  (u32)0x00000002
#define	U32_R_CAN_STATUS_ERROR_ACK		(u32)0x00000004
#define	U32_R_CAN_STATUS_BUSOFF 		(u32)0x00000008

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/


#endif /* __DEFINE_CAN_H */
