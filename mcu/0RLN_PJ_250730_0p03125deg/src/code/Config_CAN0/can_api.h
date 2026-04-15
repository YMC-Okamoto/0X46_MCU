/******************************************************************************
* DISCLAIMER
* Please refer to http://www.renesas.com/disclaimer
******************************************************************************
  Copyright (C) 2010. Renesas Electronics Corp., All Rights Reserved.
*******************************************************************************
* File Name    : R_CAN_API.H
* Version      : 1.00
* Description  :
******************************************************************************
* History
* Mar 22 '10  REA	For RX62N with new CAN API.
* Apr 15 '10  REA	R_CAN_Control 'Enter Sleep' added.
					R_CAN_TxStopMsg added.
******************************************************************************/

#ifndef R_CAN_API_H
#define R_CAN_API_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "stdint.h"
#include "ctypedef.h"

/******************************************************************************
Typedef definitions
******************************************************************************/
/* Standard data frame message definition object   */
typedef struct
{
	u32	u32_canId;
	u8	u8_canDlc;
	u8	u8_canData[8];
}ST_CAN_STD_FRAME;

/******************************************************************************
Macro definitions
******************************************************************************/
/*** CAN API ACTION TYPES ***/
#define	DISABLE							0
#define	ENABLE							1
/* Periph CAN modes */
#define	EXITSLEEP_CANMODE				2
#define	ENTERSLEEP_CANMODE 				3
#define	RESET_CANMODE					4
#define	HALT_CANMODE					5
#define	OPERATE_CANMODE					6
/* Port mode actions */
#define	CANPORT_TEST_LISTEN_ONLY		7
#define	CANPORT_TEST_0_EXT_LOOPBACK		8
#define	CANPORT_TEST_1_INT_LOOPBACK		9
#define	CANPORT_RETURN_TO_NORMAL		10

/* Local sleep mode for CAN module */
#define	CAN_NOT_SLEEP	0
#define	CAN_SLEEP 		1

/*** CAN API return values *********************/
#define		R_CAN_OK					(u32)0x00000000
#define		R_CAN_NOT_OK				(u32)0x00000001
//available
#define		R_CAN_MSGLOST				(u32)0x00000004
#define		R_CAN_NO_SENTDATA			(u32)0x00000008
#define		R_CAN_RXPOLL_TMO			(u32)0x00000010
#define		R_CAN_BAD_CH_NR 			(u32)0x00000020
#define		R_CAN_SW_BAD_MBX			(u32)0x00000040
#define		R_CAN_BAD_ACTION_TYPE		(u32)0x00000080
#define     CAN_ERR_NOT_FIFO_MODE   	(u32)0x00010000
#define     CAN_ERR_BOX_FULL        	(u32)0x00020000
#define     CAN_ERR_BOX_EMPTY       	(u32)0x00040000
#define     R_CAN_SW_BAD_MODE       	(u32)0x00080000
#define     R_CAN_SW_BAD_MODE       	(u32)0x00080000
/* CAN peripheral timeout reasons. */
#define		R_CAN_SW_WAKEUP_ERR			(u32)0x00000100
#define		R_CAN_SW_SLEEP_ERR			(u32)0x00000200
#define		R_CAN_SW_HALT_ERR			(u32)0x00000400
#define		R_CAN_SW_RST_ERR			(u32)0x00000800
#define		R_CAN_SW_TSRC_ERR			(u32)0x00001000
#define		R_CAN_SW_SET_TX_TMO			(u32)0x00002000
#define		R_CAN_SW_SET_RX_TMO			(u32)0x00004000
#define		R_CAN_SW_ABORT_ERR			(u32)0x00008000
/* CAN STATE CODES */
#define		R_CAN_STATUS_ERROR_ACTIVE   (u32)0x0000001
#define		R_CAN_STATUS_ERROR_PASSIVE  (u32)0x0000002
#define		R_CAN_STATUS_BUSOFF 		(u32)0x0000004
/********************************************************/

/* CAN Frame ID modes */
#define     STD_ID_MODE     0
#define     EXT_ID_MODE     1
#define     MIXED_ID_MODE   2

/* Mailbox search modes. */
#define 	RX_SEARCH		0
#define 	TX_SEARCH		1
#define 	MSGLOST_SEARCH	2
#define 	CHANNEL_SEARCH	3

/* CAN0 Control Register (CTLR) b9, b8 CANM[1:0] CAN Operating Mode Select. */
#define		CAN_OPERATION	0	//CAN operation mode
#define		CAN_RESET		1	//CAN reset mode
#define		CAN_HALT		2	//CAN halt mode
#define		CAN_RESET_FORCE	3	//CAN reset mode (forcible transition)

/* Frame types */
#define		DATA_FRAME	    0
#define		REMOTE_FRAME	1

/* Mailbox mode */
#define NORMAL_MAILBOX_MODE      0
#define FIFO_MAILBOX_MODE        1

/* Bit set defines */
#define		MBX_0			0x00000001
#define		MBX_1			0x00000002
#define		MBX_2			0x00000004
#define		MBX_3			0x00000008
#define		MBX_4			0x00000010
#define		MBX_5			0x00000020
#define		MBX_6			0x00000040
#define		MBX_7			0x00000080
#define		MBX_8			0x00000100
#define		MBX_9			0x00000200
#define		MBX_10			0x00000400
#define		MBX_11			0x00000800
#define		MBX_12			0x00001000
#define		MBX_13			0x00002000
#define		MBX_14			0x00004000
#define		MBX_15			0x00008000
#define		MBX_16			0x00010000
#define		MBX_17			0x00020000
#define		MBX_18			0x00040000
#define		MBX_19			0x00080000
#define		MBX_20			0x00100000
#define		MBX_21			0x00200000
#define		MBX_22			0x00400000
#define		MBX_23			0x00800000
#define		MBX_24			0x01000000
#define		MBX_25			0x02000000
#define		MBX_26			0x04000000
#define		MBX_27			0x08000000
#define		MBX_28			0x10000000
#define		MBX_29			0x20000000
#define		MBX_30			0x40000000
#define		MBX_31			0x80000000

/******************************************************************************
Constant definitions
*****************************************************************************/
/* Mem. area for bit set defines */
#define MAX_CHANNELS		32
static const u32 	u32_CAN_mboxArray[MAX_CHANNELS] = {	MBX_0,  MBX_1,  MBX_2,  MBX_3,
												MBX_4,  MBX_5,  MBX_6,  MBX_7,
												MBX_8,  MBX_9,  MBX_10, MBX_11,
												MBX_12, MBX_13, MBX_14, MBX_15,
												MBX_16, MBX_17, MBX_18, MBX_19,
												MBX_20, MBX_21, MBX_22, MBX_23,
												MBX_24, MBX_25, MBX_26, MBX_27,
												MBX_28, MBX_29, MBX_30, MBX_31,
												};

/******************************************************************************
Variable Externs
******************************************************************************/
/******************************************************************************
Function prototypes
******************************************************************************/
/*****************************************************************
			    R X   C A N   A P I
******************************************************************/
/* INITIALIZATION */
extern u32	R_CAN_Create(const u32 a_u32_chNum);
extern u32	R_CAN_PortSet(const u32 a_u32_chNum, const u32 a_u32_actionType);
extern u32	R_CAN_Control(const u32 a_u32_chNum, const u32 a_u32_actionType);
extern void	R_CAN_SetBitrate(const u32 a_u32_chNum);

/* TRANSMIT */
extern u32	R_CAN_TxSet(const u32 a_u32_chNum, const u32 a_u32_mbMode,const u32 a_u32_mboxNum, ST_CAN_STD_FRAME* p_frame, const u32 a_u32_frameType);
extern u32	R_CAN_Tx(const u32 a_u32_chNum, const u32 a_u32_mbMode, const u32 a_u32_mboxNum);
extern u32	R_CAN_TxCheck(const u32 a_u32_chNum, const u32 a_u32_mboxNum);
extern u32	R_CAN_TxStopMsg(const u32 a_u32_chNum, const u32 a_u32_mboxNum);

/* RECEIVE */
extern u32 R_CAN_RxSetFIFO(const u32 ch_nr,const u32 mb_mode,const u32 mbox_nr, const u32 fidcr0_value, const u32 fidcr1_value, const u32 frame_type,
					   const u32  mkr6_value, const u32 mkr7_value);
extern u32	R_CAN_RxSet(const u32 a_u32_chNum, const u32 a_u32_mboxNum, const u32 a_u32_sid, const u32 a_u32_frameType);
extern u32	R_CAN_RxPoll(const u32 a_u32_chNum, const u32 a_u32_mboxNum);
extern u32	R_CAN_RxRead(const u32 a_u32_chNum, const u32 a_u32_mboxNum, ST_CAN_STD_FRAME * const p_frame);
extern void	R_CAN_RxSetMask(const u32 a_u32_chNum, const u32 a_u32_mboxNum, const u32 a_u32_sidMaskValue);

/* ERRORS */
extern u32	R_CAN_CheckErr(const u32 a_u32_chNum);

#endif	/* R_CAN_API.H */
/* eof */