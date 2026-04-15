/******************************************************************************
* DISCLAIMER
* Please refer to http://www.renesas.com/disclaimer
******************************************************************************
  Copyright (C) 2010. Renesas Electronics Corp., All Rights Reserved.
*******************************************************************************
* File Name    : R_CAN_API_CFG.H
* Version      : 1.00
* Description  : Edit this file to configure the CAN API.
******************************************************************************
* History
* History       Ver.	Description
* Mar 22 '10	1.00	For RX62N with new CAN API.
* May 12, 2011 			Port to RSKRX62T.
******************************************************************************/
//Set TAB to 4 spaces!

#ifndef R_CAN_API_CFG_H
#define R_CAN_API_CFG_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
/******************************************************************************
Typedef definitions
******************************************************************************/
/* Set to 0 to use the CAN interrupts. Set to 1 if you want to poll CAN 
mailboxes for messages sent and received. */
#define USE_CAN_POLL        (0)   /* 1 polled mode, 0 use interrupts (=default). */

/* Transmit FIFO Interrupt Generation Timing Control:
   0 when every time transmission is completed (=default), 
   1 when the transmit FIFO becomes empty due to completion of transmission. */
#define CAN_CFG_TXFIFO_INT_GEN_TIMING        (0) 

/* Receive FIFO Interrupt Generation Timing Control:
   0 when every time reception is completed (=default), 
   1 when the receive FIFO becomes buffer warning by completion of reception. */
#define CAN_CFG_RXFIFO_INT_GEN_TIMING        (0)

/******************************************************************************
Macro definitions
******************************************************************************/
/* Include this line if you want to poll can mailboxes for messages received
and sent. COMMENT to use the CAN interrupts. */
//#define USE_CAN_POLL			1

/* Level */
#define CAN0_INT_LVL			2

/* In the interrupts, USE_CAN_API_SEARCH is default. Uncomment to use the
mailbox search register. Should be faster if a lot of mailboxes to check.
This is NOT verified, the opposite may be true. */
#define USE_CAN_API_SEARCH		1

/*** Board specific ports ******************************************************
Map the transceiver control pins here. Tranceivers vary in the control pins present
and these may need I/O ports assigned to them. This example has "Enable" and
"Standby" control pins on its tranceiver.

/* Configure CAN0 STBn pin.
Output. High = not standby. */
//#define CAN0_TRX_STB_PORT 	  	B
//#define CAN0_TRX_STB_PIN		0
//#define CAN0_TRX_STB_LVL		1   //High = Not standby.

/* Configure CAN0 EN pin.
Output. High to enable CAN transceiver. */
//#define CAN0_TRX_ENABLE_PORT    2
//#define CAN0_TRX_ENABLE_PIN     4
//#define CAN0_TRX_ENABLE_LVL     1   //High = Enable.

/*** Baudrate settings ********************************************************
	Calculation of baudrate:
	*********************************
	*	PCLK = 40 MHz = fcan.		*
	*	fcanclk = fcan/prescale		*
	*********************************

	Example 1)
	Desired baudrate 500 kbps.
	Selecting prescale to 4.
    fcanclk = 40000000/4
	fcanclk = 10000000 Hz
	Bitrate = fcanclk/Tqtot
	 or,
	Tqtot = fcanclk/bitrate
	Tqtot = 10000000/500000
	Tqtot = 100/5 = 20.
    Tqtot = TSEG1 + TSEG2 + SS
	Using TSEG1 = 7 Tq
		  TSEG2 = 2Tq
		  SS = 1 Tq always
		  Re-synchronization Control (SJW) should be 1-4 Tq (must be <=TSEG2).

	Example 2)
    Desired baudrate 500 kbps.
	Selecting prescale to 8.
	fcanclk = 40000000/8
	fcanclk = 5000000 Hz
	Tqtot = fcanclk/bitrate
	Tqtot = 5000000/500000
	Tqtot = 50/5 = 10.
	Tqtot = TSEG1 + TSEG2 + SS
	Using 	TSEG1 = 7 Tq
			TSEG2 = 2 Tq
			SS = 1 Tq always
			SJW = 1 Tq (<=TSEG2). */
	#define CAN_BRP		8
	#define CAN_TSEG1	7
	#define CAN_TSEG2	2
	#define CAN_SJW		1
//	*********************************
//	*	PCLK = 50 MHz = fcan.		*
//	*	fcanclk = fcan/prescale		*
//	*********************************
//
//	Example 1)
//	Desired baudrate 500 kbps.
//	Selecting prescale to 4.
//    fcanclk = 50000000/4
//	fcanclk = 12500000 Hz
//	Bitrate = fcanclk/Tqtot
//	 or,
//	Tqtot = fcanclk/bitrate
//	Tqtot = 12500000/500000
//	Tqtot = 125/5 = 25.
//    Tqtot = TSEG1 + TSEG2 + SS
//	Using TSEG1 = 16 Tq
//		  TSEG2 = 8 Tq
//		  SS = 1 Tq always
//		  Re-synchronization Control (SJW) should be 1-4 Tq (must be <=TSEG2). */

//	#define CAN_BRP		4
//	#define CAN_TSEG1	16
//	#define CAN_TSEG2	8
//	#define CAN_SJW		2

//	/*
//	Example 2)
//    Desired baudrate 500 kbps.
//	Selecting prescale to 5.
//	fcanclk = 50000000/5
//	fcanclk = 10000000 Hz
//	Tqtot = fcanclk/bitrate
//	Tqtot = 10000000/500000
//	Tqtot = 100/5 = 20.
//	Tqtot = TSEG1 + TSEG2 + SS
//	Using 	TSEG1 = 16 Tq
//			TSEG2 = 3 Tq
//			SS = 1 Tq always
//			SJW should be 1-4 Tq (<=TSEG2). *
//	#define CAN_BRP		5
//	#define CAN_TSEG1	16
//	#define CAN_TSEG2	3
//	#define CAN_SJW		2	*/


	/*
	 *	The range of each segment:	Bit time = 8 Tq to 25 Tq
	 *								SS = 1 Tq
	 *								TSEG1 = 4 Tq to 16 Tq
	 *								TSEG2 = 2 Tq to 8 Tq
	 *								SJW = 1 Tq to 4 Tq
	 *	Setting of TSEG1 and TSEG2: TSEG1 > TSEG2 >= SJW
	*/


/*** Other settings  *********************************************************/
/* Max time to poll a CAN register bit for expected value. Don't set to zero. */
#define MAX_CAN_REG_POLLTIME	4

/******************************************************************************
Constant definitions
*****************************************************************************/
#define CANBOX_TX 0				/*transmit mail box number*/
#define CANBOX_RX 9				/*receive mail box number*/
/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Function prototypes
******************************************************************************/
#endif	/* R_CAN_API_CFG_H */
/* eof */