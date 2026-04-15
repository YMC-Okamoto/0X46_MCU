/******************************************************************************
* DISCLAIMER

* This software is supplied by Renesas Electronics Corp. and is only
* intended for use with Renesas products. No other uses are authorized.

* This software is owned by Renesas Electronics Corp. and is protected under
* all applicable laws, including copyright laws.

* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES
* REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE AND NON-INFRINGEMENT.	ALL SUCH WARRANTIES ARE EXPRESSLY
* DISCLAIMED.

* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORP. NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES
* FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS
* AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

* Renesas reserves the right, without notice, to make changes to this
* software and to discontinue the availability of this software.
* By using this software, you agree to the additional terms and
* conditions found by accessing the following link:
* http://www.renesas.com/disclaimer
******************************************************************************
* Copyright (C) 2010. Renesas Electronics Corp., All Rights Reserved.
******************************************************************************
* File Name		R_CAN_API.C
* Version		1.00
* Tool-Chain	RX Standard Toolchain 1.0.0.0
* Platform		ROK5562N
* Description	CAN API function definitions
* Operation		See example usage API_DEMO.C
* Limitations	:
******************************************************************************
* History
* Mar 22 '10  REA		For RX62N with new CAN API.
* Apr 15 '10  REA		Remote frame handling added.
* Apr 22 '10  REA		Port configuration moved to config.h. User just
						sets port and pin number for the transceiver control
						ports. No need to change driver.
						R_CAN_Control: Enter Sleep added.
						R_CAN_PortSet: Modes automatically now enter and exit
							 Halt mode, so user need just one call to change
							 mode.
						R_CAN_TxStopMsg(): TRMREQ to 0, then TRMABT clear.
						R_CAN_RxSetMask(): Halt CAN before mask change, resume
							after.
						R_CAN_RxPoll(): Function rewritten to use INVALDATA flag.
							R_CAN_NOT_OK added; "No message waiting or currently
							being written".
						All RETURN values spelled out in all function headers.
						Return values added and changed for some APIs.
* Jun 7 '10	 REA		Changed TRM_ACTIVE -> SENTDATA in R_CAN_WaitTxRx() since
							TRM_ACTIVE is low for a while after TRMREQ is set high.
						Increased MAX_CAN_SW_DELAY from 0x1000 to 0x2000. If the
							TxCheck function is not used, the timer could time out
							 and the mailbox will not send properly if the user
							ignores the a_u32_apiStatus warning that WaitTxRx timed out.
						In R_CAN_WaitTxRx changed to while (...SENTDATA == 0)...
* Mar 22, 2012			Removed RESET_CAN_SW_TMR. Counter reset at function start.
* Oct 12, 2012			Changed R_CAN_Control() sleep mode, to ensure that Halt mode
							is entered before exiting sleep.
******************************************************************************/
//Set TAB = 4 spaces

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include <string.h>
#include <machine.h>
//#include "Ctypedef.h"	/*Cタイプ変数定義*/
#include "iodefine.h"
#include "config_can_rapi.h"
//#include "iodefine.h"
//#include "rx62t_iodefine_tmp.h"
#include "can_api.h"
//#include "ramdef.h"

/******************************************************************************
Typedef definitions
******************************************************************************/
/******************************************************************************
Macro definitions
******************************************************************************/
/* These macros are for determining if some can while loop times out. If they do,
the canx_sw_err variable will be non zero. This is to prevent an error in the
can peripheral and driver from blocking the CPU indefinatly. */
#define	DEC_CHK_CAN_SW_TMR		(--a_u32_canTimeoutCnt != 0)

/* Max delay waiting for CAN register to flip. Set to whatever theoretical
worst latency you could accept. It should never timeout. Do not set to 0. */
#define	MAX_CAN_SW_DELAY		(0x2000)

#define CHECK_MBX_NR            {if (mbox_nr > 31) return R_CAN_SW_BAD_MBX;}
#define CHECK_TXFIFO_MBX_NR     {if (mbox_nr > 27) return R_CAN_SW_BAD_MBX;}
#define CHECK_RXFIFO_MBX_NR     {if (((mbox_nr > 23) && (mbox_nr < 28)) || mbox_nr > 31 ) return R_CAN_SW_BAD_MBX;}
#define CHECK_MBX_MODE          {if (mb_mode > 1) return R_CAN_SW_BAD_MODE;}

/* Define a mask for MSB of a long to serve as an extended ID mode bit flag. */
/* Extended ID occupies lower 29 bits, so use this to mask off upper 3 bits. */ 
#define XID_MASK                (0xE0000000)

/* Define a mask for the 11 bits that make up a standard ID. */ 
#define SID_MASK                (0x000007FF)

/******************************************************************************
Global variables and functions imported (externs)
******************************************************************************/
/******************************************************************************
Constant definitions
*****************************************************************************/
/******************************************************************************
Global variables and functions private to the file
******************************************************************************/
/* Data */
/* Functions */
static void	CanClearSentData(const u32 a_u32_chNum, const u32 a_u32_mboxNum);
static u32	R_CAN_WaitTxRx(const u32 a_u32_chNum, const u32 a_u32_mboxNum);

static void	R_CAN_ConfigCANinterrupts(const u32 a_u32_chNum);


/******************************************************************************

						C A N 0	  F U N C T I O N S

******************************************************************************/
/*******************************************************************************
Function Name:	R_CAN_Create
Description:	Configure the CAN peripheral.
Parameters:		a_u32_chNum
Return value:	R_CAN_OK			Action completed successfully.
				R_CAN_SW_BAD_MBX	Bad mailbox number.
				R_CAN_BAD_CH_NR		The channel number does not exist.
				R_CAN_SW_RST_ERR	The CAN peripheral did not enter Reset mode.
				See also R_CAN_Control return	values.
*******************************************************************************/
u32 R_CAN_Create(const u32 a_u32_chNum)
{
	u32 a_u32_apiStatus = (R_CAN_OK);
	u32 i, j;
	/* A faulty CAN peripheral block, due to HW, FW could potentially block (hang)
	the program at a while-loop. To prevent this, a sw timer in the while-loops
	will time out enabling the CPU to continue. */
	u32 a_u32_canTimeoutCnt = (MAX_CAN_SW_DELAY);

	volatile struct st_can __evenaccess * p_canBlock;

	if (a_u32_chNum == 0)
	{
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		return	(R_CAN_BAD_CH_NR);
	}

	MSTP(CAN0) = 0;
	/* Exit Sleep mode. */
	a_u32_apiStatus |= R_CAN_Control(a_u32_chNum, EXITSLEEP_CANMODE);

	/* Sleep -> RESET mode. */
	a_u32_apiStatus |= R_CAN_Control(a_u32_chNum, RESET_CANMODE);

	/*** Setting of CAN0 Control register.***/
	/* BOM:	Bus Off recovery mode acc. to IEC11898-1 */
	p_canBlock->CTLR.BIT.BOM = 0;
	/* MBM: Select normal mailbox mode. */
	p_canBlock->CTLR.BIT.MBM = 0;
	/* IDFM: Select standard ID mode. */
	p_canBlock->CTLR.BIT.IDFM = 0;
	/*	:	0 = Overwrite mode: Latest message overwrites old.
			1 = Overrun mode: Latest message discarded. */
	p_canBlock->CTLR.BIT.MLM = 0;
	/* TPM: ID priority mode. */
	p_canBlock->CTLR.BIT.TPM = 0;
	/* TSRC: Only to be set to 1 in operation mode */
	p_canBlock->CTLR.BIT.TSRC = 0;
	/* TSPS: Update every 8 bit times */
	p_canBlock->CTLR.BIT.TSPS = 3;

	/* Set BAUDRATE */
	R_CAN_SetBitrate(a_u32_chNum);

	/* Mask invalid for all mailboxes by default. */
	p_canBlock->MKIVLR.LONG = 0xFFFFFFFF;

	/* Configure CAN interrupts. */
	R_CAN_ConfigCANinterrupts(a_u32_chNum);

	/* Reset -> HALT mode */
	a_u32_apiStatus |= R_CAN_Control(a_u32_chNum, HALT_CANMODE);
	p_canBlock->MSMR.BIT.MBSM = 0x00;		/*Receive mail box check mode*/
	/* Configure mailboxes in Halt mode. */
	for (i = 0; i < 32; i++)
	{
		p_canBlock->MB[i].ID.LONG = 0x00;
		p_canBlock->MB[i].DLC = 0x0000;
		for (j = 0; j < 8; j++)
		{
			p_canBlock->MB[i].DATA[j] = 0x00;
		}
	//	p_canBlock->MB[i].TS.WORD = 0x00;
		p_canBlock->MB[i].TS = 0x0000;
	}

	/* Halt -> OPERATION mode */
	/* Note: EST and BLIF flag go high here when stepping code in debugger. */
	a_u32_apiStatus |= R_CAN_Control(a_u32_chNum, OPERATE_CANMODE);

	/* Time Stamp Counter reset. Set the TSRC bit to 1 in CAN Operation mode. */
	p_canBlock->CTLR.BIT.TSRC = 1;
	while (( p_canBlock->CTLR.BIT.TSRC) && (DEC_CHK_CAN_SW_TMR) )
	{
		;
	}
	if ( a_u32_canTimeoutCnt == 0 )
	{
		a_u32_apiStatus |= (R_CAN_SW_TSRC_ERR);
	}

	/* Check for errors so far, report, and clear. */
	if ( p_canBlock->STR.BIT.EST )
	{
		a_u32_apiStatus |= (R_CAN_SW_RST_ERR);
	}

	/* Clear Error Interrupt Factor Judge Register. */
	if ( p_canBlock->EIFR.BYTE )
	{
		a_u32_apiStatus |= (R_CAN_SW_RST_ERR);
	}
	p_canBlock->EIFR.BYTE = 0x00;

	/* Clear Error Code Store Register. */
	if (p_canBlock->ECSR.BYTE)
	{
		a_u32_apiStatus |= (R_CAN_SW_RST_ERR);
	}
	p_canBlock->ECSR.BYTE = 0x00;

	return	a_u32_apiStatus;
}/* end R_CAN_Create() */

/***********************************************************************************
Function Name:	R_CAN_PortSet
Description:	Configures the MCU and transceiver port pins. This function is
				responsible for configuring the MCU and transceiver port pins.
				Transceiver port pins such as Enable will vary depending on design,
				and this fucntion must then be modified. The function is also used
				to enter the CAN port test modes, such as Listen Only.
Parameters:	 a_u32_chNum
				a_u32_actionTypes: ENABLE, DISABLE, CANPORT_TEST_LISTEN_ONLY,
				CANPORT_TEST_0_EXT_LOOPBACK, CANPORT_TEST_1_INT_LOOPBACK, and
				CANPORT_RETURN_TO_NORMAL which is the default; no need to call
				unless another test mode was invoked previously.
Return value:	R_CAN_OK				Action completed successfully.
				R_CAN_SW_BAD_MBX		Bad mailbox number.
				R_CAN_BAD_CH_NR			The channel number does not exist.
				R_CAN_BAD_ACTION_TYPE	No such action type exists for this function.
				R_CAN_SW_HALT_ERR		The CAN peripheral did not enter Halt mode.
				R_CAN_SW_RST_ERR		The CAN peripheral did not enter Reset mode.
				See also R_CAN_Control return	values.
***********************************************************************************/
u32 R_CAN_PortSet(const u32 a_u32_chNum, const u32 a_u32_actionType)
{
	u32 a_u32_apiStatus = (R_CAN_OK);
	volatile struct st_can __evenaccess * p_canBlock;

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

	switch ( a_u32_actionType )
	{
	case ENABLE:
	    /* Port pin function select register setting */
		MPC.PWPR.BIT.B0WI = 0U;	  /* Enable PFSWE register write */
    	MPC.PWPR.BIT.PFSWE = 1U;  /* Enable PFS register write */

		/* Enable the CTX0(PB5) and CRX0(PB6) pins. */
 		/* Set CRX0 pin */
    	PORTB.PMR.BIT.B6 = 0U;
    	PORTB.PDR.BIT.B6 = 0U;
    	MPC.PB6PFS.BIT.PSEL = 0x10U;			/* PB6: CRX0*/
    	PORTB.PMR.BIT.B6 = 1U;					/* Peripheral functions */
    	PORTB.PDR.BIT.B6 = 0U;					/* CRX0 is input */
	
	 	/* Set CTX0 pin */
    	PORTB.PMR.BIT.B5 = 0U;
    	PORTB.PDR.BIT.B5 = 0U;
   		MPC.PB5PFS.BIT.PSEL = 0x10U;			/* PB5: CTX0*/
    	PORTB.PMR.BIT.B5 = 1U;					/* Peripheral functions */
    	PORTB.PDR.BIT.B5 = 1U;					/*CTX0 is output*/	

		MPC.PB6PFS.BIT.ISEL = 1U;				/* CAN receive intterrupt input pin*/

    	MPC.PWPR.BIT.PFSWE = 0U;	  	/* disable PFS register write */
    	MPC.PWPR.BIT.B0WI = 1U;			/* disable PFSWE register write */
		break;

	case DISABLE:
		/* Configure CAN0 TX and RX pins. */

		/* Port function control register MPC is used to configure I/O pin
		0:The CTX0 and CRX0 pins are disabled. (CTX-B and CRX-B are disable)*/
	    /* Port pin function select register setting */
		MPC.PWPR.BIT.B0WI = 0U;	  /* Enable PFSWE register write */
    	MPC.PWPR.BIT.PFSWE = 1U;  /* Enable PFS register write */

		MPC.PB5PFS.BIT.PSEL = 0x00U;			/* PB5: CTX0*/
		MPC.PB6PFS.BIT.PSEL = 0x00U;			/* PB6: CRX0*/
		PORTB.PMR.BIT.B5 = 0U;
		PORTB.PMR.BIT.B6 = 0U;

    	MPC.PWPR.BIT.PFSWE = 0U;	  	/* disable PFS register write */
    	MPC.PWPR.BIT.B0WI = 1U;			/* disable PFSWE register write */	
	break;

	/* Run in Listen Only test mode. */
	case CANPORT_TEST_LISTEN_ONLY:
		a_u32_apiStatus = R_CAN_Control(0, HALT_CANMODE);
		p_canBlock->TCR.BYTE = 0x03;
		a_u32_apiStatus |= R_CAN_Control(0, OPERATE_CANMODE);
		a_u32_apiStatus |= R_CAN_PortSet(0, ENABLE);
		break;

	/* Run in External Loopback test mode. */
	case CANPORT_TEST_0_EXT_LOOPBACK:
		a_u32_apiStatus = R_CAN_Control(0, HALT_CANMODE);
		p_canBlock->TCR.BYTE = 0x05;
		a_u32_apiStatus |= R_CAN_Control(0, OPERATE_CANMODE);
		a_u32_apiStatus |= R_CAN_PortSet(0, ENABLE);
		break;

	/* Run in Internal Loopback test mode. */
	case CANPORT_TEST_1_INT_LOOPBACK:
		a_u32_apiStatus = R_CAN_Control(0, HALT_CANMODE);
		p_canBlock->TCR.BYTE = 0x07;
		a_u32_apiStatus |= R_CAN_Control(0, OPERATE_CANMODE);
		break;

	/* Return to default CAN bus mode.
	This is the default setting at CAN reset. */
	case CANPORT_RETURN_TO_NORMAL:
		a_u32_apiStatus = R_CAN_Control(0, HALT_CANMODE);
		p_canBlock->TCR.BYTE = 0x00;
		a_u32_apiStatus |= R_CAN_Control(0, OPERATE_CANMODE);
		a_u32_apiStatus |= R_CAN_PortSet(0, ENABLE);
		break;

	default:
		/* Bad action type. */
		a_u32_apiStatus = R_CAN_BAD_ACTION_TYPE;
		break;
	}
	return	a_u32_apiStatus;
}/* end R_CAN_PortSet() */

/*******************************************************************************
Function Name:	R_CAN_Control
Description:	Controls transition to CAN operating modes determined by the CAN
				Control register. For example, the Halt mode should be used to
				later configure a recieve mailbox.
Parameters:		a_u32_chNum
				a_u32_actionType: EXITSLEEP_CANMODE, ENTERSLEEP_CANMODE,
				RESET_CANMODE, HALT_CANMODE, OPERATE_CANMODE.
Return value:	R_CAN_OK				Action completed successfully.
				R_CAN_SW_BAD_MBX		Bad mailbox number.
				R_CAN_BAD_CH_NR			The channel number does not exist.
				R_CAN_BAD_ACTION_TYPE	No such action type exists for this function.
				R_CAN_SW_WAKEUP_ERR	The CAN peripheral did not wake up from Sleep mode.
				R_CAN_SW_SLEEP_ERR		The CAN peripheral did enter Sleep mode.
				R_CAN_SW_RST_ERR		The CAN peripheral did not enter Halt mode.
				R_CAN_SW_HALT_ERR		The CAN peripheral did not enter Halt mode.
				R_CAN_SW_RST_ERR		The CAN peripheral did not enter Reset mode.
				See also R_CAN_PortSet return	values.
*******************************************************************************/
u32 R_CAN_Control(const u32 a_u32_chNum, const u32 a_u32_actionType)
{
	u32 a_u32_apiStatus = (R_CAN_OK);
	u32 a_u32_canTimeoutCnt = (MAX_CAN_SW_DELAY);
	volatile struct st_can __evenaccess * p_canBlock;

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

	switch ( a_u32_actionType )
	{
	case EXITSLEEP_CANMODE:
		/* Set to, and ensure that RCAN returns in, Sleep mode.
		Write to the SLPM bit (in CAN Reset or) CAN Halt mode,
		but if we are in Sleep mode, we should already also be
		in Halt mode. */
		p_canBlock->CTLR.BIT.SLPM = (CAN_NOT_SLEEP);
		while ( (p_canBlock->STR.BIT.SLPST) && (DEC_CHK_CAN_SW_TMR) )
		{
			R_BSP_NOP();
		}
		if ( a_u32_canTimeoutCnt == 0 )
		{
			a_u32_apiStatus = (R_CAN_SW_WAKEUP_ERR);
		}
		R_CAN_Control(a_u32_chNum, RESET_CANMODE);
		break;

	case ENTERSLEEP_CANMODE:
		/* Set to, and ensure that RCAN returns in, the Sleep state.
		Write to the SLPM bit in CAN Reset or CAN Halt modes. */
		a_u32_apiStatus = R_CAN_Control(0, HALT_CANMODE);

		p_canBlock->CTLR.BIT.SLPM = (CAN_SLEEP);
		while ( (!p_canBlock->STR.BIT.SLPST) && (DEC_CHK_CAN_SW_TMR) )
		{
			R_BSP_NOP();
		}
		if ( a_u32_canTimeoutCnt == 0 )
		{
			a_u32_apiStatus = (R_CAN_SW_SLEEP_ERR);
		}
		break;

	case RESET_CANMODE:
		/* Set to, and ensure that RCAN is in, the Reset state. */
		p_canBlock->CTLR.BIT.CANM = (CAN_RESET);
		while ( (!p_canBlock->STR.BIT.RSTST) && (DEC_CHK_CAN_SW_TMR) )
		{
			;
		}
		if ( a_u32_canTimeoutCnt == 0 )
		{
			a_u32_apiStatus = (R_CAN_SW_RST_ERR);
		}
		break;

	case HALT_CANMODE:
		/* Set to, and ensure that RCAN is in, the Halt state. */
		/* The CAN module enters CAN Halt mode after waiting for the end of
		message reception or transmission. */
		p_canBlock->CTLR.BIT.CANM = (CAN_HALT);
		while ( (!p_canBlock->STR.BIT.HLTST) && (DEC_CHK_CAN_SW_TMR) )
		{
			;
		}
		if ( a_u32_canTimeoutCnt == 0 )
		{
			a_u32_apiStatus = (R_CAN_SW_HALT_ERR);
		}
		break;

	case OPERATE_CANMODE:

		/* Take CAN out of Stop mode. */
		//SYSTEM.MSTPCRB.BIT.MSTPB0  =	 0;					/*It is too late to set the CAN enable bit*/
		MSTP(CAN0) = 0;
		while ( SYSTEM.MSTPCRB.BIT.MSTPB0 && (DEC_CHK_CAN_SW_TMR) )
		{
			R_BSP_NOP();
		}

		/* Set to Operate mode. */
		p_canBlock->CTLR.BIT.CANM = (CAN_OPERATION);

		/* Ensure that RCAN is in Operation	mode. */
		while ( (p_canBlock->STR.BIT.RSTST) && (DEC_CHK_CAN_SW_TMR) )
		{
			;
		}
		if ( a_u32_canTimeoutCnt == 0 )
		{
			a_u32_apiStatus = (R_CAN_SW_RST_ERR);
		}
		break;

	default:
		a_u32_apiStatus = (R_CAN_BAD_ACTION_TYPE);
		break;
	}

	return	a_u32_apiStatus;
}/* end R_CAN_Control() */

/*******************************************************************************
Function Name:	R_CAN_TxSet
Description:	Set up a CAN mailbox to transmit.
Parameters:		Channel nr.
				Mailbox nr.
				p_frame - pointer to a data frame structure.
				remote - REMOTE_FRAME to send remote request, DATA_FRAME for
				sending normal dataframe.
Return value:	R_CAN_OK				The mailbox was set up for transmission.
				R_CAN_SW_BAD_MBX		Bad mailbox number.
				R_CAN_BAD_CH_NR			The channel number does not exist.
				R_CAN_BAD_ACTION_TYPE	No such action type exists for this
										function.
*******************************************************************************/
u32 R_CAN_TxSet(	const u32			a_u32_chNum,
					const u32			a_u32_mbMode,
					const u32			a_u32_mboxNum,
					ST_CAN_STD_FRAME*	p_frame,
					const u32			a_u32_frameType)
{
	u32	a_u32_apiStatus = (R_CAN_OK);
	u32	i;
	volatile struct st_can __evenaccess * p_canBlock;

	if( a_u32_mbMode > 1)
	{
		return (R_CAN_SW_BAD_MODE);
	}

	if (a_u32_mboxNum > 31)
	{
		return	(R_CAN_SW_BAD_MBX);
	}

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

    if((FIFO_MAILBOX_MODE == a_u32_mbMode) && (u32_CAN_mboxArray[a_u32_mboxNum] & 0x0f000000))    /* check fifo mailbox mode and using FIFO mailbox */
    {
        /* In FIFO mailbox mode, change the bits in MIER for the related FIFO only when
         * the TFE bit in TFCR is 0 and the TFEST flag is 1 */
        if((p_canBlock->TFCR.BIT.TFE == 0) && (p_canBlock->TFCR.BIT.TFEST == 1))
        {
            /* Interrupt disable the mailbox.in case it was a receive mailbox */
            p_canBlock->MIER.LONG &= ((~(u32_CAN_mboxArray[24])));
        }

        /* return if transmit fifo mailbox is full */
        if(p_canBlock->TFCR.BIT.TFFST == 1)
        {
            return (CAN_ERR_BOX_FULL);
        }

        /* Copy frame data into mailbox */
        for (i = 0; ((i < p_frame->u8_canDlc) && (i<8)); i++)
        {
            p_canBlock->MB[24].DATA[i] = p_frame->u8_canData[i];
        }

        /* Frame select: Data frame = 0, Remote = 1 */
        if (REMOTE_FRAME == a_u32_frameType)
        {
            p_canBlock->MB[24].ID.BIT.RTR = 1;
        }
        else
        {
            p_canBlock->MB[24].ID.BIT.RTR = 0;
        }

        /*** Set Mailbox ID based on ID mode ***/
        if (p_frame->u32_canId & XID_MASK)    /* Check for XID flag bit set in ID field */
        {
            /* Set message mailbox buffer Extended ID, masking off temporary XID flag bit. */
            p_canBlock->MB[24].ID.LONG = (p_frame->u32_canId & (~XID_MASK));

            if (MIXED_ID_MODE == p_canBlock->CTLR.BIT.IDFM)
            {
                p_canBlock->MB[24].ID.BIT.IDE = 1;   /* Mixed mode; select to send extended frame. */
            }
        }
        else
        {
            /* Set message mailbox buffer Standard ID.
             * Put only the lower 11 bit in the SID. */
            p_canBlock->MB[24].ID.BIT.SID = (p_frame->u32_canId & SID_MASK);

            /* Always set IDE to 0 (unless mixed mode and extended frame as above). */
            p_canBlock->MB[24].ID.BIT.IDE = 0;
        }

        /* Set the Data Length Code */
        p_canBlock->MB[24].DLC = p_frame->u8_canDlc;

        #if (USE_CAN_POLL == 0)
        /* In FIFO mailbox mode, change the bits in MIER for the related FIFO only when
         * the TFE bit in TFCR is 0 and the TFEST flag is 1 */
            if((p_canBlock->TFCR.BIT.TFE == 0) && (p_canBlock->TFCR.BIT.TFEST == 1))
            {
                /* Interrupt enable the mailbox */
                p_canBlock->MIER.LONG |= (u32_CAN_mboxArray[24]);

                /* Generate interrupt when FIFO last message transmission completed */
                if(CAN_CFG_TXFIFO_INT_GEN_TIMING == 1)
                {
                    p_canBlock->MIER.LONG |= (u32_CAN_mboxArray[25]);
                }
            }
        #endif

        if(p_canBlock->TFCR.BIT.TFEST == 1) /* confirm that the transmit FIFO empty status bit is set to 1. */
        {
            p_canBlock->TFCR.BIT.TFE = 1; /* Transmit FIFO enabled. */
        }

        R_CAN_Tx(a_u32_chNum, a_u32_mbMode, a_u32_mboxNum);
    }

    /* Content of below else will occur in 2 case:
     * 1: Transmit message by MBX normal in FIFO mailbox mode when FIFO_MAILBOX_MODE == mb_mode
     * 2: Transmit message by MBX normal in normal mailbox mode when FIFO_MAILBOX_MODE != mb_mode */	
	else
	{
		/* Wait for any previous transmission to complete. */
		a_u32_apiStatus = R_CAN_WaitTxRx(0, a_u32_mboxNum);
		
		/* Interrupt disable the mailbox.in case it was a receive mailbox */
		p_canBlock->MIER.LONG &= ~(u32_CAN_mboxArray[a_u32_mboxNum]);

		/* Clear message mailbox control register (trmreq to 0). */
		p_canBlock->MCTL[a_u32_mboxNum].BYTE = 0;

        /*** Set Mailbox ID based on ID mode ***/
        if (p_frame->u32_canId & XID_MASK)    /* Check for XID flag bit set in ID field */
        {
            /* Set message mailbox buffer Extended ID, masking off temporary XID flag bit. */
            p_canBlock->MB[a_u32_mboxNum].ID.LONG = (p_frame->u32_canId & (~XID_MASK));

            if (MIXED_ID_MODE == p_canBlock->CTLR.BIT.IDFM)
            {
                p_canBlock->MB[a_u32_mboxNum].ID.BIT.IDE = 1;   /* Mixed mode; select to send extended frame. */
            }
        }
        else
        {
            /* Set message mailbox buffer Standard ID. Put only the lower 11 bit in the SID. */
            p_canBlock->MB[a_u32_mboxNum].ID.BIT.SID = (p_frame->u32_canId & SID_MASK);

            /* Always set IDE to 0 (unless mixed mode and extended frame as above). */
            p_canBlock->MB[a_u32_mboxNum].ID.BIT.IDE = 0;
        }

		/* Set the Data Length Code */
		p_canBlock->MB[a_u32_mboxNum].DLC = p_frame->u8_canDlc;

		/* Frame select: Data frame = 0, Remote = 1 */
		if ( a_u32_frameType == (REMOTE_FRAME) )
		{
			p_canBlock->MB[a_u32_mboxNum].ID.BIT.RTR = 1;
		}
		else
		{
			p_canBlock->MB[a_u32_mboxNum].ID.BIT.RTR = 0;
		}

		/* Frame select: Standard = 0, Extended = 1 */
		p_canBlock->MB[a_u32_mboxNum].ID.BIT.IDE = 0;

		/* Copy frame data into mailbox */
		for ( i = 0; ((i < p_frame->u8_canDlc) && (i < 8)); i++ )
		{
			p_canBlock->MB[a_u32_mboxNum].DATA[i] = p_frame->u8_canData[i];
		}

#if 0								/*Disable the interrupt for transmite mailbox*/
		#ifndef USE_CAN_POLL
		/* Interrupt enable the mailbox */
		//	p_canBlock->MIER |= (u32_CAN_mboxArray[a_u32_mboxNum]);
		p_canBlock->MIER.LONG |= (u32_CAN_mboxArray[a_u32_mboxNum]);
		#endif
#endif

		R_CAN_Tx(a_u32_chNum, a_u32_mbMode, a_u32_mboxNum);
	}

	return	a_u32_apiStatus;
} /* end R_CAN_TxSet() */

/*******************************************************************************
Function Name:	R_CAN_Tx
Description:	Starts actual message transmission onto the CAN bus.
Parameters:		Channel nr.
				mbMode, the mode of mailbox
				Mailbox nr.
Return value:	R_CAN_OK			The mailbox was set to transmit a previously
									configured mailbox.
				R_CAN_SW_BAD_MBX	Bad mailbox number.
				R_CAN_BAD_CH_NR		The channel number does not exist.
				R_CAN_SW_SET_TX_TMO	Waiting for previous transmission to finish
									timed out.
				R_CAN_SW_SET_RX_TMO	Waiting for previous reception to complete
									timed out.
*******************************************************************************/
u32 R_CAN_Tx(const u32 a_u32_chNum, const u32 a_u32_mbMode,const u32 a_u32_mboxNum)
{
	u32 a_u32_apiStatus = (R_CAN_OK);
	volatile struct st_can __evenaccess * p_canBlock;

	if (a_u32_mboxNum > 31)
	{
		return	(R_CAN_SW_BAD_MBX);
	}

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

    if((FIFO_MAILBOX_MODE == a_u32_mbMode) && (u32_CAN_mboxArray[a_u32_mboxNum] & 0x0f000000))    /* check normal or fifo mode */
    {
        /* Write FFh to transmit FIFO pointer control register */
        p_canBlock->TFPCR = 0xFF;
    }
	else
	{
		/* Wait for any previous transmission to complete. */
		a_u32_apiStatus = R_CAN_WaitTxRx(0, a_u32_mboxNum);

		/* Clear SentData flag since we are about to send anew. */
		//CanClearSentData(a_u32_chNum, a_u32_mboxNum);
		p_canBlock->MCTL[a_u32_mboxNum].BYTE = 0;

		/* Set TrmReq bit to "1" */
		p_canBlock->MCTL[a_u32_mboxNum].BIT.TX.TRMREQ = 1;
	}
	return	a_u32_apiStatus;
}/* end R_CAN_Tx() */

/*****************************************************************************
Name:			R_CAN_TxCheck
Parameters:		Channel nr.
				Mailbox nr.
Description:	Use to check a mailbox for a successful data frame transmission.
				Primarily used when polling to check that message was sent, so
				that the next in series of messages can be sent. To do this when
				using CAN interrupts, this function can be called to check which
				mailbox caused the interrupt.
Return value:	R_CAN_OK			Transmission was completed successfully.
				R_CAN_SW_BAD_MBX	Bad mailbox number.
				R_CAN_BAD_CH_NR		The channel number does not exist.
				R_CAN_MSGLOST		Message was overwritten or lost.
				R_CAN_NO_SENTDATA	No message was sent.
*****************************************************************************/
u32 R_CAN_TxCheck(const u32 a_u32_chNum, const u32 a_u32_mboxNum)
{
	u32 a_u32_apiStatus = (R_CAN_OK);
	volatile struct st_can __evenaccess * p_canBlock;

	if (a_u32_mboxNum > 31)
	{
		return	(R_CAN_SW_BAD_MBX);
	}

	if ( a_u32_chNum == 0 )
	{
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		return	(R_CAN_BAD_CH_NR);
	}

	if ( p_canBlock->MCTL[a_u32_mboxNum].BIT.TX.SENTDATA == 0 )
	{
		a_u32_apiStatus = (R_CAN_NO_SENTDATA);
	}
	else
	{
		/* Clear SentData flag. */
		CanClearSentData(a_u32_chNum, a_u32_mboxNum);
	}

	return	a_u32_apiStatus;
}/* end R_CAN_TxCheck() */

/*****************************************************************************
Name:			R_CAN_TxStopMsg
Parameters:		Channel nr.
				Mailbox nr.
Description:	Stop a mailbox that has been asked to transmit a frame. If the
				message was not stopped, R_CAN_SW_ABORT_ERR is returned. Note
				that the cause of this could be that the message was already sent.
Return value:	R_CAN_OK			Action completed successfully.
				R_CAN_SW_BAD_MBX	Bad mailbox number.
				R_CAN_BAD_CH_NR		The channel number does not exist.
				R_CAN_SW_ABORT_ERR	Waiting for an abort timed out.
*****************************************************************************/
u32 R_CAN_TxStopMsg(const u32 a_u32_chNum, const u32 a_u32_mboxNum)
{
	u32 a_u32_apiStatus = (R_CAN_OK);
	u32 a_u32_canTimeoutCnt = (MAX_CAN_SW_DELAY);
	volatile struct st_can __evenaccess * p_canBlock;

	if (a_u32_mboxNum > 31)
	{
		return	(R_CAN_SW_BAD_MBX);
	}

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

	/* Clear message mailbox control register. Setting TRMREQ to 0 should abort. */
	p_canBlock->MCTL[a_u32_mboxNum].BYTE = 0;

	/* Wait for abort. */
	while ( (p_canBlock->MCTL[a_u32_mboxNum].BIT.TX.TRMABT) && (DEC_CHK_CAN_SW_TMR) )
	{
		;
	}
	if ( a_u32_canTimeoutCnt == 0 )
	{
		a_u32_apiStatus = (R_CAN_SW_ABORT_ERR);
	}
	/* Clear abort flag. */
	p_canBlock->MCTL[a_u32_mboxNum].BIT.TX.TRMABT = 0;

	return	a_u32_apiStatus;
}/* end R_CAN_TxStopMsg() */

/*****************************************************************************
Name:			CanClearSentData
Parameters:		Channel nr.
				Mailbox nr.
Description:	Use in poll mode for checking successful data frame transmission.
Return value:	CAN API code (CAN_(R_CAN_OK) if mailbox has sent.)
*****************************************************************************/
#pragma inline(CanClearSentData)
static void CanClearSentData(const u32 a_u32_chNum, const u32 a_u32_mboxNum)
{
	volatile struct st_can __evenaccess * p_canBlock;

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return;
	}

	/* Clear SentData to 0 *after* setting TrmReq to 0. */
	p_canBlock->MCTL[a_u32_mboxNum].BIT.TX.TRMREQ = 0;
	R_BSP_NOP();
	p_canBlock->MCTL[a_u32_mboxNum].BIT.TX.SENTDATA = 0;
}/* end CanClearSentData() */

/*******************************************************************************
Function Name:	R_CAN_RxSet
Description:	Set up a mailbox to receive. The API sets up a given mailbox to
				receive dataframes with the given CAN ID. Incoming data frames
				with the same ID will be stored in the mailbox.
Parameters:		a_u32_chNum
				Mailbox nr.
				p_frame - pointer to a data frame structure.
				remote - REMOTE_FRAME to listen for remote requests, DATA_FRAME
				 for receiving normal dataframes.
Return value:	R_CAN_OK			Action completed successfully.
				R_CAN_SW_BAD_MBX	Bad mailbox number.
				R_CAN_BAD_CH_NR		The channel number does not exist.
				R_CAN_SW_SET_TX_TMO	Waiting for previous transmission to finish
									timed out.
				R_CAN_SW_SET_RX_TMO	Waiting for previous reception to complete
									timed out.
*******************************************************************************/
u32 R_CAN_RxSet(	const u32	a_u32_chNum,
					const u32	a_u32_mboxNum,
					const u32	a_u32_sid,
					const u32	a_u32_frameType)
{
	u32 a_u32_apiStatus = (R_CAN_OK);
	volatile struct st_can __evenaccess * p_canBlock;

	if (a_u32_mboxNum > 31)
	{
		return	(R_CAN_SW_BAD_MBX);
	}

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

	/* Wait for any previous transmission/reception to complete. */
	a_u32_apiStatus = R_CAN_WaitTxRx(0, a_u32_mboxNum);

	/* Interrupt disable the mailbox. */
	p_canBlock->MIER.LONG &= ~(u32_CAN_mboxArray[a_u32_mboxNum]);

	/* Clear message mailbox control register. */
	p_canBlock->MCTL[a_u32_mboxNum].BYTE = 0;

	/*** Set Mailbox. ***/
	/* Set mailbox standard ID. */
	p_canBlock->MB[a_u32_mboxNum].ID.BIT.SID = a_u32_sid;

	/* Dataframe = 0, Remote frame = 1	*/
	if ( a_u32_frameType == (REMOTE_FRAME) )
	{
		p_canBlock->MB[a_u32_mboxNum].ID.BIT.RTR = 1;
	}
	else
	{
		p_canBlock->MB[a_u32_mboxNum].ID.BIT.RTR = 0;
	}

	/* Frame select: Standard = 0, Extended = 1 */
	p_canBlock->MB[a_u32_mboxNum].ID.BIT.IDE = 0;
	/********************/

#if USE_CAN_POLL == FALSE
	/* Interrupt enable the mailbox */
	p_canBlock->MIER.LONG |= (u32_CAN_mboxArray[a_u32_mboxNum]);
#endif

	/* Request to receive the frame with RecReq bit. */
	//p_canBlock->MCTL[a_u32_mboxNum].BIT.RX.recreq = 1; need to also reset newdata:
	p_canBlock->MCTL[a_u32_mboxNum].BYTE = 0x40;

	return	a_u32_apiStatus;
} /* end R_CAN_RxSet() */

/***********************************************************************************************************************
* Function Name: R_CAN_RxSetFIFO
********************************************************************************************************************//**
* @brief Set up a mailbox to receive.
* The API sets up a given FIFO mailbox to receive data frames with the given CAN 11-bit ID. Incoming data frames with
* the same ID will be stored in the mailbox.
* @param[in] ch_nr - CAN channel to use (0-2 MCU dependent).
* @param[in] mb_mode - Normal mailbox (0), FIFO mailbox (1).
* @param[in] mbox_nr - Which CAN mailbox to use. (0-32)
* @param[in] fidcr0_value - The CAN ID which the mailbox should receive.
* @param[in] fidcr1_value - The CAN ID which the mailbox should receive.
* @param[in] mkr6_value - The mask register.
* @param[in] mkr7_value - The mask register.
* @param[in] frame_type 
*  DATA_FRAME      receive a normal data frame.
*  REMOTE_FRAME    receive a remote data frame request.
* @retval R_CAN_OK                Action completed successfully.
* @retval R_CAN_SW_BAD_MBX        Bad mailbox number.
* @retval R_CAN_BAD_CH_NR         The channel number does not exist.
* @retval R_CAN_BAD_MODE          The mode number does not exist.
* @retval CAN_ERR_NOT_FIFO_MODE   Current mailbox mode is not FIFO mailbox mode.
* @details The function will first temporarily interrupt disable the mailbox. Then it sets whether to receive normal
* CAN data frames or remote frame requests. It also sets the FIFO mailbox to the given ID value. Next, it performs
* setting value for mask register, refer to section R_CAN_RxSetMask in the application note for details.
*/
u32 R_CAN_RxSetFIFO(const u32  ch_nr,
                    const u32  mb_mode,
                    const u32  mbox_nr,
                    const u32  fidcr0_value,
                    const u32  fidcr1_value,
                    const u32  frame_type,
                    const u32  mkr6_value,
                    const u32  mkr7_value)
{
    volatile struct st_can R_BSP_EVENACCESS_SFR * can_block_p;
    u32 api_status = R_CAN_OK;

	if ( mbox_nr> 31)
	{
		return	(R_CAN_SW_BAD_MBX);
	}

	if ( ch_nr == 0 )
	{
		/* Point to CAN0 peripheral block. */
		can_block_p = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}


    /* return if not FIFO_MAILBOX_MODE */
    if(FIFO_MAILBOX_MODE != mb_mode )    /* check normal or fifo mode */
    {
        return (CAN_ERR_NOT_FIFO_MODE);
    }

     /* return if mailbox is not receive fifo mailbox(28 -->31) */
    if ((mbox_nr < 28) || (mbox_nr > 31))
    {
        return (R_CAN_SW_BAD_MBX);
    }

    /* Shift to CAN Halt mode only when the CAN operation mode is not Halt or Reset */
    if((CAN_RESET != can_block_p->CTLR.BIT.CANM) && (CAN_HALT != can_block_p->CTLR.BIT.CANM))
    {
        /* Write to fidcr0, fidcr1 and mkr6, mkr7 in CAN reset mode or CAN halt mode. */
        R_CAN_Control(ch_nr, HALT_CANMODE);
    }

    /* In FIFO mailbox mode, change the bits in MIER for the related FIFO only
     * when the RFE bit in RFCR is 0 and the RFEST flag in RFCR is 1 */
    if((can_block_p->RFCR.BIT.RFE == 0) && (can_block_p->RFCR.BIT.RFEST == 1))
    {
        /* Interrupt disable the mailbox. */
        can_block_p->MIER.LONG &= (~(u32_CAN_mboxArray[28]));
    }

    /* Data frame = 0, Remote frame = 1    */
    if (REMOTE_FRAME == frame_type)
    {
        can_block_p->FIDCR0.BIT.RTR = 1;
        can_block_p->FIDCR1.BIT.RTR = 1;
    }
    else
    {
        can_block_p->FIDCR0.BIT.RTR = 0;
        can_block_p->FIDCR1.BIT.RTR = 0;
    }

    /*** Set Mailbox ID based on ID mode ***/
    /* Check for XID flag bit set in fidcr0_value argument */
    if (fidcr0_value & XID_MASK)
    {
         /* Set message mailbox buffer Extended ID, masking off temporary XID flag bit. */
        can_block_p->FIDCR0.LONG = (fidcr0_value & (~XID_MASK));
    }
    else
    {
        /* Set message mailbox buffer Standard ID */
        can_block_p->FIDCR0.BIT.SID = (fidcr0_value & SID_MASK);
    }

    /* Set IDE bit depending on if want to receive SID or XID frame. Only for mixed mode. */
    if (MIXED_ID_MODE == can_block_p->CTLR.BIT.IDFM)
    {
        if (fidcr0_value & XID_MASK)
        {
        /* Mixed mode; select to send extended frame. */
        can_block_p->FIDCR0.BIT.IDE = 1;
        }
        else
        {
        /* Always set IDE to 0 (unless mixed mode and extended frame as above). */
        can_block_p->FIDCR0.BIT.IDE = 0;
        }
    }
    else
    {
        /* When not MIXED mode, IDE should be written with 0. */
        can_block_p->FIDCR0.BIT.IDE = 0;
    }

    /* Check for XID flag bit set in fidcr1_value argument */
    if (fidcr1_value & XID_MASK)
    {
         /* Set message mailbox buffer Extended ID, masking off temporary XID flag bit. */
        can_block_p->FIDCR1.LONG = (fidcr1_value & (~XID_MASK));
    }
    else
    {
        /* Set message mailbox buffer Standard ID */
        can_block_p->FIDCR1.BIT.SID = (fidcr1_value & SID_MASK);
    }

    /* Set IDE bit depending on if want to receive SID or XID frame. Only for mixed mode. */
    if (MIXED_ID_MODE == can_block_p->CTLR.BIT.IDFM)
    {
        if (fidcr1_value & XID_MASK)
        {
        /* Mixed mode; select to send extended frame. */
        can_block_p->FIDCR1.BIT.IDE = 1;
        }
        else
        {
        /* Always set IDE to 0 (unless mixed mode and extended frame as above). */
        can_block_p->FIDCR1.BIT.IDE = 0;
        }
    }
    else
    {
        /* When not MIXED mode, IDE should be written with 0. */
        can_block_p->FIDCR1.BIT.IDE = 0;
    }

    if ((EXT_ID_MODE == can_block_p->CTLR.BIT.IDFM) || (MIXED_ID_MODE == can_block_p->CTLR.BIT.IDFM))
    {
        /* Set XID 29-bit mask value in mask register. */
        can_block_p->MKR[6].LONG = (mkr6_value & (~XID_MASK)); /* Set XID 29-bit mask value in mask register 6. */
        can_block_p->MKR[7].LONG = (mkr7_value & (~XID_MASK)); /* Set XID 29-bit mask value in mask register 7. */
    }
    else
    {
        /* Set SID 11-bit mask value in mask register. */
        can_block_p->MKR[6].BIT.SID = mkr6_value; /* Set SID 11-bit mask value in mask register 6. */
        can_block_p->MKR[7].BIT.SID = mkr7_value; /* Set SID 11-bit mask value in mask register 7. */
    }

    /* Set mailbox mask to be used. (0 = mask VALID.).
     *  Set bits 31 to 24 to 0 in FIFO mailbox mode */
    can_block_p->MKIVLR.LONG &= (~(0xFF000000));

    #if (USE_CAN_POLL == 0)
    /* In FIFO mailbox mode, change the bits in MIER for the related FIFO only
     * when the RFE bit in RFCR is 0 and the RFEST flag in RFCR is 1 */
        if((can_block_p->RFCR.BIT.RFE == 0) && (can_block_p->RFCR.BIT.RFEST == 1))
        {
            /* Interrupt enable the mailbox */
            can_block_p->MIER.LONG |= (u32_CAN_mboxArray[28]);

            /* Generate interrupt when Receive FIFO warning */
            if(CAN_CFG_RXFIFO_INT_GEN_TIMING == 1)
            {
                can_block_p->MIER.LONG |= (u32_CAN_mboxArray[29]);
            }
        }
    #endif

    /* Receive FIFO enable */
    can_block_p->RFCR.BIT.RFE = 1;

    return (api_status);
}
/******************************************************************************
 End of function R_CAN_RxSetFIFO
 *****************************************************************************/ /* end R_CAN_RxSetFIFO() */

/*******************************************************************************
Function Name:	R_CAN_RxPoll
Description:	Checks for received message in mailbox.
Parameters:		Channel nr.
				Mailbox nr.
Return value:	R_CAN_OK			There is a message waiting.
				R_CAN_NOT_OK		No message waiting.
				R_CAN_RXPOLL_TMO	Message pending but timed out.
				R_CAN_SW_BAD_MBX	Bad mailbox number.
				R_CAN_BAD_CH_NR		The channel number does not exist.
*******************************************************************************/
u32 R_CAN_RxPoll(const u32 a_u32_chNum, const u32 a_u32_mboxNum)
{
	u32 a_u32_apiStatus = R_CAN_NOT_OK;
	u32 poll_delay = MAX_CAN_REG_POLLTIME;
	volatile struct st_can __evenaccess * p_canBlock;

	if (a_u32_mboxNum > 31)
	{
		return	(R_CAN_SW_BAD_MBX);
	}

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

	/* Wait if new data is currently being received. */
	while ( (p_canBlock->MCTL[a_u32_mboxNum].BIT.RX.INVALDATA) && poll_delay )
	{
		poll_delay--;
	}
	if ( poll_delay == 0 )
	/* Still updating mailbox. Come back later. */
	{
		a_u32_apiStatus = (R_CAN_RXPOLL_TMO);
	}
	else /* Message received? */
	{
		/* If message received, tell user. */
		if ( p_canBlock->MCTL[a_u32_mboxNum].BIT.RX.NEWDATA == 1 )
		{
			a_u32_apiStatus = (R_CAN_OK);
		}
	}
	return	a_u32_apiStatus;
}/* end R_CAN_RxPoll() */

/*******************************************************************************
Function Name:	R_CAN_RxRead
Parameters:		Mailbox nr.
				p_frame: Data frame structure
Description:	Call from CAN receive interrupt. Copies received data from
				message mailbox to memory.
Return value:	R_CAN_OK			There is a message waiting.
				R_CAN_SW_BAD_MBX	Bad mailbox number.
				R_CAN_BAD_CH_NR		The channel number does not exist.
				R_CAN_MSGLOST	 Message was overwritten or lost.
*******************************************************************************/
u32 R_CAN_RxRead(	const u32			a_u32_chNum,
					const u32			a_u32_mboxNum,
					ST_CAN_STD_FRAME * p_frame	)
{
	u32 i;
	u32 a_u32_apiStatus = (R_CAN_OK);
	volatile struct st_can __evenaccess * p_canBlock;

	if (a_u32_mboxNum > 31)
	{
		return	(R_CAN_SW_BAD_MBX);
	}

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

	/* Copy received data from message mailbox to memory */
	p_frame->u32_canId = p_canBlock->MB[a_u32_mboxNum].ID.BIT.SID;
//	p_frame->u8_canDlc = p_canBlock->MB[a_u32_mboxNum].DLC.BIT.DLC;
	p_frame->u8_canDlc = p_canBlock->MB[a_u32_mboxNum].DLC;
//	for ( i = 0; i < p_canBlock->MB[a_u32_mboxNum].DLC.BIT.DLC; i++ )
	for ( i = 0; i < p_canBlock->MB[a_u32_mboxNum].DLC; i++ )
	{
		p_frame->u8_canData[i] = p_canBlock->MB[a_u32_mboxNum].DATA[i];
	}

	/* Check if message was lost/overwritten. */
	if ( p_canBlock->MCTL[a_u32_mboxNum].BIT.RX.MSGLOST )
	{
		p_canBlock->MCTL[a_u32_mboxNum].BIT.RX.MSGLOST = 0;
		a_u32_apiStatus = (R_CAN_MSGLOST);
	}

	/* Set NEWDATA bit to 0 since the mailbox was just emptied and start
	over with new RxPolls. */
	p_canBlock->MCTL[a_u32_mboxNum].BIT.RX.NEWDATA = 0;

	return	a_u32_apiStatus;
}/* end R_CAN_RxRead() */

/*******************************************************************************
Function Name:	R_CAN_RxSetMask
Description:	Set a CAN bus mask for specified mask register. Note that the
				MKIVLR register is used to disable the acceptance filtering
				function individually for each mailbox.
Parameters:		a_u32_chNum
				SID mask value. For each bit that is 1; corresponding SID bit
								is compared.
				a_u32_mboxNum			0-31. The mailbox nr translates to mask_reg_nr:
								0 for mailboxes 0-3
								1 for mailboxes 4-7
								2 for mailboxes 8-11
								3 for mailboxes 12-15
								4 for mailboxes 16-19
								5 for mailboxes 20-23
								6 for mailboxes 24-27
								7 for mailboxes 28-31
Return value:	-
*******************************************************************************/
void R_CAN_RxSetMask(	const u32 a_u32_chNum,
						const u32 a_u32_mboxNum,
						const u32 a_u32_sidMaskValue)
{
	volatile struct st_can __evenaccess * p_canBlock;

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return;
	}

	/* Write to MKR0 to MKR7 in CAN reset mode or CAN halt mode. */
	R_CAN_Control(0, HALT_CANMODE);

	/* Set mask for the goup of mailboxes. */
	p_canBlock->MKR[a_u32_mboxNum/4].BIT.SID = a_u32_sidMaskValue;

	/* Set mailbox mask to be used. (0 = mask VALID.) */
//	p_canBlock->MKIVLR &= ~(u32_CAN_mboxArray[a_u32_mboxNum]);
	p_canBlock->MKIVLR.LONG &= ~(u32_CAN_mboxArray[a_u32_mboxNum]);

	R_CAN_Control(0, OPERATE_CANMODE);

}/* end R_CAN_RxSetMask() */

/*****************************************************************************
Name:			R_CAN_WaitTxRx
Parameters:		Channel nr.
				Mailbox nr.
Description:	Wait for communicating mailbox to complete action. This would
				be apporopriate for example if a mailbox all of a sudden needs
				to be reconfigured but the user wants any pending receive or
				transmit to finish.
Return value:	R_CAN_OK			There is a message waiting.
				R_CAN_SW_BAD_MBX	Bad mailbox number.
				R_CAN_BAD_CH_NR		The channel number does not exist.
				R_CAN_SW_SET_TX_TMO	Waiting for previous transmission to finish
									timed out.
				R_CAN_SW_SET_RX_TMO	Waiting for previous reception to complete
									timed out.
*****************************************************************************/
#pragma inline(R_CAN_WaitTxRx)
static u32 R_CAN_WaitTxRx(const u32 a_u32_chNum, const u32 a_u32_mboxNum)
{
	u32 a_u32_apiStatus = (R_CAN_OK);
	u32 a_u32_canTimeoutCnt = (MAX_CAN_SW_DELAY);
	volatile struct st_can __evenaccess * p_canBlock;

	if (a_u32_mboxNum > 31)
	{
		return	(R_CAN_SW_BAD_MBX);
	}

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

	/* Wait for any previous transmission to complete. */
	if ( p_canBlock->MCTL[a_u32_mboxNum].BIT.TX.TRMREQ )
	{
		while ( (p_canBlock->MCTL[a_u32_mboxNum].BIT.TX.SENTDATA == 0) && (DEC_CHK_CAN_SW_TMR) )
		{
			;
		}
		if ( a_u32_canTimeoutCnt == 0 )
		{
			a_u32_apiStatus = R_CAN_SW_SET_TX_TMO;
		}
	}
	/* Wait for any previous reception to complete. */
	else if ( p_canBlock->MCTL[a_u32_mboxNum].BIT.TX.RECREQ )	//Strange; but iodefine has TX here..
	{
		while ( (p_canBlock->MCTL[a_u32_mboxNum].BIT.RX.INVALDATA == 1) && (DEC_CHK_CAN_SW_TMR) )
		{
			;
		}
		if ( a_u32_canTimeoutCnt == 0 )
		{
			a_u32_apiStatus = (R_CAN_SW_SET_RX_TMO);
		}
	}
	else
	{
	}
	return	a_u32_apiStatus;
}/* end R_CAN_WaitTxRx() */

/*******************************************************************************
Function Name:	R_CAN_CheckErr
Description:	Checks CAN peripheraol error state.
Parameters:		-
Return value:	0 = No error
				1 = CAN is in error active state
				2 = CAN is in error passive state
				4 = CAN is in bus-off state
*******************************************************************************/
u32 R_CAN_CheckErr(const u32	a_u32_chNum)
{
	u32 a_u32_apiStatus = R_CAN_STATUS_ERROR_ACTIVE; /* Store return	value */
	volatile struct st_can __evenaccess * p_canBlock;

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return	(R_CAN_BAD_CH_NR);
	}

	/* Check CAN error state */
	if ( p_canBlock->STR.BIT.EST )
	{
		/* Check error-passive state */
		if ( p_canBlock->STR.BIT.EPST )
		{
			a_u32_apiStatus = (R_CAN_STATUS_ERROR_PASSIVE);
		}

		/* Check bus-off state */
		else if ( p_canBlock->STR.BIT.BOST )
		{
			a_u32_apiStatus = (R_CAN_STATUS_BUSOFF);
		}
		else
		{
		}
	}

	return	a_u32_apiStatus;
} /* end R_CAN_CheckErr() */

/*******************************************************************************
Function Name:	R_CAN_SetBitrate
Description:	Sets clock speed and bit rate for CAN as defined in
				config.h.
Parameters:	 -
Return value:	-
*******************************************************************************/
void R_CAN_SetBitrate(const u32 a_u32_chNum)
{
	volatile struct st_can __evenaccess * p_canBlock;

	if ( a_u32_chNum == 0 )
	{
		/* Point to CAN0 peripheral block. */
		p_canBlock = (struct st_can __evenaccess *) 0x90200;
	}
	else
	{
		/* Point to CAN1 peripheral block. Not implemented this MCU type. */
		return;
	}

	/* Set TSEG1, TSEG2 and SJW. */
	p_canBlock->BCR.BIT.CCLKS = 0;					/* CAN クロックソース選択ビット */
	p_canBlock->BCR.BIT.BRP = CAN_BRP - 1;			/* プリスケーラ分周比選択ビット */
	p_canBlock->BCR.BIT.TSEG1 = CAN_TSEG1 - 1;		/* タイムセグメント1 制御ビット */
	p_canBlock->BCR.BIT.TSEG2 = CAN_TSEG2 - 1;		/* タイムセグメント2 制御ビット */
	p_canBlock->BCR.BIT.SJW = CAN_SJW - 1;			/* 再同期ジャンプ幅制御ビット */
}/* end R_CAN_SetBitrate() */


/**********************************************************************************
Function Name:	R_CAN_ConfigCANinterrupts
Description	 :	Configuration of CAN interrupts.

				CAN0 interrupts:
				Source	Name	Vector	Address		IER, BIT		IPR
								nr.		offset
				============================================================
				CAN0	ERS0	106		1A8		GENBE0, EN0 	IPR106
				"		RXF0	170		2A8		IER15, IEN2		IPR170
				"		TXM0	171		2AC		IER15, IEN3		IPR170
				"		RXM0	172		2B0		IER15, IEN4		IPR170
				"		TXM0	173		2B4		IER15, IEN5		IPR170
				============================================================


				CAN1 interrupts:
				Source	Name	Vector	Address		IER, BIT		IPR
								nr.		offset
				============================================================

Parameters	 :	-
Return value :	-
***********************************************************************************/
void R_CAN_ConfigCANinterrupts(const u32 a_u32_chNum)
{
	if ( a_u32_chNum == 0 )
	{

#if ( USE_CAN_POLL == 0 )
		/* Configure CAN Tx interrupt. */
		//ICU.IER[IER_CAN0_TXM0].BIT.IEN4 = 1;
		//ICU.IPR[IPR_CAN0_TXM0].BIT.IPR = 3;
		/* ..or use the macros! */
//		IEN(CAN0, TXM0) = 1;	//1 = interrupt enabled.
//		IPR(CAN0, TXM0) = CAN0_INT_LVL;	//priority

		/* Configure CAN Rx interrupt. */
		IEN(CAN0, RXM0) = 1;
		IPR(CAN0, RXM0) = (CAN0_INT_LVL);

		/* Configure CAN Error interrupt. */
//		IEN(CAN0, ERS0) = 1;
//		IPR(CAN0, ERS0) = CAN0_INT_LVL;
//		CAN0.EIER.BYTE = 0xFF;

		/* RX CAN0 uses:
		- Interrupt Priority Register 18, IPR18.
		- Interrupt Request Enable Register 7, IER07.
		- Vector 170 RXF0, and 171 TXM0.(RX66T) */
	//	- Vector 57 RXF0, and 58 TXM0.(RX62T) */
	//	ICU.IPR[18].BIT.IPR = CAN0_INT_LVL;
		ICU.IPR[170].BIT.IPR = CAN0_INT_LVL;
		/* Interrupt enable bit per mailbox (p_canBlock->MIER.LONG) set by
		R_CAN_TxSet() and R_CAN_RxSet() */
#endif
		/* Mailbox interrupt enable registers. Disable interrupts for all slots.
		They will be enabled individually by the API. */
		CAN0.MIER.LONG = 0x00000000;

	}
}/* end R_CAN_ConfigCANinterrupts() */

/*********************************************************************************

		Used CAN interrupts are normally in application. Below are templates.

**********************************************************************************/
#if 0
/*****************************************************************************
Name:			CAN0_TXM0_ISR
Parameters:		-
Returns:		-
Description:	CAN0 Transmit interrupt.
				Check which mailbox transmitted data and process it.
*****************************************************************************/
//#pragma interrupt CAN0_TXM0_ISR(vect=VECT_CAN0_TXM0, enable)
#if FAST_INTERRUPT_VECTOR == VECT_CAN0_TXM0
#pragma interrupt CAN0_TXM0_ISR(vect=VECT_CAN0_TXM0,fint)
#else
#pragma interrupt CAN0_TXM0_ISR(vect=VECT_CAN0_TXM0)
#endif
void CAN0_TXM0_ISR(void)
{
	u32 a_u32_apiStatus = (R_CAN_OK);

	a_u32_apiStatus = R_CAN_TxCheck(CAN_CH0, CANBOX_TX);
	if (a_u32_apiStatus == (R_CAN_OK))
		can0_tx_sentdata_flag = 1;
	/* Use mailbox search reg. Should be faster than above if a lot of mail-
	boxes to check. Not verified. */
}/* end CAN0_TXM0_ISR() */
#endif


#if 0
/*****************************************************************************
Name:			CAN0_ERS0_ISR
Parameters:		-
Returns:		-
Description:	CAN0 Error interrupt.
*****************************************************************************/
//#pragma interrupt	CAN0_ERS0_ISR(vect=VECT_CAN0_ERS0, enable)
#if FAST_INTERRUPT_VECTOR == VECT_CAN0_ERS0
#pragma interrupt CAN0_ERS0_ISR(vect=VECT_CAN0_ERS0,fint)
#else
#pragma interrupt CAN0_ERS0_ISR(vect=VECT_CAN0_ERS0)
#endif
void CAN0_ERS0_ISR(void)
{
	R_BSP_NOP();
}/* end CAN0_ERS0_ISR() */


/*****************************************************************************
Name:			CAN0_RXF0_ISR
Parameters:		-
Returns:		-
Description:	CAN0 Rx Fifo interrupt.
*****************************************************************************/
//#pragma interrupt	CAN0_RXF0_ISR(vect=VECT_CAN0_RXF0, enable)
#if FAST_INTERRUPT_VECTOR == VECT_CAN0_RXF0
#pragma interrupt CAN0_RXF0_ISR(vect=VECT_CAN0_RXF0,fint)
#else
#pragma interrupt CAN0_RXF0_ISR(vect=VECT_CAN0_RXF0)
#endif
void CAN0_RXF0_ISR(void)
{
	 R_BSP_NOP();
}/* end CAN0_RXF0_ISR() */

/*****************************************************************************
Name:			CAN0_TXF0_ISR
Parameters:		-
Returns:		-
Description:	CAN0 Tx Fifo interrupt.
*****************************************************************************/
//#pragma interrupt	CAN0_TXF0_ISR(vect=VECT_CAN0_TXF0, enable)
#if FAST_INTERRUPT_VECTOR == VECT_CAN0_TXF0
#pragma interrupt CAN0_TXF0_ISR(vect=VECT_CAN0_TXF0,fint)
#else
#pragma interrupt CAN0_TXF0_ISR(vect=VECT_CAN0_TXF0)
#endif
void CAN0_TXF0_ISR(void)
{
	 R_BSP_NOP();
}/* end CAN0_TXF0_ISR() */

#endif //USE_CAN_POLL
/* eof */
