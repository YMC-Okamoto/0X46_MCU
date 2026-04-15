/*==========================================================================*/
/**
 * @file	TRIGONOMETRIC_FUNCTION.c
 * @brief	ŽOŠpŠÖ”‰‰ŽZŒQ
 * @author	MCU GroupFAtsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2025.01
 */
/*==========================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "TRIGONOMETRIC_FUNCTION.h"
#include "SIN_TABLE.h"

/* Private define ------------------------------------------------------------*/

/*
 * @brief	SINƒe[ƒuƒ‹ŽQÆ
 * @param	a_u16_eangle_0p03125deg
 * @return	a_s16_result
 */
s16	S16_TRIG_Sin(u16 a_u16_eangle_0p03125deg)
{
	s16 a_s16_result = (s16)0;

	if ( a_u16_eangle_0p03125deg > (U16_ENC_EANGLE_MAX_0P03125DEG) )
	{
		a_u16_eangle_0p03125deg -= (U16_ENC_EANGLE_MAX_0P03125DEG);
		if ( a_u16_eangle_0p03125deg > (U16_ENC_EANGLE_MAX_0P03125DEG) )
		{
			a_u16_eangle_0p03125deg -= (U16_ENC_EANGLE_MAX_0P03125DEG);
		}
	}

	if( a_u16_eangle_0p03125deg <= (U16_ENC_90DEG_EXP5) )						/* ‘æ‚PÛŒÀF0`0x0B40(2880) */
	{
		a_s16_result = s16_TAB_Sin[a_u16_eangle_0p03125deg];
	}
	else if( a_u16_eangle_0p03125deg <= (U16_ENC_180DEG_EXP5) )					/* ‘æ‚QÛŒÀF0x0B41`0x1680(5760) */
	{
		a_u16_eangle_0p03125deg = (u16)( (U16_ENC_180DEG_EXP5) - a_u16_eangle_0p03125deg );
		a_s16_result = s16_TAB_Sin[a_u16_eangle_0p03125deg];
	}
	else if( a_u16_eangle_0p03125deg <= (U16_ENC_270DEG_EXP5) )					/* ‘æ‚RÛŒÀF0x8000`0x21C0(8640) */
	{
		a_u16_eangle_0p03125deg = (u16)( a_u16_eangle_0p03125deg - (U16_ENC_180DEG_EXP5) );
		a_s16_result = (s16)((s32)s16_TAB_Sin[a_u16_eangle_0p03125deg] * (s32)(-1));
	}
	else																		/* ‘æ‚SÛŒÀF0xC000`0xFFFF(65535) */
	{
		a_u16_eangle_0p03125deg = (u16)( (U16_ENC_360DEG_EXP5) - a_u16_eangle_0p03125deg );
		a_s16_result = (s16)((s32)s16_TAB_Sin[a_u16_eangle_0p03125deg] * (s32)(-1));
	}
	return	a_s16_result;
}

