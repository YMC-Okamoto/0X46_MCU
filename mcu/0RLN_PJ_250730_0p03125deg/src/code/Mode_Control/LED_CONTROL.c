/**
 * @file LED_CONTROL.c
 * @brief LED表示制御
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
/* Includes ------------------------------------------------------------------*/
#include "MCU.h"
#include "pin.h"
#include "TYPEDEF_MODE.h"
#include "DEFINE_SETTING.h"
#include "DEFINE_ABN.h"
#include "MODE_CONDUCTOR.h"
#include "DIAG_SET.h"
#include "LED_CONTROL.h"										/* LED表示制御 */

/* Private define ------------------------------------------------------------*/
#define U8_LED_INTERVAL			(u8)3							/* LED無表示インターバル	300[ms] × 3 ＝ 900[ms] */
#define U8_LED_ONOFF_TERM		(u8)3							/* ON/OFF周期(異常時)	100[ms] × 3 ＝ 300[ms] */
/* Private variables ---------------------------------------------------------*/
static u8	u8_LED_termCounter		= (u8)0;					/* ON/OFF周期カウント用カウンタ */
static u8	u8_LED_intervalCounter	= (u8)0;					/* 無表示インターバルカウント用カウンタ */
static u8	u8_LED_errorBlinkNum	= (u8)0;					/* LED点滅回数 */
static u8	u8_LED_errorBlinkNumBuf	= (u8)0;					/* LED点滅回数バッファ */
static bool	b_LED_idleTermFlg		= FALSE;					/* IDLE時ON/OFF周期カウント用 */
static u16	u16_LED_diagCodeArray[5]= { (u16)0, (u16)0, (u16)0, (u16)0, (u16)0 };
static volatile u16	u16_LED_diagCode0	= (u16)0;

static enum{
	EN_LED_INIT_STATE = 0,
	EN_LED_ON_STATE,
	EN_LED_OFF_STATE,
	EN_LED_INTERVAL_STATE,
}en_LED_state = EN_LED_INIT_STATE;

/* Private function prototypes -----------------------------------------------*/
static void v_LED_LedDisplayAtNormal(void);						/* 正常時LED表示 */
static void v_LED_LedDisplayAtIdle(void);						/* アイドル時LED表示 */
static void v_LED_LedDisplayAtAbnormal(void);					/* 異常時LED表示 */
static void v_LED_LedDisplayAtBoot(void);						/* 起動時LED表示 */
static void v_LED_LedDisplayAtShutdown(void);					/* 遮断時LED表示 */

/**
 * @brief LED表示制御
 * @detail 各モード毎のLED表示
 * @note 100[ms]sec周期
 * @param none
 * @return none
 */
void V_LED_Loop100ms(void)
{
	EN_MDC_MODE	a_en_mode	= En_MDC_GetMode();					/* 動作モード取得 */

/*======= 動作モードにより選択 ======*/
	switch( a_en_mode )
	{
	case MODE_ABNORMAL:
		v_LED_LedDisplayAtAbnormal();
		break;
	case MODE_RUN:
		v_LED_LedDisplayAtNormal();
		en_LED_state = EN_LED_INIT_STATE;
		break;
	case MODE_IDLE:
		v_LED_LedDisplayAtIdle();
		en_LED_state = EN_LED_INIT_STATE;
		break;
	case MODE_STARTUP:
	case MODE_STANDBY:
		v_LED_LedDisplayAtBoot();
		en_LED_state = EN_LED_INIT_STATE;
		break;
	case MODE_SHUTDOWN_WAIT:
	case MODE_SHUTDOWN:
		v_LED_LedDisplayAtShutdown();
		en_LED_state = EN_LED_INIT_STATE;
		break;
	default:
		v_LED_LedDisplayAtNormal();
		en_LED_state = EN_LED_INIT_STATE;
		break;
	}
}

/**
 * @brief	正常時LED表示
 * @param	none
 * @note	100msec on / 100msec OFF
 */
static void v_LED_LedDisplayAtNormal(void)
{
	if( PI_LED() == (HIGH) )
	{
		PO_LED_OFF();
	}
	else
	{
		PO_LED_ON();
	}
}

/**
 * @brief	IDLE時LED表示
 * @param	none
 * @note	100msec on / 100msec OFF
 */
static void v_LED_LedDisplayAtIdle(void)
{
	b_LED_idleTermFlg = !b_LED_idleTermFlg;
	if( b_LED_idleTermFlg == FALSE )
	{
		return;
	}

	if( PI_LED() == (HIGH) )
	{
		
		PO_LED_OFF();
	}
	else
	{
		PO_LED_ON();
	}
}

/**
 * @brief	異常時LED表示
 * @param	none
 * @note	一つの異常のみの表示
 */
static void v_LED_LedDisplayAtAbnormal(void)
{
	u16	*a_pu16_diagCode;

	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	u8_LED_termCounter++;
	if( u8_LED_termCounter < (U8_LED_ONOFF_TERM) )
	{
		return;
	}
	u8_LED_termCounter = (u8)0;

	switch( en_LED_state )
	{
	case EN_LED_INIT_STATE:
		PO_LED_OFF();
		a_pu16_diagCode = Pu16_DIAG_GetDiagCode();
		for( u8 idx = (u8)0; idx < (u8)5; idx++ )
		{
			u16_LED_diagCodeArray[idx] = *(a_pu16_diagCode + idx);
		}
		u16_LED_diagCode0 = u16_LED_diagCodeArray[0];
		u8_LED_errorBlinkNumBuf = (u8)u16_LED_diagCodeArray[0];
		u8_LED_errorBlinkNum = (u8)0;
		u8_LED_termCounter = (u8)0;
		u8_LED_intervalCounter = (u8)0;
		en_LED_state = EN_LED_ON_STATE;
	/*	break;		*/											/* あえてコメント化し、EN_LED_ON_STATEへ */

	case EN_LED_ON_STATE:
		PO_LED_ON();
		u8_LED_errorBlinkNum++;
		u8_LED_intervalCounter = (u8)0;
		en_LED_state = EN_LED_OFF_STATE;
		break;

	case EN_LED_OFF_STATE:
		PO_LED_OFF();
		u8_LED_intervalCounter = (u8)0;
		if( u8_LED_errorBlinkNum >= u8_LED_errorBlinkNumBuf )
		{
			u8_LED_errorBlinkNum = (u8)0;
			en_LED_state = EN_LED_INTERVAL_STATE;
		}
		else
		{
			en_LED_state = EN_LED_ON_STATE;
		}
		break;

	case EN_LED_INTERVAL_STATE:
		PO_LED_OFF();
		u8_LED_intervalCounter++;
		if( u8_LED_intervalCounter >= (U8_LED_INTERVAL) )
		{
			u8_LED_errorBlinkNum = (u8)0;
			u8_LED_intervalCounter = (u8)0;
			en_LED_state = EN_LED_ON_STATE;
		}
		break;

	default:
		break;
	}
}

/**
 * @brief	起動時LED表示
 * @param	none
 * @note	Always on
 */
static void v_LED_LedDisplayAtBoot(void)
{
	PO_LED_ON();
	en_LED_state = EN_LED_ON_STATE;								/* 起動時のLEDステート初期化 */
}

/**
 * @brief	遮断時LED表示
 * @param	none
 * @note	Always on
 */
static void v_LED_LedDisplayAtShutdown(void)
{
	PO_LED_OFF();
	en_LED_state = EN_LED_ON_STATE;								/* 遮断時のLEDステート初期化 */
}

