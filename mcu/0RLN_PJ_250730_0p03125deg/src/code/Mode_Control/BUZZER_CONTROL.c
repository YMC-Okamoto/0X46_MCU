/**
 * @file BUZZER_CONTROL.c
 * @brief ブザー制御
 * @author	 Actuation Group：Asai Souta
 * @date 2024.2
 */
/* Includes ------------------------------------------------------------------*/
#include "MCU.h"
#include "BUZZER_CONTROL.h"										/* ブザー制御 */
#include "CAN.h"

/* Private function prototypes -----------------------------------------------*/

static EN_BUZZER_BUZZING_PATTERN en_BUZZER_pattern = BUZZING_NONE;				/* ブザー鳴動パターン　初期値=鳴動なし */
static u8 u8_BUZZER_cycleTimeCount100ms = 0;

#define BUZZER_BYTENUMBER 4

/**
 * @brief ブザーサイクル制御
 * @detail 各モード毎のブザー制御
 * @note 100[ms]sec周期
 * @param none
 * @return none
 */
void V_BUZZER_Loop100ms(void)
{
	en_BUZZER_pattern = (EN_BUZZER_BUZZING_PATTERN)U8_CAN_GetBuzzerPattern();	/* VCU指示値受け取り */

	switch(en_BUZZER_pattern)
	{
	case BUZZING_NONE :															/* 鳴動なし */
		PO_BUZZER_OFF();														/* ブザー止める */
		u8_BUZZER_cycleTimeCount100ms = 0;												/* カウンタクリア */
		break;

	case BUZZING_PATTERN1 :														/* 鳴らす */
			PO_BUZZER_ON();
		break;
	default :																	/* 1秒毎に鳴らす */
		if ( u8_BUZZER_cycleTimeCount100ms > 20 )
		{
			u8_BUZZER_cycleTimeCount100ms = 0;
		}
		else
		if ( u8_BUZZER_cycleTimeCount100ms > 10 )
		{
			u8_BUZZER_cycleTimeCount100ms ++;
			PO_BUZZER_OFF();
		}
		else
		{
			u8_BUZZER_cycleTimeCount100ms ++;
			PO_BUZZER_ON();
		}	
		break;
	}

}
