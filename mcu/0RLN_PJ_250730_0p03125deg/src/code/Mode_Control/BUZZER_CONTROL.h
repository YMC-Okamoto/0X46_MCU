/*==========================================================================*/
/**
 * @file	BUZZER_CONTROL.h
 * @brief	ブザー制御
 * @author	 Actuation Group：Asai Souta
 * @date 2024.2
 */
/*==========================================================================*/
#ifndef __BUZZER_CONTROL_H
#define __BUZZER_CONTROL_H
/* Includes ------------------------------------------------------------------*/
#include "r_cg_macrodriver.h"
#include "ctypedef.h"

/* Private define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/
typedef enum{                                                                   /* 鳴動指示 */
    BUZZING_NONE = 0,                                                           /* 鳴らさない */
    BUZZING_PATTERN1,                                                           /* パターン1 */
    BUZZING_PATTERN2,
    BUZZING_PATTERN3,
    BUZZING_PATTERN4,
    BUZZING_PATTERN5,
    BUZZING_PATTERN6,
    BUZZING_PATTERN7,
    BUZZING_PATTERN8,
    BUZZING_PATTERN9,
    BUZZING_PATTERN10,
    BUZZING_PATTERN11,
    BUZZING_PATTERN12,
    BUZZING_PATTERN13,
    BUZZING_PATTERN14,
    BUZZING_PATTERN15,
} EN_BUZZER_BUZZING_PATTERN;

/* Private variables ---------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/
extern void V_BUZZER_Loop100ms(void);					/* ブザー制御 */

#endif /* __LED_CONTROL_H */
