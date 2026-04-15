/*==========================================================================*/
/**
 * @file	MCU.h
 * @brief	MCU(Motor Control Unit)定義ヘッダ
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 */
/*==========================================================================*/
#ifndef __MCU_H
#define	__MCU_H

#include "ctypedef.h"

#define	_00_Pm1_OUTPUT_0         (0x00U) /* output low at B1 */
#define	_02_Pm1_OUTPUT_1         (0x02U) /* output high at B1 */
/*--------------------------------------------------------------------------*/
/* 定義																		*/
/*--------------------------------------------------------------------------*/

#define	PO_UL_LO()				(PORT7.PODR.BIT.B4 = (u8)(LOW))												/* _U相出力ポートdata P7_4 */
#define	PO_VL_LO()				(PORT7.PODR.BIT.B5 = (u8)(LOW))												/* _V相出力ポートdata P7_5 */
#define	PO_WL_LO()				(PORT7.PODR.BIT.B6 = (u8)(LOW))												/* _W相出力ポートdata P7_6 */

#define	PO_UVWH_LO()			(PORT7.PODR.BYTE = ((_00_Pm1_OUTPUT_0) | (_00_Pm2_OUTPUT_0) | (_00_Pm3_OUTPUT_0)))
#define	PO_UVWL_HI()			(PORT7.PODR.BYTE = ((_10_Pm4_OUTPUT_1) | (_20_Pm5_OUTPUT_1) | (_40_Pm6_OUTPUT_1)))

#define	PO_UL_HI()				(PORT7.PODR.BIT.B4 = (u8)(HIGH))											/* _U相出力ポートdata P7_4 */
#define	PO_VL_HI()				(PORT7.PODR.BIT.B5 = (u8)(HIGH))											/* _V相出力ポートdata P7_5 */
#define	PO_WL_HI()				(PORT7.PODR.BIT.B6 = (u8)(HIGH))											/* _W相出力ポートdata P7_6 */

#define	PO_E_FET_ON()			(PORT9.PODR.BIT.B1 = (u8)(HIGH))											/* 界磁出力ON  data P9_1 */
#define	PO_E_FET_OFF()			(PORT9.PODR.BIT.B1 = (u8)(LOW))												/* 界磁出力OFF data P9_1 */

#define	SELF_HOLD_ON()			(PORT.P8 |= (_PORT_SET_BIT2))												/* 自己保持回路出力ポートHi P8_2 */
#define	SELF_HOLD_OFF()			(PORT.P8 &= (_PORT_CLEAR_BIT2))												/* 自己保持回路出力ポートLow P8_2 */

#define	MAIN_RELAY_ON()			(PORT8.PODR.BIT.B1 = (u8)(HIGH))											/* メインリレー出力ポートHi P8_1 */
#define	MAIN_RELAY_OFF()		(PORT8.PODR.BIT.B1 = (u8)(LOW))												/* メインリレー出力ポートLow P8_1 */

#define	PRECHA_FET_ON()			(PORT8.PODR.BIT.B0 = (u8)(HIGH))											/* プリチャージFET出力ポートHi P8_0 */
#define	PRECHA_FET_OFF()		(PORT8.PODR.BIT.B0 = (u8)(LOW))												/* プリチャージFET出力ポートLow P8_0 */

/* LED */
#define	PO_LED_OFF()			(PORT1.PODR.BIT.B1 = (u8)(LOW))												/* LED出力ポートOFF data P1_1 */
#define	PO_LED_ON()				(PORT1.PODR.BIT.B1 = (u8)(HIGH))											/* LED出力ポートON  data P1_1 */

/* BUZZER */
#define	PO_BUZZER_OFF()			(PORT9.PODR.BIT.B6 = (u8)(LOW))												/* ブザー出力ポートOFF data P9_6 */
#define	PO_BUZZER_ON()			(PORT9.PODR.BIT.B6 = (u8)(HIGH))											/* ブザー出力ポートON  data P9_6 */

/* 汎用入出力ポートLOW出力 */
#define	PO_GPIO2_LO()			(PORT1.PODR.BIT.B0 = (u8)(LOW))												/* 汎用入出力ポートdata P1_0 */
#define	PO_GPIO3_LO()			(PORTE.PODR.BIT.B5 = (u8)(LOW))												/* 汎用入出力ポートdata PE_5 */

/* 汎用入出力ポートHIGH出力 */
#define	PO_GPIO2_HI()			(PORT1.PODR.BIT.B0 = (u8)(HIGH))											/* 汎用入出力ポートdata P1_0 */
#define	PO_GPIO3_HI()			(PORTE.PODR.BIT.B5 = (u8)(HIGH))											/* 汎用入出力ポートdata PE_5 */

#define	U8_ENC_VALUE()			(u8)(((PORTB.PIDR.BYTE) & 0x0E) >> (u8)1)									/* ポートB bit3(U),bit2(V),bit1(W)の値を読み込みシフト */

#define	MAIN_RELAY_STATE		(u8)(PORT8.PODR.BIT.B1)														/* メインリレーポート */
#define	PRECHA_FET_STATE		(u8)(PORT8.PODR.BIT.B0)														/* プリチャージFETポート */

#define	PORT8_STATE				(PORT8.PODR.BYTE)															/* ポート8状態 */

#define	PI_ACCEL_SW()			(u8)(PORTB.PIDR.BIT.B0)  													/* アクセル SW入力ポート PB_0 */
#define	PI_KEY_SW()				(u8)(PORTA.PIDR.BIT.B5)  													/* キーSW入力ポート PA_5 */
#define	PI_BACK_SW()			(u8)(PORTA.PIDR.BIT.B4)  													/* BACK SW入力ポート PA_4 */
#define	PI_FWD_SW()				(u8)(PORTA.PIDR.BIT.B3)  													/* FWD SW入力ポート PA_3 */
#define	PI_BR()					(u8)(PORTA.PIDR.BIT.B2)  													/* BR 入力ポート PA_2 */
#define	PI_CHG()				(u8)(PORTA.PIDR.BIT.B0)  													/* CHARGER 入力ポート PA_0 */
#define	PI_LED()				(u8)(PORT1.PIDR.BIT.B1)  													/* LEDポート P1_1 */

/*  BMSトグル入力ポート */
#define	PI_BMS_TOGGLE()			(u8)(PORT8.PIDR.BIT.B2)														/* BMSトグル入力ポートdata P8_2 */

/* 汎用入出力ポートdata */
//#define	PI_GPIO1()				(u8)(PORT8.PIDR.BIT.B2)														/* 汎用入出力ポートdata P8_2 */
#define	PI_GPIO2()				(u8)(PORT1.PIDR.BIT.B0)														/* 汎用入出力ポートdata P1_0 */
#define	PI_GPIO3()				(u8)(PORTE.PIDR.BIT.B5)														/* 汎用入出力ポートdata PE_5 */

/* STATE */
#define	OFF_STATE				(FALSE)
#define	ON_STATE				(TRUE)

/* ポートレベル	*/
#define	U8_HIGH_LEVEL		(u8)0x01			/*ポートをHigh*/
#define	U8_LOW_LEVEL		(u8)0x00			/*ポートをLow*/

#endif /* __MCU_H */
