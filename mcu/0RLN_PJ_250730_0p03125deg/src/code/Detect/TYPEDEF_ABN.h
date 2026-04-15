/**
 * @file TYPEDEF_MODE.h
 * @author	MCU Group：Atsumi Takayuki
 * @date 2023.11
 */
#ifndef __TYPEDEF_ABN_H
#define __TYPEDEF_ABN_H

/* Includes ------------------------------------------------------------------*/
#include "ctypedef.h"					/* 型宣言 */

/* External define -----------------------------------------------------------*/

/* External typedef ----------------------------------------------------------*/
typedef enum{
	HEATING_PROTECT_MODE0 = 0,
	HEATING_PROTECT_MODE1,
	HEATING_PROTECT_MODE2,
	HEATING_PROTECT_MODE3,
}EN_DET_MTR_HEATING_PROTECT_MODE;


#pragma bit_order right
#pragma unpack

typedef union										/* VCU送信専用 */
{
	u32			ulong;
	struct
	{
		u32	b_warningMosOverheat			:1;		/* b0:MOS過熱警告 */
		u32	b_warningMtrOverheat			:1;		/* b1:モータ過熱警告 */
		u32	b_warningMtrThermistorOpen		:1;		/* b2:モータサーミスタ断線・短絡異常 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 回転センサ警告A相 */
		u32									:1;		/* 回転センサ警告B相 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32	b_warningCapThermistorOpen		:1;		/* b8:コンデンササーミスタ断線・短絡異常 */
		u32	b_warningLowTemp				:1;		/* b9:低温警告 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32	b_warningUnderVoltageLimit		:1;		/* b12:低電圧出力制限 */
		u32	b_warningOverVoltageLimit		:1;		/* b13:過電圧出力制限 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32	b_warningBoardOverheat			:1;		/* b16:基板過熱異常 */
		u32	b_warningBoardThermistorOpen	:1;		/* b17:基板サーミスタ断線・短絡異常 */
		u32	b_warningMosThermistorOpen		:1;		/* b18:MOSサーミスタ断線・短絡異常 */
		u32	b_warningMtrOverheatErr			:1;		/* b19:モータ過熱異常 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32	b_warningWelding				:1;		/* b24:溶着異常警告 */
		u32	b_warningSolenoid				:1;		/* b25:ソレノイド異常 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32	b_warningOutputLimited			:1;		/* b30:出力制限中 */
		u32	b_warningInspectionRequest		:1;		/* b31:点検要求フラグ */
	}bit;
}UNI_ABN_WARNINGFLG;

typedef union										/* VCU送信専用 */
{
	u32			ulong;
	struct
	{
		u32	b_abnormalMosOverheat			:1;		/* b0:MOS過熱異常 */
		u32									:1;		/* 予備 */
		u32	b_abnormalCapLowTemp			:1;		/* b2:コンデンサ低温異常 */
		u32									:1;		/* 予備 */
		u32	b_abnormalMosThermistorOpen		:1;		/* b4:MOSサーミスタ断線・短絡異常 */
		u32	b_abnormalBoardThermistorOpen	:1;		/* b5:基板サーミスタ断線・短絡異常 */
		u32	b_abnormalCapThermistorOpen		:1;		/* b6:コンデンササーミスタ断線・短絡異常 */
		u32	b_abnormalCanCommunication		:1;		/* b7:CAN受信途絶異常 */
		u32	b_abnormalMtrOverCurrent		:1;		/* b8:過電流異常 */
		u32	b_abnormalCtOffset				:1;		/* b9:CTオフセット異常 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32	b_abnormalOverVoltage			:1;		/* b12:過電圧異常 */
		u32	b_abnormalUnderVoltage			:1;		/* b13:低電圧異常 */
		u32									:1;		/* 予備 */
		u32									:1;		/* 予備 */
		u32	b_abnormalBoardOverheat			:1;		/* b16:基板過熱異常 */
		u32	b_abnormalCapOverheat			:1;		/* b17:コンデンサ過熱異常 */
		u32									:1;		/* 予備 */
		u32	b_abnormalShift					:1;		/* b19:シフト入力異常 */
		u32	b_abnormalMtrPhaseShort			:1;		/* b20:相間ショート異常 */
		u32	b_abnormalBatterytVoltage		:1;		/* b21:電池電圧差異常 */
		u32	b_abnormalExOverCurrent			:1;		/* b22:エキサイタ過電流異常 */
		u32	b_abnormalPhaseWireOpen			:1;		/* b23:相モータ断線異常 */
		u32	b_abnormalMbContactor			:1;		/* b24:MBコンタクタ異常 */
		u32	b_abnormalCharge				:1;		/* b25:チャージ異常 */
		u32	b_abnormalEncoder				:1;		/* b26:エンコーダ異常 */
		u32	b_abnormalPrechargeWaitOver		:1;		/* b27:プリチャージ異常 */
		u32									:1;		/* 予備 */
		u32	b_abnormalAccelOpen				:1;		/* b29:アクセルセンサ断線異常 */
		u32	b_abnormalAccelShort			:1;		/* b30:アクセルセンサ短絡異常 */
		u32	b_abnormalAccelUnmatch			:1;		/* b31:アクセルスイッチアンマッチ異常 */
	}bit;
}UNI_ABN_ERRORFLG;

typedef union
{
	u8			byte;
	struct
	{
		u8	b_abnormalMtrOverCurrent	:1;			/* b0:モータ過電流異常		(1:過電流) */
		u8	b_abnormalEncoder			:1;			/* b1:エンコーダ異常		(1:パターン未定義) */
		u8	b_abnormalCtUOffset			:1;			/* b2:U相CTオフセット異常	(1:異常) */
		u8	b_abnormalCtVOffset			:1;			/* b3:V相CTオフセット異常	(1:異常) */
		u8	b_abnormalMosFetOverheat	:1;			/* b4:MODFET過熱異常		(1:過熱) */
		u8	b_abnormalCapacitorOverheat	:1;			/* b5:コンデンサ過熱異常	(1:過熱) */
		u8	b_abnormalCapacitorLowTemp	:1;			/* b6:コンデンサ低温異常	(1:低温) */
		u8	b_abnormalMotorOverheat		:1;			/* b7:モータ過熱異常		(1:過熱) */
	}bit;
}UNI_DET_MTR_ABNORMALFLG;

typedef union
{
	u8			byte;
	struct
	{
		u8	b_abnormalUPhaseWireOpen	:1;			/* b0:U相モータ断線異常			(1:U相単相断線) */
		u8	b_abnormalVPhaseWireOpen	:1;			/* b1:V相モータ断線異常			(1:V相単相断線) */
		u8	b_abnormalWPhaseWireOpen	:1;			/* b2:W相モータ断線異常			(1:W相単相断線) */
		u8	b_abnormalExOverCurrent		:1;			/* b5:エキサイタ過電流異常	(1:超過) */
		u8	b_abnormalOverRevolution	:1;			/* b6:過回転異常			(1:超過) */
		u8								:4;			/* b4-7:空き */
	}bit;
}UNI_DET_MTR_ABNORMALFLG2;

typedef union
{
	u8			byte;
	struct
	{
		u8	b_warningInverterHeat		:1;			/* b0:インバータ過熱警告	(1:過熱警告) */
		u8	b_warningMotorHeat			:1;			/* b1:モータ過熱警告		(1:過熱警告) */
		u8								:6;			/* b1-7:空き */
	}bit;
}UNI_DET_MTR_WARNINGFLG;

typedef union
{
	u8			byte;
	struct
	{
		u8	b_warningBattOverheat		:1;			/* b0:バッテリ過熱異常		(1:バッテリ過熱) */
		u8								:1;			/* b1:空き */
		u8	b_warningBattCool			:1;			/* b2:バッテリ低温警告		(1:バッテリ低温警告) */
		u8	b_warningBattHeat			:1;			/* b3:バッテリ高温警告		(1:バッテリ高温警告) */
		u8								:1;			/* b4:空き */
		u8								:1;			/* b5:空き */
		u8	b_warningDischargeStop		:1;			/* b6:放電停止				(1:放電停止) */
		u8								:1;			/* b7:空き */
	}bit;
}UNI_DET_BAT_WARNINGFLG;

/* external variables --------------------------------------------------------*/

/* Grobal function prototypes ------------------------------------------------*/


#endif /* __TYPEDEF_ABN_H */

