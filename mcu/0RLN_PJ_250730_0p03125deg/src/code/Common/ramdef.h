/************************************************************************/
/*	SYSTEM		LiCATs MCU RX62T										*/
/*	FILE NAME	ramdef.h												*/
/*	内容		GLOBAL変数宣言(for DEBUG)								*/
/*																		*/
/*	起動条件	無し（関数無し）										*/
/*																		*/
/*	作  成		11.01.14												*/
/*	修  正																*/
/*	履  歴																*/
/************************************************************************/

#include "ctypedef.h"
#include "can_api.h"
#ifdef RAMDEF
  #define EXTERN
#else
  #define EXTERN extern
#endif


#pragma bit_order right
#pragma unpack



EXTERN union
{
	u16			word;
	struct
	{
		u16	b_forBenchtest					:1;		/* b0:ベンチテスト用 */
		u16	b_revolutionConstant			:1;		/* b1:モータ回転数認識固定 & トルク指令固定 */
		u16	b_ifGainOld						:1;		/* b2:界磁ゲイン旧 */
		u16	b_idqGainOld					:1;		/* b3:Idqゲイン旧 */
		u16	b_variableGainTestActivity		:1;		/* b4:可変ゲイン有効 */
		u16	b_dqGainTestActivity			:1;		/* b5:dq軸ゲインテスト有効 */
		u16	b_ifGainTestActivity			:1;		/* b6:界磁ゲインテスト有効 */
		u16	b_regenerateOrder				:1;		/* b7:負トルク指令(1:負トルク、0:正トルク) */
		u16	b_currentLimitActivity			:1;		/* b8:電流リミット有効 */
		u16	b_ifForcedGateOff				:1;		/* b9:界磁強制ゲートOFF */
		u16	b_stepResponseIfGateOffCansel	:1;		/* b10:ステップ応答時IfゲートOFF解除 */
		u16	b_stepResponseIfGateOff			:1;		/* b11:ステップ応答時IfゲートOFF */
		u16	b_stepResponseActivity			:1;		/* b12:ステップ応答有効 */
		u16	b_forcedGateOff					:1;		/* b13:強制ゲートOFF */
		u16	b_iqrefByPotentiometer			:1;		/* b14:ポテンショによるIq指令値決定(Id=0) */
		u16	b_referenceByCuW				:1;		/* b15:CuWによる指令値決定 */
	}bit;
}UNI_GL_testFlg;



EXTERN union
{
	u8			byte;
	struct
	{
		u8	b_vcuDummyOrderActivity			:1;		/* b0:VCUダミー要求有効 */
		u8	b_relayOffRequest				:1;		/* b1:リレーOFF要求(1:リレーOFF) */
		u8	b_motorStopRequest				:1;		/* b2:モータ停止要求(1:モータ停止) */
		u8	b_dischargeRequest				:1;		/* b3:ディスチャージ要求(1:ディスチャージ) */
		u8									:1;		/* b4-7:未使用 */
	}bit;
}UNI_GL_vcuTestFlg;

EXTERN u16	U16_GL_dPGainForTest;					/* d軸比例ゲインTEST用 */
EXTERN u16	U16_GL_dIGainForTest;					/* d軸積分ゲインTEST用 */
EXTERN u16	U16_GL_qPGainForTest;					/* q軸比例ゲインTEST用 */
EXTERN u16	U16_GL_qIGainForTest;					/* q軸積分ゲインTEST用 */
EXTERN u16	U16_GL_pGainTemp;						/* 界磁比例ゲインTEST用 */
EXTERN u16	U16_GL_iGainTemp;						/* 界磁積分ゲインTEST用 */
EXTERN u16	U16_GL_pGainTest;						/* 界磁比例ゲインTEST用 */
EXTERN u16	U16_GL_iGainTest;						/* 界磁積分ゲインTEST用 */

EXTERN s32	S32_GL_idDiff;							/* Id差分 */
EXTERN s32	S32_GL_iqDiff;							/* Iq差分 */

EXTERN s16	S16_GL_propoVd;
EXTERN s16	S16_GL_propoVq;

EXTERN s16	S16_GL_integVd;
EXTERN s16	S16_GL_integVq;


EXTERN u16	U16_GL_vqTable;

EXTERN s16	S16_GL_idRef_100mA;
EXTERN s16	S16_GL_iqRef_100mA;
