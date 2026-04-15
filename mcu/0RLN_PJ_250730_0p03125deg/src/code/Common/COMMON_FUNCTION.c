/*==========================================================================*/
/**
 * @file	COMMON_FUNCTION.c
 * @brief	汎用関数群
 * @author	MCU Group：Atsumi Takayuki
 * @version	VER 1.0 REV 0.0
 * @date	2023.11
 *
 */
/*==========================================================================*/
/* Includes ------------------------------------------------------------------*/
#include "COMMON_FUNCTION.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/*
 * @brief 32bitIIRフィルタ（S32・シフト版）
 * @param a_s32_newValue						: 最新値
 * @param a_s32_ave								: 平均値
 * @param a_u8_fltFactor						: 平均の重み(シフト数)
 * @param a_s64_sum								: IIRフィルタ用サム値
 * @return ST_CMF_RESULT_IIRF_S16 a_st_result	: フィルタリング結果
 * @detail 平均をとる。重みは１以上。
 * @detail 軽量化の為、平均を取る際の割り算をビットシフトで行う。
 * @detail その為、重みは直値では無く、何ビットシフトするか、を指定する。
 */
ST_CMF_RESULT_IIRF_S32 St_CMF_CalcIIRFilterS32ByShift(s32 a_s32_newValue, s32 a_s32_ave, u8 a_u8_fltFactorByShiftBits, s64 a_s64_sum)
{
	s64						a_s64_newSum;												/* 更新用合計 */
	ST_CMF_RESULT_IIRF_S32	a_st_result;												/* 戻り値：IIRフィルタ結果を格納する構造体 */

	/* 平均 ＝（新値＋今までのサム値－今までの平均）>> シフト数 */
	a_s64_newSum		= a_s64_sum + (s64)a_s32_newValue;								/* 新値をサム値に足して */
	a_s64_newSum		= a_s64_newSum - (s64)a_s32_ave;								/* 平均値を引く */
	a_st_result.s64_sum	= a_s64_newSum;													/* 今回のサム値を保存（次回計算用） */
	/*----------------------------------------------
     *	MISRA-C:2004 12.7
     *	【逸脱】意図のある符号付き変数のシフト演算
     *----------------------------------------------*/
	a_s64_newSum		= (s64)(a_s64_newSum >> a_u8_fltFactorByShiftBits);				/* 重み(シフト数)でシフトすると加重平均値が出る */
	a_st_result.s32_ave	= (s32)a_s64_newSum;											/* 平均値を保存 */

	return	a_st_result;
}

/*
 * @brief 16bitIIRフィルタ（U16・シフト版）
 * @param a_u16_newValue						: 最新値
 * @param a_u16_ave								: 平均値
 * @param a_u8_fltFactor						: 平均の重み(シフト数)
 * @param a_u32_sum								: IIRフィルタ用サム値
 * @return ST_CMF_RESULT_IIRF_U16 a_st_result	: フィルタリング結果
 * @detail 平均をとる。重みは１以上。
 * @detail 軽量化の為、平均を取る際の割り算をビットシフトで行う。
 * @detail その為、重みは直値では無く、何ビットシフトするか、を指定する。
 */
ST_CMF_RESULT_IIRF_U16 St_CMF_CalcIIRFilterU16ByShift(u16 a_u16_newValue, u16 a_u16_ave, u8 a_u8_fltFactorByShiftBits, u32 a_u32_sum)
{
	ST_CMF_RESULT_IIRF_U16	a_st_result;												/* 戻り値：IIRフィルタ結果を格納する構造体 */

	/* 平均 ＝（新値＋今までのサム値－今までの平均）>> シフト数 */
	u32 a_u32_value = a_u32_sum + (u32)a_u16_newValue;									/* 新値をサム値に足して */
	if(a_u32_value > (u32)a_u16_ave)													/* サム値が引き算可能か？ */
	{
		a_u32_value = a_u32_value - (u32)a_u16_ave;										/* 平均値を引く */
	}
	else
	{
		a_u32_value = (u32)0;															/* 計算不可 （ここには来ないはず）*/
	}
	a_st_result.u32_sum	= a_u32_value;													/* 今回のサム値を保存（次回計算用） */
	a_u32_value 		= a_u32_value >> (u32)a_u8_fltFactorByShiftBits;				/* 重み(シフト数)でシフトすると平均値が出る */
	a_st_result.u16_ave	= (u16)a_u32_value;												/* 平均値を保存 */

	return	a_st_result;
}

/*
 * @brief 16bitIIRフィルタ（S16・シフト版）
 * @param a_s16_newValue						: 最新値
 * @param a_s16_ave								: 平均値
 * @param a_u8_fltFactor						: 平均の重み(シフト数)
 * @param a_s32_sum								: IIRフィルタ用サム値
 * @return ST_CMF_RESULT_IIRF_S16 a_st_result	: フィルタリング結果
 * @detail 平均をとる。重みは１以上。
 * @detail 軽量化の為、平均を取る際の割り算をビットシフトで行う。
 * @detail その為、重みは直値では無く、何ビットシフトするか、を指定する。
 */
ST_CMF_RESULT_IIRF_S16 St_CMF_CalcIIRFilterS16ByShift(s16 a_s16_newValue, s16 a_s16_ave, u8 a_u8_fltFactorByShiftBits, s32 a_s32_sum)
{
	s32						a_s32_newSum;												/* 更新用合計 */
	ST_CMF_RESULT_IIRF_S16	a_st_result;												/* 戻り値：IIRフィルタ結果を格納する構造体 */

	/* 平均 ＝（新値＋今までのサム値－今までの平均）>> シフト数 */
	a_s32_newSum		= a_s32_sum + (s32)a_s16_newValue;								/* 新値をサム値に足して */
	a_s32_newSum		= a_s32_newSum - (s32)a_s16_ave;								/* 平均値を引く */
	a_st_result.s32_sum	= a_s32_newSum;													/* 今回のサム値を保存（次回計算用） */
	/*----------------------------------------------
     *	MISRA-C:2004 12.7
     *	【逸脱】意図のある符号付き変数のシフト演算
     *----------------------------------------------*/
	a_s32_newSum		= (s32)(a_s32_newSum >> a_u8_fltFactorByShiftBits);				/* 重み(シフト数)でシフトすると加重平均値が出る */
	a_st_result.s16_ave	= (s16)a_s32_newSum;											/* 平均値を保存 */

	return	a_st_result;
}

/*
 * @brief 16bitIIRフィルタ（U16版）
 * @param a_u16_newValue						: 最新値
 * @param a_u16_ave								: 平均値
 * @param a_u8_fltFactor						: 平均の重み
 * @param a_u32_sum								: IIRフィルタ用サム値
 * @return ST_CMF_RESULT_IIRF_U16 a_st_result	: フィルタリング結果
 * @detail 平均をとる。重みは１以上。
 */
ST_CMF_RESULT_IIRF_U16 St_CMF_CalcIIRFilterU16(u16 a_u16_newValue, u16 a_u16_ave, u8 a_u8_fltFactor, u32 a_u32_sum)
{
	ST_CMF_RESULT_IIRF_U16	a_st_result;												/* 戻り値：IIRフィルタ結果を格納する構造体 */

	a_u8_fltFactor = (u8)CMF_GETMAX( a_u8_fltFactor, (u8)1 );							/* 0割回避 */

	/* 平均 ＝（新値＋今までのサム値－今までの平均）／ 重み */
	u32 a_u32_value = a_u32_sum + (u32)a_u16_newValue;									/* 新値をサム値に足して */
	if(a_u32_value > (u32)a_u16_ave)													/* サム値が引き算可能か？ */
	{
		a_u32_value = a_u32_value - (u32)a_u16_ave;										/* 平均値を引く */
	}
	else
	{
		a_u32_value = (u32)0;															/* 計算不可 （ここには来ないはず）*/
	}
	a_st_result.u32_sum	= a_u32_value;													/* 今回のサム値を保存（次回計算用） */
	a_u32_value 		= a_u32_value / (u32)a_u8_fltFactor;							/* 重み分割ると平均値が出る */
	a_st_result.u16_ave	= (u16)a_u32_value;												/* 平均値を保存 */

	return	a_st_result;
}

/*
 * @brief 16bitIIRフィルタ（S16版）
 * @param a_s16_newValue						: 最新値
 * @param a_s16_ave								: 平均値
 * @param a_u8_fltFactor						: 平均の重み
 * @param a_s32_sum								: IIRフィルタ用サム値
 * @return ST_CMF_RESULT_IIRF_S16 a_st_result	: フィルタリング結果
 * @detail 平均をとる。重みは１以上。
 */
ST_CMF_RESULT_IIRF_S16 St_CMF_CalcIIRFilterS16(s16 a_s16_newValue, s16 a_s16_ave, u8 a_u8_fltFactor, s32 a_s32_sum)
{
	s32						a_s32_value;												/* 演算用 */
	ST_CMF_RESULT_IIRF_S16	a_st_result;												/* 戻り値：IIRフィルタ結果を格納する構造体 */

	a_u8_fltFactor = (u8)CMF_GETMAX( a_u8_fltFactor, (u8)1 );							/* 0割回避 */

	/* 平均 ＝（新値＋今までのサム値－今までの平均）／ 重み */
	a_s32_value			= a_s32_sum + (s32)a_s16_newValue;								/* 新値をサム値に足して */
	a_s32_value			= a_s32_value - (s32)a_s16_ave;									/* 平均値を引く */
	a_st_result.s32_sum	= a_s32_value;													/* 今回のサム値を保存（次回計算用） */
	a_s32_value			= (s32)(a_s32_value / (s32)a_u8_fltFactor);						/* 重み分割ると平均値が出る */
	a_st_result.s16_ave	= (s16)a_s32_value;												/* 平均値を保存 */

	return	a_st_result;
}

/**
 * @brief 配列への値セット
 * @detail 配列に対し、指定範囲に指定値をセットする。
 * @param a_u8_array	：セット対象の配列
 * @param a_u8_value	：セットする値
 * @param a_u32_length	：配列先頭からセットする範囲
 * @author T.atsumi
 * @date 2023.xx.xx
 */
void	V_CMF_Memset(u8 a_u8_array[], u8 a_u8_value, u32 a_u32_length)
{
	for(u32 a_u32_i = (u32)0; a_u32_i < a_u32_length; a_u32_i++)
	{
		a_u8_array[a_u32_i] = a_u8_value;
	}
}

/**
 * @brief 平方根演算（入力32bit、出力16bit）
 * @param unsigned long	:(u32)
 * @return unsined int	:(u16)
 * @author T.atsumi
 * @date 2023.xx.xx
 */

/*
ニュートン法を用いた平方根演算（入力32bit、出力32bit）

初期値X0を入力値Xの1/2とする。
X0 = X/2
Xn = Xn - ( Xn^2 - X ) / (2 * Xn)
   = Xn - ((Xn /2 ) - (X / 2 / Xn))
Xn ≧ Xn-1 となるまでループさせる
ループは最大20回とする

32bitの数値の平方根を32bitで出力
*/

u32 U32_CMF_Sqrt(u32 a_u32_input)
{
	u32 a_u32_output = (u32)0;
	u8 a_u8_loopCount = (u8)0;

	if( (a_u32_input == (u32)0) || (a_u32_input == (u32)1) )
	{
		a_u32_output = a_u32_input;												/* 0と1の場合は演算しないで入力値をそのまま出力値として返す */
	}
	else
	{
		u32 a_u32_x0 = a_u32_input >> (u32)1;											/* 初期値を入力値の1/2とする */
		u32 a_u32_xn = a_u32_x0;
		u32 a_u32_x  = (u32)0;

		for(a_u8_loopCount = (u8)0; a_u8_loopCount < (u8)20; a_u8_loopCount++)			/* 最大ループ20回 */
		{
			a_u32_x = a_u32_xn;
			a_u32_xn = a_u32_xn - ( (a_u32_xn / (u32)2) - (a_u32_input / (u32)2 / a_u32_xn) );

			if(a_u32_x <= a_u32_xn) break;												/* 今回計算値が前回計算値以上でループ終了 */
		}
		a_u32_output = (u16)(u32)CMF_GETMIN( a_u32_xn, (u32)0xFFFF );
	}
	return a_u32_output;
}

/**
 * @brief 2Dmapの補間
 * @param a_s16_inputData:	map読み込みの入力データ
 * @param a_u16_tabSize:	マップのサイズ
 * @param *a_s16_xData:	マップのxデータ行列名( 注：マップのｘデータは小さい→大きい順に並べる )
 * @param *a_s16_yData:	マップのyデータ行列名
 * @return a_s16_tabResult:	2Dマップ補間の結果(符号付け16ビット)
 */
s16 S16_CMF_2DMapCal( s16 a_s16_inputData, u16 a_u16_tabSize, const s16 a_s16_xData[], const s16 a_s16_yData[] )
{
	u8 a_u8_gap = (u8)0;			/* マップ比較のギャップ*/
	u8 a_u8_tabSearch = (u8)0;		/* 位置探し*/
	u8 a_u8_tabMaxNum = (u8)0;		/* マップの最大番号の定義*/
	s16 a_s16_tabResult;			/* マップ補間出力結果*/
	u8 a_u8_cmpStart = (u8)0;		/* 入力データはマップのｘデータに置く範囲*/
	u8 a_u8_cmpEnd = (u8)0;			/* 入力データはマップのｘデータに置く範囲*/
	s32 a_s32_tmp;

	a_u8_gap = (u8)(a_u16_tabSize >> (u16)1);											/*比較ギャップをマップサイズの半分に初期化する*/
	a_u8_tabMaxNum = (u8)(a_u16_tabSize - (u16)1);										/*マップの最大番号を設定する*/

/*---- mapを前半と後半を分けて、入力データがマップに置く位置を探す-----*/
	if( a_s16_inputData <= a_s16_xData[a_u8_gap] ) 										/*入力データと真ん中のデータと比較*/
	{																					/*入力データは真ん中データより小さい場合→xデータの前半で探す*/
		a_u8_cmpStart = (u8)0; 															/*開始の位置設定*/
		a_u8_cmpEnd = a_u8_gap;															/*比較endの位置を設定*/
	}
  	else
	{																					/*入力データは真ん中データより大きい場合→xデータの後半で探す*/
		a_u8_cmpStart = a_u8_gap;														/*開始の位置設定*/
		a_u8_cmpEnd = a_u8_tabMaxNum;													/*比較endの位置を設定*/
	}

	for( a_u8_tabSearch = a_u8_cmpStart; a_u8_tabSearch <= a_u8_cmpEnd; a_u8_tabSearch++ )	/*入力データとｘデータ比較範囲設定*/
	{
		if( a_s16_inputData <= a_s16_xData[a_u8_tabSearch])								/*入力データはｘデータより小さい時→入力データはマップの位置を見つけた*/
		{
			break;																		/*入力データの位置を見つけた→循環から抜け出す*/
		}
	}

/*----- 入力データがマップに置く位置により2D補間を行う -----*/
	if( a_u8_tabSearch == (u8)0 )														/*xデータ行列の0番の場合*/
	{
		a_s16_tabResult = a_s16_yData[0];												/*yの0番データ返送*/
	}
	else if( (u16)a_u8_tabSearch == a_u16_tabSize  )									/*xデータ行列の最後番の場合*/
	{
		a_s16_tabResult = a_s16_yData[a_u8_tabSearch - (u8)1];							/*yの最後番データ返送*/
	}
	else
	{																					/*その他の場合*/
		a_s32_tmp = (s32)(s16)(a_s16_yData[a_u8_tabSearch] - a_s16_yData[a_u8_tabSearch - (u8)1]);
		a_s32_tmp *= (s32)(s16)(a_s16_inputData - a_s16_xData[a_u8_tabSearch - (u8)1]);
		a_s32_tmp /= (s32)(s16)(a_s16_xData[a_u8_tabSearch] - a_s16_xData[a_u8_tabSearch - (u8)1]);
		a_s32_tmp += (s32)a_s16_yData[a_u8_tabSearch - (u8)1];
		a_s16_tabResult = (s16)a_s32_tmp;
	}

/*----- 2D補間の結果を出力 -----*/
	return a_s16_tabResult;																/*補間結果を返送*/
}

/**
 * @brief 3Dmapの補間
 * @param a_u8_keydatSize: 3Dマップにてkeydataサイズ
 * @param a_s32_inputKeydat:	入力キーデータ
 * @param a_s16_inputXdata:	入力ｘデータ( 注：3Dマップのｘデータは小さい→大きい順に並べる )
 * @param *a_pst_p:	ST_CMF_3D_MAPタイプへのポインタ
 * @return a_s16_tabResult:	3Dマップ補間の結果(符号付け16ビット)
 */
s16 S16_CMF_3DMapCal( u8 a_u8_keydatSize, s32 a_s32_inputKeydat,  s16 a_s16_inputXdata, const struct ST_CMF_3D_MAP *a_pst_p )
{
	u8 a_u8_gap = (u8)0;																/* マップ比較のギャップ */
	u8 a_u8_tabSearch = (u8)0;															/* 位置探し */
	u8 a_u8_tabMaxNum = (u8)0;															/* マップの最大番号の定義 */
	s16 a_s16_tabY1 = (s16)0;															/* 2Dマップ1の補間結果 */
	s16 a_s16_tabY2 = (s16)0;															/* 2Dマップ2の補間結果 */
	s16 a_s16_tabResult = (s16)0;														/* 3Dマップの補間結果 */

	a_u8_gap = a_u8_keydatSize >> (u8)1;												/*比較ギャップをマップサイズの半分に初期化する*/
	a_u8_tabMaxNum = a_u8_keydatSize - (u8)1;											/*マップの最大番号を設定する*/

	if( a_s32_inputKeydat <= a_pst_p -> s32_keyData )									/*キーデータは極値かの判断*/
	{														/*** 最小値より小さい場合 ***/
		a_s16_tabResult = S16_CMF_2DMapCal( a_s16_inputXdata, (u16)a_pst_p -> s16_tabSize, a_pst_p -> s16_x_p, a_pst_p -> s16_y_p );	/*最小値対応する2Dマップの計算*/
	}
	else if( a_s32_inputKeydat >= ( a_pst_p + a_u8_tabMaxNum ) -> s32_keyData )
	{														/*** 最大値より大きい場合 ***/
		a_pst_p = a_pst_p + a_u8_tabMaxNum;												/*最大値対応する2Dマップへのポインタ*/
		a_s16_tabResult = S16_CMF_2DMapCal( a_s16_inputXdata, (u16)a_pst_p -> s16_tabSize, a_pst_p -> s16_x_p, a_pst_p -> s16_y_p );	/*最大値対応する2Dマップの計算*/
	}
	else
	{														/*** 極値ではない場合 ***/
		if( a_s32_inputKeydat <= ( a_pst_p + a_u8_gap ) -> s32_keyData )				/*入力キーデータとマップの真ん中キーデータを比較する*/
		{													/*** 真ん中キーデータより小さい場合 ***/
			for( a_u8_tabSearch = (u8)0; a_u8_tabSearch <= a_u8_gap; a_u8_tabSearch++ )	/*前半で探す*/
			{
				if( a_s32_inputKeydat <= ( a_pst_p + a_u8_tabSearch ) -> s32_keyData )
				{											/*** 入力キーデータはマップでの位置を見つけた ***/
					break;
				}
			}
		}
		else
		{													/*** 真ん中キーデータより大きい場合→後半で探す ***/
			for( a_u8_tabSearch = ( a_u8_gap + (u8)1 ); a_u8_tabSearch < ( a_u8_keydatSize - (u8)1 ); a_u8_tabSearch++ )
			{
				if( a_s32_inputKeydat <= ( a_pst_p + a_u8_tabSearch ) -> s32_keyData )	/*** 入力キーデータはマップでの位置を見つけた ***/
				{
					break;
				}
			}
		}

/*----- 3Dマップの補間を行う -----*/
		a_pst_p = a_pst_p + a_u8_tabSearch;												/*見つけた位置の3Dマップへのポインタ*/
		a_s16_tabY2 = S16_CMF_2DMapCal( a_s16_inputXdata, (u16)a_pst_p -> s16_tabSize, a_pst_p -> s16_x_p, a_pst_p -> s16_y_p );	/*現在対応するｘｙマップより補間結果*/
		a_pst_p = a_pst_p - 1;															/*ポインタを一つ前へのポインタ*/
		a_s16_tabY1 = S16_CMF_2DMapCal( a_s16_inputXdata, (u16)a_pst_p -> s16_tabSize, a_pst_p -> s16_x_p, a_pst_p -> s16_y_p );	/*小さいマップの補間結果*/

		a_s16_tabResult = ( ( s32 )a_s16_tabY2 - a_s16_tabY1 ) * ( ( a_s32_inputKeydat ) - ( a_pst_p -> s32_keyData ) ) / ( ( a_pst_p + 1 ) -> s32_keyData - ( a_pst_p -> s32_keyData) ) + a_s16_tabY1;
																						/*二つマップの補間を求める*/
	}

/*----- 3Dマップの補間結果出力 -----*/
	return a_s16_tabResult ;
}

/**
 * @brief チェックサム計算
 * @detail 総和を取る(u8) ⇒ ビット反転  ⇒ １加算
 * @param a_u8_array[]	:	配列内のCS計算スタート場所へのポインタ
 * @param a_u16_length	:	CS計算サイズ	[0,65535]
 * @return a_u8_cs		:	チェックサム計算結果	[0,255]
 */
u8	U8_CMF_CalcCheckSum(u8 a_u8_array[], u16 a_u16_length)
{
	u8 a_u8_cs = (u8)0;

	/* 加算ループ */
	for(u32 a_u32_i = (u32)0; a_u32_i < (u32)a_u16_length; a_u32_i++)
	{
		a_u8_cs += a_u8_array[a_u32_i];
	}

	/* ビット反転 & 1加算 */
	a_u8_cs = ~a_u8_cs + (u8)1;

	return a_u8_cs;
}

/*
 * @brief 大きい方を返す
 * @param a_s32_value1
 * @param a_s32_value2
 * @return a_s32_value1 or a_s32_value2
 */
s32	S32_CMF_GETMAX( s32 a_s32_value1, s32 a_s32_value2 )						/* 大きい方を返す */
{
	if( a_s32_value1 > a_s32_value2 )
	{
		return	a_s32_value1;
	}
	else
	{
		return	a_s32_value2;
	}
}

/*
 * @brief 小さい方を返す
 * @param a_s32_value1
 * @param a_s32_value2
 * @return a_s32_value1 or a_s32_value2
 */
s32	S32_CMF_GETMIN(s32 a_s32_value1, s32 a_s32_value2)						/* 小さい方を返す */
{
	if( a_s32_value1 < a_s32_value2 )
	{
		return	a_s32_value1;
	}
	else
	{
		return	a_s32_value2;
	}
}

/*
 * @brief 最大値と最小値に制限させる値を返す
 * @param a_s32_value
 * @param a_s32_min
 * @param a_s32_max
 * @return a_s32_value or a_s32_min or a_s32_max
 */
s32	S32_CMF_GETLIMIT( s32 a_s32_value, s32 a_s32_min, s32 a_s32_max )						/* 最大値と最小値に制限させる値を返す */
{
	if( a_s32_value < a_s32_min )
	{
		return	a_s32_min;
	}
	else if( a_s32_value > a_s32_max )
	{
		return	a_s32_max;
	}
	else
	{
		return	a_s32_value;
	}
}

/*
 * @brief 最大値と最小値に制限させる値を返す
 * @param a_s32_value
 * @return (u32)a_s32_value or (u32)(-a_s32_value)
 */
u32	U32_CMF_GETABS( s32 a_s32_value )								/* 絶対値を返す */
{
	if( a_s32_value >= (s32)0 )
	{
		return	(u32)a_s32_value;
	}
	else
	{
		return	(u32)(-a_s32_value);
	}
}

/*
 * @brief 飽和加算
 * @note 演算結果を上下限でクリップする
 * @param a_s32_value1
 * @param a_s32_value2
 * @return a_s32_result
 */
s32	S32_CMF_QADD(s32 a_s32_value1, s32 a_s32_value2)
{
	s32 a_s32_result;
	if( (a_s32_value2 > 0) && (a_s32_value1 > ((S32_MAX) - a_s32_value2)) )
	{
		/* 上限クリップ処理 */
		a_s32_result = (S32_MAX);
	}
	else if( (a_s32_value2 < 0) && (a_s32_value1 < ((S32_MIN) - a_s32_value2)) ) 
	{
		/* 下限クリップ処理 */
		a_s32_result = (S32_MIN);
	}
	else
	{
		a_s32_result = a_s32_value1 + a_s32_value2;
	}
	return	a_s32_result;
	/* ... */
}

/*
 * @brief 飽和減算
 * @note 演算結果を上下限でクリップする
 * @param a_s32_value1
 * @param a_s32_value2
 * @return a_s32_result
 */
s32 S32_CMF_QSUB(s32 a_s32_value1, s32 a_s32_value2)
{
	s32 a_s32_result;
	if( (a_s32_value2 > 0) && (a_s32_value1 < (S32_MIN) + a_s32_value2) )
	{
		/* 下限クリップ処理 */
		a_s32_result = (S32_MIN);
	}
	else if( (a_s32_value2 < 0) && (a_s32_value1 > (S32_MAX) + a_s32_value2) )
	{
		/* 上限クリップ処理 */
		a_s32_result = (S32_MAX);
	}
	else
	{
		a_s32_result = a_s32_value1 - a_s32_value2;
	}
	return	a_s32_result;
	/* ... */
}
