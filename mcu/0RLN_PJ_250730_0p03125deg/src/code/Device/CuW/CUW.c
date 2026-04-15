/******************************************************************/
/*!
 * @file	CuW.c
 * @brief	CuW通信
 * @author	鈴木 秀彰
 * @note	modified for 09X：Atsumi.T
 * @date	2016/09/13
 */
/*******************************************************************/

/***************************************
	INCLUDE FILE
***************************************/
#include "COMMON_FUNCTION.h"		/* 汎用関数群 */

#include "CUW.h"

/***************************************
	型宣言
***************************************/
/*! @brief ロギングモード列挙体 */
typedef enum
{
	CUW_MODE_CHANGESETTING	= 1,						/*!< セッティング変更 */
	CUW_MODE_RAMMONITOR		= 4,						/*!< RAMモニタ */
	CUW_MODE_SIGNATURE		= 5,						/*!< バージョン情報送信 */
	CUW_MODE_SENDLOGOK		= 6							/*!< ロギングOK送信 */
}EN_CUW_MODE;

/*! @brief ロギング制御フラグ */
#pragma bit_order right
#pragma unpack

static union
{
	u8			byte;									/*!< バイトアクセス用 */
	struct
	{
		u8	b_chkSumErr			:1;					/*!<  b0 : チェックサム異常フラグ	(1:チェックサム不一致) */
		u8	b_rxErr				:1;					/*!<  b1 : 受信異常フラグ			(1:受信異常) */
		u8	b_modeErr			:1;					/*!<  b2 : モード異常フラグ			(1:モード異常あり) */
		u8	b_data				:1;					/*!< _b3_: 通信ありフラグ			(1:受信あり) */
		u8	b_dataOk			:1;					/*!<  b4 : 通信有効フラグ			(1:データ有効) */
		u8	b_finishedReceive	:1;					/*!<  b5 : 受信完了フラグ			(1:完了) */
		u8	b_storedTxData		:1;					/*!<  b6 : 送信データ格納完了フラグ	(1:完了)	[CuWでセット、CAN送信後にクリア] */
		u8	b_shut				:1;					/*!<  b7 : 通信遮断フラグ			(1:通信遮断) */
	}bit;
}uni_CUW_commControlFlg;

/**
  * @brief	CuW受信情報
  * @note	ID 0x63C
  */
static union
{
	u8	   u8_byteData[7];
	struct
	{
		u8	u8_header;									/*!< ヘッダ			 */
		u8	u8_cmdId;									/*!< コマンドID		 */
		u8	u8_data1;									/*!< データ1		 */
		u8	u8_data2;									/*!< データ2		 */
		u8	u8_data3;									/*!< データ3		 */
		u8	u8_data4;									/*!< データ4		 */
		u8	u8_checksum;								/*!< チェックサム	 */
	}part;
}uni_CUW_rxDataList;

/**
  * @brief	CuW送信データ
  * @note	ID 0x63D
  */
static union
{
	u8	   u8_byteData[7];
	struct
	{
		u8	u8_header;									/*!< ヘッダ			 */
		u8	u8_cmdId;									/*!< コマンドID		 */
		u8	u8_data1;									/*!< データ1		 */
		u8	u8_data2;									/*!< データ2		 */
		u8	u8_data3;									/*!< データ3		 */
		u8	u8_data4;									/*!< データ4		 */
		u8	u8_checksum;								/*!< チェックサム	 */
	}part;
}uni_CUW_txDataList;


/*! @brief 2バイトデータを1バイトずつに分解したい時用 構造体 */
typedef union
{
	u16			word;									/*!< 2バイト */
	struct
	{
		u8		low;									/*!< 下位バイト */
		u8		high;									/*!< 上位バイト */
	}byte;
}UNI_CUW_DATA16;


/***************************************
	定数宣言
***************************************/
#define U8_CUW_LOG_RX_SIZE			(u8)7									/*!< ロギングデータ受信サイズ */
#define U8_CUW_LOG_CH_SIZE			(u8)20									/*!< ロギングチャンネルサイズ */
#define U32_CUW_LOG_BASE_ADRRESS	(u32)0x00000000							/*!< ロギング対象RAMアドレスのベース：RH850は0xFEDF0000 */

#define U8_CUW_LOG_HEADER			(u8)0xFF								/*!< ロギング ヘッダ情報 */
#define U16_CUW_DEFAULT_ADDRESS		(u16)0x0000								/*!< ロギング対象アドレス 初期値 */
#define U8_CUW_LOG_OK_KEEP			(u16)500								/*!< ロギングOK信号定期送信継続時間 [10ms] */
#define U8_CUW_LOG_OK_INTERVAL		(u8)20									/*!< ロギングOK信号定期送信間隔時間 [10ms] */

#define	U8_CUW_SIGNATURE_STRING		"23091900"


/***************************************
	グローバル変数定義
***************************************/
static EN_CUW_MODE		en_CUW_logMode;										/*!< ロギングモード */

static u8				u8_CUW_rxID;										/*!< 受信コマンドID */
static u8				u8_CUW_txID;										/*!< 送信コマンドID */
static u8				u8_CUW_rxData[(U8_CUW_LOG_RX_SIZE)];				/*!< ロギング受信データ */
/* セッティング変更モード関連 */
static volatile u8		u8_CUW_changeSettingMapStyle;						/*!< マップ種類、送信データバイト数 */
static u16				u16_CUW_changeSettingAddress;						/*!< 変更データの先頭アドレス */
static u8				u8_CUW_changeSettingTotalSize;						/*!< 変更データの総バイト数 */
static u8				u8_CUW_changeSettingMapData[4]						/*!< 変更データ */
						= {(u8)0,(u8)0,(u8)0,(u8)0};
/* RAMモニタモード関連 */
static u8				u8_CUW_ramMonTotalSize;								/*!< 送信データ総バイト数 */
static u16				u16_CUW_ramMonChAdrs[(U8_CUW_LOG_CH_SIZE)];			/*!< 送信データアドレス配列		(ロギング対象変数のアドレス(下位2バイト)を格納) */
static u8				u8_CUW_ramMonChSize[(U8_CUW_LOG_CH_SIZE)];			/*!< 送信データサイズ配列		(ロギング対象変数のサイズを格納) */
static UNI_CUW_DATA16	uni_CUW_ramMonChData[(U8_CUW_LOG_CH_SIZE)];			/*!< 送信データ配列				(指定アドレスのデータを格納) */
static u8				u8_CUW_ramMonChIndex;								/*!< 送信データ情報配列インデックス */
/* バージョン情報送信モード関連 */
static u8				u8_CUW_signatureIndex;								/*!< バージョン情報送信データインデックス */
/* ロギングOK送信モード関連 */
static u16				u16_CUW_logOkKeepCounter;							/*!< ロギングOK信号定期送信継続カウンタ */
static u8				u8_CUW_logOkTimingCounter;							/*!< ロギングOK信号定期送信タイミングカウンタ */

//const char u8_CUW_signature[] = "'09X'";
const char u8_CUW_signature[] = U8_CUW_SIGNATURE_STRING;


/***************************************
	ローカル関数プロトタイプ宣言
***************************************/
static void v_CUW_CheckRxData(void);										/* 受信データ確認 */
static void v_CUW_ChangeSetting(void);										/* セッッティング変更 */
static void v_CUW_RamMonitor(void);											/* RAMモニタモード */
static void v_CUW_SendSignature(void);										/* バージョン情報送信モード */
static void v_CUW_PeriodicSendLogOk(void);									/* ロギングOK定期送信モード */
static void v_CUW_SendLogOk(void);											/* ロギングOK信号を送信する */
static void v_CUW_SetScanData( u8 a_u8_txData[]);							/* 送信データ設定 */
static bool	b_CUW_GetRxdata( u8 * a_u8_data );



/*********************************/
/*!
 * @brief	CuW通信メイン
 */
void V_CUW_Loop10ms(void)
{
	/*----- 受信データ確認 -----*/
	v_CUW_CheckRxData();													/* 受信データ確認 */

	/*----- ロギングモード別処理 -----*/
	switch( en_CUW_logMode )												/* ロギングモード別に分岐 */
	{
	case CUW_MODE_CHANGESETTING:
		/* セッティング変更 */
		v_CUW_ChangeSetting();
		break;

	case CUW_MODE_RAMMONITOR:
		/* RAMモニタ */
		v_CUW_RamMonitor();
		break;

	case CUW_MODE_SIGNATURE:
		/* バージョン情報送信 */
		v_CUW_SendSignature();
		break;

	case CUW_MODE_SENDLOGOK:
		/* ロギングOK定期送信 */
		v_CUW_PeriodicSendLogOk();
		break;

	default:
		uni_CUW_commControlFlg.bit.b_modeErr = TRUE;						/* ロギングモード異常フラグ セット */
		break;
	}
}


/*********************************/
/*!
 * @brief	PCからの受信データを確認し、対応する
 */
static void v_CUW_CheckRxData(void)
{
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( uni_CUW_commControlFlg.bit.b_finishedReceive == FALSE )
	{
		return;																/* 受信完了フラグ確認 受信データがなければ終了 */
	}

	if( b_CUW_GetRxdata( u8_CUW_rxData ) == FALSE )
	{
		return;																/* 受信データ取得 & 受信完了フラグ確認 受信データがなければ終了 */
	}

	/*----- コマンドIDの確認 -----*/
	u8_CUW_rxID = u8_CUW_rxData[1];											/* コマンドID 保存 */
	switch( u8_CUW_rxID )
	{
	case 210:
		/* RAMモニタ モード切替 要求 */
		en_CUW_logMode = CUW_MODE_RAMMONITOR;								/* RAMモニタモード 切替 */
		u8_CUW_ramMonChIndex = (u8)0;										/* 送信データ情報配列インデックス クリア */
		u8_CUW_ramMonTotalSize = u8_CUW_rxData[3];							/* 送信データ総バイト数 保存 */
		break;

	case 212:
		/* RAMモニタ アドレス指示 */
		if( en_CUW_logMode != CUW_MODE_RAMMONITOR )
		{
			break;															/* RAMモニタモードになっているか確認	異なる → 正規手順ではない → 終了 */
		}
		if( u8_CUW_ramMonChIndex >= (U8_CUW_LOG_CH_SIZE) )
		{
			break;															/* 情報配列のインデックスが規定値より大きい → 溢れる分は無視 */
		}

		u16_CUW_ramMonChAdrs[u8_CUW_ramMonChIndex] = (u16)u8_CUW_rxData[2] * (u16)256 + (u16)u8_CUW_rxData[3];	/* 送信データのアドレス 保存 */
		u8_CUW_ramMonChSize[u8_CUW_ramMonChIndex]  = u8_CUW_rxData[4];		/* 送信データのサイズ 保存 */

		u8_CUW_ramMonChIndex ++;											/* インデックス 更新 */
		break;

	case 213:
		/* RAMモニタ アドレス指示終了 */
		u8_CUW_ramMonTotalSize = u8_CUW_ramMonChIndex;						/* 指示があったチャンネル数 保存 */
		u8_CUW_ramMonChIndex = 0;											/* 送信データ情報インデックス 初期化 */
		break;

	case 200:
		/* セッティング変更 モード切替 要求 */
		en_CUW_logMode = CUW_MODE_CHANGESETTING;							/* セッティングモード 切替 */
		break;

	case 202:
		/* セッティング変更 マップ種類、先頭アドレス、データ指示 */
		u8_CUW_changeSettingMapStyle  = u8_CUW_rxData[2];											/* マップ種類 保存 */
		u16_CUW_changeSettingAddress  = (u16)u8_CUW_rxData[3] * (u16)256 + (u16)u8_CUW_rxData[4];	/* 変更データの先頭アドレス 保存 */
		u8_CUW_changeSettingTotalSize = u8_CUW_rxData[5] + 1;										/* 変更データの総バイト数 保存	 (送られてくるデータは"総バイト数-1"になっている) */
		break;

	case 203:
	case 204:
		/* セッティング変更 */
		u8_CUW_changeSettingMapData[0] = u8_CUW_rxData[2];					/* マップデータ数 保存 */
		u8_CUW_changeSettingMapData[1] = u8_CUW_rxData[3];					/* マップデータ数 保存 */
		u8_CUW_changeSettingMapData[2] = u8_CUW_rxData[4];					/* マップデータ数 保存 */
		u8_CUW_changeSettingMapData[3] = u8_CUW_rxData[5];					/* マップデータ数 保存 */
		break;

	case 252:
	case 253:
		/* バージョン情報送信モード */
		en_CUW_logMode = CUW_MODE_SIGNATURE;								/* バージョン情報送信モード 切替 */
		u8_CUW_signatureIndex = (u8)0;										/* インデックス 初期化 */
		u8_CUW_txID = (u8)215;												/* 送信コマンドID 初期化 */
		break;

	default:
		/* 既定外コード受信 */
		break;																/* 特に何もしない */
	}

	uni_CUW_commControlFlg.bit.b_finishedReceive = FALSE;					/* 受信完了フラグ クリア */

}


/**
  * @brief			CuW受信データ取得
  * @author			鈴木 秀彰
  * @param [out]	u8_data		受信データのコピー先 (参照渡し)
  * @return			True：正規データ取得 / False：未受信 or 不正データ
  */
static bool b_CUW_GetRxdata( u8 * a_u8_data )
{
	/*----------------------------------------------
	 *	MISRA-C:2004 14.7
	 *	【逸脱】主要な処理が不要となる為。
	 *----------------------------------------------*/
	if( uni_CUW_rxDataList.part.u8_header != (u8)0xFF )						/* ヘッダデータが既定値でなかったらFALSEを返して終了 */
	{
		return FALSE;
	}

	/*----- チェックサムの確認 -----*/
	if( U8_CMF_CalcCheckSum(&uni_CUW_rxDataList.u8_byteData[1], (u16)5) != uni_CUW_rxDataList.u8_byteData[6] )
	{
		/*--- チェックサム不一致 ---*/
		return FALSE;
	}
	else
	{
		/*--- チェックサム一致 ---*/
		for( u8 idx = 0; idx <= 6; idx++ )
		{
			*( a_u8_data + idx ) = uni_CUW_rxDataList.u8_byteData[idx];		/* 全データをコピー */
			uni_CUW_rxDataList.u8_byteData[idx] = (u8)0;					/* 受信の有無を判別するために、受信処理がすんだら、受信データクリア */
		}
		return TRUE;
	}
}


/*********************************/
/*!
 * @brief	セッティング変更モード 応答送信
 */
static void	v_CUW_ChangeSetting(void)
{
	u8	a_u8_txData[5];
	u8 *cp;

	switch( u8_CUW_rxID )
	{
	case 200:
		a_u8_txData[0] = (u8)201;											/* コマンドID = 201 */
		a_u8_txData[1] = (u8)0;												/* 空情報 */
		a_u8_txData[2] = (u8)0;
		a_u8_txData[3] = (u8)0;
		a_u8_txData[4] = (u8)0;
		v_CUW_SetScanData( a_u8_txData );									/* 送信データ 設定 */
		break;

	case 203:
		cp = (u8 *)u16_CUW_changeSettingAddress;							/* アドレス設定 */
		cp += (U32_CUW_LOG_BASE_ADRRESS);									/* CuWからはアドレスの下位2バイトしか送れないので、上位分を追加する */
		for( u8 idx = 0; idx < 4; idx ++)
		{
			*cp = u8_CUW_changeSettingMapData[idx];							/* 指定アドレスのデータを指定データに変更する */
			cp ++;															/* ポインタをずらす */
			u16_CUW_changeSettingAddress ++;								/* アドレスもずらす */
			u8_CUW_changeSettingTotalSize --;								/* 変更するべき残りのデータサイズ 更新 */
			if( u8_CUW_changeSettingTotalSize == (u8)0 )					/* 残りのデータサイズが0の場合 */
			{
				/*----- データ変更 完了通知 -----*/
				a_u8_txData[0] = (u8)205;									/* コマンドID = 205 */
				a_u8_txData[1] = (u8)0;										/* 空情報 */
				a_u8_txData[2] = (u8)0;										/* */
				a_u8_txData[3] = (u8)0;										/* */
				a_u8_txData[4] = (u8)0;										/* */
				v_CUW_SetScanData( a_u8_txData );							/* 送信データ 設定 */
				break;														/* Forループを抜ける */
			}
		}
		break;

	default:
		/* 既定外コード受信 */
		break;
	}

	u8_CUW_rxID = (u8)0;													/* 受信コード リセット	(次回以降、受信があった時のみ処理するようにクリアする) */
}

/*********************************/
/*!
 * @brief	指定アドレスデータの送信を行う
 */
static void v_CUW_RamMonitor(void)
{
	u8 a_u8_txData[5];
	u8 idx = (u8)0;

	switch( u8_CUW_rxID )
	{
	case 210:
		/*----- RAMモニタ切替要求 応答 -----*/
		if( u8_CUW_ramMonTotalSize <= ((U8_CUW_LOG_CH_SIZE) * (u8)2) )		/* 送信データ総バイト数要求 上限確認 */
		{
			a_u8_txData[1] = (u8)0;											/* 上限以下 → "0"を送信 */
		}
		else
		{
			a_u8_txData[1] = u8_CUW_ramMonTotalSize - ((U8_CUW_LOG_CH_SIZE) * (u8)2);	/*上限以上 → オーバー分を送信 */
		}
		u8_CUW_rxID = (u8)211;												/* 受信コマンドIDを211へ更新	(通信停止用にID=210をCuWが送信してくる。ID=211に切り替えることで、応答をした後、送信を止める) */
		a_u8_txData[0] = (u8)211;											/* コマンドID = 211 */
		a_u8_txData[2] = (u8)0;												/* 空情報 */
		a_u8_txData[3] = (u8)0;
		a_u8_txData[4] = (u8)0;
		v_CUW_SetScanData( a_u8_txData );									/* 送信データ 設定 */
		break;

	case 213:
		/*----- RAMモニタ アドレス指示終了 応答 -----*/
		a_u8_txData[0] = (u8)214;											/* コマンドID = 214 */
		a_u8_txData[1] = (u8)0;												/* 空情報 */
		a_u8_txData[2] = (u8)0;
		a_u8_txData[3] = (u8)0;
		a_u8_txData[4] = (u8)0;
		v_CUW_SetScanData( a_u8_txData );									/* 送信データ 設定 */

		u8_CUW_txID = (u8)215;												/* 送信コマンドID 設定 */
		u8_CUW_rxID = (u8)215;												/* 受信コマンドID 更新	(次のタイミングから、指定アドレスのデータ送信を開始する) */
		break;

	case 215:
		/*----- データ送信 -----*/
		if( u8_CUW_ramMonChIndex == (u8)0 )									/* インデックスが"0"の場合、データを一括で保存する		(データの送信タイミングによるズレを無くす為) */
		{
			for( idx = 0; idx < u8_CUW_ramMonTotalSize; idx ++ )
			{																/* 全てのチャンネルに対して */
				u8 *cp;
				cp = (u8 *)(u16_CUW_ramMonChAdrs[idx]);						/* アドレス設定 */
				cp += (U32_CUW_LOG_BASE_ADRRESS);							/* CuWからはアドレスの下位2バイトしか送れないので、上位分を追加する */
				if( u8_CUW_ramMonChSize[idx] == 1 )							/* サイズの確認 */
				{
					/* 1バイト */
					uni_CUW_ramMonChData[idx].word = *cp;					/* 1バイトデータをゼロ拡張して格納 */
				}
				else
				{
					clrpsw_i();												/* 割込み禁止:DI()		(データ取得中に割込みルーチン内でデータが変更されるのを防ぐ為) */
					/* 2バイト */
					uni_CUW_ramMonChData[idx].byte.low = *cp;				/* 2バイトデータの下位データ（アドレスが小さい方）を格納		 ┐奇数アドレスに2バイトでデータを取りに行くと */
					cp ++;													/*																 │偶数アドレスからのデータになってしまう為、 */
					uni_CUW_ramMonChData[idx].byte.high = *cp;				/* 2バイトデータの上位データ（アドレスが大きい方）を格納		 ┘わざわざ1バイト毎データを格納する方法に変更 */
					setpsw_i();												/* 割込み許可:EI() */
				}
			}
		}

		/*----- 送信データ 設定 -----*/
		idx = u8_CUW_ramMonChIndex;											/* インデックス 読込 */
		a_u8_txData[0] = u8_CUW_txID;										/* コマンドID 設定 */
		a_u8_txData[1] = uni_CUW_ramMonChData[idx].byte.low;				/* 1つ目データの下位1バイト 設定 */
		a_u8_txData[2] = uni_CUW_ramMonChData[idx].byte.high;				/* 1つ目データの上位1バイト 設定 */
		idx ++ ;															/* インデックス 更新 */
		a_u8_txData[3] = uni_CUW_ramMonChData[idx].byte.low;				/* 2つ目データの下位1バイト 設定 */
		a_u8_txData[4] = uni_CUW_ramMonChData[idx].byte.high;				/* 2つ目データの上位1バイト 設定 */
		idx ++ ;															/* インデックス 更新 */
		v_CUW_SetScanData( a_u8_txData );									/* 送信データ 設定 */

		if( idx < u8_CUW_ramMonTotalSize )									/* 全てのチャンネルを送信したか確認 */
		{
			u8_CUW_txID ++;													/* まだ残っている →	送信コマンドID 更新 */
			u8_CUW_ramMonChIndex = idx;										/*	インデックス 更新 */
		}
		else
		{
			u8_CUW_txID = (u8)215;											/* ひと回り終わった →	送信コマンドID リセット */
			u8_CUW_ramMonChIndex = (u8)0;									/*	インデックス リセット */
		}
		break;
	}
}


/*********************************/
/*!
 * @brief	バージョン情報を送信する
 */
static void v_CUW_SendSignature(void)
{
	u8 a_u8_txData[5];

	u8 a_u8_signatureLength = (u8)sizeof(u8_CUW_signature);					/* バージョン情報の文字数(バイト数)取得 */

	switch( u8_CUW_rxID )
	{
	case 252:
		/* バージョン情報 文字数(バイト数) 送信 */
		a_u8_txData[0] = (u8)252;											/* ID = 252 */
		a_u8_txData[1] = a_u8_signatureLength;								/* バージョン情報の文字数(バイト数) セット	*/
		a_u8_txData[2] = (u8)0;												/* 空情報 */
		a_u8_txData[3] = (u8)0;
		a_u8_txData[4] = (u8)0;
		break;

	case 253:
		/* バージョン情報 送信 */
		a_u8_txData[0] = u8_CUW_txID;										/* ID 設定 */
		for( u8 idx = 1; idx <= 4; idx++ )
		{
			a_u8_txData[idx] = u8_CUW_signature[u8_CUW_signatureIndex];		/* バージョン情報を1文字ずつ設定 */
			u8_CUW_signatureIndex ++;										/* インデックス 更新 */
		}

		if( u8_CUW_signatureIndex < a_u8_signatureLength )					/* インデックスの最大値 確認 */
		{
			u8_CUW_txID ++;													/* 最大値未満 → 指定のバイト数を送信していない → ID更新 */
		}
		else																/* 最大値以上 → 指定のバイト数を送信 */
		{
			u8_CUW_signatureIndex = 0;										/* インデックス 初期化 */
			u8_CUW_txID = 215;												/* 送信ID 初期化 */
		}
		break;
	}
	v_CUW_SetScanData( a_u8_txData );										/* 送信データ 設定 */
}


/*********************************/
/*!
 * @brief	ロギングOK信号を定期送信する
 */
static void v_CUW_PeriodicSendLogOk(void)
{
	if( u16_CUW_logOkKeepCounter != 0 )										/* ロギングOK信号定期送信モード 継続カウンタ 確認 */
	{
		u16_CUW_logOkKeepCounter -- ;
		if( --u8_CUW_logOkTimingCounter == 0 )								/* 送信タイミングカウンタ=0 確認 */
		{
			u8_CUW_logOkTimingCounter = (U8_CUW_LOG_OK_INTERVAL);			/* タイミングカウンタ リセット */
			v_CUW_SendLogOk();												/* ロギングOK信号 送信 */
		}
	}
	else
	{
		en_CUW_logMode = CUW_MODE_RAMMONITOR;								/* ロギングモードを RAMモニタ に設定 */
	}
}


/*********************************/
/*!
 * @brief	ロギングOK信号を送信する
 */
static void v_CUW_SendLogOk(void)
{
	u8 a_u8_txData[5];
	
	a_u8_txData[0] = (u8)10;												/* コマンドID = 010 */
	a_u8_txData[1] = (u8)0x4C;												/* 1バイト目 送信データ 4C = 'L' */
	a_u8_txData[2] = (u8)0x67;												/* 2バイト目 送信データ 67 = 'g' */
	a_u8_txData[3] = (u8)0x4F;												/* 3バイト目 送信データ 4F = 'O' */
	a_u8_txData[4] = (u8)0x4B;												/* 4バイト目 送信データ 4B = 'K' */
	v_CUW_SetScanData( a_u8_txData );										/* 送信データ 設定処理 */
}


/*********************************/
/*!
 * @brief			送信処理
 * @param a_u8_txData	ヘッダとチェックサム以外のデータ4バイト
 */
static void v_CUW_SetScanData( u8 a_u8_txData[] )
{
	uni_CUW_txDataList.u8_byteData[0] = (U8_CUW_LOG_HEADER);				/* ヘッダを送信データに設定 */
	u8 a_u8_cs = (u8)0;														/* チェックサム値 クリア */
	for( u8 idx = 0; idx <= 4; idx++ )
	{
		uni_CUW_txDataList.u8_byteData[idx + 1]	= a_u8_txData[idx];			/* データ 読込 & 格納 */
		a_u8_cs += a_u8_txData[idx];										/* チェックサム用に加算 */
	}
	uni_CUW_txDataList.u8_byteData[6] = ~a_u8_cs + 1;						/* チェックサムを送信データに設定 */

	uni_CUW_commControlFlg.bit.b_storedTxData = TRUE;						/* ロギング送信データ格納完了フラグ セット */


//	/* for DEBUG */
//	for( u8 idx = 0; idx <= 6; idx++ )
//	{
//		UNI_GL_txDataList.u8_byteData[idx] = uni_CUW_txDataList.u8_byteData[idx];
//	}
}


/*********************************/
/*!
 * @brief	ロギング通信関連 初期化
 */
void V_CUW_Initialize(void)
{
	/*----- 変数 初期化 -----*/
	en_CUW_logMode = CUW_MODE_SENDLOGOK;									/* ロギングモードを ロギングOK信号定期送信モード に設定 */

	for( s8 idx = (s8)(U8_CUW_LOG_CH_SIZE) - (s8)1; idx >= (s8)0; idx-- )
	{
		u16_CUW_ramMonChAdrs[idx] = (U16_CUW_DEFAULT_ADDRESS);				/* アドレス格納変数 初期化 */
	}

	u16_CUW_logOkKeepCounter	= (U8_CUW_LOG_OK_KEEP);						/* ロギングOK信号定期送信継続カウンタ 初期化 */
	u8_CUW_logOkTimingCounter	= (U8_CUW_LOG_OK_INTERVAL);					/* ロギングOK信号定期送信タイミングカウンタ 初期化 */
	uni_CUW_commControlFlg.bit.b_storedTxData = FALSE;						/* ロギング送信データ格納完了フラグ クリア */
	uni_CUW_commControlFlg.bit.b_finishedReceive = FALSE;					/* ロギング受信完了フラグ クリア */
}

/*********************************/
/*!
 * @brief	ロギング受信完了フラグ 提供
 */
bool B_CUW_IsFinishedReceive(void)
{
	return	uni_CUW_commControlFlg.bit.b_finishedReceive;					/* ロギング受信完了フラグ 提供 */
}


/*********************************/
/*!
 * @brief	ロギング受信完了フラグ セット
 */
void V_CUW_SetFinishedReceive(void)
{
	uni_CUW_commControlFlg.bit.b_finishedReceive = TRUE;					/* ロギング受信完了フラグ セット */
}


/*********************************/
/*!
 * @brief	ロギング通信異常フラグ セット
 */
void V_CUW_SetRxErr(void)
{
	uni_CUW_commControlFlg.bit.b_rxErr = TRUE;								/* ロギング通信異常フラグ セット */
}


/*********************************/
/*!
 * @brief	ロギング送信データ格納完了フラグ クリア
 */
void V_CUW_ClearStoredTxData(void)
{
	uni_CUW_commControlFlg.bit.b_storedTxData = FALSE;						/* ロギング送信データ格納完了フラグ クリア */
}

/*********************************/
/*!
 * @brief	ロギング送信データ格納完了フラグ 提供
 */
bool B_CUW_IsStoredTxData(void)
{
	return	uni_CUW_commControlFlg.bit.b_storedTxData;						/* ロギング送信データ格納完了フラグ 提供 */
}

/**
 * @brief 送信データ提供（ポインタ渡し）
 * @param *a_pu8_data
 * @return none
 */
void	V_CUW_GetTxData(u8 *a_pu8_data)
{
	for( u8 idx = (u8)0; idx < (u8)7; idx++)
	{
		*( a_pu8_data + idx ) = uni_CUW_txDataList.u8_byteData[idx];
	}
}

/**
 * @brief 受信データ格納（ポインタ渡し）
 * @param *a_pu8_data[idx]
 * @return none
 */
void	V_CUW_StoreRxData(u8 *a_pu8_data)
{
	for( u8 idx = (u8)0; idx < (u8)7; idx++ )
	{
		uni_CUW_rxDataList.u8_byteData[idx] = *( a_pu8_data + idx );
	}
}
