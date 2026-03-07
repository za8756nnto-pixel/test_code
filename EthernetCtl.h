/*
 * EthernetCtl.h
 *
 *  Created on: Mar 7, 2026
 *      Author: user
 *
 *  Note: LWIPを使用しない独自ETHスタック実装版
 *        (DMAなし・FreeRTASポーリングベース)
 *        lwip/ip4_addr.h の依存を除去し、独自IP型を使用
 *
 *  変更点 (元ヘッダからの修正):
 *    - `lwip/ip4_addr.h` / `lwip/err.h` 依存を除去
 *    - ip4_addr_t → EthIp4Addr_t (独自定義) に置換
 *    - ip_addr_t  → EthIp4Addr_t に統一
 *    - err_t      → bool に変更
 *    - typedef sturct → typedef struct (typo修正)
 *    - enum区切りを ; → , に修正
 *    - extern "C" { の { 抜けを修正
 *    - EEthernetCtl_ (Eが2つ) → EthernetCtl_ に修正
 *    - EthernetCtl_StartDhcoWhithFallback → StartDhcpWithFallback (typo修正)
 */

#ifndef INC_ETHERNETCTL_H_
#define INC_ETHERNETCTL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================
 * LWIPの ip4_addr_t 互換型 (LWIP不要)
 * ============================================================ */
typedef struct {
    uint32_t addr;   /*!< ネットワークバイトオーダー (big-endian) */
} EthIp4Addr_t;

/** IPアドレスをセットするマクロ (a.b.c.d 形式) */
#define ETH_IP4_ADDR(addr_ptr, a, b, c, d)          \
    do {                                             \
        (addr_ptr)->addr = ((uint32_t)(d) << 24) |  \
                           ((uint32_t)(c) << 16) |  \
                           ((uint32_t)(b) <<  8) |  \
                           ((uint32_t)(a));          \
    } while(0)

/** IPアドレスを uint8_t[4] から設定 */
#define ETH_IP4_ADDR_FROM_BYTES(addr_ptr, b0, b1, b2, b3) \
    ETH_IP4_ADDR(addr_ptr, b0, b1, b2, b3)

/** IPアドレスの各オクテット取得 */
#define ETH_IP4_ADDR_GET_BYTE(addr_ptr, idx) \
    (((uint8_t*)&(addr_ptr)->addr)[idx])

/* ============================================================
 * エラーコード (lwip/err.h 互換)
 * ============================================================ */
typedef enum {
    ETH_ERR_OK       =  0,   /*!< 成功 */
    ETH_ERR_MEM      = -1,   /*!< メモリ不足 */
    ETH_ERR_TIMEOUT  = -3,   /*!< タイムアウト */
    ETH_ERR_ARG      = -7,   /*!< 不正引数 */
    ETH_ERR_IF       = -11,  /*!< インタフェースエラー */
    ETH_ERR_NOLINK   = -14,  /*!< リンクなし */
} EthErr_t;

/* ============================================================
 * デフォルトポート定義
 * ============================================================ */
#define ETHERNETCTL_TCP_ECHO_PORT_DEFAULT   ((uint16_t)5000)
#define ETHERNETCTL_UDP_ECHO_PORT_DEFAULT   ((uint16_t)5001)

/* ============================================================
 * リクエスト列挙型
 * ============================================================ */
typedef enum
{
    ETH_REQ_NONE            = 0,
    ETH_REQ_LED_BLACKOUT_ON,
    ETH_REQ_LED_BLACKOUT_OFF
} EthernetCtl_Request;

/* ============================================================
 * ネットワーク状態構造体
 * ============================================================ */
typedef struct {
    bool           link_up;  /*!< リンク状態 */
    bool           is_dhcp;  /*!< true=DHCPで取得, false=フォールバックIP */
    uint8_t        mac[6];   /*!< MACアドレス */
    EthIp4Addr_t   ip;       /*!< IPアドレス */
    EthIp4Addr_t   mask;     /*!< サブネットマスク */
    EthIp4Addr_t   gw;       /*!< デフォルトゲートウェイ */
} EthernetCtl_Status;

/* ============================================================
 * Public API
 * ============================================================ */

/**
 * @brief  MACアドレスを設定する
 *         ETHタスク起動前に呼ぶこと
 * @param  mac  6バイトのMACアドレス
 * @retval true=成功, false=失敗
 */
bool EthernetCtl_SetMac(const uint8_t mac[6]);

/**
 * @brief  DHCP でIPアドレスを取得する。失敗した場合はフォールバックIPを使用
 *         内部でETHタスクを生成する。一度だけ呼ぶこと。
 * @param  timeout_ms     DHCPタイムアウト時間 [ms]
 * @param  fallback_ip    DHCP失敗時のIPアドレス (NULL不可)
 * @param  fallback_mask  DHCP失敗時のサブネットマスク (NULL不可)
 * @param  fallback_gw    DHCP失敗時のゲートウェイ (NULL不可)
 * @retval true=DHCPで取得成功, false=フォールバックIPを使用
 */
bool EthernetCtl_StartDhcpWithFallback(uint32_t timeout_ms,
                                       const EthIp4Addr_t* fallback_ip,
                                       const EthIp4Addr_t* fallback_mask,
                                       const EthIp4Addr_t* fallback_gw);

/**
 * @brief  現在のネットワーク状態を取得する
 * @param  out  結果を格納する構造体ポインタ (NULL不可)
 */
void EthernetCtl_GetStatus(EthernetCtl_Status* out);

/**
 * @brief  IPアドレスを文字列で取得する (例: "192.168.1.100")
 * @param  buf  書き込みバッファ
 * @param  len  バッファサイズ (最低16バイト推奨)
 */
void EthernetCtl_GetIpString(char* buf, size_t len);

/**
 * @brief  指定IPへICMP Echo Request (ping) を送信する
 *         応答はICMP Replyとして自動受信される (ポーリングで処理)
 * @param  dest  宛先IPアドレス (NULL不可)
 * @retval ETH_ERR_OK=送信成功, その他=エラー
 */
EthErr_t EthernetCtl_PingSend(const EthIp4Addr_t* dest);

/**
 * @brief  非同期リクエストをセットする (別タスクからの通知用)
 * @param  req  リクエスト種別
 */
void EthernetCtl_RequestSet(EthernetCtl_Request req);

/**
 * @brief  非同期リクエストを取得し、同時にクリアする (ETHタスク内で使用)
 * @retval リクエスト種別 (ETH_REQ_NONE=なし)
 */
EthernetCtl_Request EthernetCtl_RequestGetAndClear(void);

/**
 * @brief  LAN8742A PHYのLEDブラックアウト (消灯) を設定する
 *         LAN8742A の Special Control/Status Register を操作
 * @param  enable  true=消灯(Blackout), false=通常点灯
 * @retval true=成功, false=PHYアクセス失敗
 */
bool EthernetCtl_Lan8742Blackout_Set(bool enable);

/**
 * @brief  LAN8742A PHYのLEDブラックアウトを解除し、DHCP再取得を行う
 *         リンクダウン → LED復元 → リンクアップ → DHCP のシーケンスを実行
 * @param  timeout_ms     DHCPタイムアウト時間 [ms]
 * @param  fallback_ip    DHCP失敗時のIPアドレス (NULL不可)
 * @param  fallback_mask  DHCP失敗時のサブネットマスク (NULL不可)
 * @param  fallback_gw    DHCP失敗時のゲートウェイ (NULL不可)
 * @retval true=DHCP成功, false=フォールバックIP使用
 */
bool EthernetCtl_Lan8742Blackout_RestoreWithDhcp(uint32_t timeout_ms,
                                                  const EthIp4Addr_t* fallback_ip,
                                                  const EthIp4Addr_t* fallback_mask,
                                                  const EthIp4Addr_t* fallback_gw);

#ifdef __cplusplus
}
#endif

#endif /* INC_ETHERNETCTL_H_ */
