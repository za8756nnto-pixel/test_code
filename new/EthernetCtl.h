/*
 * EthernetCtl.h
 *
 *  Created on: Mar 7, 2026
 *      Author: user
 *
 *  LWIPなし・FreeRTOSポーリングベース ETHドライバ
 *  対象MCU : STM32H745/H755 デュアルコア (CM7で動作)
 *  対象PHY : LAN8742A (CubeMX lan8742コンポーネント使用)
 *  使用HAL : HAL_ETH_ReadData / HAL_ETH_Transmit
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
 * IPアドレス型 (LWIP不要・独自定義)
 * ============================================================ */
typedef struct {
    uint32_t addr;  /*!< バイト列格納: [0]=a, [1]=b, [2]=c, [3]=d */
} EthIp4Addr_t;

/** 例: ETH_IP4_ADDR(&x, 192, 168, 1, 100) */
#define ETH_IP4_ADDR(p, a, b, c, d)               \
    do {                                           \
        ((uint8_t*)&(p)->addr)[0] = (uint8_t)(a); \
        ((uint8_t*)&(p)->addr)[1] = (uint8_t)(b); \
        ((uint8_t*)&(p)->addr)[2] = (uint8_t)(c); \
        ((uint8_t*)&(p)->addr)[3] = (uint8_t)(d); \
    } while(0)

/* ============================================================
 * エラーコード
 * ============================================================ */
typedef enum {
    ETH_ERR_OK      =  0,
    ETH_ERR_MEM     = -1,
    ETH_ERR_TIMEOUT = -3,
    ETH_ERR_ARG     = -7,
    ETH_ERR_IF      = -11,
    ETH_ERR_NOLINK  = -14,
} EthErr_t;

/* ============================================================
 * ポート定義
 * ============================================================ */
#define ETHERNETCTL_TCP_ECHO_PORT_DEFAULT  ((uint16_t)5000)
#define ETHERNETCTL_UDP_ECHO_PORT_DEFAULT  ((uint16_t)5001)

/* ============================================================
 * リクエスト列挙型
 * ============================================================ */
typedef enum {
    ETH_REQ_NONE            = 0,
    ETH_REQ_LED_BLACKOUT_ON,
    ETH_REQ_LED_BLACKOUT_OFF
} EthernetCtl_Request;

/* ============================================================
 * ネットワーク状態構造体
 * ============================================================ */
typedef struct {
    bool         link_up;  /*!< リンク状態 */
    bool         is_dhcp;  /*!< true=DHCP取得, false=フォールバックIP */
    uint8_t      mac[6];   /*!< MACアドレス */
    EthIp4Addr_t ip;
    EthIp4Addr_t mask;
    EthIp4Addr_t gw;
} EthernetCtl_Status;

/* ============================================================
 * Public API
 * ============================================================ */

/**
 * @brief  MACアドレスを設定する
 *         EthernetCtl_StartDhcpWithFallback() より前に呼ぶこと
 */
bool EthernetCtl_SetMac(const uint8_t mac[6]);

/**
 * @brief  ETH初期化 + DHCP取得。失敗時はフォールバックIPを使用。
 *         内部でFreeRTOSタスクを生成する。一度だけ呼ぶこと。
 *         osKernelStart() 後 (スケジューラ起動後タスク内) に呼ぶこと。
 * @param  timeout_ms    DHCPタイムアウト[ms]
 * @param  fallback_ip   DHCP失敗時IP   (NULL不可)
 * @param  fallback_mask DHCP失敗時Mask (NULL不可)
 * @param  fallback_gw   DHCP失敗時GW   (NULL不可)
 * @retval true=DHCP成功, false=フォールバックIP使用
 */
bool EthernetCtl_StartDhcpWithFallback(uint32_t timeout_ms,
                                        const EthIp4Addr_t *fallback_ip,
                                        const EthIp4Addr_t *fallback_mask,
                                        const EthIp4Addr_t *fallback_gw);

/**
 * @brief  現在のネットワーク状態を取得する (スレッドセーフ)
 */
void EthernetCtl_GetStatus(EthernetCtl_Status *out);

/**
 * @brief  IPアドレスを文字列で取得する (例: "192.168.1.100")
 *         buf は最低16バイト用意すること
 */
void EthernetCtl_GetIpString(char *buf, size_t len);

/**
 * @brief  指定IPへICMP Echo Request (ping) を送信する
 * @retval ETH_ERR_OK=送信成功, その他=エラー
 */
EthErr_t EthernetCtl_PingSend(const EthIp4Addr_t *dest);

/**
 * @brief  非同期リクエストをセット (割り込みハンドラ等から呼んでOK)
 */
void EthernetCtl_RequestSet(EthernetCtl_Request req);

/**
 * @brief  非同期リクエストを取得してクリアする (ETHタスク内部で使用)
 */
EthernetCtl_Request EthernetCtl_RequestGetAndClear(void);

/**
 * @brief  LAN8742A PHY LEDをブラックアウト(消灯)または復帰させる
 *
 *         LAN8742A の PHYSCSR (Reg0x11) の LED制御ビットを操作する。
 *         enable=true  : LED強制OFF (ブラックアウト)
 *         enable=false : LED通常動作に復帰
 *
 * @retval true=成功, false=PHYアクセス失敗
 */
bool EthernetCtl_Lan8742Blackout_Set(bool enable);

/**
 * @brief  LAN8742A LEDブラックアウト解除 + DHCP再取得
 * @retval true=DHCP成功, false=フォールバックIP使用
 */
bool EthernetCtl_Lan8742Blackout_RestoreWithDhcp(uint32_t timeout_ms,
                                                  const EthIp4Addr_t *fallback_ip,
                                                  const EthIp4Addr_t *fallback_mask,
                                                  const EthIp4Addr_t *fallback_gw);

#ifdef __cplusplus
}
#endif

#endif /* INC_ETHERNETCTL_H_ */
