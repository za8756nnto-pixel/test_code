/*
 * EthernetCtl.c
 *
 *  Created on: Mar 7, 2026
 *      Author: user
 *
 *  LWIPなし・FreeRTOSポーリングベース ETHドライバ
 *  対象MCU : STM32H745/H755 デュアルコア (CM7で動作)
 *  対象PHY : LAN8742A (RMII)
 *
 *  受信アーキテクチャ:
 *    HAL_ETH_ReadData() を使用したポーリング受信。
 *    HAL は受信時に以下の2つのweakコールバックを呼ぶ:
 *      HAL_ETH_RxAllocateCallback() : 受信バッファアドレスを HAL に渡す
 *      HAL_ETH_RxLinkCallback()     : 受信完了データを受け取る
 *    本ファイルでこれら2関数をオーバーライドする。
 *
 *  送信:
 *    HAL_ETH_Transmit() (ポーリング・ブロッキング) を使用。
 *
 *  DCacheについて:
 *    DCacheは無効のまま使用する。
 *    有効にする場合はDMAディスクリプタ/バッファ領域のMPU設定が必要。
 */

#include "EthernetCtl.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 * コンパイル設定
 * ============================================================ */
#define ETHCTL_PHY_ADDR         0x00U   /*!< LAN8742A MDIOアドレス (要確認) */
#define ETHCTL_LINK_POLL_MS     500U    /*!< PHYリンク監視周期 */
#define ETHCTL_TASK_STACK       1024U   /*!< ETHタスクスタック(ワード) */
#define ETHCTL_TASK_PRI         (tskIDLE_PRIORITY + 3U)

/* RxバッファはHALが1フレーム分を1バッファで受け取る前提 */
#define ETHCTL_RX_BUF_SIZE      1536U
#define ETHCTL_RX_BUF_COUNT     ETH_RX_DESC_CNT   /* HAL定義(通常4) */
#define ETHCTL_TX_BUF_SIZE      1536U

/* ============================================================
 * LAN8742A PHY レジスタ
 * ============================================================ */
#define LAN8742_REG_BCR     0x00U
#define LAN8742_REG_BSR     0x01U
#define LAN8742_BCR_PWRDN   (1U << 11)  /*!< Power Down: PHYパワーダウン */
#define LAN8742_BSR_LINK    (1U <<  2)  /*!< Link Status */
#define LAN8742_BSR_ANEG    (1U <<  5)  /*!< Auto-Negotiation Complete */

/* ============================================================
 * Ethernet プロトコル定数
 * ============================================================ */
#define ETYPE_ARP           0x0806U
#define ETYPE_IP            0x0800U
#define IPPROTO_ICMP        1U
#define IPPROTO_UDP         17U
#define ICMP_ECHO_REQ       8U
#define ICMP_ECHO_REP       0U
#define ARP_OP_REQ          1U
#define ARP_OP_REP          2U
#define DHCP_SRV_PORT       67U
#define DHCP_CLI_PORT       68U
#define DHCP_OP_REQ         1U
#define DHCP_OP_REP         2U
#define DHCP_MSG_DISCOVER   1U
#define DHCP_MSG_OFFER      2U
#define DHCP_MSG_REQUEST    3U
#define DHCP_MSG_ACK        5U
#define DHCP_MSG_NAK        6U

/* ============================================================
 * パケット構造体 (packed)
 * ============================================================ */
#pragma pack(push, 1)
typedef struct { uint8_t dst[6]; uint8_t src[6]; uint16_t type; } EthHdr_t;
typedef struct {
    uint8_t  ver_ihl; uint8_t  tos;   uint16_t total_len;
    uint16_t id;      uint16_t flags; uint8_t  ttl;
    uint8_t  proto;   uint16_t csum;
    uint8_t  src[4];  uint8_t  dst[4];
} IpHdr_t;
typedef struct { uint16_t sp; uint16_t dp; uint16_t len; uint16_t csum; } UdpHdr_t;
typedef struct {
    uint8_t  htype; uint8_t ptype; uint8_t hlen; uint8_t plen;
    uint16_t oper;
    uint8_t  sha[6]; uint8_t spa[4]; uint8_t tha[6]; uint8_t tpa[4];
} ArpPkt_t;
typedef struct { uint8_t type; uint8_t code; uint16_t csum; uint16_t id; uint16_t seq; } IcmpHdr_t;
typedef struct {
    uint8_t  op; uint8_t htype; uint8_t hlen; uint8_t hops;
    uint32_t xid; uint16_t secs; uint16_t flags;
    uint8_t  ciaddr[4]; uint8_t yiaddr[4]; uint8_t siaddr[4]; uint8_t giaddr[4];
    uint8_t  chaddr[16]; uint8_t sname[64]; uint8_t file[128];
    uint8_t  magic[4]; uint8_t options[308];
} DhcpPkt_t;
#pragma pack(pop)

/* ============================================================
 * DHCP ステートマシン
 * ============================================================ */
typedef enum {
    DHCP_ST_IDLE = 0,
    DHCP_ST_OFFER_WAIT,
    DHCP_ST_ACK_WAIT,
    DHCP_ST_BOUND,
    DHCP_ST_FAILED,
} DhcpSt_t;

/* ============================================================
 * Rx バッファ管理
 *   HAL_ETH_RxAllocateCallback で HAL にバッファを渡し、
 *   HAL_ETH_RxLinkCallback で受信データポインタを受け取る。
 *   シンプルな固定バッファプールで管理する。
 * ============================================================ */
typedef struct {
    uint8_t  data[ETHCTL_RX_BUF_SIZE];
    bool     in_use;
} RxBuf_t;

/* 4バイトアライン。DCacheを無効にしているためsection指定不要 */
static RxBuf_t s_rxpool[ETHCTL_RX_BUF_COUNT] __attribute__((aligned(4)));

/* HAL_ETH_RxLinkCallback で受け取った受信済みバッファポインタ */
static uint8_t *s_rx_received_buf  = NULL;
static uint16_t s_rx_received_len  = 0;

/* ============================================================
 * Tx バッファ (送信組み立て用)
 * ============================================================ */
static uint8_t s_txbuf[ETHCTL_TX_BUF_SIZE] __attribute__((aligned(4)));

/* ============================================================
 * HAL ETH ハンドル・送信設定
 * ============================================================ */
static ETH_HandleTypeDef     s_heth;
static ETH_TxPacketConfigTypeDef s_txcfg;

/* DMAディスクリプタ: 4バイトアライン */
static ETH_DMADescTypeDef s_rxdesc[ETH_RX_DESC_CNT] __attribute__((aligned(4)));
static ETH_DMADescTypeDef s_txdesc[ETH_TX_DESC_CNT] __attribute__((aligned(4)));

/* ============================================================
 * ネットワーク状態
 * ============================================================ */
static EthernetCtl_Status s_status;
static SemaphoreHandle_t  s_mutex = NULL;

/* ============================================================
 * DHCP 管理
 * ============================================================ */
static DhcpSt_t      s_dhcp_st         = DHCP_ST_IDLE;
static TickType_t    s_dhcp_deadline   = 0;
static uint32_t      s_dhcp_timeout_ms = 10000U;
static uint8_t       s_dhcp_offered[4];
static uint8_t       s_dhcp_server[4];
static EthIp4Addr_t  s_fb_ip, s_fb_mask, s_fb_gw;
static bool          s_dhcp_renew = false;

/* XID: 固定値 (複数デバイスが同一LAN上にいる場合は要ランダム化) */
static const uint32_t DHCP_XID = 0xEC7A1B2CU;

/* ============================================================
 * その他
 * ============================================================ */
static volatile EthernetCtl_Request s_request = ETH_REQ_NONE;
static uint16_t  s_ping_seq      = 0;
static bool      s_blackout      = false;

static const uint8_t MAC_BCAST[6]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t IP_BCAST[4]   = {0xFF,0xFF,0xFF,0xFF};
static const uint8_t IP_ZERO[4]    = {0x00,0x00,0x00,0x00};
static const uint8_t DHCP_MAGIC[4] = {0x63,0x82,0x53,0x63};

/* ============================================================
 * 内部プロトタイプ
 * ============================================================ */
static bool     prv_hw_init(void);
static bool     prv_phy_rd(uint32_t reg, uint32_t *v);
static bool     prv_phy_wr(uint32_t reg, uint32_t v);
static bool     prv_phy_link(void);
static void     prv_link_up(void);
static void     prv_link_down(void);
static void     prv_poll_rx(void);
static void     prv_dispatch(uint8_t *buf, uint16_t len);
static void     prv_arp(uint8_t *buf, uint16_t len);
static void     prv_ip(uint8_t *buf, uint16_t len);
static void     prv_icmp(uint8_t *buf, uint16_t len, IpHdr_t *iph);
static void     prv_udp(uint8_t *buf, uint16_t len, IpHdr_t *iph);
static void     prv_dhcp_rx(DhcpPkt_t *dhcp, uint16_t len);
static void     prv_dhcp_discover(void);
static void     prv_dhcp_request(void);
static void     prv_dhcp_fallback(void);
static bool     prv_tx(uint8_t *buf, uint16_t len);
static uint16_t prv_ip_csum(const void *buf, uint16_t len);
static uint16_t prv_icmp_csum(IcmpHdr_t *h, const uint8_t *data, uint16_t dlen);
static uint16_t prv_htons(uint16_t v);
static uint32_t prv_htonl(uint32_t v);
static uint16_t prv_ntohs(uint16_t v);
static bool     prv_ip_eq(const uint8_t *a, const uint8_t *b);
static uint8_t *prv_ip_b(const EthIp4Addr_t *a);
static void     prv_eth_task(void *arg);

/* ============================================================
 * HAL RxAllocate コールバック (weak オーバーライド)
 *   HAL_ETH_ReadData() 内部から呼ばれる。
 *   空きバッファのアドレスを *buff に書いて返す。
 *   空きがなければ *buff = NULL を返す (その受信フレームは破棄される)。
 * ============================================================ */
void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
    *buff = NULL;
    for (uint32_t i = 0; i < ETHCTL_RX_BUF_COUNT; i++) {
        if (!s_rxpool[i].in_use) {
            s_rxpool[i].in_use = true;
            *buff = s_rxpool[i].data;
            return;
        }
    }
    /* 空きなし: HAL はこのディスクリプタをスキップする */
}

/* ============================================================
 * HAL RxLink コールバック (weak オーバーライド)
 *   HAL_ETH_ReadData() 内部から呼ばれる。
 *   1フレーム分のデータが buff / Length で渡される。
 *   pStart/pEnd は LWIP pbuf チェーン用だが、ここでは使わない。
 *   受信済みデータをポーリングループへ渡すために静的変数に保存する。
 *
 *   注意: 1回のポーリングで複数フレームが来た場合、最後の1フレームのみ
 *         処理される。実用上は 1ms ポーリングで問題ないが、高負荷時は
 *         s_rx_received_buf をキューに変更することを検討すること。
 * ============================================================ */
void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd,
                             uint8_t *buff, uint16_t Length)
{
    (void)pStart;
    (void)pEnd;

    /* 前回フレームが未処理の場合は破棄してバッファ解放 */
    if (s_rx_received_buf != NULL) {
        for (uint32_t i = 0; i < ETHCTL_RX_BUF_COUNT; i++) {
            if (s_rxpool[i].data == s_rx_received_buf) {
                s_rxpool[i].in_use = false;
                break;
            }
        }
    }
    s_rx_received_buf = buff;
    s_rx_received_len = Length;
}

/* ============================================================
 * Public: MACアドレス設定
 * ============================================================ */
bool EthernetCtl_SetMac(const uint8_t mac[6])
{
    if (mac == NULL) return false;
    memcpy(s_status.mac, mac, 6);
    return true;
}

/* ============================================================
 * Public: DHCP開始 (メイン初期化・ブロッキング)
 * ============================================================ */
bool EthernetCtl_StartDhcpWithFallback(uint32_t timeout_ms,
                                        const EthIp4Addr_t *fallback_ip,
                                        const EthIp4Addr_t *fallback_mask,
                                        const EthIp4Addr_t *fallback_gw)
{
    if (!fallback_ip || !fallback_mask || !fallback_gw) return false;

    s_dhcp_timeout_ms = timeout_ms;
    s_fb_ip   = *fallback_ip;
    s_fb_mask = *fallback_mask;
    s_fb_gw   = *fallback_gw;

    /* ミューテックス生成 */
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) return false;
    }

    /* HW初期化 */
    if (!prv_hw_init()) return false;

    /* ETHポーリングタスク生成 */
    if (xTaskCreate(prv_eth_task, "EthCtl",
                    ETHCTL_TASK_STACK, NULL,
                    ETHCTL_TASK_PRI, NULL) != pdPASS) {
        return false;
    }

    /* DHCP完了 or フォールバック適用までブロッキング待機
     * タイムアウトはDHCPタイムアウト + PHYリンクアップ猶予3秒 */
    TickType_t deadline = xTaskGetTickCount()
                          + pdMS_TO_TICKS(timeout_ms + 3000U);
    while (xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (s_dhcp_st == DHCP_ST_BOUND || s_dhcp_st == DHCP_ST_FAILED) {
            return (s_dhcp_st == DHCP_ST_BOUND);
        }
    }
    /* タイムアウト: フォールバックを強制適用 */
    prv_dhcp_fallback();
    return false;
}

/* ============================================================
 * Public: ステータス取得
 * ============================================================ */
void EthernetCtl_GetStatus(EthernetCtl_Status *out)
{
    if (!out) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, &s_status, sizeof(EthernetCtl_Status));
    xSemaphoreGive(s_mutex);
}

/* ============================================================
 * Public: IP文字列取得
 * ============================================================ */
void EthernetCtl_GetIpString(char *buf, size_t len)
{
    if (!buf || len < 16U) return;
    uint8_t *p = prv_ip_b(&s_status.ip);
    snprintf(buf, len, "%d.%d.%d.%d", p[0], p[1], p[2], p[3]);
}

/* ============================================================
 * Public: Ping送信
 * ============================================================ */
EthErr_t EthernetCtl_PingSend(const EthIp4Addr_t *dest)
{
    if (!dest)                                     return ETH_ERR_ARG;
    if (!s_status.link_up)                         return ETH_ERR_NOLINK;
    if (prv_ip_eq(prv_ip_b(&s_status.ip), IP_ZERO)) return ETH_ERR_IF;

    static const uint8_t payload[] = "EthernetCtl_Ping";
    uint16_t plen  = (uint16_t)(sizeof(payload) - 1U);
    uint16_t total = (uint16_t)(sizeof(EthHdr_t) + sizeof(IpHdr_t) +
                                sizeof(IcmpHdr_t) + plen);
    if (total > ETHCTL_TX_BUF_SIZE) return ETH_ERR_MEM;

    memset(s_txbuf, 0, total);
    EthHdr_t  *eh  = (EthHdr_t  *)s_txbuf;
    IpHdr_t   *ih  = (IpHdr_t   *)(s_txbuf + sizeof(EthHdr_t));
    IcmpHdr_t *ic  = (IcmpHdr_t *)((uint8_t*)ih + sizeof(IpHdr_t));
    uint8_t   *dat = (uint8_t*)ic + sizeof(IcmpHdr_t);

    /* 宛先MACはブロードキャスト(ARPキャッシュなし簡易実装) */
    memcpy(eh->dst, MAC_BCAST,             6);
    memcpy(eh->src, s_status.mac,          6);
    eh->type = prv_htons(ETYPE_IP);

    ih->ver_ihl   = 0x45;
    ih->total_len = prv_htons((uint16_t)(sizeof(IpHdr_t) + sizeof(IcmpHdr_t) + plen));
    ih->id        = prv_htons((uint16_t)(0x3C00U + s_ping_seq));
    ih->ttl       = 64;
    ih->proto     = IPPROTO_ICMP;
    memcpy(ih->src, prv_ip_b(&s_status.ip), 4);
    memcpy(ih->dst, prv_ip_b(dest),          4);
    ih->csum      = prv_ip_csum(ih, sizeof(IpHdr_t));

    ic->type = ICMP_ECHO_REQ;
    ic->id   = prv_htons(0xECA0);
    ic->seq  = prv_htons(s_ping_seq++);
    memcpy(dat, payload, plen);
    ic->csum = prv_icmp_csum(ic, dat, plen);

    return prv_tx(s_txbuf, total) ? ETH_ERR_OK : ETH_ERR_IF;
}

/* ============================================================
 * Public: リクエスト操作
 * ============================================================ */
void EthernetCtl_RequestSet(EthernetCtl_Request req)
{
    taskENTER_CRITICAL();
    s_request = req;
    taskEXIT_CRITICAL();
}

EthernetCtl_Request EthernetCtl_RequestGetAndClear(void)
{
    taskENTER_CRITICAL();
    EthernetCtl_Request r = s_request;
    s_request = ETH_REQ_NONE;
    taskEXIT_CRITICAL();
    return r;
}

/* ============================================================
 * Public: LAN8742A Blackout制御
 * ============================================================ */
bool EthernetCtl_Lan8742Blackout_Set(bool enable)
{
    uint32_t bcr = 0;
    if (!prv_phy_rd(LAN8742_REG_BCR, &bcr)) return false;

    if (enable) bcr |=  LAN8742_BCR_PWRDN;
    else        bcr &= ~LAN8742_BCR_PWRDN;

    if (!prv_phy_wr(LAN8742_REG_BCR, bcr)) return false;

    s_blackout = enable;
    if (enable) {
        prv_link_down();
    } else {
        vTaskDelay(pdMS_TO_TICKS(300)); /* PHY起動待ち */
    }
    return true;
}

bool EthernetCtl_Lan8742Blackout_RestoreWithDhcp(uint32_t timeout_ms,
                                                  const EthIp4Addr_t *fallback_ip,
                                                  const EthIp4Addr_t *fallback_mask,
                                                  const EthIp4Addr_t *fallback_gw)
{
    if (!fallback_ip || !fallback_mask || !fallback_gw) return false;

    s_dhcp_timeout_ms = timeout_ms;
    s_fb_ip   = *fallback_ip;
    s_fb_mask = *fallback_mask;
    s_fb_gw   = *fallback_gw;

    if (s_blackout) {
        if (!EthernetCtl_Lan8742Blackout_Set(false)) return false;
    }
    s_dhcp_renew = true;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms + 3000U);
    while (xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (s_dhcp_st == DHCP_ST_BOUND || s_dhcp_st == DHCP_ST_FAILED) {
            return (s_dhcp_st == DHCP_ST_BOUND);
        }
    }
    return false;
}

/* ============================================================
 * 内部: FreeRTOS ETHタスク
 * ============================================================ */
static void prv_eth_task(void *arg)
{
    (void)arg;
    TickType_t link_tick = xTaskGetTickCount();
    bool       last_link = false;

    vTaskDelay(pdMS_TO_TICKS(200)); /* PHY安定待ち */

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* ─ (1) PHY リンク監視 ─ */
        if ((now - link_tick) >= pdMS_TO_TICKS(ETHCTL_LINK_POLL_MS)) {
            link_tick = now;
            if (!s_blackout) {
                bool cur = prv_phy_link();
                if ( cur && !last_link) prv_link_up();
                if (!cur &&  last_link) prv_link_down();
                last_link = cur;
            }
        }

        /* ─ (2) DHCP再取得要求 ─ */
        if (s_dhcp_renew && s_status.link_up) {
            s_dhcp_renew   = false;
            s_dhcp_st      = DHCP_ST_OFFER_WAIT;
            s_dhcp_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(s_dhcp_timeout_ms);
            prv_dhcp_discover();
        }

        /* ─ (3) DHCPタイムアウト監視 ─ */
        now = xTaskGetTickCount();
        if ((s_dhcp_st == DHCP_ST_OFFER_WAIT || s_dhcp_st == DHCP_ST_ACK_WAIT)
            && (now >= s_dhcp_deadline)) {
            prv_dhcp_fallback();
        }

        /* ─ (4) 受信ポーリング ─ */
        if (s_status.link_up) {
            prv_poll_rx();
        }

        /* ─ (5) リクエスト処理 ─ */
        EthernetCtl_Request req = EthernetCtl_RequestGetAndClear();
        switch (req) {
        case ETH_REQ_LED_BLACKOUT_ON:
            EthernetCtl_Lan8742Blackout_Set(true);
            break;
        case ETH_REQ_LED_BLACKOUT_OFF:
            EthernetCtl_Lan8742Blackout_RestoreWithDhcp(
                s_dhcp_timeout_ms, &s_fb_ip, &s_fb_mask, &s_fb_gw);
            break;
        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ============================================================
 * 内部: HW初期化
 *   GPIO設定は MX_GPIO_Init() (CubeMX生成) で済んでいるため
 *   ここでは HAL_ETH_Init のみ行う。
 *   MspInit は HAL_ETH_MspInit() (下記) でクロックを有効化する。
 * ============================================================ */
static bool prv_hw_init(void)
{
    s_heth.Instance            = ETH;
    s_heth.Init.MACAddr        = s_status.mac;
    s_heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    s_heth.Init.TxDesc         = s_txdesc;
    s_heth.Init.RxDesc         = s_rxdesc;
    s_heth.Init.RxBuffLen      = ETHCTL_RX_BUF_SIZE;

    if (HAL_ETH_Init(&s_heth) != HAL_OK) return false;

    /* Tx設定: CRC/Padを自動挿入 */
    memset(&s_txcfg, 0, sizeof(s_txcfg));
    s_txcfg.Attributes   = ETH_TX_PACKETS_FEATURES_CRCPAD;
    s_txcfg.CRCPadCtrl   = ETH_CRC_PAD_INSERT;

    return true;
}

/* ============================================================
 * HAL_ETH_MspInit オーバーライド
 *   HAL_ETH_Init() 内部から呼ばれる。
 *   GPIO設定はCubeMX生成のMX_GPIO_Init()で済んでいるため、
 *   ここではETHクロック有効化のみ行う。
 * ============================================================ */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
    (void)heth;
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();
}

/* ============================================================
 * 内部: PHY SMI読み書き
 * ============================================================ */
static bool prv_phy_rd(uint32_t reg, uint32_t *v)
{
    return HAL_ETH_ReadPHYRegister(&s_heth, ETHCTL_PHY_ADDR, reg, v) == HAL_OK;
}
static bool prv_phy_wr(uint32_t reg, uint32_t v)
{
    return HAL_ETH_WritePHYRegister(&s_heth, ETHCTL_PHY_ADDR, reg, v) == HAL_OK;
}

/* ============================================================
 * 内部: PHY リンク確認 (BSR Bit2, 2回読み)
 * ============================================================ */
static bool prv_phy_link(void)
{
    uint32_t bsr = 0;
    prv_phy_rd(LAN8742_REG_BSR, &bsr); /* 1回目: ラッチクリア */
    prv_phy_rd(LAN8742_REG_BSR, &bsr); /* 2回目: 確定値       */
    return (bsr & LAN8742_BSR_LINK) != 0U;
}

/* ============================================================
 * 内部: LINK UP
 * ============================================================ */
static void prv_link_up(void)
{
    /* Auto-Negotiation 完了待ち (最大1秒) */
    for (int i = 0; i < 100; i++) {
        uint32_t bsr = 0;
        prv_phy_rd(LAN8742_REG_BSR, &bsr);
        if (bsr & LAN8742_BSR_ANEG) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    HAL_ETH_Start(&s_heth);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_status.link_up = true;
    s_status.is_dhcp = false;
    xSemaphoreGive(s_mutex);

    /* DHCP開始 */
    s_dhcp_st       = DHCP_ST_OFFER_WAIT;
    s_dhcp_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(s_dhcp_timeout_ms);
    prv_dhcp_discover();
}

/* ============================================================
 * 内部: LINK DOWN
 * ============================================================ */
static void prv_link_down(void)
{
    HAL_ETH_Stop(&s_heth);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_status.link_up = false;
    s_status.is_dhcp = false;
    memset(&s_status.ip,   0, sizeof(EthIp4Addr_t));
    memset(&s_status.mask, 0, sizeof(EthIp4Addr_t));
    memset(&s_status.gw,   0, sizeof(EthIp4Addr_t));
    xSemaphoreGive(s_mutex);

    s_dhcp_st = DHCP_ST_IDLE;
}

/* ============================================================
 * 内部: 受信ポーリング
 *   HAL_ETH_ReadData() を呼ぶと、内部で
 *     HAL_ETH_RxAllocateCallback() → バッファ確保
 *     HAL_ETH_RxLinkCallback()     → データ受け渡し
 *   が呼ばれ、s_rx_received_buf / s_rx_received_len にセットされる。
 *   受信データがなければ HAL_ERROR が返る。
 * ============================================================ */
static void prv_poll_rx(void)
{
    void *appbuf = NULL;

    /* 受信フレームがある間ループ (最大 ETH_RX_DESC_CNT 回) */
    for (uint32_t i = 0; i < (uint32_t)ETH_RX_DESC_CNT; i++) {

        s_rx_received_buf = NULL;
        s_rx_received_len = 0;

        if (HAL_ETH_ReadData(&s_heth, &appbuf) != HAL_OK) {
            break; /* 受信フレームなし */
        }

        /* RxLinkCallback でセットされたデータを処理 */
        if (s_rx_received_buf != NULL && s_rx_received_len > sizeof(EthHdr_t)) {
            prv_dispatch(s_rx_received_buf, s_rx_received_len);
        }

        /* バッファ解放 */
        if (s_rx_received_buf != NULL) {
            for (uint32_t j = 0; j < ETHCTL_RX_BUF_COUNT; j++) {
                if (s_rxpool[j].data == s_rx_received_buf) {
                    s_rxpool[j].in_use = false;
                    break;
                }
            }
            s_rx_received_buf = NULL;
        }
    }
}

/* ============================================================
 * 内部: フレーム振り分け
 * ============================================================ */
static void prv_dispatch(uint8_t *buf, uint16_t len)
{
    EthHdr_t *eh   = (EthHdr_t *)buf;
    uint16_t  type = prv_ntohs(eh->type);

    /* 宛先が自分 or ブロードキャスト以外は捨てる */
    if (memcmp(eh->dst, s_status.mac, 6) != 0 &&
        memcmp(eh->dst, MAC_BCAST,    6) != 0) return;

    if      (type == ETYPE_ARP) prv_arp(buf, len);
    else if (type == ETYPE_IP)  prv_ip(buf, len);
}

/* ============================================================
 * 内部: ARP Reply 返送
 * ============================================================ */
static void prv_arp(uint8_t *buf, uint16_t len)
{
    if (len < (uint16_t)(sizeof(EthHdr_t) + sizeof(ArpPkt_t))) return;

    EthHdr_t *eh  = (EthHdr_t *)buf;
    ArpPkt_t *arp = (ArpPkt_t *)(buf + sizeof(EthHdr_t));

    if (prv_ntohs(arp->oper) != ARP_OP_REQ)                   return;
    if (!prv_ip_eq(arp->tpa, prv_ip_b(&s_status.ip)))         return;
    if (prv_ip_eq(prv_ip_b(&s_status.ip), IP_ZERO))           return;

    uint8_t  rep[sizeof(EthHdr_t) + sizeof(ArpPkt_t)];
    EthHdr_t *re = (EthHdr_t *)rep;
    ArpPkt_t *ra = (ArpPkt_t *)(rep + sizeof(EthHdr_t));

    memcpy(re->dst, eh->src,       6);
    memcpy(re->src, s_status.mac,  6);
    re->type = prv_htons(ETYPE_ARP);

    ra->htype = prv_htons(1U);
    ra->ptype = prv_htons(0x0800U);
    ra->hlen  = 6; ra->plen = 4;
    ra->oper  = prv_htons(ARP_OP_REP);
    memcpy(ra->sha, s_status.mac,              6);
    memcpy(ra->spa, prv_ip_b(&s_status.ip),    4);
    memcpy(ra->tha, arp->sha,                  6);
    memcpy(ra->tpa, arp->spa,                  4);

    prv_tx(rep, (uint16_t)sizeof(rep));
}

/* ============================================================
 * 内部: IP振り分け
 * ============================================================ */
static void prv_ip(uint8_t *buf, uint16_t len)
{
    if (len < (uint16_t)(sizeof(EthHdr_t) + sizeof(IpHdr_t))) return;

    IpHdr_t *iph = (IpHdr_t *)(buf + sizeof(EthHdr_t));

    bool for_me = prv_ip_eq(iph->dst, prv_ip_b(&s_status.ip))
               || prv_ip_eq(iph->dst, IP_BCAST);

    bool dhcp_rx = (s_dhcp_st == DHCP_ST_OFFER_WAIT
                 || s_dhcp_st == DHCP_ST_ACK_WAIT);

    if (iph->proto == IPPROTO_ICMP && for_me) {
        prv_icmp(buf, len, iph);
    } else if (iph->proto == IPPROTO_UDP && (for_me || dhcp_rx)) {
        prv_udp(buf, len, iph);
    }
}

/* ============================================================
 * 内部: ICMP Echo Reply 返送
 * ============================================================ */
static void prv_icmp(uint8_t *buf, uint16_t len, IpHdr_t *iph)
{
    uint16_t   ip_hl  = (uint16_t)((iph->ver_ihl & 0x0FU) * 4U);
    IcmpHdr_t *ic     = (IcmpHdr_t *)((uint8_t*)iph + ip_hl);
    uint16_t   iclen  = prv_ntohs(iph->total_len) - ip_hl;

    if (iclen < sizeof(IcmpHdr_t))    return;
    if (ic->type != ICMP_ECHO_REQ)   return;

    uint8_t *payload = (uint8_t*)ic + sizeof(IcmpHdr_t);
    uint16_t plen    = iclen - (uint16_t)sizeof(IcmpHdr_t);

    uint16_t total = (uint16_t)(sizeof(EthHdr_t) + sizeof(IpHdr_t)
                                + sizeof(IcmpHdr_t) + plen);
    if (total > ETHCTL_TX_BUF_SIZE) return;

    memset(s_txbuf, 0, total);
    EthHdr_t  *re  = (EthHdr_t  *)s_txbuf;
    IpHdr_t   *ri  = (IpHdr_t   *)(s_txbuf + sizeof(EthHdr_t));
    IcmpHdr_t *ric = (IcmpHdr_t *)((uint8_t*)ri + sizeof(IpHdr_t));
    uint8_t   *rd  = (uint8_t*)ric + sizeof(IcmpHdr_t);

    EthHdr_t *oeh = (EthHdr_t *)buf;
    memcpy(re->dst, oeh->src,              6);
    memcpy(re->src, s_status.mac,          6);
    re->type = prv_htons(ETYPE_IP);

    ri->ver_ihl   = 0x45;
    ri->total_len = prv_htons((uint16_t)(sizeof(IpHdr_t) + sizeof(IcmpHdr_t) + plen));
    ri->id        = prv_htons(0x1EC7U);
    ri->ttl       = 64;
    ri->proto     = IPPROTO_ICMP;
    memcpy(ri->src, prv_ip_b(&s_status.ip), 4);
    memcpy(ri->dst, iph->src,                4);
    ri->csum      = prv_ip_csum(ri, sizeof(IpHdr_t));

    ric->type = ICMP_ECHO_REP;
    ric->code = 0;
    ric->id   = ic->id;
    ric->seq  = ic->seq;
    memcpy(rd, payload, plen);
    ric->csum = prv_icmp_csum(ric, rd, plen);

    prv_tx(s_txbuf, total);
}

/* ============================================================
 * 内部: UDP → DHCP振り分け
 * ============================================================ */
static void prv_udp(uint8_t *buf, uint16_t len, IpHdr_t *iph)
{
    uint16_t  ip_hl = (uint16_t)((iph->ver_ihl & 0x0FU) * 4U);
    UdpHdr_t *udp   = (UdpHdr_t *)((uint8_t*)iph + ip_hl);

    if (len < (uint16_t)(sizeof(EthHdr_t) + ip_hl + sizeof(UdpHdr_t))) return;

    if (prv_ntohs(udp->sp) == DHCP_SRV_PORT &&
        prv_ntohs(udp->dp) == DHCP_CLI_PORT)
    {
        DhcpPkt_t *dhcp = (DhcpPkt_t *)((uint8_t*)udp + sizeof(UdpHdr_t));
        uint16_t   dlen = prv_ntohs(udp->len) - (uint16_t)sizeof(UdpHdr_t);
        prv_dhcp_rx(dhcp, dlen);
    }
}

/* ============================================================
 * 内部: DHCP応答処理
 * ============================================================ */
static void prv_dhcp_rx(DhcpPkt_t *dhcp, uint16_t len)
{
    if (len < (uint16_t)offsetof(DhcpPkt_t, options)) return;
    if (dhcp->op != DHCP_OP_REP)                       return;
    if (prv_htonl(dhcp->xid) != DHCP_XID)              return;
    if (memcmp(dhcp->magic, DHCP_MAGIC, 4) != 0)        return;

    uint8_t  msg    = 0;
    uint8_t  srv[4] = {0}, sub[4] = {0}, gw[4] = {0};
    bool     hsrv   = false, hsub = false, hgw = false;

    uint8_t *o = dhcp->options;
    uint8_t *e = dhcp->options + sizeof(dhcp->options);
    while (o < e && *o != 255U) {
        uint8_t code = *o++;
        if (code == 0U) continue;
        if (o >= e) break;
        uint8_t olen = *o++;
        if (o + olen > e) break;
        switch (code) {
        case 53: if (olen >= 1) msg    = o[0];                        break;
        case 54: if (olen >= 4) { memcpy(srv, o, 4); hsrv = true; }  break;
        case  1: if (olen >= 4) { memcpy(sub, o, 4); hsub = true; }  break;
        case  3: if (olen >= 4) { memcpy(gw,  o, 4); hgw  = true; }  break;
        }
        o += olen;
    }

    /* OFFER → REQUEST */
    if (msg == DHCP_MSG_OFFER && s_dhcp_st == DHCP_ST_OFFER_WAIT) {
        memcpy(s_dhcp_offered, dhcp->yiaddr, 4);
        if (hsrv) memcpy(s_dhcp_server, srv, 4);
        s_dhcp_st = DHCP_ST_ACK_WAIT;
        prv_dhcp_request();
        return;
    }

    /* ACK → IPアドレス確定 */
    if (msg == DHCP_MSG_ACK && s_dhcp_st == DHCP_ST_ACK_WAIT) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        memcpy(&s_status.ip.addr,   dhcp->yiaddr, 4);
        if (hsub) memcpy(&s_status.mask.addr, sub, 4);
        if (hgw)  memcpy(&s_status.gw.addr,   gw,  4);
        s_status.is_dhcp = true;
        xSemaphoreGive(s_mutex);
        s_dhcp_st = DHCP_ST_BOUND;
        return;
    }

    if (msg == DHCP_MSG_NAK) prv_dhcp_fallback();
}

/* ============================================================
 * 内部: DHCP Discover 送信
 * ============================================================ */
static void prv_dhcp_discover(void)
{
    uint16_t total = (uint16_t)(sizeof(EthHdr_t) + sizeof(IpHdr_t) +
                                sizeof(UdpHdr_t) + sizeof(DhcpPkt_t));
    if (total > ETHCTL_TX_BUF_SIZE) return;

    memset(s_txbuf, 0, total);
    EthHdr_t  *eh   = (EthHdr_t  *)s_txbuf;
    IpHdr_t   *ih   = (IpHdr_t   *)(s_txbuf + sizeof(EthHdr_t));
    UdpHdr_t  *uh   = (UdpHdr_t  *)((uint8_t*)ih + sizeof(IpHdr_t));
    DhcpPkt_t *dhcp = (DhcpPkt_t *)((uint8_t*)uh + sizeof(UdpHdr_t));

    memcpy(eh->dst, MAC_BCAST,    6);
    memcpy(eh->src, s_status.mac, 6);
    eh->type = prv_htons(ETYPE_IP);

    ih->ver_ihl   = 0x45;
    ih->total_len = prv_htons((uint16_t)(sizeof(IpHdr_t) + sizeof(UdpHdr_t) + sizeof(DhcpPkt_t)));
    ih->id        = prv_htons(0xDC01U);
    ih->ttl       = 64;
    ih->proto     = IPPROTO_UDP;
    memset(ih->dst, 0xFF, 4);
    ih->csum      = prv_ip_csum(ih, sizeof(IpHdr_t));

    uh->sp  = prv_htons(DHCP_CLI_PORT);
    uh->dp  = prv_htons(DHCP_SRV_PORT);
    uh->len = prv_htons((uint16_t)(sizeof(UdpHdr_t) + sizeof(DhcpPkt_t)));

    dhcp->op    = DHCP_OP_REQ;
    dhcp->htype = 1; dhcp->hlen = 6;
    dhcp->xid   = prv_htonl(DHCP_XID);
    dhcp->flags = prv_htons(0x8000U); /* Broadcast flag */
    memcpy(dhcp->chaddr, s_status.mac, 6);
    memcpy(dhcp->magic,  DHCP_MAGIC,   4);

    uint8_t *o = dhcp->options;
    *o++ = 53; *o++ = 1; *o++ = DHCP_MSG_DISCOVER;
    *o++ = 55; *o++ = 3; *o++ = 1; *o++ = 3; *o++ = 6;
    *o++ = 255;

    prv_tx(s_txbuf, total);
}

/* ============================================================
 * 内部: DHCP Request 送信
 * ============================================================ */
static void prv_dhcp_request(void)
{
    uint16_t total = (uint16_t)(sizeof(EthHdr_t) + sizeof(IpHdr_t) +
                                sizeof(UdpHdr_t) + sizeof(DhcpPkt_t));
    if (total > ETHCTL_TX_BUF_SIZE) return;

    memset(s_txbuf, 0, total);
    EthHdr_t  *eh   = (EthHdr_t  *)s_txbuf;
    IpHdr_t   *ih   = (IpHdr_t   *)(s_txbuf + sizeof(EthHdr_t));
    UdpHdr_t  *uh   = (UdpHdr_t  *)((uint8_t*)ih + sizeof(IpHdr_t));
    DhcpPkt_t *dhcp = (DhcpPkt_t *)((uint8_t*)uh + sizeof(UdpHdr_t));

    memcpy(eh->dst, MAC_BCAST,    6);
    memcpy(eh->src, s_status.mac, 6);
    eh->type = prv_htons(ETYPE_IP);

    ih->ver_ihl   = 0x45;
    ih->total_len = prv_htons((uint16_t)(sizeof(IpHdr_t) + sizeof(UdpHdr_t) + sizeof(DhcpPkt_t)));
    ih->id        = prv_htons(0xDC02U);
    ih->ttl       = 64;
    ih->proto     = IPPROTO_UDP;
    memset(ih->dst, 0xFF, 4);
    ih->csum      = prv_ip_csum(ih, sizeof(IpHdr_t));

    uh->sp  = prv_htons(DHCP_CLI_PORT);
    uh->dp  = prv_htons(DHCP_SRV_PORT);
    uh->len = prv_htons((uint16_t)(sizeof(UdpHdr_t) + sizeof(DhcpPkt_t)));

    dhcp->op    = DHCP_OP_REQ;
    dhcp->htype = 1; dhcp->hlen = 6;
    dhcp->xid   = prv_htonl(DHCP_XID);
    dhcp->flags = prv_htons(0x8000U);
    memcpy(dhcp->chaddr, s_status.mac, 6);
    memcpy(dhcp->magic,  DHCP_MAGIC,   4);

    uint8_t *o = dhcp->options;
    *o++ = 53; *o++ = 1; *o++ = DHCP_MSG_REQUEST;
    *o++ = 50; *o++ = 4; memcpy(o, s_dhcp_offered, 4); o += 4;
    *o++ = 54; *o++ = 4; memcpy(o, s_dhcp_server,  4); o += 4;
    *o++ = 55; *o++ = 3; *o++ = 1; *o++ = 3; *o++ = 6;
    *o++ = 255;

    prv_tx(s_txbuf, total);
}

/* ============================================================
 * 内部: DHCP失敗 → フォールバックIP設定
 * ============================================================ */
static void prv_dhcp_fallback(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_status.ip      = s_fb_ip;
    s_status.mask    = s_fb_mask;
    s_status.gw      = s_fb_gw;
    s_status.is_dhcp = false;
    xSemaphoreGive(s_mutex);
    s_dhcp_st = DHCP_ST_FAILED;
}

/* ============================================================
 * 内部: フレーム送信
 * ============================================================ */
static bool prv_tx(uint8_t *buf, uint16_t len)
{
    ETH_BufferTypeDef tb;
    tb.buffer = buf;
    tb.len    = len;
    tb.next   = NULL;

    s_txcfg.Length   = len;
    s_txcfg.TxBuffer = &tb;

    return HAL_ETH_Transmit(&s_heth, &s_txcfg, 100U) == HAL_OK;
}

/* ============================================================
 * 内部: チェックサム計算
 * ============================================================ */
static uint16_t prv_ip_csum(const void *buf, uint16_t len)
{
    const uint16_t *p = (const uint16_t *)buf;
    uint32_t sum = 0;
    while (len > 1U)  { sum += *p++; len -= 2U; }
    if (len != 0U)     sum += *(const uint8_t *)p;
    while (sum >> 16)  sum  = (sum & 0xFFFFU) + (sum >> 16);
    return (uint16_t)(~sum);
}

static uint16_t prv_icmp_csum(IcmpHdr_t *h, const uint8_t *data, uint16_t dlen)
{
    /* ヘッダのcsumsフィールドを0にして計算 */
    IcmpHdr_t tmp = *h; tmp.csum = 0;
    uint32_t sum = 0;
    const uint16_t *p;
    uint16_t n;

    p = (const uint16_t *)&tmp; n = sizeof(IcmpHdr_t);
    while (n > 1U) { sum += *p++; n -= 2U; }

    p = (const uint16_t *)data; n = dlen;
    while (n > 1U) { sum += *p++; n -= 2U; }
    if (n != 0U) sum += *(const uint8_t *)p;

    while (sum >> 16) sum = (sum & 0xFFFFU) + (sum >> 16);
    return (uint16_t)(~sum);
}

/* ============================================================
 * 内部: ユーティリティ
 * ============================================================ */
static uint16_t prv_htons(uint16_t v)
{
    return (uint16_t)((v >> 8U) | (v << 8U));
}
static uint32_t prv_htonl(uint32_t v)
{
    return ((v >> 24U) & 0xFFU)        |
           (((v >> 16U) & 0xFFU) << 8U) |
           (((v >>  8U) & 0xFFU) << 16U)|
           ((v & 0xFFU) << 24U);
}
static uint16_t prv_ntohs(uint16_t v) { return prv_htons(v); }
static bool     prv_ip_eq(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 4) == 0;
}
static uint8_t *prv_ip_b(const EthIp4Addr_t *a)
{
    return (uint8_t *)&a->addr;
}
