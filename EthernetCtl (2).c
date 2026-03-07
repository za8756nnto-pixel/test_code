/*
 * EthernetCtl.c
 *
 *  Created on: Mar 7, 2026
 *      Author: user
 *
 *  LWIPなし・DMAなし・FreeRTOSポーリングベース ETHドライバ
 *  対象MCU: STM32H745/H755 (デュアルコア CM7で動作)
 *  対象PHY: LAN8742A (RMII)
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
#define ETHERNETCTL_PHY_ADDR            0x00U   /*!< LAN8742A MDIOアドレス */
#define ETHERNETCTL_LINK_POLL_MS        500U    /*!< リンク監視周期 */
#define ETHERNETCTL_TASK_STACK          1024U
#define ETHERNETCTL_TASK_PRIORITY       (tskIDLE_PRIORITY + 3)

#define ETHERNETCTL_RX_BUF_SIZE         1536U
#define ETHERNETCTL_RX_BUF_COUNT        4U
#define ETHERNETCTL_TX_BUF_SIZE         1536U

/* ============================================================
 * LAN8742A PHY レジスタ定義
 * ============================================================ */
#define LAN8742_REG_BCR                 0x00U   /* Basic Control */
#define LAN8742_REG_BSR                 0x01U   /* Basic Status */
#define LAN8742_REG_PHYID1              0x02U
#define LAN8742_REG_PHYID2              0x03U
#define LAN8742_REG_SCSR                0x1FU   /* Special Control/Status */

#define LAN8742_BCR_SOFT_RESET          (1U << 15)
#define LAN8742_BCR_AUTONEG_EN          (1U << 12)
#define LAN8742_BCR_POWER_DOWN          (1U << 11)  /*!< Blackout用 */
#define LAN8742_BSR_LINK_STATUS         (1U <<  2)
#define LAN8742_BSR_AUTONEG_COMPLETE    (1U <<  5)
#define LAN8742_SCSR_SPEED_MASK         (0x07U << 2)
#define LAN8742_SCSR_100FD              (0x06U << 2)
#define LAN8742_SCSR_100HD              (0x05U << 2)
#define LAN8742_SCSR_10FD               (0x02U << 2)  /* unused but documented */

/* ============================================================
 * Ethernetフレーム定数
 * ============================================================ */
#define ETYPE_ARP                       0x0806U
#define ETYPE_IP                        0x0800U
#define IP_PROTO_ICMP                   1U
#define IP_PROTO_UDP                    17U
#define ICMP_ECHO_REQUEST               8U
#define ICMP_ECHO_REPLY                 0U
#define ARP_OP_REQUEST                  1U
#define ARP_OP_REPLY                    2U
#define DHCP_SERVER_PORT                67U
#define DHCP_CLIENT_PORT                68U
#define DHCP_OP_REQUEST                 1U
#define DHCP_OP_REPLY                   2U
#define DHCP_MSG_DISCOVER               1U
#define DHCP_MSG_OFFER                  2U
#define DHCP_MSG_REQUEST                3U
#define DHCP_MSG_ACK                    5U
#define DHCP_MSG_NAK                    6U

/* ============================================================
 * パケット構造体
 * ============================================================ */
#pragma pack(push, 1)

typedef struct { uint8_t dst[6]; uint8_t src[6]; uint16_t type; } EthHdr_t;

typedef struct {
    uint8_t  ver_ihl; uint8_t  tos;      uint16_t total_len;
    uint16_t id;      uint16_t frag_off; uint8_t  ttl;
    uint8_t  proto;   uint16_t csum;
    uint8_t  src[4];  uint8_t  dst[4];
} IpHdr_t;

typedef struct {
    uint16_t src_port; uint16_t dst_port;
    uint16_t length;   uint16_t checksum;
} UdpHdr_t;

typedef struct {
    uint8_t  htype; uint8_t  ptype; uint8_t hlen; uint8_t  plen;
    uint16_t oper;
    uint8_t  sha[6]; uint8_t spa[4];
    uint8_t  tha[6]; uint8_t tpa[4];
} ArpPkt_t;

typedef struct {
    uint8_t  type; uint8_t  code;
    uint16_t csum; uint16_t id; uint16_t seq;
} IcmpHdr_t;

typedef struct {
    uint8_t  op; uint8_t htype; uint8_t hlen; uint8_t hops;
    uint32_t xid;
    uint16_t secs; uint16_t flags;
    uint8_t  ciaddr[4]; uint8_t yiaddr[4];
    uint8_t  siaddr[4]; uint8_t giaddr[4];
    uint8_t  chaddr[16];
    uint8_t  sname[64]; uint8_t file[128];
    uint8_t  magic[4];
    uint8_t  options[308];
} DhcpPkt_t;

#pragma pack(pop)

/* ============================================================
 * DHCP ステートマシン
 * ============================================================ */
typedef enum {
    DHCP_ST_IDLE = 0,
    DHCP_ST_DISCOVER_SENT,
    DHCP_ST_OFFER_WAIT,
    DHCP_ST_REQUEST_SENT,
    DHCP_ST_ACK_WAIT,
    DHCP_ST_BOUND,
    DHCP_ST_FAILED,
} DhcpState_t;

/* ============================================================
 * 内部変数
 * ============================================================ */
/* HAL */
static ETH_HandleTypeDef        s_heth;
static ETH_TxPacketConfig       s_txcfg;

/* Rx/Txバッファ
 * DCacheを無効にしているため、通常のRAM配置でOK。
 * section属性・リンカスクリプト追記・MPU設定は不要。
 * 4バイトアラインのみ維持する。                        */
static uint8_t s_rx_buf[ETHERNETCTL_RX_BUF_COUNT][ETHERNETCTL_RX_BUF_SIZE]
    __attribute__((aligned(4)));
static uint8_t s_tx_buf[ETHERNETCTL_TX_BUF_SIZE]
    __attribute__((aligned(4)));

/* DMAディスクリプタ
 * DCacheを無効にしているため、通常のRAM配置でOK。
 * (DCacheが有効な場合はNon-Cacheable領域への配置が必須だが不要) */
static ETH_DMADescTypeDef s_rx_desc[ETHERNETCTL_RX_BUF_COUNT]
    __attribute__((aligned(4)));
static ETH_DMADescTypeDef s_tx_desc[ETHERNETCTL_TX_BUF_COUNT]
    __attribute__((aligned(4)));

/* ネットワーク状態 */
static EthernetCtl_Status  s_status;
static SemaphoreHandle_t   s_mutex = NULL;

/* DHCP */
static DhcpState_t   s_dhcp_st        = DHCP_ST_IDLE;
static TickType_t    s_dhcp_deadline  = 0;
static uint32_t      s_dhcp_timeout_ms = 10000;
static uint8_t       s_dhcp_offered[4];
static uint8_t       s_dhcp_server[4];
static EthIp4Addr_t  s_fallback_ip;
static EthIp4Addr_t  s_fallback_mask;
static EthIp4Addr_t  s_fallback_gw;
static bool          s_dhcp_renew     = false;
static const uint32_t s_dhcp_xid      = 0xEC7A1B2C;

/* リクエスト */
static volatile EthernetCtl_Request s_request = ETH_REQ_NONE;

/* pingシーケンス番号 */
static uint16_t s_ping_seq = 0;

/* Blackout状態 */
static bool s_blackout_active = false;

/* ============================================================
 * 定数
 * ============================================================ */
static const uint8_t MAC_BCAST[6]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t IP_BCAST[4]   = {0xFF,0xFF,0xFF,0xFF};
static const uint8_t IP_ZERO[4]    = {0x00,0x00,0x00,0x00};
static const uint8_t DHCP_MAGIC[4] = {0x63,0x82,0x53,0x63};

/* ============================================================
 * 内部プロトタイプ
 * ============================================================ */
static void         prv_task(void *arg);
static bool         prv_hw_init(void);
static void         prv_gpio_init(void);
static bool         prv_phy_rd(uint32_t reg, uint32_t *val);
static bool         prv_phy_wr(uint32_t reg, uint32_t val);
static bool         prv_phy_link(void);
static void         prv_link_up(void);
static void         prv_link_down(void);
static void         prv_poll_rx(void);
static void         prv_frame(uint8_t *buf, uint16_t len);
static void         prv_arp(uint8_t *buf, uint16_t len);
static void         prv_ip(uint8_t *buf, uint16_t len);
static void         prv_icmp_rx(uint8_t *buf, uint16_t len, IpHdr_t *iph);
static void         prv_udp(uint8_t *buf, uint16_t len, IpHdr_t *iph);
static void         prv_dhcp_rx(DhcpPkt_t *dhcp, uint16_t len);
static void         prv_dhcp_discover(void);
static void         prv_dhcp_request(void);
static void         prv_dhcp_fallback(void);
static bool         prv_tx(uint8_t *buf, uint16_t len);
static uint16_t     prv_ip_csum(const void *buf, uint16_t len);
static uint16_t     prv_icmp_csum(IcmpHdr_t *h, const uint8_t *data, uint16_t dlen);
static uint16_t     prv_htons(uint16_t v);
static uint32_t     prv_htonl(uint32_t v);
static uint16_t     prv_ntohs(uint16_t v);
static bool         prv_ip_eq(const uint8_t *a, const uint8_t *b);
static uint8_t*     prv_ip_bytes(const EthIp4Addr_t *a);

/* ============================================================
 * Public: MACアドレス設定
 * ============================================================ */
bool EthernetCtl_SetMac(const uint8_t mac[6])
{
    if (!mac) return false;
    xSemaphoreTake(s_mutex != NULL ? s_mutex : (SemaphoreHandle_t)1,
                   s_mutex != NULL ? portMAX_DELAY : 0);
    memcpy(s_status.mac, mac, 6);
    if (s_mutex) xSemaphoreGive(s_mutex);
    return true;
}

/* ============================================================
 * Public: DHCP開始 (メイン初期化関数)
 * ============================================================ */
bool EthernetCtl_StartDhcpWithFallback(uint32_t timeout_ms,
                                        const EthIp4Addr_t* fallback_ip,
                                        const EthIp4Addr_t* fallback_mask,
                                        const EthIp4Addr_t* fallback_gw)
{
    if (!fallback_ip || !fallback_mask || !fallback_gw) return false;

    /* フォールバック保存 */
    s_dhcp_timeout_ms = timeout_ms;
    s_fallback_ip     = *fallback_ip;
    s_fallback_mask   = *fallback_mask;
    s_fallback_gw     = *fallback_gw;

    /* 状態初期化 */
    memset(&s_status, 0, sizeof(s_status));

    /* ミューテックス */
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) return false;
    }

    /* GPIO / HWイニシャライズ */
    prv_gpio_init();
    if (!prv_hw_init()) return false;

    /* ETHタスク起動 */
    xTaskCreate(prv_task, "EthCtl",
                ETHERNETCTL_TASK_STACK, NULL,
                ETHERNETCTL_TASK_PRIORITY, NULL);

    /* リンクアップ & DHCP完了をブロック待機 */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms + 3000);
    while (xTaskGetTickCount() < deadline)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        EthernetCtl_Status st;
        EthernetCtl_GetStatus(&st);
        if (st.link_up && (s_dhcp_st == DHCP_ST_BOUND ||
                           s_dhcp_st == DHCP_ST_FAILED))
        {
            return st.is_dhcp;
        }
    }
    /* タイムアウト → フォールバック適用済み */
    return false;
}

/* ============================================================
 * Public: ステータス取得
 * ============================================================ */
void EthernetCtl_GetStatus(EthernetCtl_Status* out)
{
    if (!out) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, &s_status, sizeof(EthernetCtl_Status));
    xSemaphoreGive(s_mutex);
}

/* ============================================================
 * Public: IPアドレス文字列取得
 * ============================================================ */
void EthernetCtl_GetIpString(char* buf, size_t len)
{
    if (!buf || len < 16) return;
    uint8_t *p = prv_ip_bytes(&s_status.ip);
    snprintf(buf, len, "%d.%d.%d.%d", p[0], p[1], p[2], p[3]);
}

/* ============================================================
 * Public: Ping送信
 * ============================================================ */
EthErr_t EthernetCtl_PingSend(const EthIp4Addr_t* dest)
{
    if (!dest) return ETH_ERR_ARG;
    if (!s_status.link_up) return ETH_ERR_NOLINK;

    uint8_t *my_ip  = prv_ip_bytes(&s_status.ip);
    uint8_t *dst_ip = prv_ip_bytes(dest);

    /* ゼロIPはまだ設定されていない */
    if (prv_ip_eq(my_ip, IP_ZERO)) return ETH_ERR_IF;

    /* ICMP Echo Request フレーム組み立て */
    static const uint8_t ping_data[] = "EthernetCtl_Ping";
    uint16_t plen  = sizeof(ping_data) - 1;
    uint16_t total = sizeof(EthHdr_t) + sizeof(IpHdr_t) +
                     sizeof(IcmpHdr_t) + plen;
    if (total > ETHERNETCTL_TX_BUF_SIZE) return ETH_ERR_MEM;

    memset(s_tx_buf, 0, total);

    EthHdr_t  *eth  = (EthHdr_t  *)s_tx_buf;
    IpHdr_t   *ip   = (IpHdr_t   *)(s_tx_buf + sizeof(EthHdr_t));
    IcmpHdr_t *icmp = (IcmpHdr_t *)((uint8_t*)ip + sizeof(IpHdr_t));
    uint8_t   *data = (uint8_t*)icmp + sizeof(IcmpHdr_t);

    /* Ethernet: 宛先MACは暫定ブロードキャスト
     * (本来はARPテーブルを引くが、同一サブネット簡易実装) */
    memcpy(eth->dst, MAC_BCAST, 6);
    memcpy(eth->src, s_status.mac, 6);
    eth->type = prv_htons(ETYPE_IP);

    /* IP */
    ip->ver_ihl   = 0x45;
    ip->total_len = prv_htons(sizeof(IpHdr_t) + sizeof(IcmpHdr_t) + plen);
    ip->id        = prv_htons(0x3C00 + s_ping_seq);
    ip->ttl       = 64;
    ip->proto     = IP_PROTO_ICMP;
    memcpy(ip->src, my_ip,  4);
    memcpy(ip->dst, dst_ip, 4);
    ip->csum      = prv_ip_csum(ip, sizeof(IpHdr_t));

    /* ICMP */
    icmp->type = ICMP_ECHO_REQUEST;
    icmp->code = 0;
    icmp->id   = prv_htons(0xECA0);
    icmp->seq  = prv_htons(s_ping_seq++);
    memcpy(data, ping_data, plen);
    icmp->csum = prv_icmp_csum(icmp, data, plen);

    return prv_tx(s_tx_buf, total) ? ETH_ERR_OK : ETH_ERR_IF;
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
 * Public: LAN8742A LED Blackout設定
 *   LAN8742A の BCR Bit11 (Power Down) を使ってPHYを
 *   パワーダウンさせることでLINK LEDを消灯させる
 *   ※ PHYが提供するLED制御レジスタがある場合はそちらを使う
 * ============================================================ */
bool EthernetCtl_Lan8742Blackout_Set(bool enable)
{
    uint32_t bcr = 0;
    if (!prv_phy_rd(LAN8742_REG_BCR, &bcr)) return false;

    if (enable) {
        bcr |=  LAN8742_BCR_POWER_DOWN;  /* PHYパワーダウン → LED消灯 */
    } else {
        bcr &= ~LAN8742_BCR_POWER_DOWN;  /* PHY復帰 */
    }

    if (!prv_phy_wr(LAN8742_REG_BCR, bcr)) return false;

    s_blackout_active = enable;

    if (enable) {
        /* ソフト的にリンクダウン状態にする */
        prv_link_down();
    } else {
        /* PHY起動待ち */
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    return true;
}

/* ============================================================
 * Public: LAN8742A Blackout解除 + DHCP再取得
 * ============================================================ */
bool EthernetCtl_Lan8742Blackout_RestoreWithDhcp(uint32_t timeout_ms,
                                                   const EthIp4Addr_t* fallback_ip,
                                                   const EthIp4Addr_t* fallback_mask,
                                                   const EthIp4Addr_t* fallback_gw)
{
    if (!fallback_ip || !fallback_mask || !fallback_gw) return false;

    /* フォールバック更新 */
    s_dhcp_timeout_ms = timeout_ms;
    s_fallback_ip     = *fallback_ip;
    s_fallback_mask   = *fallback_mask;
    s_fallback_gw     = *fallback_gw;

    /* Blackout解除 */
    if (s_blackout_active) {
        if (!EthernetCtl_Lan8742Blackout_Set(false)) return false;
    }

    /* DHCP再取得をリクエスト */
    s_dhcp_renew = true;

    /* 完了待ち */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms + 3000);
    while (xTaskGetTickCount() < deadline)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (s_dhcp_st == DHCP_ST_BOUND || s_dhcp_st == DHCP_ST_FAILED) {
            return s_status.is_dhcp;
        }
    }
    return false;
}

/* ============================================================
 * 内部: FreeRTOS タスク本体
 * ============================================================ */
static void prv_task(void *arg)
{
    (void)arg;
    TickType_t link_tick = xTaskGetTickCount();
    bool       last_link = false;

    vTaskDelay(pdMS_TO_TICKS(200)); /* PHY安定待ち */

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        /* ── (1) リンク監視 ── */
        if ((now - link_tick) >= pdMS_TO_TICKS(ETHERNETCTL_LINK_POLL_MS))
        {
            link_tick = now;
            if (!s_blackout_active)
            {
                bool cur = prv_phy_link();
                if ( cur && !last_link) prv_link_up();
                if (!cur &&  last_link) prv_link_down();
                last_link = cur;
            }
        }

        /* ── (2) DHCP 再取得要求 ── */
        if (s_dhcp_renew && s_status.link_up)
        {
            s_dhcp_renew   = false;
            s_dhcp_st      = DHCP_ST_DISCOVER_SENT;
            s_dhcp_deadline = xTaskGetTickCount() +
                              pdMS_TO_TICKS(s_dhcp_timeout_ms);
            prv_dhcp_discover();
        }

        /* ── (3) DHCP タイムアウト監視 ── */
        now = xTaskGetTickCount();
        if ((s_dhcp_st == DHCP_ST_OFFER_WAIT ||
             s_dhcp_st == DHCP_ST_ACK_WAIT) &&
            (now >= s_dhcp_deadline))
        {
            prv_dhcp_fallback();
        }

        /* ── (4) 受信ポーリング ── */
        if (s_status.link_up) prv_poll_rx();

        /* ── (5) リクエスト処理 ── */
        EthernetCtl_Request req = EthernetCtl_RequestGetAndClear();
        switch (req)
        {
        case ETH_REQ_LED_BLACKOUT_ON:
            EthernetCtl_Lan8742Blackout_Set(true);
            break;
        case ETH_REQ_LED_BLACKOUT_OFF:
            /* フォールバックはデフォルト値を使用 */
            EthernetCtl_Lan8742Blackout_RestoreWithDhcp(
                s_dhcp_timeout_ms,
                &s_fallback_ip, &s_fallback_mask, &s_fallback_gw);
            break;
        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ============================================================
 * 内部: HWイニシャライズ
 * ============================================================ */
static bool prv_hw_init(void)
{
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();

    s_heth.Instance            = ETH;
    s_heth.Init.MACAddr        = s_status.mac;
    s_heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    s_heth.Init.TxDesc         = s_tx_desc;
    s_heth.Init.RxDesc         = s_rx_desc;
    s_heth.Init.RxBuffLen      = ETHERNETCTL_RX_BUF_SIZE;

    if (HAL_ETH_Init(&s_heth) != HAL_OK) return false;

    for (uint32_t i = 0; i < ETHERNETCTL_RX_BUF_COUNT; i++) {
        HAL_ETH_DescAssignMemory(&s_heth, i, s_rx_buf[i], NULL);
    }

    memset(&s_txcfg, 0, sizeof(s_txcfg));
    s_txcfg.Attributes    = ETH_TX_PACKETS_FEATURES_CSUM |
                            ETH_TX_PACKETS_FEATURES_CRCPAD;
    s_txcfg.ChecksumCtrl  = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    s_txcfg.CRCPadCtrl    = ETH_CRC_PAD_INSERT;

    return true;
}

/* ============================================================
 * 内部: GPIO初期化 (Nucleo-H745ZI-Q RMII配線)
 *   ボードが異なる場合はここを変更する
 * ============================================================ */
static void prv_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF11_ETH;

    g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_13;                           HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5; HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_11 | GPIO_PIN_13;             HAL_GPIO_Init(GPIOG, &g);
}

/* ============================================================
 * 内部: PHY SMI アクセス
 * ============================================================ */
static bool prv_phy_rd(uint32_t reg, uint32_t *val)
{
    return HAL_ETH_ReadPHYRegister(&s_heth, ETHERNETCTL_PHY_ADDR,
                                   reg, val) == HAL_OK;
}

static bool prv_phy_wr(uint32_t reg, uint32_t val)
{
    return HAL_ETH_WritePHYRegister(&s_heth, ETHERNETCTL_PHY_ADDR,
                                    reg, val) == HAL_OK;
}

/* ============================================================
 * 内部: PHY リンク状態確認 (BSR Bit2, 2回読み)
 * ============================================================ */
static bool prv_phy_link(void)
{
    uint32_t bsr = 0;
    prv_phy_rd(LAN8742_REG_BSR, &bsr); /* 1回目: ラッチクリア */
    prv_phy_rd(LAN8742_REG_BSR, &bsr); /* 2回目: 確定値 */
    return (bsr & LAN8742_BSR_LINK_STATUS) != 0;
}

/* ============================================================
 * 内部: LINK UP 処理
 * ============================================================ */
static void prv_link_up(void)
{
    /* Auto-Negotiation 完了待ち (最大1秒) */
    for (int i = 0; i < 100; i++) {
        uint32_t bsr = 0;
        prv_phy_rd(LAN8742_REG_BSR, &bsr);
        if (bsr & LAN8742_BSR_AUTONEG_COMPLETE) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    HAL_ETH_Start(&s_heth);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_status.link_up = true;
    s_status.is_dhcp = false;
    xSemaphoreGive(s_mutex);

    /* DHCP Discover 開始 */
    s_dhcp_st       = DHCP_ST_DISCOVER_SENT;
    s_dhcp_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(s_dhcp_timeout_ms);
    prv_dhcp_discover();
}

/* ============================================================
 * 内部: LINK DOWN 処理
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
 * ============================================================ */
static void prv_poll_rx(void)
{
    for (uint32_t i = 0; i < ETHERNETCTL_RX_BUF_COUNT; i++)
    {
        ETH_BufferTypeDef rb = { .buffer = s_rx_buf[i],
                                 .len    = ETHERNETCTL_RX_BUF_SIZE,
                                 .next   = NULL };
        uint32_t rlen = 0;

        if (HAL_ETH_GetRxDataBuffer(&s_heth, &rb) != HAL_OK) break;
        HAL_ETH_GetRxDataLength(&s_heth, &rlen);
        HAL_ETH_BuildRxDescriptors(&s_heth);

        if (rlen > sizeof(EthHdr_t)) {
            prv_frame(rb.buffer, (uint16_t)rlen);
        }
    }
}

/* ============================================================
 * 内部: フレーム振り分け
 * ============================================================ */
static void prv_frame(uint8_t *buf, uint16_t len)
{
    EthHdr_t *eh   = (EthHdr_t *)buf;
    uint16_t  type = prv_ntohs(eh->type);

    /* 宛先MACが自分かブロードキャストか */
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
    if (len < sizeof(EthHdr_t) + sizeof(ArpPkt_t)) return;

    EthHdr_t *eh  = (EthHdr_t *)buf;
    ArpPkt_t *arp = (ArpPkt_t *)(buf + sizeof(EthHdr_t));

    if (prv_ntohs(arp->oper) != ARP_OP_REQUEST)           return;
    if (!prv_ip_eq(arp->tpa, prv_ip_bytes(&s_status.ip))) return;
    if (prv_ip_eq(prv_ip_bytes(&s_status.ip), IP_ZERO))   return;

    uint8_t  rep[sizeof(EthHdr_t) + sizeof(ArpPkt_t)];
    EthHdr_t *re  = (EthHdr_t *)rep;
    ArpPkt_t *ra  = (ArpPkt_t *)(rep + sizeof(EthHdr_t));

    memcpy(re->dst, eh->src,       6);
    memcpy(re->src, s_status.mac,  6);
    re->type = prv_htons(ETYPE_ARP);

    ra->htype = prv_htons(1);
    ra->ptype = prv_htons(0x0800);
    ra->hlen  = 6; ra->plen = 4;
    ra->oper  = prv_htons(ARP_OP_REPLY);
    memcpy(ra->sha, s_status.mac,              6);
    memcpy(ra->spa, prv_ip_bytes(&s_status.ip),4);
    memcpy(ra->tha, arp->sha,                  6);
    memcpy(ra->tpa, arp->spa,                  4);

    prv_tx(rep, (uint16_t)sizeof(rep));
}

/* ============================================================
 * 内部: IP処理
 * ============================================================ */
static void prv_ip(uint8_t *buf, uint16_t len)
{
    if (len < sizeof(EthHdr_t) + sizeof(IpHdr_t)) return;

    IpHdr_t *iph = (IpHdr_t *)(buf + sizeof(EthHdr_t));

    bool for_me = prv_ip_eq(iph->dst, prv_ip_bytes(&s_status.ip)) ||
                  prv_ip_eq(iph->dst, IP_BCAST);

    /* DHCP応答はブロードキャスト宛で来ることがあるので通す */
    bool dhcp_active = (s_dhcp_st == DHCP_ST_OFFER_WAIT ||
                        s_dhcp_st == DHCP_ST_ACK_WAIT);

    if (iph->proto == IP_PROTO_ICMP && for_me) {
        prv_icmp_rx(buf, len, iph);
    } else if (iph->proto == IP_PROTO_UDP && (for_me || dhcp_active)) {
        prv_udp(buf, len, iph);
    }
}

/* ============================================================
 * 内部: ICMP Echo Reply 返送
 * ============================================================ */
static void prv_icmp_rx(uint8_t *buf, uint16_t len, IpHdr_t *iph)
{
    uint16_t  ip_hl   = (iph->ver_ihl & 0x0F) * 4;
    IcmpHdr_t *icmp   = (IcmpHdr_t *)((uint8_t*)iph + ip_hl);
    uint16_t  icmp_len = prv_ntohs(iph->total_len) - ip_hl;

    if (icmp_len < sizeof(IcmpHdr_t)) return;
    if (icmp->type != ICMP_ECHO_REQUEST) return;

    uint8_t  *payload = (uint8_t*)icmp + sizeof(IcmpHdr_t);
    uint16_t  plen    = icmp_len - sizeof(IcmpHdr_t);

    uint16_t total = sizeof(EthHdr_t) + sizeof(IpHdr_t) +
                     sizeof(IcmpHdr_t) + plen;
    if (total > ETHERNETCTL_TX_BUF_SIZE) return;

    memset(s_tx_buf, 0, total);

    EthHdr_t  *re   = (EthHdr_t  *)s_tx_buf;
    IpHdr_t   *ri   = (IpHdr_t   *)(s_tx_buf + sizeof(EthHdr_t));
    IcmpHdr_t *ric  = (IcmpHdr_t *)((uint8_t*)ri + sizeof(IpHdr_t));
    uint8_t   *rdat = (uint8_t*)ric + sizeof(IcmpHdr_t);

    EthHdr_t *orig_eh = (EthHdr_t *)buf;
    memcpy(re->dst, orig_eh->src,              6);
    memcpy(re->src, s_status.mac,              6);
    re->type = prv_htons(ETYPE_IP);

    ri->ver_ihl   = 0x45;
    ri->total_len = prv_htons((uint16_t)(sizeof(IpHdr_t)+sizeof(IcmpHdr_t)+plen));
    ri->id        = prv_htons(0x1EC7);
    ri->ttl       = 64;
    ri->proto     = IP_PROTO_ICMP;
    memcpy(ri->src, prv_ip_bytes(&s_status.ip), 4);
    memcpy(ri->dst, iph->src,                   4);
    ri->csum      = prv_ip_csum(ri, sizeof(IpHdr_t));

    ric->type = ICMP_ECHO_REPLY;
    ric->code = 0;
    ric->id   = icmp->id;
    ric->seq  = icmp->seq;
    memcpy(rdat, payload, plen);
    ric->csum = prv_icmp_csum(ric, rdat, plen);

    prv_tx(s_tx_buf, total);
}

/* ============================================================
 * 内部: UDP → DHCP振り分け
 * ============================================================ */
static void prv_udp(uint8_t *buf, uint16_t len, IpHdr_t *iph)
{
    uint16_t ip_hl = (iph->ver_ihl & 0x0F) * 4;
    UdpHdr_t *udp  = (UdpHdr_t *)((uint8_t*)iph + ip_hl);

    if (len < sizeof(EthHdr_t) + ip_hl + sizeof(UdpHdr_t)) return;

    if (prv_ntohs(udp->src_port) == DHCP_SERVER_PORT &&
        prv_ntohs(udp->dst_port) == DHCP_CLIENT_PORT)
    {
        DhcpPkt_t *dhcp = (DhcpPkt_t *)((uint8_t*)udp + sizeof(UdpHdr_t));
        uint16_t   dlen = prv_ntohs(udp->length) - (uint16_t)sizeof(UdpHdr_t);
        prv_dhcp_rx(dhcp, dlen);
    }
}

/* ============================================================
 * 内部: DHCP応答処理
 * ============================================================ */
static void prv_dhcp_rx(DhcpPkt_t *dhcp, uint16_t len)
{
    if (len < (uint16_t)offsetof(DhcpPkt_t, options)) return;
    if (dhcp->op != DHCP_OP_REPLY)                     return;
    if (prv_htonl(dhcp->xid) != s_dhcp_xid)            return;
    if (memcmp(dhcp->magic, DHCP_MAGIC, 4) != 0)        return;

    /* オプション解析 */
    uint8_t  msg_type   = 0;
    uint8_t  srv_id[4]  = {0};
    uint8_t  subnet[4]  = {0};
    uint8_t  gw[4]      = {0};
    bool     has_srv    = false;
    bool     has_sub    = false;
    bool     has_gw     = false;

    uint8_t *o = dhcp->options;
    uint8_t *e = dhcp->options + sizeof(dhcp->options);

    while (o < e && *o != 255) {
        uint8_t code = *o++;
        if (code == 0) continue;
        if (o >= e) break;
        uint8_t olen = *o++;
        if (o + olen > e) break;
        switch (code) {
        case 53: if (olen >= 1) msg_type = o[0];                        break;
        case 54: if (olen >= 4) { memcpy(srv_id, o, 4); has_srv = true; } break;
        case  1: if (olen >= 4) { memcpy(subnet, o, 4); has_sub = true; } break;
        case  3: if (olen >= 4) { memcpy(gw,     o, 4); has_gw  = true; } break;
        }
        o += olen;
    }

    /* OFFER → REQUEST送信 */
    if (msg_type == DHCP_MSG_OFFER && s_dhcp_st == DHCP_ST_OFFER_WAIT) {
        memcpy(s_dhcp_offered, dhcp->yiaddr, 4);
        if (has_srv) memcpy(s_dhcp_server, srv_id, 4);
        s_dhcp_st = DHCP_ST_REQUEST_SENT;
        prv_dhcp_request();
        s_dhcp_st = DHCP_ST_ACK_WAIT;
        return;
    }

    /* ACK → IPアドレス確定 */
    if (msg_type == DHCP_MSG_ACK && s_dhcp_st == DHCP_ST_ACK_WAIT) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        memcpy(&s_status.ip.addr,   dhcp->yiaddr,             4);
        if (has_sub) memcpy(&s_status.mask.addr, subnet, 4);
        if (has_gw)  memcpy(&s_status.gw.addr,  gw,     4);
        s_status.is_dhcp = true;
        xSemaphoreGive(s_mutex);
        s_dhcp_st = DHCP_ST_BOUND;
        return;
    }

    /* NAK */
    if (msg_type == DHCP_MSG_NAK) {
        prv_dhcp_fallback();
    }
}

/* ============================================================
 * 内部: DHCP Discover 送信
 * ============================================================ */
static void prv_dhcp_discover(void)
{
    uint16_t total = sizeof(EthHdr_t) + sizeof(IpHdr_t) +
                     sizeof(UdpHdr_t) + sizeof(DhcpPkt_t);
    if (total > ETHERNETCTL_TX_BUF_SIZE) return;

    memset(s_tx_buf, 0, total);

    EthHdr_t  *eh   = (EthHdr_t  *)s_tx_buf;
    IpHdr_t   *ip   = (IpHdr_t   *)(s_tx_buf + sizeof(EthHdr_t));
    UdpHdr_t  *udp  = (UdpHdr_t  *)((uint8_t*)ip + sizeof(IpHdr_t));
    DhcpPkt_t *dhcp = (DhcpPkt_t *)((uint8_t*)udp + sizeof(UdpHdr_t));

    memcpy(eh->dst, MAC_BCAST,   6);
    memcpy(eh->src, s_status.mac,6);
    eh->type = prv_htons(ETYPE_IP);

    ip->ver_ihl   = 0x45;
    ip->total_len = prv_htons(sizeof(IpHdr_t)+sizeof(UdpHdr_t)+sizeof(DhcpPkt_t));
    ip->id        = prv_htons(0xDC01);
    ip->ttl       = 64;
    ip->proto     = IP_PROTO_UDP;
    memset(ip->dst, 0xFF, 4);
    ip->csum      = prv_ip_csum(ip, sizeof(IpHdr_t));

    udp->src_port = prv_htons(DHCP_CLIENT_PORT);
    udp->dst_port = prv_htons(DHCP_SERVER_PORT);
    udp->length   = prv_htons(sizeof(UdpHdr_t) + sizeof(DhcpPkt_t));

    dhcp->op    = DHCP_OP_REQUEST;
    dhcp->htype = 1; dhcp->hlen = 6;
    dhcp->xid   = prv_htonl(s_dhcp_xid);
    dhcp->flags = prv_htons(0x8000);
    memcpy(dhcp->chaddr, s_status.mac, 6);
    memcpy(dhcp->magic,  DHCP_MAGIC,   4);

    uint8_t *o = dhcp->options;
    *o++ = 53; *o++ = 1; *o++ = DHCP_MSG_DISCOVER;
    *o++ = 55; *o++ = 3; *o++ = 1; *o++ = 3; *o++ = 6;
    *o++ = 255;

    prv_tx(s_tx_buf, total);
    s_dhcp_st = DHCP_ST_OFFER_WAIT;
}

/* ============================================================
 * 内部: DHCP Request 送信
 * ============================================================ */
static void prv_dhcp_request(void)
{
    uint16_t total = sizeof(EthHdr_t) + sizeof(IpHdr_t) +
                     sizeof(UdpHdr_t) + sizeof(DhcpPkt_t);
    if (total > ETHERNETCTL_TX_BUF_SIZE) return;

    memset(s_tx_buf, 0, total);

    EthHdr_t  *eh   = (EthHdr_t  *)s_tx_buf;
    IpHdr_t   *ip   = (IpHdr_t   *)(s_tx_buf + sizeof(EthHdr_t));
    UdpHdr_t  *udp  = (UdpHdr_t  *)((uint8_t*)ip + sizeof(IpHdr_t));
    DhcpPkt_t *dhcp = (DhcpPkt_t *)((uint8_t*)udp + sizeof(UdpHdr_t));

    memcpy(eh->dst, MAC_BCAST,    6);
    memcpy(eh->src, s_status.mac, 6);
    eh->type = prv_htons(ETYPE_IP);

    ip->ver_ihl   = 0x45;
    ip->total_len = prv_htons(sizeof(IpHdr_t)+sizeof(UdpHdr_t)+sizeof(DhcpPkt_t));
    ip->id        = prv_htons(0xDC02);
    ip->ttl       = 64;
    ip->proto     = IP_PROTO_UDP;
    memset(ip->dst, 0xFF, 4);
    ip->csum      = prv_ip_csum(ip, sizeof(IpHdr_t));

    udp->src_port = prv_htons(DHCP_CLIENT_PORT);
    udp->dst_port = prv_htons(DHCP_SERVER_PORT);
    udp->length   = prv_htons(sizeof(UdpHdr_t) + sizeof(DhcpPkt_t));

    dhcp->op    = DHCP_OP_REQUEST;
    dhcp->htype = 1; dhcp->hlen = 6;
    dhcp->xid   = prv_htonl(s_dhcp_xid);
    dhcp->flags = prv_htons(0x8000);
    memcpy(dhcp->chaddr, s_status.mac, 6);
    memcpy(dhcp->magic,  DHCP_MAGIC,   4);

    uint8_t *o = dhcp->options;
    *o++ = 53; *o++ = 1; *o++ = DHCP_MSG_REQUEST;
    *o++ = 50; *o++ = 4; memcpy(o, s_dhcp_offered, 4); o += 4;
    *o++ = 54; *o++ = 4; memcpy(o, s_dhcp_server,  4); o += 4;
    *o++ = 55; *o++ = 3; *o++ = 1; *o++ = 3; *o++ = 6;
    *o++ = 255;

    prv_tx(s_tx_buf, total);
}

/* ============================================================
 * 内部: DHCP失敗 → フォールバック適用
 * ============================================================ */
static void prv_dhcp_fallback(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_status.ip      = s_fallback_ip;
    s_status.mask    = s_fallback_mask;
    s_status.gw      = s_fallback_gw;
    s_status.is_dhcp = false;
    xSemaphoreGive(s_mutex);
    s_dhcp_st = DHCP_ST_FAILED;
}

/* ============================================================
 * 内部: フレーム送信
 * ============================================================ */
static bool prv_tx(uint8_t *buf, uint16_t len)
{
    ETH_BufferTypeDef tb = { .buffer = buf, .len = len, .next = NULL };
    s_txcfg.Length   = len;
    s_txcfg.TxBuffer = &tb;
    return HAL_ETH_Transmit(&s_heth, &s_txcfg, 100) == HAL_OK;
}

/* ============================================================
 * 内部: チェックサム計算
 * ============================================================ */
static uint16_t prv_ip_csum(const void *buf, uint16_t len)
{
    const uint16_t *p = (const uint16_t *)buf;
    uint32_t sum = 0;
    while (len > 1)  { sum += *p++; len -= 2; }
    if (len)          sum += *(const uint8_t*)p;
    while (sum >> 16) sum  = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)(~sum);
}

static uint16_t prv_icmp_csum(IcmpHdr_t *h, const uint8_t *data, uint16_t dlen)
{
    uint32_t sum = 0;
    /* ICMPヘッダ (checksumフィールドは0で計算) */
    IcmpHdr_t tmp = *h; tmp.csum = 0;
    const uint16_t *p = (const uint16_t *)&tmp;
    uint16_t n = sizeof(IcmpHdr_t);
    while (n > 1) { sum += *p++; n -= 2; }
    /* ペイロード */
    p = (const uint16_t *)data; n = dlen;
    while (n > 1) { sum += *p++; n -= 2; }
    if (n) sum += *(const uint8_t*)p;
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)(~sum);
}

/* ============================================================
 * 内部: ユーティリティ
 * ============================================================ */
static uint16_t prv_htons(uint16_t v) { return (uint16_t)((v>>8)|(v<<8)); }
static uint32_t prv_htonl(uint32_t v) {
    return ((v>>24)&0xFF)|(((v>>16)&0xFF)<<8)|(((v>>8)&0xFF)<<16)|((v&0xFF)<<24);
}
static uint16_t prv_ntohs(uint16_t v) { return prv_htons(v); }
static bool     prv_ip_eq(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 4) == 0;
}
static uint8_t* prv_ip_bytes(const EthIp4Addr_t *a) {
    return (uint8_t*)&a->addr;
}
