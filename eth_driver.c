/**
 * @file    eth_driver.c
 * @brief   STM32H7 ETH Driver (No LWIP, No DMA, Polling-based)
 *
 *  実装機能:
 *    - LINK UP / LINK DOWN 検出・コールバック
 *    - DHCP Discover → Offer → Request → ACK シーケンス
 *    - DHCP失敗時フォールバック (ローカルIPアドレス)
 *    - ARP 応答 (ARPリクエストに対してReplyを返す)
 *    - ICMP Echo (pingに応答する)
 *    - DMAなし・ポーリングベース
 *
 *  デュアルコア運用:
 *    - CM7コアで動作させることを想定
 *    - ETH_Task() を CM7側 FreeRTOSタスクとして登録
 */

#include "eth_driver.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 * 内部定数
 * ============================================================ */
static const uint8_t MAC_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t MAC_ZERO[6]      = {0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t IP_BROADCAST[4]  = {0xFF,0xFF,0xFF,0xFF};
static const uint8_t IP_ZERO[4]       = {0x00,0x00,0x00,0x00};
static const uint8_t DHCP_MAGIC[4]    = {0x63,0x82,0x53,0x63};

/* DHCP内部状態 */
typedef enum {
    DHCP_STATE_IDLE = 0,
    DHCP_STATE_DISCOVER,
    DHCP_STATE_OFFER_WAIT,
    DHCP_STATE_REQUEST,
    DHCP_STATE_ACK_WAIT,
    DHCP_STATE_BOUND,
    DHCP_STATE_FAILED
} DhcpState_t;

/* ============================================================
 * 内部変数
 * ============================================================ */
static ETH_HandleTypeDef        s_heth;
static ETH_TxPacketConfig       s_txconfig;

/* Rx/Tx バッファ (4バイトアライン) */
static uint8_t s_rx_buf[ETH_RX_BUF_COUNT][ETH_RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t s_tx_buf[ETH_TX_BUF_SIZE]                    __attribute__((aligned(4)));

/* ETH DMA ディスクリプタ */
static ETH_DMADescTypeDef  s_rx_desc[ETH_RX_BUF_COUNT] __attribute__((section(".RxDecripSection")));
static ETH_DMADescTypeDef  s_tx_desc[ETH_TX_BUF_COUNT] __attribute__((section(".TxDecripSection")));

/* ネット設定 */
static EthNetConfig_t s_netcfg;

/* DHCP状態 */
static DhcpState_t  s_dhcp_state   = DHCP_STATE_IDLE;
static uint32_t     s_dhcp_xid     = 0xA1B2C3D4; /* 固定XID (要ランダム化) */
static TickType_t   s_dhcp_timeout = 0;
static uint8_t      s_dhcp_server_ip[4];
static uint8_t      s_dhcp_offered_ip[4];
static bool         s_dhcp_renew_req = false;

/* PHYアドレス (LAN8742A等に合わせる) */
#define PHY_ADDRESS     0x00

/* コールバック */
static EthLinkCallback_t s_link_cb = NULL;
static EthIpCallback_t   s_ip_cb   = NULL;

/* ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;

/* ============================================================
 * 内部プロトタイプ
 * ============================================================ */
static HAL_StatusTypeDef prv_eth_hw_init(void);
static void              prv_eth_gpio_init(void);
static void              prv_eth_clk_init(void);
static bool              prv_phy_read(uint32_t reg, uint32_t *val);
static bool              prv_phy_write(uint32_t reg, uint32_t val);
static bool              prv_phy_check_link(void);
static void              prv_handle_link_up(void);
static void              prv_handle_link_down(void);
static void              prv_poll_rx(void);
static void              prv_process_frame(uint8_t *buf, uint16_t len);
static void              prv_process_arp(uint8_t *buf, uint16_t len);
static void              prv_process_ip(uint8_t *buf, uint16_t len);
static void              prv_process_icmp(uint8_t *buf, uint16_t len,
                                          IpHeader_t *iph);
static void              prv_process_udp(uint8_t *buf, uint16_t len,
                                         IpHeader_t *iph);
static void              prv_process_dhcp_reply(DhcpPacket_t *dhcp, uint16_t len);
static void              prv_dhcp_send_discover(void);
static void              prv_dhcp_send_request(void);
static void              prv_dhcp_set_fallback(void);
static bool              prv_send_frame(uint8_t *buf, uint16_t len);
static uint16_t          prv_ip_checksum(const void *buf, uint16_t len);
static uint16_t          prv_icmp_checksum(IcmpHeader_t *icmp,
                                           uint8_t *payload, uint16_t plen);
static bool              prv_ip_match(const uint8_t *a, const uint8_t *b);
static void              prv_ip_copy(uint8_t *dst, const uint8_t *src);
static void              prv_mac_copy(uint8_t *dst, const uint8_t *src);
static uint16_t          prv_htons(uint16_t v);
static uint32_t          prv_htonl(uint32_t v);
static uint16_t          prv_ntohs(uint16_t v);

/* ============================================================
 * Public: 初期化
 * ============================================================ */
HAL_StatusTypeDef ETH_Driver_Init(void)
{
    HAL_StatusTypeDef ret;

    /* ネット設定初期化 */
    memset(&s_netcfg, 0, sizeof(s_netcfg));
    s_netcfg.mac[0] = ETH_MAC_ADDR0;
    s_netcfg.mac[1] = ETH_MAC_ADDR1;
    s_netcfg.mac[2] = ETH_MAC_ADDR2;
    s_netcfg.mac[3] = ETH_MAC_ADDR3;
    s_netcfg.mac[4] = ETH_MAC_ADDR4;
    s_netcfg.mac[5] = ETH_MAC_ADDR5;
    s_netcfg.link    = ETH_LINK_DOWN;
    s_netcfg.ip_state = ETH_IP_STATE_NONE;

    /* ミューテックス作成 */
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return HAL_ERROR;

    /* GPIO / クロック初期化 */
    prv_eth_gpio_init();
    prv_eth_clk_init();

    /* ETH ペリフェラル初期化 */
    ret = prv_eth_hw_init();
    if (ret != HAL_OK) return ret;

    return HAL_OK;
}

/* ============================================================
 * Public: FreeRTOS タスク本体
 * ============================================================ */
void ETH_Task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t   link_check_tick = xTaskGetTickCount();
    bool         last_link       = false;

    /* PHY リセット待ち */
    vTaskDelay(pdMS_TO_TICKS(200));

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        /* ─────────────────────────────────────
         * (1) リンク状態ポーリング
         * ───────────────────────────────────── */
        if ((now - link_check_tick) >= pdMS_TO_TICKS(ETH_LINK_CHECK_PERIOD_MS))
        {
            link_check_tick = now;
            bool cur_link = prv_phy_check_link();

            if (cur_link && !last_link) {
                prv_handle_link_up();
            } else if (!cur_link && last_link) {
                prv_handle_link_down();
            }
            last_link = cur_link;
        }

        /* ─────────────────────────────────────
         * (2) DHCP 再試行要求
         * ───────────────────────────────────── */
        if (s_dhcp_renew_req && s_netcfg.link == ETH_LINK_UP)
        {
            s_dhcp_renew_req = false;
            s_dhcp_state     = DHCP_STATE_DISCOVER;
            s_dhcp_timeout   = xTaskGetTickCount();
            prv_dhcp_send_discover();
        }

        /* ─────────────────────────────────────
         * (3) DHCP ステートマシン
         * ───────────────────────────────────── */
        if (s_netcfg.link == ETH_LINK_UP)
        {
            now = xTaskGetTickCount();

            switch (s_dhcp_state)
            {
            case DHCP_STATE_OFFER_WAIT:
            case DHCP_STATE_ACK_WAIT:
                /* タイムアウト確認 */
                if ((now - s_dhcp_timeout) >= pdMS_TO_TICKS(DHCP_TIMEOUT_MS))
                {
                    /* DHCP失敗 → フォールバック */
                    prv_dhcp_set_fallback();
                }
                break;

            default:
                break;
            }
        }

        /* ─────────────────────────────────────
         * (4) 受信ポーリング
         * ───────────────────────────────────── */
        if (s_netcfg.link == ETH_LINK_UP)
        {
            prv_poll_rx();
        }

        /* CPUを明け渡す (最小スリープ) */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ============================================================
 * Public: API
 * ============================================================ */
EthLinkState_t ETH_GetLinkState(void)
{
    return s_netcfg.link;
}

void ETH_GetNetConfig(EthNetConfig_t *cfg)
{
    if (!cfg) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(cfg, &s_netcfg, sizeof(EthNetConfig_t));
    xSemaphoreGive(s_mutex);
}

void ETH_RegisterLinkCallback(EthLinkCallback_t cb)
{
    s_link_cb = cb;
}

void ETH_RegisterIpCallback(EthIpCallback_t cb)
{
    s_ip_cb = cb;
}

void ETH_RequestDhcpRenew(void)
{
    s_dhcp_renew_req = true;
}

void ETH_ForceLinkUp(void)
{
    if (s_netcfg.link == ETH_LINK_DOWN) {
        prv_handle_link_up();
    }
}

void ETH_ForceLinkDown(void)
{
    if (s_netcfg.link == ETH_LINK_UP) {
        prv_handle_link_down();
    }
}

void ETH_GetIpString(char *buf, size_t len)
{
    snprintf(buf, len, "%d.%d.%d.%d",
             s_netcfg.ip[0], s_netcfg.ip[1],
             s_netcfg.ip[2], s_netcfg.ip[3]);
}

/* ============================================================
 * 内部: HWイニシャライズ
 * ============================================================ */
static HAL_StatusTypeDef prv_eth_hw_init(void)
{
    HAL_StatusTypeDef ret;

    s_heth.Instance             = ETH;
    s_heth.Init.MACAddr         = s_netcfg.mac;
    s_heth.Init.MediaInterface  = HAL_ETH_RMII_MODE;   /* RMII / MII は要変更 */
    s_heth.Init.TxDesc          = s_tx_desc;
    s_heth.Init.RxDesc          = s_rx_desc;
    s_heth.Init.RxBuffLen       = ETH_RX_BUF_SIZE;

    ret = HAL_ETH_Init(&s_heth);
    if (ret != HAL_OK) return ret;

    /* Rxバッファアドレスを設定 */
    for (uint32_t i = 0; i < ETH_RX_BUF_COUNT; i++) {
        HAL_ETH_DescAssignMemory(&s_heth, i, s_rx_buf[i], NULL);
    }

    /* Txパケット設定 */
    memset(&s_txconfig, 0, sizeof(s_txconfig));
    s_txconfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM |
                            ETH_TX_PACKETS_FEATURES_CRCPAD;
    s_txconfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    s_txconfig.CRCPadCtrl   = ETH_CRC_PAD_INSERT;

    return HAL_OK;
}

/* ============================================================
 * 内部: GPIO / CLK (ボードに合わせて要変更)
 * ============================================================ */
static void prv_eth_gpio_init(void)
{
    /* ─────────────────────────────────────────────────────────────
     * RMII ピン例 (STM32H745 Nucleo-H745ZI-Q)
     *   PA1  → ETH_REF_CLK (AF11)
     *   PA2  → ETH_MDIO    (AF11)
     *   PA7  → ETH_CRS_DV  (AF11)
     *   PB13 → ETH_TXD1    (AF11)
     *   PC1  → ETH_MDC     (AF11)
     *   PC4  → ETH_RXD0    (AF11)
     *   PC5  → ETH_RXD1    (AF11)
     *   PG11 → ETH_TX_EN   (AF11)
     *   PG13 → ETH_TXD0    (AF11)
     * ─────────────────────────────────────────────────────────────
     * ※ HAL_ETH_MspInit() のオーバーライドで実装しても可
     */
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF11_ETH;

    /* PA1, PA2, PA7 */
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PB13 */
    gpio.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* PC1, PC4, PC5 */
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* PG11, PG13 */
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOG, &gpio);
}

static void prv_eth_clk_init(void)
{
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();
}

/* ============================================================
 * 内部: PHY 読み書き (SMI/MDIO)
 * ============================================================ */
static bool prv_phy_read(uint32_t reg, uint32_t *val)
{
    return (HAL_ETH_ReadPHYRegister(&s_heth, PHY_ADDRESS, reg, val) == HAL_OK);
}

static bool prv_phy_write(uint32_t reg, uint32_t val)
{
    return (HAL_ETH_WritePHYRegister(&s_heth, PHY_ADDRESS, reg, val) == HAL_OK);
}

/* ============================================================
 * 内部: PHY リンク状態確認
 *   PHY Basic Status Register (Reg1) Bit1 = Link Status
 * ============================================================ */
static bool prv_phy_check_link(void)
{
    uint32_t bsr = 0;
    /* BSR は2回読みで確定 (ラッチクリア) */
    prv_phy_read(0x01, &bsr);
    prv_phy_read(0x01, &bsr);
    return ((bsr & 0x0004) != 0); /* Bit2: Link Status */
}

/* ============================================================
 * 内部: LINK UP 処理
 * ============================================================ */
static void prv_handle_link_up(void)
{
    /* PHY Auto-Negotiation 完了待ち */
    uint32_t bsr = 0;
    for (int i = 0; i < 100; i++) {
        prv_phy_read(0x01, &bsr);
        if (bsr & 0x0020) break; /* Bit5: Auto-Neg Complete */
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* ETH Start */
    HAL_ETH_Start(&s_heth);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_netcfg.link = ETH_LINK_UP;
    xSemaphoreGive(s_mutex);

    if (s_link_cb) s_link_cb(ETH_LINK_UP);

    /* DHCP開始 */
    s_dhcp_state   = DHCP_STATE_DISCOVER;
    s_dhcp_timeout = xTaskGetTickCount();
    prv_dhcp_send_discover();
}

/* ============================================================
 * 内部: LINK DOWN 処理
 * ============================================================ */
static void prv_handle_link_down(void)
{
    HAL_ETH_Stop(&s_heth);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_netcfg.link     = ETH_LINK_DOWN;
    s_netcfg.ip_state = ETH_IP_STATE_NONE;
    memset(s_netcfg.ip,   0, 4);
    memset(s_netcfg.gw,   0, 4);
    memset(s_netcfg.mask, 0, 4);
    xSemaphoreGive(s_mutex);

    s_dhcp_state = DHCP_STATE_IDLE;

    if (s_link_cb) s_link_cb(ETH_LINK_DOWN);
}

/* ============================================================
 * 内部: 受信ポーリング
 * ============================================================ */
static void prv_poll_rx(void)
{
    ETH_BufferTypeDef rx_buf;
    uint32_t          rx_len = 0;
    HAL_StatusTypeDef st;

    /* 受信フレームがある間ループ */
    for (int i = 0; i < ETH_RX_BUF_COUNT; i++)
    {
        rx_buf.buffer = s_rx_buf[i];
        rx_buf.len    = ETH_RX_BUF_SIZE;
        rx_buf.next   = NULL;
        rx_len        = 0;

        st = HAL_ETH_GetRxDataBuffer(&s_heth, &rx_buf);
        if (st != HAL_OK) break;

        HAL_ETH_GetRxDataLength(&s_heth, &rx_len);
        HAL_ETH_BuildRxDescriptors(&s_heth);

        if (rx_len > sizeof(EthHeader_t)) {
            prv_process_frame(rx_buf.buffer, (uint16_t)rx_len);
        }
    }
}

/* ============================================================
 * 内部: フレーム処理ディスパッチ
 * ============================================================ */
static void prv_process_frame(uint8_t *buf, uint16_t len)
{
    if (len < sizeof(EthHeader_t)) return;

    EthHeader_t *eth  = (EthHeader_t *)buf;
    uint16_t     type = prv_ntohs(eth->type);

    /* 宛先MACが自分か broadcast かチェック */
    if (memcmp(eth->dst, s_netcfg.mac, 6) != 0 &&
        memcmp(eth->dst, MAC_BROADCAST, 6) != 0)
    {
        return;
    }

    switch (type)
    {
    case ETH_ETYPE_ARP:
        prv_process_arp(buf, len);
        break;
    case ETH_ETYPE_IP:
        prv_process_ip(buf, len);
        break;
    default:
        break;
    }
}

/* ============================================================
 * 内部: ARP 処理 (ARPリクエストにReplyを返す)
 * ============================================================ */
static void prv_process_arp(uint8_t *buf, uint16_t len)
{
    if (len < sizeof(EthHeader_t) + sizeof(ArpPacket_t)) return;

    EthHeader_t *eth = (EthHeader_t *)buf;
    ArpPacket_t *arp = (ArpPacket_t *)(buf + sizeof(EthHeader_t));

    /* ARPリクエストで、対象IPが自分のIPの場合のみ応答 */
    if (prv_ntohs(arp->oper) != ARP_OP_REQUEST) return;
    if (!prv_ip_match(arp->tpa, s_netcfg.ip))   return;
    if (prv_ip_match(s_netcfg.ip, IP_ZERO))     return;

    /* 返信バッファ作成 */
    uint8_t  reply[sizeof(EthHeader_t) + sizeof(ArpPacket_t)];
    EthHeader_t *r_eth = (EthHeader_t *)reply;
    ArpPacket_t *r_arp = (ArpPacket_t *)(reply + sizeof(EthHeader_t));

    /* Ethernetヘッダ */
    prv_mac_copy(r_eth->dst, eth->src);
    prv_mac_copy(r_eth->src, s_netcfg.mac);
    r_eth->type = prv_htons(ETH_ETYPE_ARP);

    /* ARPペイロード */
    r_arp->htype = prv_htons(1);      /* Ethernet */
    r_arp->ptype = prv_htons(0x0800); /* IPv4 */
    r_arp->hlen  = 6;
    r_arp->plen  = 4;
    r_arp->oper  = prv_htons(ARP_OP_REPLY);
    prv_mac_copy(r_arp->sha, s_netcfg.mac);
    prv_ip_copy(r_arp->spa, s_netcfg.ip);
    prv_mac_copy(r_arp->tha, arp->sha);
    prv_ip_copy(r_arp->tpa, arp->spa);

    prv_send_frame(reply, (uint16_t)sizeof(reply));
}

/* ============================================================
 * 内部: IP処理
 * ============================================================ */
static void prv_process_ip(uint8_t *buf, uint16_t len)
{
    if (len < sizeof(EthHeader_t) + sizeof(IpHeader_t)) return;

    IpHeader_t *iph = (IpHeader_t *)(buf + sizeof(EthHeader_t));

    /* 宛先IPが自分 or ブロードキャスト か確認 */
    bool for_me = prv_ip_match(iph->dst_ip, s_netcfg.ip) ||
                  prv_ip_match(iph->dst_ip, IP_BROADCAST);

    /* DHCP試行中はブロードキャストのUDPも受け取る */
    if (!for_me && s_dhcp_state != DHCP_STATE_ACK_WAIT &&
                   s_dhcp_state != DHCP_STATE_OFFER_WAIT)
    {
        return;
    }

    switch (iph->protocol)
    {
    case IP_PROTO_ICMP:
        if (for_me) prv_process_icmp(buf, len, iph);
        break;
    case IP_PROTO_UDP:
        prv_process_udp(buf, len, iph);
        break;
    default:
        break;
    }
}

/* ============================================================
 * 内部: ICMP Echo (Ping応答)
 * ============================================================ */
static void prv_process_icmp(uint8_t *buf, uint16_t len,
                              IpHeader_t *iph)
{
    uint16_t ip_hlen  = (iph->ver_ihl & 0x0F) * 4;
    uint8_t *icmp_ptr = (uint8_t *)iph + ip_hlen;
    uint16_t icmp_len = prv_ntohs(iph->total_len) - ip_hlen;

    if (icmp_len < sizeof(IcmpHeader_t)) return;

    IcmpHeader_t *icmp = (IcmpHeader_t *)icmp_ptr;
    if (icmp->type != ICMP_TYPE_ECHO_REQ) return;

    /* ペイロードデータ */
    uint8_t  *payload   = icmp_ptr + sizeof(IcmpHeader_t);
    uint16_t  plen      = icmp_len - sizeof(IcmpHeader_t);

    /* 返信フレームサイズ計算 */
    uint16_t total_frame = sizeof(EthHeader_t) + sizeof(IpHeader_t) +
                           sizeof(IcmpHeader_t) + plen;
    if (total_frame > ETH_TX_BUF_SIZE) return;

    memset(s_tx_buf, 0, total_frame);

    EthHeader_t  *r_eth  = (EthHeader_t *)s_tx_buf;
    IpHeader_t   *r_ip   = (IpHeader_t  *)(s_tx_buf + sizeof(EthHeader_t));
    IcmpHeader_t *r_icmp = (IcmpHeader_t*)(s_tx_buf + sizeof(EthHeader_t) +
                                            sizeof(IpHeader_t));
    uint8_t      *r_data  = (uint8_t *)r_icmp + sizeof(IcmpHeader_t);

    /* Ethernet ヘッダ */
    EthHeader_t *orig_eth = (EthHeader_t *)buf;
    prv_mac_copy(r_eth->dst, orig_eth->src);
    prv_mac_copy(r_eth->src, s_netcfg.mac);
    r_eth->type = prv_htons(ETH_ETYPE_IP);

    /* IP ヘッダ */
    r_ip->ver_ihl   = 0x45;
    r_ip->tos       = 0;
    r_ip->total_len = prv_htons(sizeof(IpHeader_t) + sizeof(IcmpHeader_t) + plen);
    r_ip->id        = prv_htons(0x1234);
    r_ip->frag_off  = 0;
    r_ip->ttl       = 64;
    r_ip->protocol  = IP_PROTO_ICMP;
    r_ip->checksum  = 0;
    prv_ip_copy(r_ip->src_ip, s_netcfg.ip);
    prv_ip_copy(r_ip->dst_ip, iph->src_ip);
    r_ip->checksum  = prv_ip_checksum(r_ip, sizeof(IpHeader_t));

    /* ICMP Echo Reply */
    r_icmp->type     = ICMP_TYPE_ECHO_REP;
    r_icmp->code     = 0;
    r_icmp->checksum = 0;
    r_icmp->id       = icmp->id;
    r_icmp->seq      = icmp->seq;
    memcpy(r_data, payload, plen);
    r_icmp->checksum = prv_icmp_checksum(r_icmp, r_data, plen);

    prv_send_frame(s_tx_buf, total_frame);
}

/* ============================================================
 * 内部: UDP処理 (DHCP Offerなど)
 * ============================================================ */
static void prv_process_udp(uint8_t *buf, uint16_t len,
                             IpHeader_t *iph)
{
    uint16_t ip_hlen = (iph->ver_ihl & 0x0F) * 4;
    uint8_t *udp_ptr = (uint8_t *)iph + ip_hlen;
    uint16_t remain  = prv_ntohs(iph->total_len) - ip_hlen;

    if (remain < sizeof(UdpHeader_t)) return;

    UdpHeader_t *udp     = (UdpHeader_t *)udp_ptr;
    uint16_t     src_p   = prv_ntohs(udp->src_port);
    uint16_t     dst_p   = prv_ntohs(udp->dst_port);

    /* DHCP応答: src=67(server), dst=68(client) */
    if (src_p == DHCP_SERVER_PORT && dst_p == DHCP_CLIENT_PORT)
    {
        uint8_t     *dhcp_ptr = udp_ptr + sizeof(UdpHeader_t);
        uint16_t     dhcp_len = prv_ntohs(udp->length) - sizeof(UdpHeader_t);
        DhcpPacket_t *dhcp    = (DhcpPacket_t *)dhcp_ptr;
        prv_process_dhcp_reply(dhcp, dhcp_len);
    }
}

/* ============================================================
 * 内部: DHCP応答処理
 * ============================================================ */
static void prv_process_dhcp_reply(DhcpPacket_t *dhcp, uint16_t len)
{
    if (len < offsetof(DhcpPacket_t, options)) return;
    if (dhcp->op != DHCP_OP_REPLY)             return;
    if (prv_htonl(dhcp->xid) != s_dhcp_xid)   return;

    /* MagicCookie 確認 */
    if (memcmp(dhcp->magic, DHCP_MAGIC, 4) != 0) return;

    /* オプション解析 */
    uint8_t  msg_type   = 0;
    uint8_t  server_id[4] = {0};
    uint8_t  subnet[4]    = {0};
    uint8_t  gw[4]        = {0};
    uint8_t  dns[4]       = {0};
    bool     has_server   = false;
    bool     has_subnet   = false;
    bool     has_gw       = false;

    uint8_t *opt = dhcp->options;
    uint8_t *end = dhcp->options + sizeof(dhcp->options);

    while (opt < end && *opt != 255)
    {
        uint8_t code = *opt++;
        if (code == 0) continue; /* PAD */

        if (opt >= end) break;
        uint8_t olen = *opt++;

        if (opt + olen > end) break;

        switch (code)
        {
        case 53: /* DHCP Message Type */
            if (olen >= 1) msg_type = opt[0];
            break;
        case 54: /* Server Identifier */
            if (olen >= 4) { memcpy(server_id, opt, 4); has_server = true; }
            break;
        case 1:  /* Subnet Mask */
            if (olen >= 4) { memcpy(subnet, opt, 4); has_subnet = true; }
            break;
        case 3:  /* Router (GW) */
            if (olen >= 4) { memcpy(gw, opt, 4); has_gw = true; }
            break;
        case 6:  /* DNS */
            if (olen >= 4) memcpy(dns, opt, 4);
            break;
        default: break;
        }
        opt += olen;
    }

    /* ─── DHCP OFFER 処理 ─── */
    if (msg_type == DHCP_MSG_OFFER &&
        s_dhcp_state == DHCP_STATE_OFFER_WAIT)
    {
        prv_ip_copy(s_dhcp_offered_ip, dhcp->yiaddr);
        if (has_server) prv_ip_copy(s_dhcp_server_ip, server_id);

        /* DHCPリクエスト送信 */
        s_dhcp_state   = DHCP_STATE_REQUEST;
        s_dhcp_timeout = xTaskGetTickCount();
        prv_dhcp_send_request();
        s_dhcp_state   = DHCP_STATE_ACK_WAIT;
        return;
    }

    /* ─── DHCP ACK 処理 ─── */
    if (msg_type == DHCP_MSG_ACK &&
        s_dhcp_state == DHCP_STATE_ACK_WAIT)
    {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        prv_ip_copy(s_netcfg.ip,   dhcp->yiaddr);
        if (has_subnet) prv_ip_copy(s_netcfg.mask, subnet);
        if (has_gw)     prv_ip_copy(s_netcfg.gw,   gw);
        s_netcfg.ip_state = ETH_IP_STATE_DHCP_OK;
        xSemaphoreGive(s_mutex);

        s_dhcp_state = DHCP_STATE_BOUND;

        if (s_ip_cb) s_ip_cb(&s_netcfg);
        return;
    }

    /* DHCP NAK */
    if (msg_type == DHCP_MSG_NAK) {
        prv_dhcp_set_fallback();
    }
}

/* ============================================================
 * 内部: DHCP Discover 送信
 * ============================================================ */
static void prv_dhcp_send_discover(void)
{
    /* フレーム全体サイズ */
    uint16_t total = sizeof(EthHeader_t) + sizeof(IpHeader_t) +
                     sizeof(UdpHeader_t) + sizeof(DhcpPacket_t);
    if (total > ETH_TX_BUF_SIZE) return;

    memset(s_tx_buf, 0, total);

    EthHeader_t  *eth  = (EthHeader_t  *)s_tx_buf;
    IpHeader_t   *ip   = (IpHeader_t   *)(s_tx_buf + sizeof(EthHeader_t));
    UdpHeader_t  *udp  = (UdpHeader_t  *)((uint8_t *)ip + sizeof(IpHeader_t));
    DhcpPacket_t *dhcp = (DhcpPacket_t *)((uint8_t *)udp + sizeof(UdpHeader_t));

    /* Ethernet */
    prv_mac_copy(eth->dst, MAC_BROADCAST);
    prv_mac_copy(eth->src, s_netcfg.mac);
    eth->type = prv_htons(ETH_ETYPE_IP);

    /* IP */
    ip->ver_ihl   = 0x45;
    ip->tos       = 0;
    ip->total_len = prv_htons(sizeof(IpHeader_t) + sizeof(UdpHeader_t) +
                               sizeof(DhcpPacket_t));
    ip->id        = prv_htons(0xABCD);
    ip->frag_off  = 0;
    ip->ttl       = 64;
    ip->protocol  = IP_PROTO_UDP;
    memset(ip->src_ip, 0, 4);
    memset(ip->dst_ip, 0xFF, 4);
    ip->checksum  = prv_ip_checksum(ip, sizeof(IpHeader_t));

    /* UDP */
    udp->src_port = prv_htons(DHCP_CLIENT_PORT);
    udp->dst_port = prv_htons(DHCP_SERVER_PORT);
    udp->length   = prv_htons(sizeof(UdpHeader_t) + sizeof(DhcpPacket_t));
    udp->checksum = 0; /* チェックサム省略 (RFC2131許可) */

    /* DHCP */
    dhcp->op    = DHCP_OP_REQUEST;
    dhcp->htype = 1;  /* Ethernet */
    dhcp->hlen  = 6;
    dhcp->hops  = 0;
    dhcp->xid   = prv_htonl(s_dhcp_xid);
    dhcp->secs  = 0;
    dhcp->flags = prv_htons(0x8000); /* Broadcast flag */
    memcpy(dhcp->chaddr, s_netcfg.mac, 6);
    memcpy(dhcp->magic, DHCP_MAGIC, 4);

    /* DHCP Options: Message Type=Discover, End */
    uint8_t *opt = dhcp->options;
    *opt++ = 53; *opt++ = 1; *opt++ = DHCP_MSG_DISCOVER;  /* Msg Type */
    *opt++ = 55; *opt++ = 3;                               /* Parameter Request */
    *opt++ = 1;  /* Subnet */
    *opt++ = 3;  /* Router */
    *opt++ = 6;  /* DNS */
    *opt++ = 255; /* End */

    prv_send_frame(s_tx_buf, total);
    s_dhcp_state = DHCP_STATE_OFFER_WAIT;
}

/* ============================================================
 * 内部: DHCP Request 送信
 * ============================================================ */
static void prv_dhcp_send_request(void)
{
    uint16_t total = sizeof(EthHeader_t) + sizeof(IpHeader_t) +
                     sizeof(UdpHeader_t) + sizeof(DhcpPacket_t);
    if (total > ETH_TX_BUF_SIZE) return;

    memset(s_tx_buf, 0, total);

    EthHeader_t  *eth  = (EthHeader_t  *)s_tx_buf;
    IpHeader_t   *ip   = (IpHeader_t   *)(s_tx_buf + sizeof(EthHeader_t));
    UdpHeader_t  *udp  = (UdpHeader_t  *)((uint8_t *)ip + sizeof(IpHeader_t));
    DhcpPacket_t *dhcp = (DhcpPacket_t *)((uint8_t *)udp + sizeof(UdpHeader_t));

    /* Ethernet */
    prv_mac_copy(eth->dst, MAC_BROADCAST);
    prv_mac_copy(eth->src, s_netcfg.mac);
    eth->type = prv_htons(ETH_ETYPE_IP);

    /* IP */
    ip->ver_ihl   = 0x45;
    ip->tos       = 0;
    ip->total_len = prv_htons(sizeof(IpHeader_t) + sizeof(UdpHeader_t) +
                               sizeof(DhcpPacket_t));
    ip->id        = prv_htons(0xABCE);
    ip->frag_off  = 0;
    ip->ttl       = 64;
    ip->protocol  = IP_PROTO_UDP;
    memset(ip->src_ip, 0, 4);
    memset(ip->dst_ip, 0xFF, 4);
    ip->checksum  = prv_ip_checksum(ip, sizeof(IpHeader_t));

    /* UDP */
    udp->src_port = prv_htons(DHCP_CLIENT_PORT);
    udp->dst_port = prv_htons(DHCP_SERVER_PORT);
    udp->length   = prv_htons(sizeof(UdpHeader_t) + sizeof(DhcpPacket_t));
    udp->checksum = 0;

    /* DHCP */
    dhcp->op    = DHCP_OP_REQUEST;
    dhcp->htype = 1;
    dhcp->hlen  = 6;
    dhcp->hops  = 0;
    dhcp->xid   = prv_htonl(s_dhcp_xid);
    dhcp->secs  = 0;
    dhcp->flags = prv_htons(0x8000);
    memcpy(dhcp->chaddr, s_netcfg.mac, 6);
    memcpy(dhcp->magic, DHCP_MAGIC, 4);

    /* Options */
    uint8_t *opt = dhcp->options;
    *opt++ = 53; *opt++ = 1; *opt++ = DHCP_MSG_REQUEST;    /* Msg Type */
    *opt++ = 50; *opt++ = 4;                               /* Requested IP */
    memcpy(opt, s_dhcp_offered_ip, 4); opt += 4;
    *opt++ = 54; *opt++ = 4;                               /* Server ID */
    memcpy(opt, s_dhcp_server_ip, 4); opt += 4;
    *opt++ = 55; *opt++ = 3;                               /* Param Request */
    *opt++ = 1; *opt++ = 3; *opt++ = 6;
    *opt++ = 255; /* End */

    prv_send_frame(s_tx_buf, total);
}

/* ============================================================
 * 内部: DHCP失敗 → フォールバックIP設定
 * ============================================================ */
static void prv_dhcp_set_fallback(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_netcfg.ip[0]   = ETH_DEFAULT_IP0;
    s_netcfg.ip[1]   = ETH_DEFAULT_IP1;
    s_netcfg.ip[2]   = ETH_DEFAULT_IP2;
    s_netcfg.ip[3]   = ETH_DEFAULT_IP3;
    s_netcfg.gw[0]   = ETH_DEFAULT_GW0;
    s_netcfg.gw[1]   = ETH_DEFAULT_GW1;
    s_netcfg.gw[2]   = ETH_DEFAULT_GW2;
    s_netcfg.gw[3]   = ETH_DEFAULT_GW3;
    s_netcfg.mask[0] = ETH_DEFAULT_MASK0;
    s_netcfg.mask[1] = ETH_DEFAULT_MASK1;
    s_netcfg.mask[2] = ETH_DEFAULT_MASK2;
    s_netcfg.mask[3] = ETH_DEFAULT_MASK3;
    s_netcfg.ip_state = ETH_IP_STATE_STATIC;
    xSemaphoreGive(s_mutex);

    s_dhcp_state = DHCP_STATE_FAILED;

    if (s_ip_cb) s_ip_cb(&s_netcfg);
}

/* ============================================================
 * 内部: フレーム送信
 * ============================================================ */
static bool prv_send_frame(uint8_t *buf, uint16_t len)
{
    ETH_BufferTypeDef tx_buf;
    tx_buf.buffer = buf;
    tx_buf.len    = len;
    tx_buf.next   = NULL;

    s_txconfig.Length    = len;
    s_txconfig.TxBuffer  = &tx_buf;

    return (HAL_ETH_Transmit(&s_heth, &s_txconfig, 100) == HAL_OK);
}

/* ============================================================
 * 内部: チェックサム計算
 * ============================================================ */
static uint16_t prv_ip_checksum(const void *buf, uint16_t len)
{
    const uint16_t *ptr = (const uint16_t *)buf;
    uint32_t sum = 0;

    while (len > 1) {
        sum += *ptr++;
        len -= 2;
    }
    if (len == 1) {
        sum += *(const uint8_t *)ptr;
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return (uint16_t)(~sum);
}

static uint16_t prv_icmp_checksum(IcmpHeader_t *icmp,
                                   uint8_t *payload, uint16_t plen)
{
    /* ICMP全体をチェックサム計算 */
    uint32_t sum  = 0;
    uint16_t total = sizeof(IcmpHeader_t) + plen;

    /* ICMPヘッダ部 */
    uint8_t  tmp[sizeof(IcmpHeader_t)];
    memcpy(tmp, icmp, sizeof(IcmpHeader_t));
    const uint16_t *ptr = (const uint16_t *)tmp;
    uint16_t hlen = sizeof(IcmpHeader_t);
    while (hlen > 1) { sum += *ptr++; hlen -= 2; }

    /* ペイロード部 */
    ptr  = (const uint16_t *)payload;
    hlen = plen;
    while (hlen > 1) { sum += *ptr++; hlen -= 2; }
    if (hlen == 1) sum += *(const uint8_t *)ptr;

    (void)total;
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)(~sum);
}

/* ============================================================
 * 内部: ユーティリティ
 * ============================================================ */
static bool prv_ip_match(const uint8_t *a, const uint8_t *b)
{
    return (memcmp(a, b, 4) == 0);
}

static void prv_ip_copy(uint8_t *dst, const uint8_t *src)
{
    memcpy(dst, src, 4);
}

static void prv_mac_copy(uint8_t *dst, const uint8_t *src)
{
    memcpy(dst, src, 6);
}

/* エンディアン変換 (ホスト=リトルエンディアン想定) */
static uint16_t prv_htons(uint16_t v)
{
    return (uint16_t)((v >> 8) | (v << 8));
}

static uint32_t prv_htonl(uint32_t v)
{
    return ((v >> 24) & 0xFF)       |
           (((v >> 16) & 0xFF) << 8) |
           (((v >>  8) & 0xFF) << 16)|
           ((v & 0xFF) << 24);
}

static uint16_t prv_ntohs(uint16_t v)
{
    return prv_htons(v);
}
