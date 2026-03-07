/**
 * @file    eth_driver.h
 * @brief   STM32H7 ETH Driver (No LWIP, No DMA, Polling-based)
 *          対象: STM32H745/H755 デュアルコア
 *          FreeRTOS タスク内ポーリング処理
 */

#ifndef ETH_DRIVER_H
#define ETH_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* ============================================================
 * Configuration
 * ============================================================ */

/** MACアドレス (要変更) */
#define ETH_MAC_ADDR0   0x02
#define ETH_MAC_ADDR1   0x00
#define ETH_MAC_ADDR2   0x00
#define ETH_MAC_ADDR3   0x00
#define ETH_MAC_ADDR4   0x00
#define ETH_MAC_ADDR5   0x01

/** デフォルトIPアドレス (DHCP失敗時のフォールバック) */
#define ETH_DEFAULT_IP0     192
#define ETH_DEFAULT_IP1     168
#define ETH_DEFAULT_IP2     1
#define ETH_DEFAULT_IP3     100

#define ETH_DEFAULT_GW0     192
#define ETH_DEFAULT_GW1     168
#define ETH_DEFAULT_GW2     1
#define ETH_DEFAULT_GW3     1

#define ETH_DEFAULT_MASK0   255
#define ETH_DEFAULT_MASK1   255
#define ETH_DEFAULT_MASK2   255
#define ETH_DEFAULT_MASK3   0

/** DHCP タイムアウト (ms) */
#define DHCP_TIMEOUT_MS     10000

/** リンク監視周期 (ms) */
#define ETH_LINK_CHECK_PERIOD_MS    500

/** Rx/Tx バッファサイズ */
#define ETH_RX_BUF_SIZE     1536
#define ETH_TX_BUF_SIZE     1536
#define ETH_RX_BUF_COUNT    4
#define ETH_TX_BUF_COUNT    4

/** ETHタスクスタックサイズ・優先度 */
#define ETH_TASK_STACK_SIZE     1024
#define ETH_TASK_PRIORITY       (tskIDLE_PRIORITY + 3)

/* ============================================================
 * Ethernet フレーム定数
 * ============================================================ */
#define ETH_ETYPE_ARP       0x0806
#define ETH_ETYPE_IP        0x0800

#define IP_PROTO_ICMP       1
#define IP_PROTO_UDP        17

#define ICMP_TYPE_ECHO_REQ  8
#define ICMP_TYPE_ECHO_REP  0

#define ARP_OP_REQUEST      1
#define ARP_OP_REPLY        2

#define DHCP_SERVER_PORT    67
#define DHCP_CLIENT_PORT    68

#define DHCP_OP_REQUEST     1
#define DHCP_OP_REPLY       2

/* DHCP メッセージタイプ */
#define DHCP_MSG_DISCOVER   1
#define DHCP_MSG_OFFER      2
#define DHCP_MSG_REQUEST    3
#define DHCP_MSG_ACK        5
#define DHCP_MSG_NAK        6

/* ============================================================
 * パケット構造体 (packed)
 * ============================================================ */
#pragma pack(push, 1)

typedef struct {
    uint8_t  dst[6];
    uint8_t  src[6];
    uint16_t type;
} EthHeader_t;

typedef struct {
    uint8_t  ver_ihl;
    uint8_t  tos;
    uint16_t total_len;
    uint16_t id;
    uint16_t frag_off;
    uint8_t  ttl;
    uint8_t  protocol;
    uint16_t checksum;
    uint8_t  src_ip[4];
    uint8_t  dst_ip[4];
} IpHeader_t;

typedef struct {
    uint16_t src_port;
    uint16_t dst_port;
    uint16_t length;
    uint16_t checksum;
} UdpHeader_t;

typedef struct {
    uint8_t  htype;
    uint8_t  ptype;
    uint8_t  hlen;
    uint8_t  plen;
    uint16_t oper;
    uint8_t  sha[6];
    uint8_t  spa[4];
    uint8_t  tha[6];
    uint8_t  tpa[4];
} ArpPacket_t;

typedef struct {
    uint8_t  type;
    uint8_t  code;
    uint16_t checksum;
    uint16_t id;
    uint16_t seq;
} IcmpHeader_t;

typedef struct {
    uint8_t  op;
    uint8_t  htype;
    uint8_t  hlen;
    uint8_t  hops;
    uint32_t xid;
    uint16_t secs;
    uint16_t flags;
    uint8_t  ciaddr[4];
    uint8_t  yiaddr[4];
    uint8_t  siaddr[4];
    uint8_t  giaddr[4];
    uint8_t  chaddr[16];
    uint8_t  sname[64];
    uint8_t  file[128];
    uint8_t  magic[4];
    uint8_t  options[308];
} DhcpPacket_t;

#pragma pack(pop)

/* ============================================================
 * 状態型
 * ============================================================ */
typedef enum {
    ETH_LINK_DOWN = 0,
    ETH_LINK_UP
} EthLinkState_t;

typedef enum {
    ETH_IP_STATE_NONE = 0,
    ETH_IP_STATE_DHCP_TRYING,
    ETH_IP_STATE_DHCP_OK,
    ETH_IP_STATE_STATIC
} EthIpState_t;

typedef struct {
    uint8_t         mac[6];
    uint8_t         ip[4];
    uint8_t         gw[4];
    uint8_t         mask[4];
    EthLinkState_t  link;
    EthIpState_t    ip_state;
} EthNetConfig_t;

/* ============================================================
 * コールバック型
 * ============================================================ */
typedef void (*EthLinkCallback_t)(EthLinkState_t state);
typedef void (*EthIpCallback_t)(const EthNetConfig_t *cfg);

/* ============================================================
 * Public API
 * ============================================================ */

/**
 * @brief  ETHドライバ初期化 (FreeRTOSタスク起動前に呼ぶ)
 */
HAL_StatusTypeDef ETH_Driver_Init(void);

/**
 * @brief  ETH処理タスク (FreeRTOS タスク関数)
 *         xTaskCreate() で登録して使用する
 */
void ETH_Task(void *pvParameters);

/**
 * @brief  リンク状態を取得
 */
EthLinkState_t ETH_GetLinkState(void);

/**
 * @brief  現在のネットワーク設定を取得
 */
void ETH_GetNetConfig(EthNetConfig_t *cfg);

/**
 * @brief  リンク変化コールバック登録
 */
void ETH_RegisterLinkCallback(EthLinkCallback_t cb);

/**
 * @brief  IP取得コールバック登録
 */
void ETH_RegisterIpCallback(EthIpCallback_t cb);

/**
 * @brief  DHCP再試行を要求
 */
void ETH_RequestDhcpRenew(void);

/**
 * @brief  強制LINK UP (テスト用)
 */
void ETH_ForceLinkUp(void);

/**
 * @brief  強制LINK DOWN (テスト用)
 */
void ETH_ForceLinkDown(void);

/**
 * @brief  IPアドレスを文字列で取得 (例: "192.168.1.100")
 */
void ETH_GetIpString(char *buf, size_t len);

#endif /* ETH_DRIVER_H */
