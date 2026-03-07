/**
 * @file    main.c  (使用例)
 * @brief   STM32H7 デュアルコア CM7側 エントリポイント
 *
 *  ビルド環境想定:
 *    - STM32CubeIDE / STM32CubeMX 生成コードと組み合わせる
 *    - FreeRTOS (CMSIS-RTOS v2 または vanilla FreeRTOS)
 *    - HAL ドライバ (stm32h7xx_hal_eth.c が必要)
 *
 *  リンカスクリプト追記:
 *    .RxDecripSection (uncached RAM推奨)
 *    .TxDecripSection (uncached RAM推奨)
 */

#include "main.h"
#include "eth_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* ============================================================
 * コールバック実装
 * ============================================================ */

/**
 * @brief LINK UP/DOWN 変化時に呼ばれる
 */
static void on_link_change(EthLinkState_t state)
{
    if (state == ETH_LINK_UP)
    {
        /* LINK UP: LEDを点灯するなどの処理 */
        printf("[ETH] LINK UP\r\n");
        /* HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET); */
    }
    else
    {
        /* LINK DOWN: IPアドレス無効化などの処理 */
        printf("[ETH] LINK DOWN\r\n");
        /* HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET); */
    }
}

/**
 * @brief DHCP/IP取得完了時に呼ばれる
 */
static void on_ip_assigned(const EthNetConfig_t *cfg)
{
    char ip_str[16], gw_str[16], mask_str[16];

    snprintf(ip_str,   sizeof(ip_str),   "%d.%d.%d.%d",
             cfg->ip[0],   cfg->ip[1],   cfg->ip[2],   cfg->ip[3]);
    snprintf(gw_str,   sizeof(gw_str),   "%d.%d.%d.%d",
             cfg->gw[0],   cfg->gw[1],   cfg->gw[2],   cfg->gw[3]);
    snprintf(mask_str, sizeof(mask_str), "%d.%d.%d.%d",
             cfg->mask[0], cfg->mask[1], cfg->mask[2], cfg->mask[3]);

    if (cfg->ip_state == ETH_IP_STATE_DHCP_OK)
    {
        printf("[ETH] DHCP OK  IP=%s GW=%s MASK=%s\r\n",
               ip_str, gw_str, mask_str);
    }
    else
    {
        printf("[ETH] DHCP NG  Fallback IP=%s GW=%s MASK=%s\r\n",
               ip_str, gw_str, mask_str);
    }
}

/* ============================================================
 * ETH 監視タスク (オプション)
 *   リンク状態やIPを定期的にログ出力する例
 * ============================================================ */
static void monitor_task(void *pvParameters)
{
    (void)pvParameters;
    EthNetConfig_t cfg;
    char ip_str[16];

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(5000)); /* 5秒ごとに状態表示 */

        ETH_GetNetConfig(&cfg);
        ETH_GetIpString(ip_str, sizeof(ip_str));

        printf("[ETH] Link=%s  IP=%s  State=%d\r\n",
               (cfg.link == ETH_LINK_UP) ? "UP" : "DOWN",
               ip_str,
               (int)cfg.ip_state);

        /* DHCP再試行例: 一定時間後に再取得したい場合 */
        /* ETH_RequestDhcpRenew(); */
    }
}

/* ============================================================
 * CM7 メイン
 * ============================================================ */
int main(void)
{
    /* HAL / システム初期化 (CubeMX生成コードを呼ぶ) */
    HAL_Init();
    SystemClock_Config();      /* CubeMX生成 */
    /* MX_GPIO_Init(); */       /* CubeMX生成 */
    /* MX_USART3_UART_Init(); */ /* デバッグUART */

    /* ETH ドライバ初期化 */
    if (ETH_Driver_Init() != HAL_OK)
    {
        /* 初期化失敗 → エラー処理 */
        Error_Handler();
    }

    /* コールバック登録 */
    ETH_RegisterLinkCallback(on_link_change);
    ETH_RegisterIpCallback(on_ip_assigned);

    /* ETH タスク生成 (CM7コアで動作) */
    xTaskCreate(ETH_Task,
                "ETH",
                ETH_TASK_STACK_SIZE,
                NULL,
                ETH_TASK_PRIORITY,
                NULL);

    /* 監視タスク生成 (オプション) */
    xTaskCreate(monitor_task,
                "ETH_MON",
                256,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    /* スケジューラ開始 */
    vTaskStartScheduler();

    /* ここには来ない */
    for (;;) {}
}

/* ============================================================
 * DHCP 手動操作の例 (別タスクから呼べる)
 * ============================================================ */

/* DHCP再取得を要求する例 */
void App_RequestDhcpRenew(void)
{
    ETH_RequestDhcpRenew();
}

/* 強制リンクUP/DOWN (テスト・デバッグ用) */
void App_SimulateLinkUp(void)
{
    ETH_ForceLinkUp();
}

void App_SimulateLinkDown(void)
{
    ETH_ForceLinkDown();
}
