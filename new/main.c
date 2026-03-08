/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EthernetCtl.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * デュアルコア起動同期シーケンス
 * 片コアのみでデバッグする場合はこの define をコメントアウトすること
 */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  #ifndef HSEM_ID_0
    #define HSEM_ID_0 (0U)
  #endif
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;

osThreadId defaultTaskHandle;
uint32_t   defaultTaskBuffer[512];
osStaticThreadDef_t defaultTaskControlBlock;

osThreadId Ethernet_TaskHandle;
uint32_t   Ethernet_TaskBuffer[2048];
osStaticThreadDef_t Ethernet_TaskControlBlock;

/* USER CODE BEGIN PV */

/*
 * フォールバックIPアドレス (DHCP失敗時に使用)
 * スタートアップタスクから EthernetCtl_StartDhcpWithFallback() に渡す
 */
static EthIp4Addr_t s_fb_ip;
static EthIp4Addr_t s_fb_mask;
static EthIp4Addr_t s_fb_gw;

/*
 * MACアドレス
 * ★ 製品ごとにユニークな値を設定すること ★
 * 同一LAN上に同じMACアドレスのデバイスが2台以上あると通信不能になる
 */
static const uint8_t s_mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const *argument);
void StartEthernet_Task(void const *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  アプリケーションエントリポイント
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /*
   * ICache 有効化
   *   ETHデータとは無関係。コード実行速度の向上のみが目的。
   */
  SCB_EnableICache();

  /*
   * DCache は無効のまま使用する
   *   ポーリングベースのETH実装ではDCacheなしで問題ない。
   *   有効にする場合は ETH DMAバッファ領域をMPUで
   *   Non-Cacheable に設定する必要がある。
   */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* CPU2(CM4)がstopモードに入るまで待機 */
  timeout = 0xFFFF;
  while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if (timeout < 0) { Error_Handler(); }
#endif
/* USER CODE END Boot_Mode_Sequence_1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0, 0);
  /* CPU2(CM4)が起動するまで待機 */
  timeout = 0xFFFF;
  while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if (timeout < 0) { Error_Handler(); }
#endif
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /*
   * フォールバックIPアドレス設定
   * ★ 実際の環境に合わせて変更すること ★
   */
  ETH_IP4_ADDR(&s_fb_ip,   192, 168,   1, 100);
  ETH_IP4_ADDR(&s_fb_mask, 255, 255, 255,   0);
  ETH_IP4_ADDR(&s_fb_gw,   192, 168,   1,   1);

  /*
   * MACアドレス設定
   * スケジューラ起動前・タスク生成前に呼ぶこと
   */
  EthernetCtl_SetMac(s_mac);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */
  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */
  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* defaultTask 生成 */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0,
                    512, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /*
   * Ethernet_Task 生成
   *
   * ★ 優先度は必ず osPriorityAboveNormal 以上にすること ★
   *   osPriorityIdle のままでは他タスクに実行機会を奪われ
   *   ポーリングが滞り、受信が正常に動作しない。
   */
  osThreadStaticDef(Ethernet_Task, StartEthernet_Task, osPriorityAboveNormal, 0,
                    2048, Ethernet_TaskBuffer, &Ethernet_TaskControlBlock);
  Ethernet_TaskHandle = osThreadCreate(osThread(Ethernet_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* COM1 初期化 (printf用) */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) { Error_Handler(); }

  /* USER ボタン (EXTI割り込みモード) */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  osKernelStart();

  /* ここには到達しない */
  while (1) {}
}

/* ============================================================
 * GPIO 初期化
 *
 * ETH RMII ピン配線 (Nucleo-H745ZI-Q):
 *   PC1  = ETH_MDC
 *   PC4  = ETH_RXD0
 *   PC5  = ETH_RXD1
 *   PA1  = ETH_REF_CLK
 *   PA2  = ETH_MDIO
 *   PA7  = ETH_CRS_DV
 *   PB13 = ETH_TXD1
 *   PG11 = ETH_TX_EN
 *   PG13 = ETH_TXD0
 *
 * ★ GPIO Speed は必ず VERY_HIGH にすること ★
 *   LOW のままでは 100Mbps RMII 信号の立ち上がりが追いつかず
 *   通信が不安定になる。
 * ============================================================ */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* ── ETH: PC1(MDC), PC4(RXD0), PC5(RXD1) ── */
  GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* ── ETH: PA1(REF_CLK), PA2(MDIO), PA7(CRS_DV) ── */
  GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ── ETH: PB13(TXD1) ── */
  GPIO_InitStruct.Pin       = GPIO_PIN_13;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ── ETH: PG11(TX_EN), PG13(TXD0) ── */
  GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_13;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* ── USB OTG FS: PA8(SOF), PA11(DM), PA12(DP) ── LOW でOK */
  GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* ============================================================
 * システムクロック設定 (HSI, PLL無効, VOS3)
 * ============================================================ */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK    | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1   | RCC_CLOCKTYPE_PCLK2
                                   | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  USERボタン割り込みコールバック
  *         押下ごとに LAN8742A LED Blackout ON/OFF をトグルする例。
  *         実際のアプリケーションに応じて変更すること。
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BUTTON_USER_PIN)
  {
    static bool blackout = false;
    blackout = !blackout;
    EthernetCtl_RequestSet(blackout ? ETH_REQ_LED_BLACKOUT_ON
                                    : ETH_REQ_LED_BLACKOUT_OFF);
  }
}

/* USER CODE END 4 */

/* ============================================================
 * defaultTask
 *   ETH関連の処理は一切持たない。
 *   アプリケーション固有の処理をここに記述すること。
 * ============================================================ */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

  for (;;)
  {
    /* ここにアプリケーション固有の処理を記述する */
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* ============================================================
 * Ethernet_Task
 *   ETH初期化・DHCP取得・状態監視をすべてここで完結する。
 *
 *   フロー:
 *     1. EthernetCtl_StartDhcpWithFallback() で初期化+DHCP取得
 *        (ブロッキング: 最大 timeout_ms + 3秒)
 *     2. 結果をシリアル出力
 *     3. 以降は定期的にETH状態を監視・出力するループ
 *        (実際の受信ポーリングは EthernetCtl 内部タスクが担う)
 * ============================================================ */
void StartEthernet_Task(void const *argument)
{
  /* USER CODE BEGIN StartEthernet_Task */
  (void)argument;

  /* ── (1) ETH HW初期化 + DHCP取得 ───────────────────────── */
  bool dhcp_ok = EthernetCtl_StartDhcpWithFallback(
      10000U,    /* DHCPタイムアウト: 10秒 */
      &s_fb_ip,
      &s_fb_mask,
      &s_fb_gw
  );

  /* ── (2) 初期化結果をシリアル出力 ──────────────────────── */
  char ip_str[16];
  EthernetCtl_GetIpString(ip_str, sizeof(ip_str));

  if (dhcp_ok) {
    printf("[ETH] DHCP OK           IP=%s\r\n", ip_str);
  } else {
    printf("[ETH] DHCP NG(Fallback) IP=%s\r\n", ip_str);
  }

  /* ── (3) ETH状態監視ループ ─────────────────────────────── */
  for (;;)
  {
    EthernetCtl_Status st;
    EthernetCtl_GetStatus(&st);

    if (st.link_up)
    {
      EthernetCtl_GetIpString(ip_str, sizeof(ip_str));
      printf("[ETH] Link=UP  IP=%-15s  %s\r\n",
             ip_str,
             st.is_dhcp ? "(DHCP)" : "(Fallback)");
    }
    else
    {
      printf("[ETH] Link=DOWN\r\n");
    }

    osDelay(5000);
  }
  /* USER CODE END StartEthernet_Task */
}

/* ============================================================
 * TIM6 → HAL_IncTick コールバック (CubeMX生成)
 * ============================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) { HAL_IncTick(); }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/* ============================================================
 * Error Handler
 * ============================================================ */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif
