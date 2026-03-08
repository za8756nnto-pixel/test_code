/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 512 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId Etahernet_TaskHandle;
uint32_t Etahernet_TaskBuffer[ 2048 ];
osStaticThreadDef_t Etahernet_TaskControlBlock;

/* USER CODE BEGIN PV */

/* フォールバックIPアドレス (DHCP失敗時に使用) */
static EthIp4Addr_t s_fb_ip;
static EthIp4Addr_t s_fb_mask;
static EthIp4Addr_t s_fb_gw;

/* MACアドレス (製品ごとにユニーク値を設定すること) */
static const uint8_t s_mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
void StartEtahernet_Task(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* ICache: コード実行の高速化。ETHデータとは無関係なので有効にする */
  SCB_EnableICache();

  /* DCache: 無効のまま使用する
   * ETHポーリング実装ではDCacheなしで動作に問題ない。
   * 有効にする場合はMPUでETH DMA領域をNon-Cacheableに設定が必要。 */
  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /* フォールバックIPアドレス設定 (DHCP失敗時のローカルIP) */
  ETH_IP4_ADDR(&s_fb_ip,    192, 168,   1, 100);
  ETH_IP4_ADDR(&s_fb_mask,  255, 255, 255,   0);
  ETH_IP4_ADDR(&s_fb_gw,    192, 168,   1,   1);

  /* MACアドレス設定 (スケジューラ起動前・タスク生成前に呼ぶこと) */
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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512,
                    defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Etahernet_Task
   * 優先度: osPriorityAboveNormal
   * osPriorityIdle のままだと他タスクに実行機会を奪われ
   * ポーリングが滞るため必ず Idle より高い優先度にすること */
  osThreadStaticDef(Etahernet_Task, StartEtahernet_Task, osPriorityAboveNormal, 0, 2048,
                    Etahernet_TaskBuffer, &Etahernet_TaskControlBlock);
  Etahernet_TaskHandle = osThreadCreate(osThread(Etahernet_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8N1) */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1) {}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
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

/**
  * @brief GPIO Initialization Function
  * @retval None
  *
  * 変更点: ETH用GPIO (PC1/4/5, PA1/2/7, PB13, PG11/13) の Speed を
  *         GPIO_SPEED_FREQ_LOW → GPIO_SPEED_FREQ_VERY_HIGH に修正。
  *         100Mbps Ethernet では VERY_HIGH が必須。
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* PC1(MDC), PC4(RXD0), PC5(RXD1) --- VERY_HIGH必須 */
  GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PA1(REF_CLK), PA2(MDIO), PA7(CRS_DV) --- VERY_HIGH必須 */
  GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB13(TXD1) --- VERY_HIGH必須 */
  GPIO_InitStruct.Pin       = GPIO_PIN_13;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PA8(SOF), PA11(DM), PA12(DP) --- USB OTG FS, LOWでOK */
  GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PG11(TX_EN), PG13(TXD0) --- VERY_HIGH必須 */
  GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_13;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  USER ボタン押下コールバック
  *         押下ごとにLED Blackout ON/OFFをトグルする例
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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  for (;;)
  {
    /* ── 5秒ごとにETH状態をシリアル出力 ── */
    EthernetCtl_Status st;
    EthernetCtl_GetStatus(&st);

    if (st.link_up)
    {
      char ip_str[16];
      EthernetCtl_GetIpString(ip_str, sizeof(ip_str));
      printf("[APP] Link=UP  IP=%-15s  %s\r\n",
             ip_str,
             st.is_dhcp ? "(DHCP)" : "(Fallback)");
      BSP_LED_On(LED_GREEN);
      BSP_LED_Off(LED_RED);
    }
    else
    {
      printf("[APP] Link=DOWN\r\n");
      BSP_LED_Off(LED_GREEN);
      BSP_LED_On(LED_RED);
    }

    osDelay(5000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartEtahernet_Task */
/**
  * @brief  Function implementing the Etahernet_Task thread.
  * @param  argument: Not used
  * @retval None
  *
  *  処理概要:
  *   1. EthernetCtl_StartDhcpWithFallback() を呼ぶ
  *      → 内部でHAL_ETH_Init / FreeRTOSタスク生成 / DHCP シーケンスを実行
  *      → DHCP成功またはタイムアウト(フォールバックIP)までブロック
  *   2. 結果をLEDとシリアルで通知
  *   3. 以降はポーリングループで待機
  *      (実際の受信ポーリングはEthernetCtl内部タスクが担う)
  */
/* USER CODE END Header_StartEtahernet_Task */
void StartEtahernet_Task(void const * argument)
{
  /* USER CODE BEGIN StartEtahernet_Task */

  /* ── ETH初期化 + DHCP取得 ──────────────────────────────
   * この呼び出しはブロッキング (最大 timeout_ms + 3000ms)
   * 戻り値: true=DHCP成功, false=フォールバックIP使用
   * ────────────────────────────────────────────────── */
  bool dhcp_ok = EthernetCtl_StartDhcpWithFallback(
      10000U,       /* DHCPタイムアウト: 10秒 */
      &s_fb_ip,
      &s_fb_mask,
      &s_fb_gw
  );

  char ip_str[16];
  EthernetCtl_GetIpString(ip_str, sizeof(ip_str));

  if (dhcp_ok) {
    printf("[ETH] DHCP OK    IP=%s\r\n", ip_str);
    BSP_LED_On(LED_GREEN);
    BSP_LED_Off(LED_YELLOW);
    BSP_LED_Off(LED_RED);
  } else {
    printf("[ETH] DHCP NG    Fallback IP=%s\r\n", ip_str);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_On(LED_YELLOW);   /* 黄: フォールバックIP使用中 */
    BSP_LED_Off(LED_RED);
  }

  /* 以降はアプリケーション処理 or 待機 */
  for (;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartEtahernet_Task */
}

/**
  * @brief  Period elapsed callback (TIM6 → HAL_IncTick)
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  Error Handler
  */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
