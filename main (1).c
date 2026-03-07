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

/* ETHフォールバックIPアドレス設定 (DHCP失敗時に使用) */
static EthIp4Addr_t s_fb_ip;
static EthIp4Addr_t s_fb_mask;
static EthIp4Addr_t s_fb_gw;

/* MACアドレス (要変更: 製品ごとにユニークな値を設定すること) */
static const uint8_t s_mac_addr[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };

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
   * ETHポーリング実装ではDCacheなしで十分動作する。
   * DCacheを有効にする場合はMPUでETH DMA領域をNon-Cacheableに
   * 設定する必要があるが、その複雑さを避けるため無効のまま運用する。 */
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
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
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

  /* フォールバックIPアドレス設定 (DHCP失敗時に割り当てられる固定IP) */
  ETH_IP4_ADDR(&s_fb_ip,   192, 168,   1, 100);
  ETH_IP4_ADDR(&s_fb_mask, 255, 255, 255,   0);
  ETH_IP4_ADDR(&s_fb_gw,   192, 168,   1,   1);

  /* MACアドレス設定 (タスク起動前に必ず呼ぶこと) */
  EthernetCtl_SetMac(s_mac_addr);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Etahernet_Task */
  osThreadStaticDef(Etahernet_Task, StartEtahernet_Task, osPriorityAboveNormal, 0, 2048, Etahernet_TaskBuffer, &Etahernet_TaskControlBlock);
  Etahernet_TaskHandle = osThreadCreate(osThread(Etahernet_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  *
  * 変更点: ETH用GPIO (PC1/4/5, PA1/2/7, PB13, PG11/13) の Speed を
  *         GPIO_SPEED_FREQ_LOW → GPIO_SPEED_FREQ_VERY_HIGH に修正。
  *         100Mbps Ethernet は高速GPIOが必須。LOWのままだと信号品質が劣化し
  *         リンクアップ不安定やパケットエラーの原因になる。
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : PC1 PC4 PC5 (ETH: MDC, RXD0, RXD1)
   * Speed: VERY_HIGH必須 (変更: LOW → VERY_HIGH) */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* ← 修正 */
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 (ETH: REF_CLK, MDIO, CRS_DV)
   * Speed: VERY_HIGH必須 (変更: LOW → VERY_HIGH) */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* ← 修正 */
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 (ETH: TXD1)
   * Speed: VERY_HIGH必須 (変更: LOW → VERY_HIGH) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* ← 修正 */
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 (USB OTG FS: SOF, DM, DP)
   * USB用はLOWのままで問題なし */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 (ETH: TX_EN, TXD0)
   * Speed: VERY_HIGH必須 (変更: LOW → VERY_HIGH) */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* ← 修正 */
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  for(;;)
  {
    /* USER CODE: アプリケーション処理をここに記述する */

    /* ── ETH状態確認と表示例 (5秒ごと) ── */
    EthernetCtl_Status st;
    EthernetCtl_GetStatus(&st);

    if (st.link_up)
    {
      char ip_str[16];
      EthernetCtl_GetIpString(ip_str, sizeof(ip_str));
      printf("[APP] Link=UP  IP=%s  %s\r\n",
             ip_str,
             st.is_dhcp ? "(DHCP)" : "(Fallback)");

      /* LED表示: リンクUP=緑点灯 */
      BSP_LED_On(LED_GREEN);
      BSP_LED_Off(LED_RED);
    }
    else
    {
      printf("[APP] Link=DOWN\r\n");

      /* LED表示: リンクDOWN=赤点灯 */
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
  * 処理概要:
  *   1. EthernetCtl_StartDhcpWithFallback() でHW初期化 + DHCP取得
  *      → DHCP成功: 取得したIPで動作
  *      → DHCP失敗: フォールバックIP (192.168.1.100) で動作
  *   2. 以降はポーリングループで以下を処理:
  *      - LINK UP/DOWN 監視
  *      - ARP Reply 返送 (ping前のARP解決)
  *      - ICMP Echo Reply 返送 (pingに応答)
  *      - DHCP応答受信
  *      - ETH_REQ_LED_BLACKOUT_ON/OFF リクエスト処理
  */
/* USER CODE END Header_StartEtahernet_Task */
void StartEtahernet_Task(void const * argument)
{
  /* USER CODE BEGIN StartEtahernet_Task */

  /* ── ETH初期化 + DHCP取得 ──────────────────────────────────────
   *  この関数内で:
   *    - HAL_ETH_Init()
   *    - PHY Auto-Negotiation 待ち
   *    - DHCP Discover → Offer → Request → ACK シーケンス
   *  が実行される。DHCP失敗時は自動的にフォールバックIPが設定される。
   *
   *  ※ この呼び出しはリンクアップ + DHCP完了までブロックする。
   *    タイムアウト (10000ms) を超えるとフォールバックIPで抜ける。
   * ──────────────────────────────────────────────────────────── */
  bool dhcp_ok = EthernetCtl_StartDhcpWithFallback(
      10000,        /* DHCPタイムアウト: 10秒 */
      &s_fb_ip,
      &s_fb_mask,
      &s_fb_gw
  );

  if (dhcp_ok)
  {
    char ip_str[16];
    EthernetCtl_GetIpString(ip_str, sizeof(ip_str));
    printf("[ETH] DHCP OK  IP=%s\r\n", ip_str);
    BSP_LED_On(LED_GREEN);
    BSP_LED_Off(LED_YELLOW);
  }
  else
  {
    char ip_str[16];
    EthernetCtl_GetIpString(ip_str, sizeof(ip_str));
    printf("[ETH] DHCP NG  Fallback IP=%s\r\n", ip_str);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_On(LED_YELLOW);  /* 黄色: フォールバックIP使用中 */
  }

  /* ── ポーリングループ ────────────────────────────────────────
   *  EthernetCtl_StartDhcpWithFallback() 内部でETHタスクが
   *  既に起動済みのため、ここでは補助的な監視のみを行う。
   *
   *  例: ユーザボタン押下でLED Blackout ON/OFFを切り替える場合は
   *      EthernetCtl_RequestSet() を割り込みハンドラから呼ぶ。
   * ──────────────────────────────────────────────────────────── */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartEtahernet_Task */
}

/* USER CODE BEGIN 4 (再掲: ユーザボタン割り込みハンドラ例) */
/**
  * @brief  USER ボタン押下時の割り込みコールバック
  *         押下ごとに LED Blackout ON/OFF をトグルする例
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* USER CODE BEGIN HAL_GPIO_EXTI_Callback */
  if (GPIO_Pin == BUTTON_USER_PIN) /* main.h で定義されるボタンピン */
  {
    static bool blackout = false;
    blackout = !blackout;
    EthernetCtl_RequestSet(blackout
                           ? ETH_REQ_LED_BLACKOUT_ON
                           : ETH_REQ_LED_BLACKOUT_OFF);
  }
  /* USER CODE END HAL_GPIO_EXTI_Callback */
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
