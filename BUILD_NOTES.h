/**
 * =============================================================
 * STM32H7 ETH Driver (No LWIP) ビルド・移植ノート
 * =============================================================
 *
 * ─────────────────────────────────────────────────────────────
 * 1. 必要ファイル構成
 * ─────────────────────────────────────────────────────────────
 *
 *   プロジェクト/
 *   ├── Core/
 *   │   ├── Src/
 *   │   │   ├── main.c          ← main_example.c を参考に改変
 *   │   │   └── eth_driver.c    ← 追加
 *   │   └── Inc/
 *   │       └── eth_driver.h    ← 追加
 *   └── ...
 *
 * ─────────────────────────────────────────────────────────────
 * 2. STM32CubeMX 設定
 * ─────────────────────────────────────────────────────────────
 *
 *  [ETH]
 *   - Mode: RMII (または MII。ボードに合わせる)
 *   - NVIC: ETH global interrupt → 無効でOK (ポーリングのため)
 *   - DMA: 使用しない
 *   - HAL_ETH_Init はドライバ内で呼ぶため CubeMX生成の
 *     MX_ETH_Init() は削除または空実装にする
 *
 *  [FreeRTOS]
 *   - RTOS: FreeRTOS (CMSIS-RTOS v2 or Vanilla)
 *   - Heap: heap_4 推奨
 *   - configTOTAL_HEAP_SIZE: 最低 32KB
 *
 *  [MPU] ※STM32H7では重要
 *   - ETH DMAディスクリプタ領域を Non-cacheable に設定
 *   - 推奨: .RxDecripSection / .TxDecripSection を
 *     0x30040000 (SRAM3) に配置し、MPUでキャッシュ無効化
 *
 * ─────────────────────────────────────────────────────────────
 * 3. リンカスクリプト (.ld) 追記例
 * ─────────────────────────────────────────────────────────────
 *
 * STM32H745XIHx_FLASH.ld に以下を追加:
 *
 *   /* ETH DMA Descriptors: Non-Cacheable SRAM */
 *   .RxDecripSection (NOLOAD) :
 *   {
 *     . = ALIGN(4);
 *     *(.RxDecripSection)
 *     . = ALIGN(4);
 *   } >RAM_D2
 *
 *   .TxDecripSection (NOLOAD) :
 *   {
 *     . = ALIGN(4);
 *     *(.TxDecripSection)
 *     . = ALIGN(4);
 *   } >RAM_D2
 *
 *  ※ RAM_D2 = AHB SRAM (0x30000000) はETH DMAがアクセス可能
 *  ※ RAM_D1 (DTCM 0x20000000) は ETH DMAからアクセス不可なので注意
 *
 * ─────────────────────────────────────────────────────────────
 * 4. MPU設定例 (Non-Cacheable)
 * ─────────────────────────────────────────────────────────────
 *
 * SystemClock_Config() の前か、HAL_Init() 後に呼ぶ:
 *
 *   void MPU_Config(void)
 *   {
 *       MPU_Region_InitTypeDef mpu = {0};
 *       HAL_MPU_Disable();
 *
 *       // SRAM3 (ETH descriptor/buffer) → Non-cacheable
 *       mpu.Enable           = MPU_REGION_ENABLE;
 *       mpu.BaseAddress      = 0x30040000;
 *       mpu.Size             = MPU_REGION_SIZE_256B;
 *       mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
 *       mpu.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
 *       mpu.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
 *       mpu.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
 *       mpu.Number           = MPU_REGION_NUMBER0;
 *       mpu.TypeExtField     = MPU_TEX_LEVEL1;
 *       mpu.SubRegionDisable = 0x00;
 *       mpu.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
 *       HAL_MPU_ConfigRegion(&mpu);
 *
 *       HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
 *   }
 *
 * ─────────────────────────────────────────────────────────────
 * 5. PHY 依存部の変更
 * ─────────────────────────────────────────────────────────────
 *
 *  eth_driver.c の以下を使用PHYに合わせて変更:
 *
 *  #define PHY_ADDRESS  0x00  ← PHYのMDIOアドレス
 *
 *  prv_phy_check_link():
 *    LAN8742A → Reg1 Bit2 で判定 (実装済)
 *    KSZ8081  → Reg1 Bit2 で判定 (同じ)
 *    DP83848  → Reg1 Bit2 で判定 (同じ)
 *    RTL8201  → Reg1 Bit2 で判定 (同じ)
 *
 *  prv_handle_link_up():
 *    Auto-Negotiation 完了は Reg1 Bit5 で判定 (共通)
 *    速度/Full-Half duplex を PHY固有レジスタで取得し
 *    HAL_ETH_SetMDIOClockRange() 等で調整が必要な場合がある
 *
 * ─────────────────────────────────────────────────────────────
 * 6. デュアルコア (H745/H755) 運用
 * ─────────────────────────────────────────────────────────────
 *
 *  ETH ペリフェラルは CM7 側で占有する
 *  CM4 側から ETH_GetNetConfig() を呼ぶ場合は、
 *  共有メモリ (SRAM3等) 経由でコピーするか、
 *  HSEMハードウェアセマフォで排他制御を追加する
 *
 *  推奨構成:
 *    CM7: ETH_Task() を動かす
 *    CM4: 共有変数からIPアドレスを参照するだけ
 *
 * ─────────────────────────────────────────────────────────────
 * 7. メモリ使用量 (参考)
 * ─────────────────────────────────────────────────────────────
 *
 *  Rx バッファ: ETH_RX_BUF_SIZE × ETH_RX_BUF_COUNT
 *             = 1536 × 4 = 6144 bytes
 *  Tx バッファ: ETH_TX_BUF_SIZE = 1536 bytes
 *  DMA描述子:  ETH_DMADescTypeDef × (4+4) ≒ 320 bytes
 *  ETHタスクスタック: 1024 × 4 = 4096 bytes
 *  合計:       ≒ 12KB  (LWIPと比較し大幅削減)
 *
 * ─────────────────────────────────────────────────────────────
 * 8. 動作確認手順
 * ─────────────────────────────────────────────────────────────
 *
 *  (1) LANケーブル接続前にビルド・書き込み
 *  (2) シリアルモニタ (115200bps) を開く
 *  (3) LANケーブル接続 → "[ETH] LINK UP" 表示を確認
 *  (4) DHCP環境なら "[ETH] DHCP OK  IP=xxx.xxx.xxx.xxx" 表示
 *      DHCP環境なしなら 10秒後 "[ETH] DHCP NG  Fallback IP=192.168.1.100"
 *  (5) PC から ping xxx.xxx.xxx.xxx → 応答確認
 *  (6) LANケーブル抜き → "[ETH] LINK DOWN" 表示確認
 *  (7) LANケーブル再挿し → LINK UP → DHCP シーケンス自動再開
 */
