/* ============================================================
   ICM-45686 使用例（main.c に組み込む）
   ============================================================

   CubeMX での設定:
     SPI: Mode=Full-Duplex Master, CPOL=High, CPHA=2Edge (= Mode3)
          BaudRate: 10MHz 以下推奨（ICM-45686最大24MHz）
     GPIO (CS ピン): Output Push-Pull
     GPIO (INT1 ピン): Input, 割り込みモード Rising Edge
   ============================================================ */

#include "icm45686.h"

/* ---- グローバル変数 ---- */
ICM45686_Handle imu;
ICM45686_Data   imu_data;
volatile bool   data_ready_flag = false;

/* ================================================================
   パターンA: DATA_READY 割り込み方式（推奨）
   ================================================================
   CubeMX: INT1ピンを GPIO EXTI、Rising Edge に設定

   HAL_GPIO_EXTI_Callback で data_ready_flag を立て、
   メインループで読む。
   ================================================================ */

// stm32xxxx_it.c に追記、または main.c に書く
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == IMU_INT1_Pin) {  // ← CubeMXで設定したピン名
        data_ready_flag = true;
    }
}

void app_main_interrupt_style(void) {

    // --- 初期化 ---
    if (ICM45686_Init(&imu, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin) != HAL_OK) {
        Error_Handler();  // センサー未検出
    }

    // --- キャリブレーション（水平静止状態で1回だけ）---
    HAL_Delay(100);
    ICM45686_Calibrate(&imu, 1000);  // 1000サンプル平均

    // --- メインループ ---
    while (1) {
        if (data_ready_flag) {
            data_ready_flag = false;
            ICM45686_ReadData(&imu, &imu_data);

            // 使用例
            float ax = imu_data.accel.x;  // [m/s^2]
            float gx = imu_data.gyro.x;   // [deg/s]
            (void)ax; (void)gx;
        }
    }
}

/* ================================================================
   パターンB: ポーリング方式（割り込みを使いたくない場合）
   ================================================================ */

void app_main_polling_style(void) {

    if (ICM45686_Init(&imu, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin) != HAL_OK) {
        Error_Handler();
    }

    HAL_Delay(100);
    ICM45686_Calibrate(&imu, 1000);

    while (1) {
        // DATA_READYを確認してから読む（直接ポーリングとの差）
        if (ICM45686_IsDataReady(&imu)) {
            ICM45686_ReadData(&imu, &imu_data);
        }
        // ← HAL_Delay() は不要。センサー側のODRで自然にレートが決まる
    }
}

/* ================================================================
   スパイク除去の閾値カスタマイズ例
   ================================================================ */
void customize_threshold(void) {
    // Init後に変更可能
    imu.spike_accel_threshold = 10.0f;  // より厳しく（10 m/s^2）
    imu.spike_gyro_threshold  = 200.0f; // より厳しく（200 deg/s）
    // ※ 激しい動きの用途では緩める必要あり
}
