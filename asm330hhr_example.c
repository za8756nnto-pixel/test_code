/* ============================================================
   ASM330HHR 使用例（main.c に組み込む）
   ============================================================

   CubeMX での設定（ICM-45686と共通）:
     SPI: Mode=Full-Duplex Master, CPOL=High, CPHA=2Edge (= Mode3)
          BaudRate: 10MHz 以下推奨
     GPIO (CS ピン): Output Push-Pull, 初期値 HIGH
     GPIO (INT1 ピン): Input, 割り込みモード Rising Edge
   ============================================================ */

#include "asm330hhr.h"

ASM330_Handle  imu;
ASM330_Data    imu_data;
volatile bool  data_ready_flag = false;

/* ================================================================
   パターンA: DATA_READY 割り込み方式（推奨）
   ================================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == IMU_INT1_Pin) {
        data_ready_flag = true;
    }
}

void app_main_interrupt_style(void) {

    if (ASM330_Init(&imu, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin) != HAL_OK) {
        Error_Handler();
    }

    HAL_Delay(100);
    ASM330_Calibrate(&imu, 1000);

    while (1) {
        if (data_ready_flag) {
            data_ready_flag = false;
            ASM330_ReadData(&imu, &imu_data);

            float gx = imu_data.gyro.x;   // [deg/s]
            float ax = imu_data.accel.x;  // [m/s^2]
            (void)gx; (void)ax;
        }
    }
}

/* ================================================================
   パターンB: ポーリング方式
   ================================================================ */

void app_main_polling_style(void) {

    if (ASM330_Init(&imu, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin) != HAL_OK) {
        Error_Handler();
    }

    HAL_Delay(100);
    ASM330_Calibrate(&imu, 1000);

    while (1) {
        if (ASM330_IsDataReady(&imu)) {
            ASM330_ReadData(&imu, &imu_data);
        }
    }
}

/* ================================================================
   ICM-45686 と ASM330HHR を同時使用する場合の例
   ================================================================ */
#include "icm45686.h"

ICM45686_Handle imu_icm;
ASM330_Handle   imu_asm;
ICM45686_Data   data_icm;
ASM330_Data     data_asm;

void app_dual_imu(void) {
    // それぞれ別のCSピンを使用
    ICM45686_Init(&imu_icm, &hspi1, ICM_CS_GPIO_Port, ICM_CS_Pin);
    ASM330_Init  (&imu_asm, &hspi1, ASM_CS_GPIO_Port, ASM_CS_Pin);

    // キャリブレーション（両方）
    HAL_Delay(100);
    ICM45686_Calibrate(&imu_icm, 1000);
    ASM330_Calibrate  (&imu_asm,  1000);

    while (1) {
        if (ICM45686_IsDataReady(&imu_icm)) {
            ICM45686_ReadData(&imu_icm, &data_icm);
        }
        if (ASM330_IsDataReady(&imu_asm)) {
            ASM330_ReadData(&imu_asm, &data_asm);
        }
        // 2つのセンサー値を比較・フュージョンするなど
    }
}
