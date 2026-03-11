/* ============================================================
   IIM-42653 使用例（main.c に組み込む）
   ============================================================
   CubeMX 設定は ICM-45686 / ASM330HHR と同じ:
     SPI: CPOL=High, CPHA=2Edge (Mode3), BaudRate ≤ 10MHz
     CS ピン: Output Push-Pull, 初期値 HIGH
     INT1ピン: EXTI Rising Edge
   ============================================================ */

#include "iim42653.h"

IIM_Handle   imu;
IIM_Data     imu_data;
volatile bool data_ready_flag = false;

/* ================================================================
   パターンA: DATA_READY 割り込み方式（推奨）
   ================================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == IMU_INT1_Pin) {
        data_ready_flag = true;
    }
}

void app_main(void) {

    if (IIM42653_Init(&imu, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin) != HAL_OK) {
        // ⚠️ HAL_ERRORの場合: WHO_AM_I不一致
        // → iim42653.h の IIM42653_WHO_AM_I_VAL をデータシートで確認して修正
        Error_Handler();
    }

    HAL_Delay(100);
    IIM42653_Calibrate(&imu, 1000);

    while (1) {
        if (data_ready_flag) {
            data_ready_flag = false;
            IIM42653_ReadData(&imu, &imu_data);

            float gx = imu_data.gyro.x;
            float ax = imu_data.accel.x;
            (void)gx; (void)ax;
        }
    }
}

/* ================================================================
   3センサー同時使用の例
   ================================================================ */
#include "icm45686.h"
#include "asm330hhr.h"

ICM45686_Handle imu_icm45686;
ASM330_Handle   imu_asm330;
IIM_Handle      imu_iim42653;

void app_triple_imu(void) {
    ICM45686_Init(&imu_icm45686, &hspi1, ICM_CS_GPIO_Port,  ICM_CS_Pin);
    ASM330_Init  (&imu_asm330,   &hspi1, ASM_CS_GPIO_Port,  ASM_CS_Pin);
    IIM42653_Init(&imu_iim42653, &hspi1, IIM_CS_GPIO_Port,  IIM_CS_Pin);

    HAL_Delay(200);

    ICM45686_Calibrate(&imu_icm45686, 1000);
    ASM330_Calibrate  (&imu_asm330,   1000);
    IIM42653_Calibrate(&imu_iim42653, 1000);

    ICM45686_Data d_icm;
    ASM330_Data   d_asm;
    IIM_Data      d_iim;

    while (1) {
        if (ICM45686_IsDataReady(&imu_icm45686)) ICM45686_ReadData(&imu_icm45686, &d_icm);
        if (ASM330_IsDataReady  (&imu_asm330))   ASM330_ReadData  (&imu_asm330,   &d_asm);
        if (IIM42653_IsDataReady(&imu_iim42653)) IIM42653_ReadData(&imu_iim42653, &d_iim);
    }
}
