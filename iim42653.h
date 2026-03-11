#ifndef IIM42653_H
#define IIM42653_H

#include "stm32xxxx_hal.h"  // ← 使用するSTM32シリーズに合わせて変更 (例: stm32f4xx_hal.h)
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ============================================================
   IIM-42653 レジスタ定義
   ※ ICM-42688P / IIM-42652 と同一レジスタ体系
   ※ フィルタ詳細設定はBank1/Bank2が必要（初期化内で自動切替）
   ============================================================ */

/* --- Bank 0 (通常使用) --- */
#define IIM_REG_DEVICE_CONFIG       0x01  // SW_RESET
#define IIM_REG_DRIVE_CONFIG        0x02  // SPI スルーレート設定
#define IIM_REG_INT_CONFIG          0x06  // INT1ピン設定
#define IIM_REG_FIFO_CONFIG         0x16  // FIFOモード
#define IIM_REG_TEMP_DATA1          0x1D
#define IIM_REG_TEMP_DATA0          0x1E
#define IIM_REG_ACCEL_DATA_X1       0x1F  // 加速度データ先頭 (～0x24まで連続6バイト)
#define IIM_REG_GYRO_DATA_X1        0x25  // ジャイロデータ先頭 (～0x2Aまで連続6バイト)
#define IIM_REG_INT_STATUS          0x2D  // DATA_READY ステータス
#define IIM_REG_FIFO_COUNTH         0x2E
#define IIM_REG_FIFO_COUNTL         0x2F
#define IIM_REG_FIFO_DATA           0x30
#define IIM_REG_SIGNAL_PATH_RESET   0x4B  // FIFO/センサーリセット
#define IIM_REG_INTF_CONFIG0        0x4C  // UI_SIFS_CFG: I2C無効化
#define IIM_REG_INTF_CONFIG1        0x4D  // CLKSEL
#define IIM_REG_PWR_MGMT0           0x4E  // センサーON/OFF
#define IIM_REG_GYRO_CONFIG0        0x4F  // ジャイロ ODR/FS
#define IIM_REG_ACCEL_CONFIG0       0x50  // 加速度 ODR/FS
#define IIM_REG_GYRO_CONFIG1        0x51  // GYRO_UI_FILT_ORD, GYRO_DEC2_M2_ORD
#define IIM_REG_GYRO_ACCEL_CONFIG0  0x52  // ACCEL_UI_FILT_BW, GYRO_UI_FILT_BW
#define IIM_REG_ACCEL_CONFIG1       0x53  // ACCEL_UI_FILT_ORD
#define IIM_REG_INT_CONFIG0         0x63  // DRDY クリア方式
#define IIM_REG_INT_CONFIG1         0x64  // INT_TPULSE_DURATION
#define IIM_REG_INT_SOURCE0         0x65  // UI_DRDY_INT1_EN など
#define IIM_REG_WHO_AM_I            0x75

/* --- バンク切り替えレジスタ --- */
#define IIM_REG_REG_BANK_SEL        0x76

/* --- Bank 1 (AAFジャイロ設定) --- */
#define IIM_B1_REG_GYRO_CONFIG_STATIC2   0x0B  // GYRO_AAF_DIS, GYRO_NF_DIS
#define IIM_B1_REG_GYRO_CONFIG_STATIC3   0x0C  // GYRO_AAF_DELT[5:0]
#define IIM_B1_REG_GYRO_CONFIG_STATIC4   0x0D  // GYRO_AAF_DELTSQR[7:0]
#define IIM_B1_REG_GYRO_CONFIG_STATIC5   0x0E  // GYRO_AAF_BITSHIFT[3:0], GYRO_AAF_DELTSQR[11:8]

/* --- Bank 2 (AAF加速度設定) --- */
#define IIM_B2_REG_ACCEL_CONFIG_STATIC2  0x03  // ACCEL_AAF_DIS, ACCEL_AAF_DELT[5:0]
#define IIM_B2_REG_ACCEL_CONFIG_STATIC3  0x04  // ACCEL_AAF_DELTSQR[7:0]
#define IIM_B2_REG_ACCEL_CONFIG_STATIC4  0x05  // ACCEL_AAF_BITSHIFT[3:0], ACCEL_AAF_DELTSQR[11:8]

/* ============================================================
   IIM-42653 WHO_AM_I 値
   ============================================================ */
#define IIM42653_WHO_AM_I_VAL       0xDB  // ← データシートで要確認。IIM-42652は0xE5

/* ============================================================
   設定値
   ============================================================ */

/* ジャイロ ODR (GYRO_CONFIG0 [3:0]) */
#define IIM_GYRO_ODR_32kHz          0x01
#define IIM_GYRO_ODR_16kHz          0x02
#define IIM_GYRO_ODR_8kHz           0x03
#define IIM_GYRO_ODR_4kHz           0x04
#define IIM_GYRO_ODR_2kHz           0x05
#define IIM_GYRO_ODR_1kHz           0x06  // ← 今回使用
#define IIM_GYRO_ODR_200Hz          0x07
#define IIM_GYRO_ODR_100Hz          0x08

/* ジャイロ フルスケール (GYRO_CONFIG0 [7:5]) — IIM-42653固有 (最大4000dps) */
#define IIM_GYRO_FS_4000DPS         (0x00 << 5)
#define IIM_GYRO_FS_2000DPS         (0x01 << 5)
#define IIM_GYRO_FS_1000DPS         (0x02 << 5)
#define IIM_GYRO_FS_500DPS          (0x03 << 5)
#define IIM_GYRO_FS_250DPS          (0x04 << 5)
#define IIM_GYRO_FS_125DPS          (0x05 << 5)
#define IIM_GYRO_FS_62_5DPS         (0x06 << 5)
#define IIM_GYRO_FS_31_25DPS        (0x07 << 5)

/* 加速度 ODR (ACCEL_CONFIG0 [3:0]) */
#define IIM_ACCEL_ODR_32kHz         0x01
#define IIM_ACCEL_ODR_16kHz         0x02
#define IIM_ACCEL_ODR_8kHz          0x03
#define IIM_ACCEL_ODR_4kHz          0x04
#define IIM_ACCEL_ODR_2kHz          0x05
#define IIM_ACCEL_ODR_1kHz          0x06
#define IIM_ACCEL_ODR_200Hz         0x07
#define IIM_ACCEL_ODR_100Hz         0x08

/* 加速度 フルスケール (ACCEL_CONFIG0 [7:5]) — IIM-42653固有 (最大32g) */
#define IIM_ACCEL_FS_32G            (0x00 << 5)
#define IIM_ACCEL_FS_16G            (0x01 << 5)
#define IIM_ACCEL_FS_8G             (0x02 << 5)
#define IIM_ACCEL_FS_4G             (0x03 << 5)

/* GYRO_UI_FILT_BW (GYRO_ACCEL_CONFIG0 [7:4]) — デジタルLPF帯域幅 */
#define IIM_GYRO_UI_FILT_BW_ODR_2   (0x00 << 4)  // BW = ODR/2  (最大帯域)
#define IIM_GYRO_UI_FILT_BW_ODR_4   (0x01 << 4)  // BW = ODR/4
#define IIM_GYRO_UI_FILT_BW_ODR_5   (0x02 << 4)  // BW = ODR/5  ← 1kHz時 200Hz
#define IIM_GYRO_UI_FILT_BW_ODR_8   (0x03 << 4)  // BW = ODR/8  ← 1kHz時 125Hz 推奨
#define IIM_GYRO_UI_FILT_BW_ODR_10  (0x04 << 4)  // BW = ODR/10 ← 1kHz時 100Hz
#define IIM_GYRO_UI_FILT_BW_ODR_16  (0x05 << 4)  // BW = ODR/16
#define IIM_GYRO_UI_FILT_BW_ODR_20  (0x06 << 4)  // BW = ODR/20
#define IIM_GYRO_UI_FILT_BW_ODR_40  (0x07 << 4)  // BW = ODR/40 (最もノイズ低減)

/* ACCEL_UI_FILT_BW (GYRO_ACCEL_CONFIG0 [3:0]) — デジタルLPF帯域幅 */
#define IIM_ACCEL_UI_FILT_BW_ODR_2  0x00
#define IIM_ACCEL_UI_FILT_BW_ODR_4  0x01
#define IIM_ACCEL_UI_FILT_BW_ODR_5  0x02
#define IIM_ACCEL_UI_FILT_BW_ODR_8  0x03
#define IIM_ACCEL_UI_FILT_BW_ODR_10 0x04
#define IIM_ACCEL_UI_FILT_BW_ODR_16 0x05
#define IIM_ACCEL_UI_FILT_BW_ODR_20 0x06
#define IIM_ACCEL_UI_FILT_BW_ODR_40 0x07

/* ============================================================
   データ構造体
   ============================================================ */
typedef struct {
    float x, y, z;
} IIM_Vec3;

typedef struct {
    IIM_Vec3 accel;  // [m/s^2]
    IIM_Vec3 gyro;   // [deg/s]
    float    temp;   // [°C]
} IIM_Data;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    float accel_scale;
    float gyro_scale;

    IIM_Vec3 accel_offset;
    IIM_Vec3 gyro_offset;

    IIM_Data last_data;
    bool     first_read;

    float spike_accel_threshold;  // [m/s^2]
    float spike_gyro_threshold;   // [deg/s]
} IIM_Handle;

/* ============================================================
   関数プロトタイプ
   ============================================================ */
HAL_StatusTypeDef IIM42653_Init(IIM_Handle *dev,
                                 SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port,
                                 uint16_t cs_pin);

HAL_StatusTypeDef IIM42653_Calibrate(IIM_Handle *dev, uint16_t samples);
HAL_StatusTypeDef IIM42653_ReadData(IIM_Handle *dev, IIM_Data *out);
bool              IIM42653_IsDataReady(IIM_Handle *dev);

#endif /* IIM42653_H */
