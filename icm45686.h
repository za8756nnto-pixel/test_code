#ifndef ICM45686_H
#define ICM45686_H

#include "stm32xxxx_hal.h"  // ← 使用するSTM32シリーズに合わせて変更 (例: stm32f4xx_hal.h)
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ============================================================
   ICM-45686 レジスタ定義
   ============================================================ */
#define ICM45686_REG_DEVICE_CONFIG      0x01
#define ICM45686_REG_INT_CONFIG         0x06
#define ICM45686_REG_FIFO_CONFIG        0x16
#define ICM45686_REG_ACCEL_CONFIG0      0x21
#define ICM45686_REG_GYRO_CONFIG0       0x20
#define ICM45686_REG_GYRO_CONFIG1       0x23
#define ICM45686_REG_ACCEL_CONFIG1      0x24
#define ICM45686_REG_INT_SOURCE0        0x65
#define ICM45686_REG_PWR_MGMT0          0x10
#define ICM45686_REG_WHO_AM_I           0x75
#define ICM45686_REG_ACCEL_DATA_X1      0x0B
#define ICM45686_REG_GYRO_DATA_X1       0x11
#define ICM45686_REG_FIFO_COUNTH        0x2E
#define ICM45686_REG_FIFO_COUNTL        0x2F
#define ICM45686_REG_FIFO_DATA          0x30
#define ICM45686_REG_INT_STATUS         0x19

#define ICM45686_WHO_AM_I_VAL           0xE9

/* ============================================================
   設定値
   ============================================================ */
// ODR (Output Data Rate)
#define ICM45686_ODR_1kHz               0x06
#define ICM45686_ODR_2kHz               0x05
#define ICM45686_ODR_4kHz               0x04

// ジャイロ フルスケール
#define ICM45686_GYRO_FS_2000DPS        0x00
#define ICM45686_GYRO_FS_1000DPS        0x01
#define ICM45686_GYRO_FS_500DPS         0x02
#define ICM45686_GYRO_FS_250DPS         0x03

// 加速度 フルスケール
#define ICM45686_ACCEL_FS_16G           0x00
#define ICM45686_ACCEL_FS_8G            0x01
#define ICM45686_ACCEL_FS_4G            0x02
#define ICM45686_ACCEL_FS_2G            0x03

// DLPF帯域幅 (GYRO_CONFIG1 / ACCEL_CONFIG1)
// 値が小さいほどカットオフ周波数が低い（ノイズ除去強め）
#define ICM45686_DLPF_BW_180Hz          0x00
#define ICM45686_DLPF_BW_121Hz          0x01
#define ICM45686_DLPF_BW_73Hz           0x02
#define ICM45686_DLPF_BW_53Hz           0x03
#define ICM45686_DLPF_BW_34Hz           0x04
#define ICM45686_DLPF_BW_25Hz           0x05
#define ICM45686_DLPF_BW_16Hz           0x06

/* ============================================================
   データ構造体
   ============================================================ */
typedef struct {
    float x, y, z;
} ICM45686_Vec3;

typedef struct {
    ICM45686_Vec3 accel;   // [m/s^2]
    ICM45686_Vec3 gyro;    // [deg/s]
    float temp;            // [°C]
} ICM45686_Data;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    // スケールファクタ（初期化時に自動設定）
    float accel_scale;
    float gyro_scale;

    // キャリブレーションオフセット
    ICM45686_Vec3 accel_offset;
    ICM45686_Vec3 gyro_offset;

    // スパイク除去用（前回値）
    ICM45686_Data last_data;
    bool          first_read;

    // スパイク除去閾値
    float spike_accel_threshold;  // [m/s^2]  例: 20.0
    float spike_gyro_threshold;   // [deg/s]  例: 500.0
} ICM45686_Handle;

/* ============================================================
   関数プロトタイプ
   ============================================================ */
HAL_StatusTypeDef ICM45686_Init(ICM45686_Handle *dev,
                                 SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port,
                                 uint16_t cs_pin);

HAL_StatusTypeDef ICM45686_Calibrate(ICM45686_Handle *dev, uint16_t samples);
HAL_StatusTypeDef ICM45686_ReadData(ICM45686_Handle *dev, ICM45686_Data *out);
bool              ICM45686_IsDataReady(ICM45686_Handle *dev);

// 内部関数（直接呼ばなくてよい）
static HAL_StatusTypeDef _icm_write(ICM45686_Handle *dev, uint8_t reg, uint8_t val);
static HAL_StatusTypeDef _icm_read(ICM45686_Handle *dev, uint8_t reg,
                                    uint8_t *buf, uint16_t len);

#endif /* ICM45686_H */
