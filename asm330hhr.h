#ifndef ASM330HHR_H
#define ASM330HHR_H

#include "stm32xxxx_hal.h"  // ← 使用するSTM32シリーズに合わせて変更 (例: stm32f4xx_hal.h)
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ============================================================
   ASM330HHR レジスタ定義 (ASM330LHHファミリー共通)
   ============================================================ */
#define ASM330_REG_WHO_AM_I         0x0F
#define ASM330_REG_INT1_CTRL        0x0D
#define ASM330_REG_INT2_CTRL        0x0E
#define ASM330_REG_CTRL1_XL         0x10  // 加速度 ODR/FS
#define ASM330_REG_CTRL2_G          0x11  // ジャイロ ODR/FS
#define ASM330_REG_CTRL3_C          0x12  // BDU, SW_RESET, IF_INC など
#define ASM330_REG_CTRL4_C          0x13  // LPF1_SEL_G など
#define ASM330_REG_CTRL6_C          0x15  // FTYPE[2:0] (ジャイロLPF1帯域)
#define ASM330_REG_CTRL7_G          0x16  // HP_EN_G など
#define ASM330_REG_CTRL8_XL         0x17  // 加速度LPF2/HPF設定
#define ASM330_REG_CTRL9_XL         0x18  // I2C無効化 (SPI専用時に使用)
#define ASM330_REG_STATUS_REG       0x1E  // XLDA, GDA ビット
#define ASM330_REG_OUTX_L_G         0x22  // ジャイロ X低バイト (～0x27まで連続)
#define ASM330_REG_OUTX_L_A         0x28  // 加速度 X低バイト (～0x2Dまで連続)

// ASM330HHR の WHO_AM_I 値
// ※ ASM330LHH = 0x6B, ASM330HHR = 0x6B (同じ場合が多い。データシートで要確認)
#define ASM330HHR_WHO_AM_I_VAL      0x6B

/* ============================================================
   設定値
   ============================================================ */

// ジャイロ ODR (CTRL2_G [7:4])
#define ASM330_GYRO_ODR_OFF         0x00
#define ASM330_GYRO_ODR_12_5Hz      0x10
#define ASM330_GYRO_ODR_26Hz        0x20
#define ASM330_GYRO_ODR_52Hz        0x30
#define ASM330_GYRO_ODR_104Hz       0x40
#define ASM330_GYRO_ODR_208Hz       0x50
#define ASM330_GYRO_ODR_416Hz       0x60
#define ASM330_GYRO_ODR_833Hz       0x70
#define ASM330_GYRO_ODR_1666Hz      0x80
#define ASM330_GYRO_ODR_3333Hz      0x90
#define ASM330_GYRO_ODR_6666Hz      0xA0

// ジャイロ フルスケール (CTRL2_G [3:2])
#define ASM330_GYRO_FS_250DPS       0x00
#define ASM330_GYRO_FS_500DPS       0x04
#define ASM330_GYRO_FS_1000DPS      0x08
#define ASM330_GYRO_FS_2000DPS      0x0C

// 加速度 ODR (CTRL1_XL [7:4])
#define ASM330_ACCEL_ODR_OFF        0x00
#define ASM330_ACCEL_ODR_12_5Hz     0x10
#define ASM330_ACCEL_ODR_26Hz       0x20
#define ASM330_ACCEL_ODR_52Hz       0x30
#define ASM330_ACCEL_ODR_104Hz      0x40
#define ASM330_ACCEL_ODR_208Hz      0x50
#define ASM330_ACCEL_ODR_416Hz      0x60
#define ASM330_ACCEL_ODR_833Hz      0x70
#define ASM330_ACCEL_ODR_1666Hz     0x80

// 加速度 フルスケール (CTRL1_XL [3:2])
#define ASM330_ACCEL_FS_2G          0x00
#define ASM330_ACCEL_FS_4G          0x08
#define ASM330_ACCEL_FS_8G          0x0C
#define ASM330_ACCEL_FS_16G         0x04

// ジャイロ LPF1 帯域幅 (CTRL6_C [2:0]) ← LPF1_SEL_G=1 時に有効
// ODR=1666Hzのとき (カットオフ周波数の目安)
#define ASM330_GYRO_LPF1_BW_WIDE    0x00  // ~315 Hz
#define ASM330_GYRO_LPF1_BW_237Hz   0x01
#define ASM330_GYRO_LPF1_BW_173Hz   0x02
#define ASM330_GYRO_LPF1_BW_100Hz   0x03  // 推奨（ノイズ低減と応答のバランス）
#define ASM330_GYRO_LPF1_BW_70Hz    0x04
#define ASM330_GYRO_LPF1_BW_51Hz    0x05
#define ASM330_GYRO_LPF1_BW_30Hz    0x06
#define ASM330_GYRO_LPF1_BW_NARROW  0x07  // ~16 Hz (最もノイズ低減)

/* ============================================================
   データ構造体
   ============================================================ */
typedef struct {
    float x, y, z;
} ASM330_Vec3;

typedef struct {
    ASM330_Vec3 accel;  // [m/s^2]
    ASM330_Vec3 gyro;   // [deg/s]
} ASM330_Data;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    float accel_scale;  // LSB → m/s^2
    float gyro_scale;   // LSB → deg/s

    ASM330_Vec3 accel_offset;
    ASM330_Vec3 gyro_offset;

    ASM330_Data last_data;
    bool        first_read;

    float spike_accel_threshold;  // [m/s^2]
    float spike_gyro_threshold;   // [deg/s]
} ASM330_Handle;

/* ============================================================
   関数プロトタイプ
   ============================================================ */
HAL_StatusTypeDef ASM330_Init(ASM330_Handle *dev,
                               SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef *cs_port,
                               uint16_t cs_pin);

HAL_StatusTypeDef ASM330_Calibrate(ASM330_Handle *dev, uint16_t samples);
HAL_StatusTypeDef ASM330_ReadData(ASM330_Handle *dev, ASM330_Data *out);
bool              ASM330_IsDataReady(ASM330_Handle *dev);

#endif /* ASM330HHR_H */
