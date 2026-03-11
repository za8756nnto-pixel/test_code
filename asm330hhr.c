#include "asm330hhr.h"

/* ============================================================
   内部ヘルパー：SPI CS制御 + 送受信
   ============================================================ */

static inline void _cs_low(ASM330_Handle *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void _cs_high(ASM330_Handle *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief レジスタ1バイト書き込み
 *        ASM330HHR: bit7=0 で書き込み（ICM-45686と同じ）
 */
static HAL_StatusTypeDef _asm_write(ASM330_Handle *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & 0x7F, val };
    _cs_low(dev);
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(dev->hspi, buf, 2, 10);
    _cs_high(dev);
    return ret;
}

/**
 * @brief レジスタ複数バイト読み出し
 *        ASM330HHR: bit7=1 で読み出し
 *        ※ CTRL3_C の IF_INC=1(デフォルト) でアドレス自動インクリメント
 */
static HAL_StatusTypeDef _asm_read(ASM330_Handle *dev, uint8_t reg,
                                    uint8_t *buf, uint16_t len) {
    uint8_t addr = reg | 0x80;
    _cs_low(dev);
    HAL_SPI_Transmit(dev->hspi, &addr, 1, 10);
    HAL_StatusTypeDef ret = HAL_SPI_Receive(dev->hspi, buf, len, 10);
    _cs_high(dev);
    return ret;
}

/* ============================================================
   初期化
   ============================================================ */

/**
 * @brief ASM330HHR 初期化
 *
 * ICM-45686との主な違い:
 *   ① BDU (Block Data Update) ビットが重要 → torn read防止
 *   ② ジャイロLPF1の有効化はCTRL4_C + CTRL6_Cの2レジスタ
 *   ③ SPI専用時はCTRL9_XLでI2C無効化を推奨
 *   ④ DATA_READY割り込みはINT1_CTRLで設定
 */
HAL_StatusTypeDef ASM330_Init(ASM330_Handle *dev,
                               SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef *cs_port,
                               uint16_t cs_pin) {
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    dev->first_read = true;

    dev->spike_accel_threshold = 20.0f;   // [m/s^2]
    dev->spike_gyro_threshold  = 500.0f;  // [deg/s]

    _cs_high(dev);
    HAL_Delay(15);  // ブート完了待ち（最大10ms）

    // --- WHO_AM_I 確認 ---
    uint8_t who = 0;
    _asm_read(dev, ASM330_REG_WHO_AM_I, &who, 1);
    if (who != ASM330HHR_WHO_AM_I_VAL) {
        return HAL_ERROR;
    }

    // --- ソフトウェアリセット ---
    // CTRL3_C bit0 = SW_RESET
    _asm_write(dev, ASM330_REG_CTRL3_C, 0x01);
    HAL_Delay(10);

    // --- CTRL3_C 設定 ---
    // bit6: BDU=1   → 【重要】上位・下位バイトが同一サンプルを保証
    // bit2: IF_INC=1 → レジスタアドレス自動インクリメント（デフォルトだが明示）
    _asm_write(dev, ASM330_REG_CTRL3_C, 0x44);

    // --- SPI専用時：I2C無効化（ノイズ低減に効果あり） ---
    // CTRL9_XL bit2: I2C_DISABLE=1
    _asm_write(dev, ASM330_REG_CTRL9_XL, 0x04);

    // --- ジャイロ設定：ODR=1666Hz, FS=2000dps ---
    // CTRL2_G: [7:4]=ODR_G, [3:2]=FS_G
    _asm_write(dev, ASM330_REG_CTRL2_G,
               ASM330_GYRO_ODR_1666Hz | ASM330_GYRO_FS_2000DPS);

    // --- 加速度設定：ODR=1666Hz, FS=16G ---
    // CTRL1_XL: [7:4]=ODR_XL, [3:2]=FS_XL
    _asm_write(dev, ASM330_REG_CTRL1_XL,
               ASM330_ACCEL_ODR_1666Hz | ASM330_ACCEL_FS_16G);

    // --- ジャイロ LPF1 有効化（2段階設定）---
    // ステップ1: CTRL4_C bit1 = LPF1_SEL_G = 1
    _asm_write(dev, ASM330_REG_CTRL4_C, 0x02);

    // ステップ2: CTRL6_C [2:0] = FTYPE = 帯域幅選択
    // 100Hz カットオフ (1666Hz ODRに対して適切)
    _asm_write(dev, ASM330_REG_CTRL6_C, ASM330_GYRO_LPF1_BW_100Hz);

    // --- 加速度 LPF2 有効化 ---
    // CTRL8_XL bit2: HP_SLOPE_XL_EN=0, LPF2_XL_EN=1
    // [7:5]=HPCF_XL (LPF2カットオフ選択) → 0b100 = ODR/45
    _asm_write(dev, ASM330_REG_CTRL8_XL, 0x84);  // LPF2有効 + ODR/45

    // --- DATA_READY 割り込み有効化 (INT1ピン) ---
    // INT1_CTRL bit1: INT1_DRDY_G=1 (ジャイロDATA_READY → INT1)
    // INT1_CTRL bit0: INT1_DRDY_XL=1 (加速度DATA_READY → INT1)
    _asm_write(dev, ASM330_REG_INT1_CTRL, 0x03);

    // --- スケールファクタ ---
    // 2000dps, 16bit signed → 感度 = 70 mdps/LSB = 0.070 deg/s/LSB
    dev->gyro_scale  = 70.0f / 1000.0f;       // [deg/s per LSB]
    // 16G, 16bit signed → 感度 = 0.488 mg/LSB → m/s^2
    dev->accel_scale = (16.0f * 9.80665f) / 32768.0f;

    // オフセット初期値
    dev->accel_offset.x = dev->accel_offset.y = dev->accel_offset.z = 0.0f;
    dev->gyro_offset.x  = dev->gyro_offset.y  = dev->gyro_offset.z  = 0.0f;

    HAL_Delay(5);
    return HAL_OK;
}

/* ============================================================
   DATA_READY チェック
   ============================================================ */

/**
 * @brief STATUS_REGのGDA(bit1)でジャイロDATA_READYを確認
 *        GDA=1 かつ XLDA=1 の両方を確認
 */
bool ASM330_IsDataReady(ASM330_Handle *dev) {
    uint8_t status = 0;
    _asm_read(dev, ASM330_REG_STATUS_REG, &status, 1);
    // bit1: GDA (ジャイロ), bit0: XLDA (加速度)
    return (status & 0x03) == 0x03;
}

/* ============================================================
   キャリブレーション
   ============================================================ */

HAL_StatusTypeDef ASM330_Calibrate(ASM330_Handle *dev, uint16_t samples) {
    double ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    ASM330_Data raw;

    for (uint16_t i = 0; i < samples; i++) {
        uint32_t t = HAL_GetTick();
        while (!ASM330_IsDataReady(dev)) {
            if (HAL_GetTick() - t > 5) break;
        }
        ASM330_ReadData(dev, &raw);
        ax += raw.accel.x;
        ay += raw.accel.y;
        az += raw.accel.z;
        gx += raw.gyro.x;
        gy += raw.gyro.y;
        gz += raw.gyro.z;
        HAL_Delay(1);
    }

    dev->accel_offset.x = (float)(ax / samples);
    dev->accel_offset.y = (float)(ay / samples);
    dev->accel_offset.z = (float)(az / samples) - 9.80665f;  // 重力分を除く

    dev->gyro_offset.x  = (float)(gx / samples);
    dev->gyro_offset.y  = (float)(gy / samples);
    dev->gyro_offset.z  = (float)(gz / samples);

    dev->first_read = true;
    return HAL_OK;
}

/* ============================================================
   データ読み取り（スパイク除去つき）
   ============================================================ */

/**
 * @brief センサーデータ読み取り
 *
 * ASM330HHRの読み取り順序:
 *   OUTX_L_G(0x22) ～ OUTZ_H_G(0x27): ジャイロ 6バイト
 *   OUTX_L_A(0x28) ～ OUTZ_H_A(0x2D): 加速度 6バイト
 *   連続アドレスなので12バイト一括読みが可能（IF_INC=1）
 *
 * BDU=1 により、読み取り中に新サンプルで上書きされない
 * （上位バイトを読むまで下位バイトがロックされる）
 */
HAL_StatusTypeDef ASM330_ReadData(ASM330_Handle *dev, ASM330_Data *out) {
    uint8_t buf[12];

    // OUTX_L_G(0x22)から12バイト連続読み出し
    HAL_StatusTypeDef ret = _asm_read(dev, ASM330_REG_OUTX_L_G, buf, 12);
    if (ret != HAL_OK) return ret;

    // ジャイロ: Little-Endian (L先, H後)
    int16_t raw_gx = (int16_t)((buf[1]  << 8) | buf[0]);
    int16_t raw_gy = (int16_t)((buf[3]  << 8) | buf[2]);
    int16_t raw_gz = (int16_t)((buf[5]  << 8) | buf[4]);
    // 加速度: Little-Endian
    int16_t raw_ax = (int16_t)((buf[7]  << 8) | buf[6]);
    int16_t raw_ay = (int16_t)((buf[9]  << 8) | buf[8]);
    int16_t raw_az = (int16_t)((buf[11] << 8) | buf[10]);

    // ⚠️ ICM-45686との違い: ASM330HHRはLittle-Endian（低バイトが先）

    ASM330_Data current;
    current.gyro.x  = raw_gx * dev->gyro_scale  - dev->gyro_offset.x;
    current.gyro.y  = raw_gy * dev->gyro_scale  - dev->gyro_offset.y;
    current.gyro.z  = raw_gz * dev->gyro_scale  - dev->gyro_offset.z;
    current.accel.x = raw_ax * dev->accel_scale - dev->accel_offset.x;
    current.accel.y = raw_ay * dev->accel_scale - dev->accel_offset.y;
    current.accel.z = raw_az * dev->accel_scale - dev->accel_offset.z;

    // --- スパイク除去 ---
    if (!dev->first_read) {
        float dg_x = fabsf(current.gyro.x  - dev->last_data.gyro.x);
        float dg_y = fabsf(current.gyro.y  - dev->last_data.gyro.y);
        float dg_z = fabsf(current.gyro.z  - dev->last_data.gyro.z);
        float da_x = fabsf(current.accel.x - dev->last_data.accel.x);
        float da_y = fabsf(current.accel.y - dev->last_data.accel.y);
        float da_z = fabsf(current.accel.z - dev->last_data.accel.z);

        if (dg_x > dev->spike_gyro_threshold  ||
            dg_y > dev->spike_gyro_threshold  ||
            dg_z > dev->spike_gyro_threshold  ||
            da_x > dev->spike_accel_threshold ||
            da_y > dev->spike_accel_threshold ||
            da_z > dev->spike_accel_threshold) {
            *out = dev->last_data;  // スパイク：前回値を返す
            return HAL_OK;
        }
    }

    dev->last_data = current;
    dev->first_read = false;
    *out = current;
    return HAL_OK;
}
