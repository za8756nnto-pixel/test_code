#include "icm45686.h"

/* ============================================================
   内部ヘルパー：SPI CS制御 + 送受信
   ============================================================ */

static inline void _cs_low(ICM45686_Handle *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void _cs_high(ICM45686_Handle *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief レジスタ1バイト書き込み
 */
static HAL_StatusTypeDef _icm_write(ICM45686_Handle *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & 0x7F, val };  // bit7=0 で書き込み
    _cs_low(dev);
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(dev->hspi, buf, 2, 10);
    _cs_high(dev);
    return ret;
}

/**
 * @brief レジスタ複数バイト読み出し
 */
static HAL_StatusTypeDef _icm_read(ICM45686_Handle *dev, uint8_t reg,
                                    uint8_t *buf, uint16_t len) {
    uint8_t addr = reg | 0x80;  // bit7=1 で読み出し
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
 * @brief ICM-45686 初期化
 *
 * やること:
 *   1. WHO_AM_I 確認
 *   2. デバイスリセット
 *   3. Low Noiseモード有効化
 *   4. ODR / フルスケール設定
 *   5. DLPF（ローパスフィルタ）有効化
 *   6. DATA_READY割り込み有効化
 */
HAL_StatusTypeDef ICM45686_Init(ICM45686_Handle *dev,
                                 SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port,
                                 uint16_t cs_pin) {
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    dev->first_read = true;

    // スパイク除去デフォルト閾値
    dev->spike_accel_threshold = 20.0f;   // 20 m/s^2  (約2G分の急変)
    dev->spike_gyro_threshold  = 500.0f;  // 500 deg/s

    _cs_high(dev);
    HAL_Delay(10);

    // --- WHO_AM_I 確認 ---
    uint8_t who = 0;
    _icm_read(dev, ICM45686_REG_WHO_AM_I, &who, 1);
    if (who != ICM45686_WHO_AM_I_VAL) {
        return HAL_ERROR;  // センサーが見つからない
    }

    // --- ソフトウェアリセット ---
    _icm_write(dev, ICM45686_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(10);

    // --- 電源管理：ジャイロ＋加速度 Low Noiseモード ---
    // bit[3:2]=11 (Accel LN), bit[1:0]=11 (Gyro LN)
    _icm_write(dev, ICM45686_REG_PWR_MGMT0, 0x0F);
    HAL_Delay(1);

    // --- ジャイロ設定：ODR=1kHz, FS=2000dps ---
    // GYRO_CONFIG0: [7:5]=FS_SEL, [3:0]=ODR
    _icm_write(dev, ICM45686_REG_GYRO_CONFIG0,
               (ICM45686_GYRO_FS_2000DPS << 5) | ICM45686_ODR_1kHz);

    // --- 加速度設定：ODR=1kHz, FS=16G ---
    // ACCEL_CONFIG0: [7:5]=FS_SEL, [3:0]=ODR
    _icm_write(dev, ICM45686_REG_ACCEL_CONFIG0,
               (ICM45686_ACCEL_FS_16G << 5) | ICM45686_ODR_1kHz);

    // --- DLPF 有効化（重要！ スパイク低減の要） ---
    // GYRO_CONFIG1:  bit0=DLPF_EN, [3:1]=帯域幅
    // → 53Hz カットオフ（1kHz ODRに対して十分）
    _icm_write(dev, ICM45686_REG_GYRO_CONFIG1,
               0x01 | (ICM45686_DLPF_BW_53Hz << 1));

    // ACCEL_CONFIG1: bit0=DLPF_EN, [3:1]=帯域幅
    _icm_write(dev, ICM45686_REG_ACCEL_CONFIG1,
               0x01 | (ICM45686_DLPF_BW_53Hz << 1));

    // --- DATA_READY 割り込み有効化 ---
    // INT_CONFIG (0x06): INT1をプッシュプル、アクティブHigh
    _icm_write(dev, ICM45686_REG_INT_CONFIG, 0x02);
    // INT_SOURCE0 (0x65): bit3=UI_DRDY_INT1_EN
    _icm_write(dev, ICM45686_REG_INT_SOURCE0, 0x08);

    // --- スケールファクタ計算 ---
    // 16G フルスケール → 感度 = 2048 LSB/g → m/s^2換算
    dev->accel_scale = (16.0f * 9.80665f) / 32768.0f;
    // 2000dps フルスケール → 感度 = 16.4 LSB/(deg/s)
    dev->gyro_scale  = 2000.0f / 32768.0f;

    // オフセット初期値
    dev->accel_offset.x = dev->accel_offset.y = dev->accel_offset.z = 0.0f;
    dev->gyro_offset.x  = dev->gyro_offset.y  = dev->gyro_offset.z  = 0.0f;

    HAL_Delay(5);
    return HAL_OK;
}

/* ============================================================
   キャリブレーション（静止状態で呼ぶ）
   ============================================================ */

/**
 * @brief オフセットキャリブレーション
 * @param samples 平均化サンプル数（例: 1000）
 * @note  センサーを水平静止状態で呼ぶこと
 */
HAL_StatusTypeDef ICM45686_Calibrate(ICM45686_Handle *dev, uint16_t samples) {
    double ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    ICM45686_Data raw;

    for (uint16_t i = 0; i < samples; i++) {
        // DATA_READY待ち（最大5ms）
        uint32_t t = HAL_GetTick();
        while (!ICM45686_IsDataReady(dev)) {
            if (HAL_GetTick() - t > 5) break;
        }
        ICM45686_ReadData(dev, &raw);
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
    // Z軸は重力分（9.80665）を差し引く
    dev->accel_offset.z = (float)(az / samples) - 9.80665f;

    dev->gyro_offset.x  = (float)(gx / samples);
    dev->gyro_offset.y  = (float)(gy / samples);
    dev->gyro_offset.z  = (float)(gz / samples);

    dev->first_read = true;  // リセット
    return HAL_OK;
}

/* ============================================================
   DATA_READY チェック
   ============================================================ */

/**
 * @brief INT_STATUSレジスタでDATA_READYを確認
 * @return true: 新データあり
 *
 * 使い方: 割り込みハンドラの代わりにポーリングで使う場合
 */
bool ICM45686_IsDataReady(ICM45686_Handle *dev) {
    uint8_t status = 0;
    _icm_read(dev, ICM45686_REG_INT_STATUS, &status, 1);
    return (status & 0x08) != 0;  // bit3: UI_DRDY
}

/* ============================================================
   データ読み取り（スパイク除去つき）
   ============================================================ */

/**
 * @brief センサーデータ読み取り
 * @note  DATA_READY確認後に呼ぶこと
 *        スパイク除去：前回値との差が閾値を超えたらスキップ
 */
HAL_StatusTypeDef ICM45686_ReadData(ICM45686_Handle *dev, ICM45686_Data *out) {
    uint8_t buf[12];  // Accel(6) + Gyro(6)

    // --- アクセル生データ読み取り (0x0B〜0x10) ---
    HAL_StatusTypeDef ret = _icm_read(dev, ICM45686_REG_ACCEL_DATA_X1, buf, 12);
    if (ret != HAL_OK) return ret;

    // 16bit符号付き整数に変換
    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t raw_gx = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t raw_gy = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gz = (int16_t)((buf[10] << 8) | buf[11]);

    // 物理量に変換 + オフセット除去
    ICM45686_Data current;
    current.accel.x = raw_ax * dev->accel_scale - dev->accel_offset.x;
    current.accel.y = raw_ay * dev->accel_scale - dev->accel_offset.y;
    current.accel.z = raw_az * dev->accel_scale - dev->accel_offset.z;
    current.gyro.x  = raw_gx * dev->gyro_scale  - dev->gyro_offset.x;
    current.gyro.y  = raw_gy * dev->gyro_scale  - dev->gyro_offset.y;
    current.gyro.z  = raw_gz * dev->gyro_scale  - dev->gyro_offset.z;

    // --- スパイク除去 ---
    if (!dev->first_read) {
        float da_x = fabsf(current.accel.x - dev->last_data.accel.x);
        float da_y = fabsf(current.accel.y - dev->last_data.accel.y);
        float da_z = fabsf(current.accel.z - dev->last_data.accel.z);
        float dg_x = fabsf(current.gyro.x  - dev->last_data.gyro.x);
        float dg_y = fabsf(current.gyro.y  - dev->last_data.gyro.y);
        float dg_z = fabsf(current.gyro.z  - dev->last_data.gyro.z);

        float th_a = dev->spike_accel_threshold;
        float th_g = dev->spike_gyro_threshold;

        if (da_x > th_a || da_y > th_a || da_z > th_a ||
            dg_x > th_g || dg_y > th_g || dg_z > th_g) {
            // スパイク検出：前回値を返す
            *out = dev->last_data;
            return HAL_OK;
        }
    }

    dev->last_data = current;
    dev->first_read = false;
    *out = current;
    return HAL_OK;
}
