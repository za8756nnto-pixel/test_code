#include "iim42653.h"

/* ============================================================
   内部ヘルパー：SPI CS制御 + 送受信
   ============================================================ */

static inline void _cs_low(IIM_Handle *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void _cs_high(IIM_Handle *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef _iim_write(IIM_Handle *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & 0x7F, val };  // bit7=0: 書き込み
    _cs_low(dev);
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(dev->hspi, buf, 2, 10);
    _cs_high(dev);
    HAL_Delay(1);  // バンク切り替え後の安定待ち
    return ret;
}

static HAL_StatusTypeDef _iim_read(IIM_Handle *dev, uint8_t reg,
                                    uint8_t *buf, uint16_t len) {
    uint8_t addr = reg | 0x80;  // bit7=1: 読み出し
    _cs_low(dev);
    HAL_SPI_Transmit(dev->hspi, &addr, 1, 10);
    HAL_StatusTypeDef ret = HAL_SPI_Receive(dev->hspi, buf, len, 10);
    _cs_high(dev);
    return ret;
}

/**
 * @brief レジスタバンク切り替え
 * @param bank 0〜4
 *
 * IIM-42653はICM-42688P系と同じバンク方式。
 * Bank0: 通常データ読み書き
 * Bank1: ジャイロAAF（アンチエイリアスフィルタ）設定
 * Bank2: 加速度AAF設定
 * 設定後は必ずBank0に戻すこと。
 */
static void _iim_set_bank(IIM_Handle *dev, uint8_t bank) {
    _iim_write(dev, IIM_REG_REG_BANK_SEL, bank);
}

/* ============================================================
   初期化
   ============================================================ */

/**
 * @brief IIM-42653 初期化
 *
 * ICM-45686 / ASM330HHR との主な違い:
 *   ① レジスタバンク切り替えが必要（Bank0〜4）
 *   ② AAF（アンチエイリアスフィルタ）はBank1/2に存在
 *   ③ UIフィルタ（GYRO_UI_FILT_BW）はBank0のGYRO_ACCEL_CONFIG0で設定
 *   ④ フルスケールが最大4000dps/32g（ICM-45686と同等）
 *   ⑤ Big-Endian（ICM-45686と同じ、ASM330HHRと逆）
 */
HAL_StatusTypeDef IIM42653_Init(IIM_Handle *dev,
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
    HAL_Delay(15);

    // Bank0 を確実に選択
    _iim_set_bank(dev, 0);

    // --- WHO_AM_I 確認 ---
    uint8_t who = 0;
    _iim_read(dev, IIM_REG_WHO_AM_I, &who, 1);
    if (who != IIM42653_WHO_AM_I_VAL) {
        // ⚠️ WHO_AM_Iが一致しない場合はデータシートで値を確認して
        //    iim42653.h の IIM42653_WHO_AM_I_VAL を修正すること
        return HAL_ERROR;
    }

    // --- ソフトウェアリセット ---
    _iim_write(dev, IIM_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(10);
    _iim_set_bank(dev, 0);  // リセット後に再度Bank0を選択

    // --- I2C無効化（SPI専用時に推奨） ---
    // INTF_CONFIG0 bit[1:0] = 0b11: I2C無効、SPI専用
    _iim_write(dev, IIM_REG_INTF_CONFIG0, 0x03);

    // --- 電源管理：ジャイロ＋加速度 Low Noiseモード ---
    // PWR_MGMT0: [3:2]=ACCEL_MODE=11(LN), [1:0]=GYRO_MODE=11(LN)
    _iim_write(dev, IIM_REG_PWR_MGMT0, 0x0F);
    HAL_Delay(1);  // LNモード移行待ち（最大200μs）

    // --- ジャイロ設定：ODR=1kHz, FS=2000dps ---
    _iim_write(dev, IIM_REG_GYRO_CONFIG0,
               IIM_GYRO_FS_2000DPS | IIM_GYRO_ODR_1kHz);

    // --- 加速度設定：ODR=1kHz, FS=16G ---
    _iim_write(dev, IIM_REG_ACCEL_CONFIG0,
               IIM_ACCEL_FS_16G | IIM_ACCEL_ODR_1kHz);

    // --- デジタルUIフィルタ設定（Bank0で設定可能）---
    // GYRO_CONFIG1: [3:2]=GYRO_UI_FILT_ORD=11(3次butterworth), bit0=温度フィルタ
    _iim_write(dev, IIM_REG_GYRO_CONFIG1, 0x0A);   // 3次フィルタ有効

    // ACCEL_CONFIG1: [3:2]=ACCEL_UI_FILT_ORD=11(3次)
    _iim_write(dev, IIM_REG_ACCEL_CONFIG1, 0x08);

    // GYRO_ACCEL_CONFIG0: [7:4]=GYRO_UI_FILT_BW, [3:0]=ACCEL_UI_FILT_BW
    // ODR/10 → 1kHz ODR時に100Hzカットオフ（ノイズと応答のバランス）
    _iim_write(dev, IIM_REG_GYRO_ACCEL_CONFIG0,
               IIM_GYRO_UI_FILT_BW_ODR_10 | IIM_ACCEL_UI_FILT_BW_ODR_10);

    // =====================================================
    // AAF（アンチエイリアスフィルタ）設定 — Bank1/Bank2
    // 1kHz ODR に対して適切な値（約258Hz カットオフ）を設定
    // 値の計算: delt=6, deltsqr=36, bitshift=10 (データシートTable参照)
    // =====================================================

    // --- Bank1: ジャイロAAF ---
    _iim_set_bank(dev, 1);
    _iim_write(dev, IIM_B1_REG_GYRO_CONFIG_STATIC2, 0x00);  // AAF有効(bit1=0), NF有効(bit0=0)
    _iim_write(dev, IIM_B1_REG_GYRO_CONFIG_STATIC3, 6);     // GYRO_AAF_DELT = 6
    _iim_write(dev, IIM_B1_REG_GYRO_CONFIG_STATIC4, 36);    // GYRO_AAF_DELTSQR[7:0] = 36
    _iim_write(dev, IIM_B1_REG_GYRO_CONFIG_STATIC5,
               (10 << 4) | 0x00);   // GYRO_AAF_BITSHIFT=10, GYRO_AAF_DELTSQR[11:8]=0

    // --- Bank2: 加速度AAF ---
    _iim_set_bank(dev, 2);
    // ACCEL_CONFIG_STATIC2: [7:2]=AAF_DELT, [0]=AAF_DIS(0=有効)
    _iim_write(dev, IIM_B2_REG_ACCEL_CONFIG_STATIC2, (6 << 1) | 0x00);  // DELT=6, 有効
    _iim_write(dev, IIM_B2_REG_ACCEL_CONFIG_STATIC3, 36);   // ACCEL_AAF_DELTSQR[7:0]
    _iim_write(dev, IIM_B2_REG_ACCEL_CONFIG_STATIC4,
               (10 << 4) | 0x00);   // ACCEL_AAF_BITSHIFT=10

    // --- Bank0に戻す（必須）---
    _iim_set_bank(dev, 0);

    // --- DATA_READY割り込み設定 ---
    // INT_CONFIG: INT1をプッシュプル、アクティブHigh、パルスモード
    _iim_write(dev, IIM_REG_INT_CONFIG, 0x02);

    // INT_SOURCE0: bit3=UI_DRDY_INT1_EN
    _iim_write(dev, IIM_REG_INT_SOURCE0, 0x08);

    // INT_CONFIG0: DATA_READYをステータス読み出しでクリア
    _iim_write(dev, IIM_REG_INT_CONFIG0, 0x00);

    // --- スケールファクタ ---
    // 2000dps, 16bit: 1 LSB = 2000/32768 = 0.061035 deg/s
    dev->gyro_scale  = 2000.0f / 32768.0f;
    // 16G, 16bit: 1 LSB = (16 * 9.80665) / 32768 m/s^2
    dev->accel_scale = (16.0f * 9.80665f) / 32768.0f;

    dev->accel_offset.x = dev->accel_offset.y = dev->accel_offset.z = 0.0f;
    dev->gyro_offset.x  = dev->gyro_offset.y  = dev->gyro_offset.z  = 0.0f;

    HAL_Delay(5);
    return HAL_OK;
}

/* ============================================================
   DATA_READY チェック
   ============================================================ */

bool IIM42653_IsDataReady(IIM_Handle *dev) {
    uint8_t status = 0;
    // INT_STATUS (0x2D): bit3=DATA_RDY_INT
    _iim_read(dev, IIM_REG_INT_STATUS, &status, 1);
    return (status & 0x08) != 0;
}

/* ============================================================
   キャリブレーション
   ============================================================ */

HAL_StatusTypeDef IIM42653_Calibrate(IIM_Handle *dev, uint16_t samples) {
    double ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    IIM_Data raw;

    for (uint16_t i = 0; i < samples; i++) {
        uint32_t t = HAL_GetTick();
        while (!IIM42653_IsDataReady(dev)) {
            if (HAL_GetTick() - t > 5) break;
        }
        IIM42653_ReadData(dev, &raw);
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
    dev->accel_offset.z = (float)(az / samples) - 9.80665f;

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
 * IIM-42653のレジスタレイアウト（Bank0）:
 *   0x1F ACCEL_DATA_X1 (高バイト)
 *   0x20 ACCEL_DATA_X0 (低バイト)
 *   0x21 ACCEL_DATA_Y1
 *   0x22 ACCEL_DATA_Y0
 *   0x23 ACCEL_DATA_Z1
 *   0x24 ACCEL_DATA_Z0
 *   0x25 GYRO_DATA_X1  (高バイト)
 *   0x26 GYRO_DATA_X0  (低バイト)
 *   ...
 *
 * Big-Endian（ICM-45686と同じ）
 * → (buf[N]<<8 | buf[N+1]) で正しい符号付き16bit値になる
 */
HAL_StatusTypeDef IIM42653_ReadData(IIM_Handle *dev, IIM_Data *out) {
    uint8_t buf[12];

    // ACCEL_DATA_X1(0x1F)から12バイト連続読み出し
    // → Accel 6バイト + Gyro 6バイト
    HAL_StatusTypeDef ret = _iim_read(dev, IIM_REG_ACCEL_DATA_X1, buf, 12);
    if (ret != HAL_OK) return ret;

    // Big-Endian: 高バイト先
    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t raw_gx = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t raw_gy = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gz = (int16_t)((buf[10] << 8) | buf[11]);

    IIM_Data current;
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

        if (da_x > dev->spike_accel_threshold ||
            da_y > dev->spike_accel_threshold ||
            da_z > dev->spike_accel_threshold ||
            dg_x > dev->spike_gyro_threshold  ||
            dg_y > dev->spike_gyro_threshold  ||
            dg_z > dev->spike_gyro_threshold) {
            *out = dev->last_data;
            return HAL_OK;
        }
    }

    dev->last_data = current;
    dev->first_read = false;
    *out = current;
    return HAL_OK;
}
