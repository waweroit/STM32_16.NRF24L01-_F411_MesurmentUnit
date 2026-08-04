/**
 * Ciastkolog.pl (https://github.com)
 * 
 * MIT License (MIT) - Copyright (c) 2016 sheinz
 */
#include "BMP280.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

/* Rejestry BMP280/BME280 */
#define BMP280_REG_CALIB_TP       0x88U
#define BME280_REG_CALIB_H1       0xA1U
#define BME280_REG_CALIB_H2       0xE1U
#define BMP280_REG_ID             0xD0U
#define BMP280_REG_RESET          0xE0U
#define BMP280_REG_CTRL_HUM       0xF2U
#define BMP280_REG_STATUS         0xF3U
#define BMP280_REG_CTRL_MEAS      0xF4U
#define BMP280_REG_CONFIG         0xF5U
#define BMP280_REG_DATA           0xF7U

#define BMP280_RESET_VALUE        0xB6U
#define BMP280_STATUS_IM_UPDATE   0x01U
#define BMP280_STATUS_MEASURING   0x08U
#define BMP280_MODE_MASK          0x03U

#define BMP280_I2C_TIMEOUT_MS     25U
#define BMP280_RESET_TIMEOUT_MS   150U
#define BMP280_STATUS_POLL_MS     2U

#define BMP280_CALIB_TP_SIZE      24U
#define BME280_CALIB_H2_SIZE      7U

float tempCalibration     = 0.0f;
float pressureCalibration = 0.0f;
float humidityCalibration = 0.0f;
float altitudeCalibration = 0.0f;

BMP280_HandleTypedef bmp280;
float BMP280_temperature, BMP280_pressure, BMP280_humidity;

static uint16_t bmp280_i2c_address(const BMP280_HandleTypedef *dev)
{
    return (uint16_t)(dev->addr << 1U);
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int16_t read_s16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

static bool bmp280_read_registers(BMP280_HandleTypedef *dev, uint8_t reg,
                                  uint8_t *data, uint16_t length)
{
    if (dev == NULL || dev->i2c == NULL || data == NULL || length == 0U) {
        return false;
    }

    return HAL_I2C_Mem_Read(dev->i2c, bmp280_i2c_address(dev), reg,
                            I2C_MEMADD_SIZE_8BIT, data, length,
                            BMP280_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool bmp280_write_u8(BMP280_HandleTypedef *dev, uint8_t reg, uint8_t value)
{
    if (dev == NULL || dev->i2c == NULL) {
        return false;
    }

    return HAL_I2C_Mem_Write(dev->i2c, bmp280_i2c_address(dev), reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1U,
                             BMP280_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool bmp280_read_u8(BMP280_HandleTypedef *dev, uint8_t reg, uint8_t *value)
{
    return bmp280_read_registers(dev, reg, value, 1U);
}

static bool bmp280_read_calibration(BMP280_HandleTypedef *dev)
{
    uint8_t data[BMP280_CALIB_TP_SIZE];

    if (!bmp280_read_registers(dev, BMP280_REG_CALIB_TP, data, sizeof(data))) {
        return false;
    }

    dev->dig_T1 = read_u16_le(&data[0]);
    dev->dig_T2 = read_s16_le(&data[2]);
    dev->dig_T3 = read_s16_le(&data[4]);
    dev->dig_P1 = read_u16_le(&data[6]);
    dev->dig_P2 = read_s16_le(&data[8]);
    dev->dig_P3 = read_s16_le(&data[10]);
    dev->dig_P4 = read_s16_le(&data[12]);
    dev->dig_P5 = read_s16_le(&data[14]);
    dev->dig_P6 = read_s16_le(&data[16]);
    dev->dig_P7 = read_s16_le(&data[18]);
    dev->dig_P8 = read_s16_le(&data[20]);
    dev->dig_P9 = read_s16_le(&data[22]);

    return true;
}

static bool bmp280_read_humidity_calibration(BMP280_HandleTypedef *dev)
{
    uint8_t data[BME280_CALIB_H2_SIZE];

    if (!bmp280_read_u8(dev, BME280_REG_CALIB_H1, &dev->dig_H1) ||
        !bmp280_read_registers(dev, BME280_REG_CALIB_H2, data, sizeof(data))) {
        return false;
    }

    dev->dig_H2 = read_s16_le(&data[0]);
    dev->dig_H3 = data[2];
    dev->dig_H4 = (int16_t)(((int16_t)(int8_t)data[3] << 4) | (data[4] & 0x0FU));
    dev->dig_H5 = (int16_t)(((int16_t)(int8_t)data[5] << 4) | (data[4] >> 4));
    dev->dig_H6 = (int8_t)data[6];

    return true;
}

static bool bmp280_detect_device(BMP280_HandleTypedef *dev)
{
    if (!bmp280_read_u8(dev, BMP280_REG_ID, &dev->id)) {
        return false;
    }

    return dev->id == BMP280_CHIP_ID || dev->id == BME280_CHIP_ID;
}

static bool bmp280_reset_device(BMP280_HandleTypedef *dev)
{
    uint32_t startTick;

    if (!bmp280_write_u8(dev, BMP280_REG_RESET, BMP280_RESET_VALUE)) {
        return false;
    }

    startTick = HAL_GetTick();
    for (;;) {
        uint8_t status;

        if (!bmp280_read_u8(dev, BMP280_REG_STATUS, &status)) {
            return false;
        }

        if ((status & BMP280_STATUS_IM_UPDATE) == 0U) {
            return true;
        }

        if ((HAL_GetTick() - startTick) >= BMP280_RESET_TIMEOUT_MS) {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(BMP280_STATUS_POLL_MS));
    }
}

static bool bmp280_apply_configuration(BMP280_HandleTypedef *dev,
                                       const bmp280_params_t *params)
{
    BMP280_Mode initialMode = params->mode;
    uint8_t config;
    uint8_t ctrlMeas;

    if (initialMode == BMP280_MODE_FORCED) {
        initialMode = BMP280_MODE_SLEEP;
    }

    config = (uint8_t)(((uint8_t)params->standby << 5U) |
                       ((uint8_t)params->filter << 2U));

    if (!bmp280_write_u8(dev, BMP280_REG_CONFIG, config)) {
        return false;
    }

    if (dev->id == BME280_CHIP_ID &&
        !bmp280_write_u8(dev, BMP280_REG_CTRL_HUM,
                         (uint8_t)params->oversampling_humidity)) {
        return false;
    }

    ctrlMeas = (uint8_t)(((uint8_t)params->oversampling_temperature << 5U) |
                         ((uint8_t)params->oversampling_pressure << 2U) |
                         (uint8_t)initialMode);

    return bmp280_write_u8(dev, BMP280_REG_CTRL_MEAS, ctrlMeas);
}

void bmp280_init_default_params(bmp280_params_t *params)
{
    if (params == NULL) {
        return;
    }

    params->mode = BMP280_MODE_NORMAL;
    params->filter = BMP280_FILTER_OFF;
    params->oversampling_pressure = BMP280_STANDARD;
    params->oversampling_temperature = BMP280_STANDARD;
    params->oversampling_humidity = BMP280_STANDARD;
    params->standby = BMP280_STANDBY_250;
}

bool bmp280_init(BMP280_HandleTypedef *dev, const bmp280_params_t *params)
{
    if (dev == NULL || params == NULL || dev->i2c == NULL) {
        return false;
    }

    if (dev->addr != BMP280_I2C_ADDRESS_0 && dev->addr != BMP280_I2C_ADDRESS_1) {
        return false;
    }

    if (!bmp280_detect_device(dev) || !bmp280_reset_device(dev) ||
        !bmp280_read_calibration(dev)) {
        return false;
    }

    if (dev->id == BME280_CHIP_ID && !bmp280_read_humidity_calibration(dev)) {
        return false;
    }

    dev->params = *params;
    dev->initialized = bmp280_apply_configuration(dev, params);
    return dev->initialized;
}

bool bmp280_force_measurement(BMP280_HandleTypedef *dev)
{
    uint8_t ctrlMeas;

    if (dev == NULL || !bmp280_read_u8(dev, BMP280_REG_CTRL_MEAS, &ctrlMeas)) {
        return false;
    }

    ctrlMeas = (uint8_t)((ctrlMeas & (uint8_t)~BMP280_MODE_MASK) |
                         (uint8_t)BMP280_MODE_FORCED);

    return bmp280_write_u8(dev, BMP280_REG_CTRL_MEAS, ctrlMeas);
}

bool bmp280_get_measuring_status(BMP280_HandleTypedef *dev, bool *is_measuring)
{
    uint8_t status;

    if (dev == NULL || is_measuring == NULL ||
        !bmp280_read_u8(dev, BMP280_REG_STATUS, &status)) {
        return false;
    }

    *is_measuring = (status & BMP280_STATUS_MEASURING) != 0U;
    return true;
}

bool bmp280_is_measuring(BMP280_HandleTypedef *dev)
{
    bool isMeasuring = false;

    if (!bmp280_get_measuring_status(dev, &isMeasuring)) {
        return false;
    }

    return isMeasuring;
}

static inline int32_t compensate_temperature(BMP280_HandleTypedef *dev, int32_t adc_temp, int32_t *fine_temp)
{
    int32_t var1, var2;

    var1 = ((((adc_temp >> 3) - ((int32_t)dev->dig_T1 << 1))) * (int32_t)dev->dig_T2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t)dev->dig_T1) * ((adc_temp >> 4) - (int32_t)dev->dig_T1)) >> 12) * (int32_t)dev->dig_T3) >> 14;

    *fine_temp = var1 + var2;
    return (*fine_temp * 5 + 128) >> 8;
}

static inline uint32_t compensate_pressure(BMP280_HandleTypedef *dev, int32_t adc_press, int32_t fine_temp)
{
    int64_t var1, var2, p;

    var1 = (int64_t)fine_temp - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dev->dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dev->dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)dev->dig_P7 << 4);

    return p;
}

static inline uint32_t compensate_humidity(BMP280_HandleTypedef *dev, int32_t adc_hum, int32_t fine_temp)
{
    int32_t v_x1_u32r;

    // 1. Wyliczenie wartości bazowej z uwzględnieniem temperatury fine_temp
    v_x1_u32r = (fine_temp - ((int32_t)76800));

    // 2. Oficjalny, wieloetapowy wzór kompensacji Bosch dla BME280
    v_x1_u32r = (((((adc_hum << 14) - (((int32_t)dev->dig_H4) << 20) -
                    (((int32_t)dev->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)dev->dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)dev->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)dev->dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)dev->dig_H1)) >> 4));

    // 3. Zabezpieczenie przed przekroczeniem fizycznych barier (poniżej 0%)
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);

    // 4. Zabezpieczenie przed przekroczeniem fizycznych barier (powyżej 100%)
    //    Wartość 419430400 w matematyce stałoprzecinkowej Bosch odpowiada dokładnie 100% wilgotności
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    // 5. Zwrócenie wyniku przesuniętego o 12 bitów
    return (uint32_t)(v_x1_u32r >> 12);
}


bool bmp280_read_fixed(BMP280_HandleTypedef *dev, int32_t *temperature,
                       uint32_t *pressure, uint32_t *humidity)
{
    uint8_t data[8];
    uint16_t dataLength;
    int32_t adcPressure;
    int32_t adcTemperature;
    int32_t fineTemperature;

    if (dev == NULL || temperature == NULL || pressure == NULL) {
        return false;
    }

    if (dev->id != BME280_CHIP_ID) {
        if (humidity != NULL) {
            *humidity = 0U;
        }
        humidity = NULL;
    }

    dataLength = humidity != NULL ? 8U : 6U;
    if (!bmp280_read_registers(dev, BMP280_REG_DATA, data, dataLength)) {
        return false;
    }

    adcPressure = (int32_t)(((uint32_t)data[0] << 12U) |
                            ((uint32_t)data[1] << 4U) |
                            ((uint32_t)data[2] >> 4U));
    adcTemperature = (int32_t)(((uint32_t)data[3] << 12U) |
                               ((uint32_t)data[4] << 4U) |
                               ((uint32_t)data[5] >> 4U));

    *temperature = compensate_temperature(dev, adcTemperature, &fineTemperature);
    *pressure = compensate_pressure(dev, adcPressure, fineTemperature);

    if (humidity != NULL) {
        int32_t adcHumidity = (int32_t)(((uint16_t)data[6] << 8U) | data[7]);
        *humidity = compensate_humidity(dev, adcHumidity, fineTemperature);
    }

    return true;
}

bool bmp280_read_float(BMP280_HandleTypedef *dev, float *temperature,
                       float *pressure, float *humidity)
{
    int32_t fixedTemperature;
    uint32_t fixedPressure;
    uint32_t fixedHumidity;

    if (dev == NULL || temperature == NULL || pressure == NULL) {
        return false;
    }

    if (!bmp280_read_fixed(dev, &fixedTemperature, &fixedPressure,
                           humidity != NULL ? &fixedHumidity : NULL)) {
        return false;
    }

    *temperature = (float)fixedTemperature / 100.0f;
    *pressure = (float)fixedPressure / 256.0f;

    if (humidity != NULL) {
        *humidity = (float)fixedHumidity / 1024.0f;
    }

    return true;
}
