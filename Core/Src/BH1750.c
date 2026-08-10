/**
 * Ciastkolog.pl (https://github.com)
 *
 * MIT License (MIT) - Copyright (c) 2016 sheinz
 *
 * BH1750 I2C Digital Light Sensor Driver for STM32 HAL + FreeRTOS
 *
 * BH1750 Protocol:
 *   1. Send command byte (0x10 = Continuous H-Resolution, 0x11 = Continuous Resolution)
 *   2. Wait for conversion time (typically 120ms for H-Resolution mode)
 *   3. Read 2 bytes of data (MSB first)
 *   4. Calculate lux = data / 1.2
 */

#include "BH1750.h"

/* BH1750 I2C device handle */
BH1750_HandleTypedef bh1750;

/**
 * @brief  Write a single command byte to the BH1750.
 *         The BH1750 does not use internal register addresses;
 *         it accepts a single command byte directly.
 * @param  dev:      Pointer to BH1750 device handle.
 * @param  command:  Command byte to send.
 * @retval BH1750_Error: BH1750_OK on success, error code otherwise.
 */
static BH1750_Error BH1750_WriteCommand(BH1750_HandleTypedef *dev, uint8_t command)
{
    HAL_StatusTypeDef status;

    if (dev == NULL || dev->hi2c == NULL) {
        return BH1750_ERROR_INVALID_PARAM;
    }

    status = HAL_I2C_Master_Transmit(
        dev->hi2c,
        (uint16_t)(dev->address << 1U),
        &command,
        1U,
        BH1750_I2C_TIMEOUT
    );

    return (status == HAL_OK) ? BH1750_OK : BH1750_ERROR_I2C;
}

/**
 * @brief  Read raw 16-bit light data from BH1750.
 * @param  dev:   Pointer to BH1750 device handle.
 * @param  data:  Pointer to store the 16-bit raw value (MSB first).
 * @retval BH1750_Error: BH1750_OK on success, error code otherwise.
 */
static BH1750_Error BH1750_ReadRawData(BH1750_HandleTypedef *dev, uint16_t *data)
{
    uint8_t rawData[2];
    HAL_StatusTypeDef status;

    if (dev == NULL || data == NULL || dev->hi2c == NULL) {
        return BH1750_ERROR_INVALID_PARAM;
    }

    /* BH1750 returns 2 bytes: MSB first, then LSB */
    status = HAL_I2C_Master_Receive(
        dev->hi2c,
        (uint16_t)(dev->address << 1U),
        rawData,
        2U,
        BH1750_I2C_TIMEOUT
    );

    if (status != HAL_OK) {
        return BH1750_ERROR_I2C;
    }

    /* Combine MSB and LSB into 16-bit value */
    *data = ((uint16_t)rawData[0] << 8) | (uint16_t)rawData[1];
    return BH1750_OK;
}

/**
 * @brief  Initialize the BH1750 sensor.
 *         Powers on the device, resets it, and sets the measurement mode.
 * @param  dev: Pointer to the BH1750 device handle.
 * @retval BH1750_Error: BH1750_OK on success, error code otherwise.
 */
BH1750_Error BH1750_Init(BH1750_HandleTypedef *dev)
{
    BH1750_Error err;

    if (dev == NULL || dev->hi2c == NULL) {
        return BH1750_ERROR_INVALID_PARAM;
    }

    /* Set default mode if not already set */
    if (dev->mode == 0) {
        dev->mode = BH1750_DEFAULT_MODE;
    }

    /* Power On the sensor */
    err = BH1750_WriteCommand(dev, BH1750_CMD_POWER_ON);
    if (err != BH1750_OK) {
        return err;
    }
    /* Brief delay after power-on (sensor needs ~3ms to stabilize). */
    vTaskDelay(pdMS_TO_TICKS(BH1750_INIT_DELAY_MS));

    /* Reset the sensor. */
    err = BH1750_WriteCommand(dev, BH1750_CMD_RESET);
    if (err != BH1750_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(BH1750_INIT_DELAY_MS));

    /* Set measurement mode */
    err = BH1750_SetMode(dev, dev->mode);
    if (err != BH1750_OK) {
        return err;
    }

    dev->initialized = true;
    return BH1750_OK;
}

/**
 * @brief  Read the illuminance value from the BH1750 sensor.
 *         Waits for conversion, reads 2 bytes, and calculates lux = data / 1.2.
 * @param  dev: Pointer to the BH1750 device handle.
 * @retval float: Illuminance in lux. Returns -1.0f on error.
 */
float BH1750_ReadLux(BH1750_HandleTypedef *dev)
{
    uint16_t rawData;
    BH1750_Error err;

    if (dev == NULL || !dev->initialized) {
        return -1.0f;
    }

    /* Wait for conversion to complete based on the selected mode */
    vTaskDelay(pdMS_TO_TICKS(dev->conversionTimeMs));

    /* Read raw 16-bit data from sensor */
    err = BH1750_ReadRawData(dev, &rawData);
    if (err != BH1750_OK) {
        return -1.0f;
    }

    /* Calculate lux: Lux = RawData / 1.2 */
    float lux = (float)rawData / 1.2f;

    return lux;
}

/**
 * @brief  Read illuminance in lux from BH1750 (integer version).
 * @param  dev: Pointer to the BH1750 device handle.
 * @retval int: Illuminance in lux. Returns -1 on error.
 */
int BH1750_ReadLuxInt(BH1750_HandleTypedef *dev)
{
    float lux = BH1750_ReadLux(dev);

    if (lux < 0.0f) {
        return -1;
    }

    return (int)lux;
}

/**
 * @brief  Reset the BH1750 data register.
 *         The device must be powered on before issuing the reset command.
 * @param  dev: Pointer to the BH1750 device handle.
 * @retval BH1750_Error: BH1750_OK on success, error code otherwise.
 */
BH1750_Error BH1750_Reset(BH1750_HandleTypedef *dev)
{
    BH1750_Error err;

    if (dev == NULL || dev->hi2c == NULL) {
        return BH1750_ERROR_INVALID_PARAM;
    }

    err = BH1750_WriteCommand(dev, BH1750_CMD_POWER_ON);
    if (err != BH1750_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(BH1750_INIT_DELAY_MS));
    return BH1750_WriteCommand(dev, BH1750_CMD_RESET);
}

/**
 * @brief  Power off the BH1750 sensor.
 * @param  dev: Pointer to the BH1750 device handle.
 * @retval BH1750_Error: BH1750_OK on success, error code otherwise.
 */
BH1750_Error BH1750_PowerOff(BH1750_HandleTypedef *dev)
{
    if (dev == NULL) {
        return BH1750_ERROR_INVALID_PARAM;
    }

    dev->initialized = false;
    return BH1750_WriteCommand(dev, BH1750_CMD_POWER_OFF);
}

/**
 * @brief  Set the measurement mode of the BH1750 sensor.
 * @param  dev:  Pointer to the BH1750 device handle.
 * @param  mode: One of BH1750_CMD_* constants.
 * @retval BH1750_Error: BH1750_OK on success, error code otherwise.
 */
BH1750_Error BH1750_SetMode(BH1750_HandleTypedef *dev, BH1750_Mode mode)
{
    BH1750_Error err;

    if (dev == NULL) {
        return BH1750_ERROR_INVALID_PARAM;
    }

    /* Validate mode */
    if (mode != BH1750_CMD_CONTINUOUS_H_RES &&
        mode != BH1750_CMD_CONTINUOUS_H_RES2 &&
        mode != BH1750_CMD_CONTINUOUS_L_RES &&
        mode != BH1750_CMD_ONE_TIME_H_RES &&
        mode != BH1750_CMD_ONE_TIME_H_RES2 &&
        mode != BH1750_CMD_ONE_TIME_L_RES) {
        return BH1750_ERROR_INVALID_PARAM;
    }

    err = BH1750_WriteCommand(dev, mode);
    if (err != BH1750_OK) {
        return err;
    }

    /* Update conversion time based on mode */
    switch (mode) {
        case BH1750_CMD_CONTINUOUS_H_RES:
        case BH1750_CMD_CONTINUOUS_H_RES2:
        case BH1750_CMD_ONE_TIME_H_RES:
        case BH1750_CMD_ONE_TIME_H_RES2:
            dev->conversionTimeMs = 180U;
            break;
        case BH1750_CMD_CONTINUOUS_L_RES:
        case BH1750_CMD_ONE_TIME_L_RES:
            dev->conversionTimeMs = 16U;
            break;
        default:
            dev->conversionTimeMs = 180U;
            break;
    }

    dev->mode = mode;
    return BH1750_OK;
}
