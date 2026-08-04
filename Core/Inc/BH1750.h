/**
 * Ciastkolog.pl (https://github.com)
 * 
 * MIT License (MIT) - Copyright (c) 2016 sheinz
 * 
 * BH1750 I2C Digital Light Sensor Driver
 * Compatible with STM32 HAL and FreeRTOS
 */
#ifndef __BH1750_H__
#define __BH1750_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

/**
 * FreeRTOS Includes - required for proper task sleeping
 */
#include "FreeRTOS.h"
#include "task.h"

/* BH1750 I2C Address */
#define BH1750_I2C_ADDRESS_LOW   0x23U  /* ADDR pin to GND */
#define BH1750_I2C_ADDRESS_HIGH  0x5CU  /* ADDR pin to VCC */
#define BH1750_I2C_ADDRESS       BH1750_I2C_ADDRESS_LOW

/* BH1750 Commands */
#define BH1750_CMD_POWER_OFF          0x00
#define BH1750_CMD_POWER_ON           0x01
#define BH1750_CMD_RESET              0x07

/* BH1750 Measurement Modes */
#define BH1750_CMD_CONTINUOUS_H_RES   0x10  /* Continuous measurement mode (1 lx resolution, 120 ms) */
#define BH1750_CMD_CONTINUOUS_H_RES2  0x11  /* Continuous measurement mode (0.5 lx resolution, 120 ms) */
#define BH1750_CMD_CONTINUOUS_L_RES   0x13  /* Continuous measurement mode (4 lx resolution, 16 ms, low sensitivity) */
#define BH1750_CMD_ONE_TIME_H_RES     0x20  /* One time measurement mode (1 lx resolution, 120 ms) */
#define BH1750_CMD_ONE_TIME_H_RES2    0x21  /* One time measurement mode (0.5 lx resolution, 120 ms) */
#define BH1750_CMD_ONE_TIME_L_RES     0x23  /* One time measurement mode (4 lx resolution, 16 ms, low sensitivity) */

/* Default measurement mode */
#define BH1750_DEFAULT_MODE           BH1750_CMD_CONTINUOUS_H_RES

/* Timing constants (in milliseconds) */
#define BH1750_MEASUREMENT_TIME_MS    120   /* Time needed for a measurement in High-Resolution mode */
#define BH1750_INIT_DELAY_MS          5     /* Delay after power-on/reset */
#define BH1750_I2C_TIMEOUT            1000  /* I2C operation timeout in ms */

/* Measurement mode enumeration */
typedef enum {
    BH1750_MODE_CONTINUOUS_HIGH_RES   = BH1750_CMD_CONTINUOUS_H_RES,
    BH1750_MODE_CONTINUOUS_HIGH_RES_2 = BH1750_CMD_CONTINUOUS_H_RES2,
    BH1750_MODE_CONTINUOUS_LOW_RES    = BH1750_CMD_CONTINUOUS_L_RES,
    BH1750_MODE_ONE_TIME_HIGH_RES     = BH1750_CMD_ONE_TIME_H_RES,
    BH1750_MODE_ONE_TIME_HIGH_RES_2   = BH1750_CMD_ONE_TIME_H_RES2,
    BH1750_MODE_ONE_TIME_LOW_RES      = BH1750_CMD_ONE_TIME_L_RES
} BH1750_Mode;

/* Return codes */
typedef enum {
    BH1750_OK = 0,
    BH1750_ERROR_I2C,
    BH1750_ERROR_TIMEOUT,
    BH1750_ERROR_INVALID_PARAM
} BH1750_Error;

/* BH1750 device handle */
typedef struct {
    I2C_HandleTypeDef *hi2c;            /* I2C peripheral handle (e.g., &hi2c1) */
    uint8_t address;                    /* 7-bit I2C address */
    BH1750_Mode mode;                   /* Current measurement mode */
    uint16_t conversionTimeMs;          /* Conversion time in ms for current mode */
    bool initialized;                   /* Initialization state flag */
} BH1750_HandleTypedef;

/* External instance */
extern BH1750_HandleTypedef bh1750;

/* Function declarations */

/**
 * @brief  Initialize the BH1750 sensor.
 *         Powers on the device, resets it, and sets the measurement mode.
 * @param  dev: Pointer to the BH1750 device handle.
 * @retval true on success, false on error.
 */
BH1750_Error BH1750_Init(BH1750_HandleTypedef *dev);

/**
 * @brief  Read the illuminance value from the BH1750 sensor.
 *         Waits for conversion, reads 2 bytes, and calculates lux = raw / 1.2.
 * @param  dev: Pointer to the BH1750 device handle.
 * @return float: Illuminance in lux. Returns -1.0f on error.
 */
float BH1750_ReadLux(BH1750_HandleTypedef *dev);

/**
 * @brief  Read illuminance in lux from BH1750 (integer version).
 * @param  dev: Pointer to the BH1750 device handle.
 * @return int: Illuminance in lux. Returns -1 on error.
 */
int BH1750_ReadLuxInt(BH1750_HandleTypedef *dev);

/**
 * @brief  Power off the BH1750 sensor.
 * @param  dev: Pointer to the BH1750 device handle.
 * @retval true if successful, false otherwise.
 */
BH1750_Error BH1750_PowerOff(BH1750_HandleTypedef *dev);

/**
 * @brief  Reset the BH1750 sensor (resets measurement timing register).
 * @param  dev: Pointer to the BH1750 device handle.
 * @retval true if successful, false otherwise.
 */
BH1750_Error BH1750_Reset(BH1750_HandleTypedef *dev);

/**
 * @brief  Set the measurement mode and update conversion time.
 * @param  dev:  Pointer to the BH1750 device handle.
 * @param  mode: Measurement mode to set.
 * @retval true if successful, false otherwise.
 */
BH1750_Error BH1750_SetMode(BH1750_HandleTypedef *dev, BH1750_Mode mode);

#endif  // __BH1750_H__
