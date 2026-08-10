/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "spi.h"
#include "i2c.h"
#include "UsbDebug.h"
#include "SecureCommunication.h"
#include "Communication.h"
#include "CommunicationLink.h"
#include "NrfLink.h"
#include "CommunicationDevices.h"
#include "BMP280.h"
#include "BH1750.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMM_FLAG_RADIO_INIT_REQUEST (1u << 0)
#define COMM_FLAG_BATCH_END          (1u << 1)

#define SENSOR_FLAG_RADIO_READY      (1u << 2)
#define SENSOR_FLAG_RADIO_ERROR      (1u << 3)
#define SENSOR_FLAG_TX_COMPLETE      (1u << 4)
#define SENSOR_FLAG_TX_ERROR         (1u << 5)

#define DEVICE_POWER_STABILIZATION_MS   500u
#define SENSOR_CYCLE_DELAY_MS            60000u
#define SENSOR_INIT_RETRY_COUNT              5u
#define SENSOR_INIT_RETRY_DELAY_MS          100u
#define I2C_RESTART_DELAY_MS                 10u
#define BMP280_MEASUREMENT_TIMEOUT_MS       100u
#define RADIO_INIT_TIMEOUT_MS             5000u
#define COMMUNICATION_SESSION_TIMEOUT_MS  5000u
#define TX_COMPLETE_TIMEOUT_MS            6000u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint8_t ToggleLed = 1u;

/*
 * Select this firmware instance from the central device table.
 * currentDevice is Device 0x02, rootDevice is Device 0x01.
 */
static const CommunicationDevice_t * const rootDevice = &communicationDevices[0];
static const CommunicationDevice_t * const currentDevice = &communicationDevices[1];

static SecureCommunication_t secureCommunication;
static NrfLink_t nrfLink;
static Communication_t communication;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommunicationTa */
osThreadId_t CommunicationTaHandle;
const osThreadAttr_t CommunicationTa_attributes = {
  .name = "CommunicationTa",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ReadSensorsTask */
osThreadId_t ReadSensorsTaskHandle;
const osThreadAttr_t ReadSensorsTask_attributes = {
  .name = "ReadSensorsTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void LogCommunicationError(CommunicationStatus_t status);
static bool InitializeSensors(void);
static bool WaitForBmp280MeasurementComplete(void);
static bool QueueMeasurement(SecureMessageType_t messageType, float value, const char *name);
static bool QueueMeasurements(float lux, float temperature, float pressure, float humidity);
static void RestartI2CPeripheral(void);
static void ExternalDevicesPowerOn(void);
static void ExternalDevicesPowerOff(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartCommunicationTask(void *argument);
void StartReadSensorsTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	bmp280.i2c = &hi2c1;               // uchwyt I2C (np. &hi2c1, &hi2c2)
	bmp280.addr = BMP280_I2C_ADDRESS_0; // adres 0x76 (SDO do GND) lub ADDRESS_1 (0x77)
	bmp280_init_default_params(&bmp280.params);
	bmp280.params.mode = BMP280_MODE_FORCED;

	bh1750.hi2c = &hi2c1;
	bh1750.address = BH1750_I2C_ADDRESS;
	bh1750.mode = BH1750_MODE_CONTINUOUS_HIGH_RES;
	bh1750.initialized = false;

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  (void)UsbDebug_CreateOsObjects();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CommunicationTa */
  CommunicationTaHandle = osThreadNew(StartCommunicationTask, NULL, &CommunicationTa_attributes);

  /* creation of ReadSensorsTask */
  ReadSensorsTaskHandle = osThreadNew(StartReadSensorsTask, NULL, &ReadSensorsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

  osDelay(10000u);
  DebugPrintf("Device 0x%02X started\r\n", currentDevice->deviceId);

  uint32_t lastLedToggle = HAL_GetTick();

  for (;;)
  {
    uint32_t now = HAL_GetTick();

    UsbDebug_Process();

    if (ToggleLed != 0u && (uint32_t)(now - lastLedToggle) >= 15000u)
    {
      lastLedToggle = now;
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
      Debug("Mesurment device alive\r\n");
    }

    osDelay(2u);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCommunicationTask */
/**
* @brief Function implementing the CommunicationTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommunicationTask */
void StartCommunicationTask(void *argument)
{
  /* USER CODE BEGIN StartCommunicationTask */
  CommunicationMessage_t receivedMessage;
  CommunicationLink_t linkInterface;
  NRF24_Status_t nrfStatus;

  (void)argument;
  osDelay(10000u);

  /* Software/security layer is initialized only once per MCU boot. */
  if (!SecureCommunication_Init(&secureCommunication, currentDevice->deviceId, currentDevice->deviceKey))
  {
    Debug("SecureCommunication init failed: boot-counter Flash storage (sector 7, 0x08060000)\r\n");
    for (;;)
    {
      osDelay(1000u);
    }
  }

  if (!SecureCommunication_AuthorizePeer(&secureCommunication, rootDevice->deviceId, rootDevice->deviceKey))
  {
    Debug("Secure peer authorization failed\r\n");
    for (;;)
    {
      osDelay(1000u);
    }
  }

  /* Communication queues and routing also live for the whole MCU boot. */
  linkInterface = NrfLink_AsCommunicationLink(&nrfLink);
  if (!Communication_Init(&communication, &secureCommunication, &linkInterface) || !Communication_AddRoute(&communication, rootDevice->deviceId, rootDevice->nrfAddress, NRF_LINK_ADDRESS_SIZE))
  {
    Debug("Communication routing init failed\r\n");
    for (;;)
    {
      osDelay(1000u);
    }
  }

  DebugPrintf("Communication initialized: logical ID=0x%02X\r\n", currentDevice->deviceId);

  for (;;)
  {
    bool batchEnded = false;
    bool txError = false;
    uint32_t sessionStartTick;

    (void)osThreadFlagsWait(COMM_FLAG_RADIO_INIT_REQUEST, osFlagsWaitAny, osWaitForever);
    (void)Communication_ResetTxQueue(&communication);

    nrfStatus = NrfLink_Init(&nrfLink, &hspi1, NRF24_CE_GPIO_Port, NRF24_CE_Pin, SPI1_NRF24_CS_GPIO_Port, SPI1_NRF24_CS_Pin, currentDevice->nrfAddress);
    if (nrfStatus != NRF24_OK)
    {
      DebugPrintf("NRF init failed: %s\r\n", NRF24_StatusToString(nrfStatus));
      osThreadFlagsSet(ReadSensorsTaskHandle, SENSOR_FLAG_RADIO_ERROR);
      continue;
    }

    Debug("NRF initialized for measurement cycle\r\n");
    osThreadFlagsSet(ReadSensorsTaskHandle, SENSOR_FLAG_RADIO_READY);
    sessionStartTick = HAL_GetTick();

    while (true)
    {
      uint32_t pendingFlags = osThreadFlagsGet();

      if ((pendingFlags & COMM_FLAG_BATCH_END) != 0u)
      {
        osThreadFlagsClear(COMM_FLAG_BATCH_END);
        batchEnded = true;
      }

      if (batchEnded && Communication_IsTxQueueEmpty(&communication))
      {
        break;
      }

      if ((HAL_GetTick() - sessionStartTick) >= COMMUNICATION_SESSION_TIMEOUT_MS)
      {
        Debug("Communication session timeout\r\n");
        txError = true;
        (void)Communication_ResetTxQueue(&communication);
        break;
      }

      CommunicationStatus_t status = Communication_Process(&communication);
      if (status != COMMUNICATION_OK && status != COMMUNICATION_IDLE)
      {
        LogCommunicationError(status);
        txError = true;
      }

      while (Communication_TryReceive(&communication, &receivedMessage))
      {
        DebugMessage("RX", receivedMessage.sourceId, receivedMessage.destinationId, (uint8_t)receivedMessage.messageType, receivedMessage.payload, receivedMessage.payloadLength);
      }

      osDelay(1u);
    }

    nrfStatus = NRF24_StopListening(&nrfLink.radio);
    if (nrfStatus != NRF24_OK)
    {
      DebugPrintf("NRF stop listening failed: %s\r\n", NRF24_StatusToString(nrfStatus));
      txError = true;
    }

    uint32_t resultFlags = SENSOR_FLAG_TX_COMPLETE;
    if (txError)
    {
      resultFlags |= SENSOR_FLAG_TX_ERROR;
    }

    osThreadFlagsSet(ReadSensorsTaskHandle, resultFlags);
  }
  /* USER CODE END StartCommunicationTask */
}

/* USER CODE BEGIN Header_StartReadSensorsTask */
/**
* @brief Function implementing the ReadSensorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadSensorsTask */
void StartReadSensorsTask(void *argument)
{
  /* USER CODE BEGIN StartReadSensorsTask */
  (void)argument;

  while (!Communication_IsInitialized(&communication))
  {
    osDelay(100u);
  }

  for (;;)
  {
    float lux;
    uint32_t flags;

    ExternalDevicesPowerOn();

    if (!InitializeSensors())
    {
      ExternalDevicesPowerOff();
      osDelay(SENSOR_CYCLE_DELAY_MS);
      continue;
    }

    lux = BH1750_ReadLux(&bh1750);
    if (lux < 0.0f)
    {
      Debug("BH1750 read error\r\n");
      ExternalDevicesPowerOff();
      osDelay(SENSOR_CYCLE_DELAY_MS);
      continue;
    }

    if (!bmp280_force_measurement(&bmp280))
    {
      Debug("BMP280 force measurement failed\r\n");
      ExternalDevicesPowerOff();
      osDelay(SENSOR_CYCLE_DELAY_MS);
      continue;
    }

    if (!WaitForBmp280MeasurementComplete())
    {
      ExternalDevicesPowerOff();
      osDelay(SENSOR_CYCLE_DELAY_MS);
      continue;
    }

    if (!bmp280_read_float(&bmp280, &BMP280_temperature, &BMP280_pressure, &BMP280_humidity))
    {
      Debug("BMP280 read error\r\n");
      ExternalDevicesPowerOff();
      osDelay(SENSOR_CYCLE_DELAY_MS);
      continue;
    }

    osThreadFlagsSet(CommunicationTaHandle, COMM_FLAG_RADIO_INIT_REQUEST);
    flags = osThreadFlagsWait(SENSOR_FLAG_RADIO_READY | SENSOR_FLAG_RADIO_ERROR, osFlagsWaitAny, RADIO_INIT_TIMEOUT_MS);

    if ((flags & osFlagsError) != 0u)
    {
      Debug("Radio initialization timeout\r\n");
      ExternalDevicesPowerOff();
      osDelay(SENSOR_CYCLE_DELAY_MS);
      continue;
    }

    if ((flags & SENSOR_FLAG_RADIO_ERROR) != 0u)
    {
      Debug("Radio initialization failed for measurement cycle\r\n");
      ExternalDevicesPowerOff();
      osDelay(SENSOR_CYCLE_DELAY_MS);
      continue;
    }

    if (!QueueMeasurements(lux, BMP280_temperature, BMP280_pressure / 100.0f, BMP280_humidity))
    {
      Debug("Not all measurements were queued\r\n");
    }

    osThreadFlagsSet(CommunicationTaHandle, COMM_FLAG_BATCH_END);
    flags = osThreadFlagsWait(SENSOR_FLAG_TX_COMPLETE | SENSOR_FLAG_TX_ERROR, osFlagsWaitAny, TX_COMPLETE_TIMEOUT_MS);

    if ((flags & osFlagsError) != 0u)
    {
      Debug("Communication completion timeout\r\n");
    }
    else if ((flags & SENSOR_FLAG_TX_ERROR) != 0u)
    {
      Debug("Measurement cycle completed with communication error\r\n");
    }
    else
    {
      Debug("Measurement cycle transmitted successfully\r\n");
    }

    ExternalDevicesPowerOff();
    osDelay(SENSOR_CYCLE_DELAY_MS);
  }
  /* USER CODE END StartReadSensorsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static bool InitializeSensors(void)
{
    BH1750_Error bhStatus = BH1750_ERROR_I2C;
    bmp280_params_t bmpParams;
    bool bmpInitialized = false;
    uint32_t attempt;

    bh1750.initialized = false;
    bmp280.initialized = false;
    bmp280_init_default_params(&bmpParams);
    bmpParams.mode = BMP280_MODE_FORCED;

    for (attempt = 1u; attempt <= SENSOR_INIT_RETRY_COUNT; ++attempt)
    {
        bh1750.initialized = false;
        bhStatus = BH1750_Init(&bh1750);
        if (bhStatus == BH1750_OK)
        {
            break;
        }

        DebugPrintf("BH1750 init failed attempt %lu/%lu: err=%u I2Cerr=0x%08lX state=%lu\r\n",
                    (unsigned long)attempt,
                    (unsigned long)SENSOR_INIT_RETRY_COUNT,
                    (unsigned int)bhStatus,
                    (unsigned long)HAL_I2C_GetError(&hi2c1),
                    (unsigned long)HAL_I2C_GetState(&hi2c1));

        if (attempt < SENSOR_INIT_RETRY_COUNT)
        {
            RestartI2CPeripheral();
            osDelay(SENSOR_INIT_RETRY_DELAY_MS);
        }
    }

    if (bhStatus != BH1750_OK)
    {
        return false;
    }
    Debug("BH1750 initialized successfully\r\n");

    for (attempt = 1u; attempt <= SENSOR_INIT_RETRY_COUNT; ++attempt)
    {
        bmp280.initialized = false;
        if (bmp280_init(&bmp280, &bmpParams))
        {
            bmpInitialized = true;
            break;
        }

        DebugPrintf("BMP280 init failed attempt %lu/%lu: I2Cerr=0x%08lX state=%lu\r\n",
                    (unsigned long)attempt,
                    (unsigned long)SENSOR_INIT_RETRY_COUNT,
                    (unsigned long)HAL_I2C_GetError(&hi2c1),
                    (unsigned long)HAL_I2C_GetState(&hi2c1));

        if (attempt < SENSOR_INIT_RETRY_COUNT)
        {
            RestartI2CPeripheral();
            osDelay(SENSOR_INIT_RETRY_DELAY_MS);
        }
    }

    if (!bmpInitialized)
    {
        return false;
    }
    Debug("BMP280 initialized successfully\r\n");
    DebugPrintf("BMP280 calib: ID=0x%02X T1=%u P1=%u H1=%u\r\n", (unsigned int)bmp280.id, (unsigned int)bmp280.dig_T1, (unsigned int)bmp280.dig_P1, (unsigned int)bmp280.dig_H1);

    return true;
}

static bool WaitForBmp280MeasurementComplete(void)
{
    bool isMeasuring = false;
    bool measurementStarted = false;
    uint32_t startTick = HAL_GetTick();

    for (;;)
    {
        if (!bmp280_get_measuring_status(&bmp280, &isMeasuring))
        {
            Debug("BMP280 status read error\r\n");
            return false;
        }

        if (isMeasuring)
        {
            measurementStarted = true;
        }
        else if (measurementStarted)
        {
            return true;
        }

        if ((HAL_GetTick() - startTick) >= BMP280_MEASUREMENT_TIMEOUT_MS)
        {
            Debug("BMP280 measurement timeout\r\n");
            return false;
        }

        osDelay(1u);
    }
}

static bool QueueMeasurement(SecureMessageType_t messageType, float value, const char *name)
{
    char buffer[32];
    int length = snprintf(buffer, sizeof(buffer), "%.2f", value);

    if (length <= 0 || length >= (int)sizeof(buffer))
    {
        DebugPrintf("Failed to format %s\r\n", name);
        return false;
    }

    if (!Communication_Send(&communication, rootDevice->deviceId, messageType, buffer, (uint16_t)length))
    {
        DebugPrintf("Failed to queue %s\r\n", name);
        return false;
    }

    DebugMessage(name, currentDevice->deviceId, rootDevice->deviceId, (uint8_t)messageType, (const uint8_t *)buffer, (uint16_t)length);
    return true;
}

static bool QueueMeasurements(float lux, float temperature, float pressure, float humidity)
{
    bool result = true;

    if (!QueueMeasurement(MESSAGE_TYPE_LIGHT_MESURMENT, lux, "TX QUEUED Lux"))
    {
        result = false;
    }
    if (!QueueMeasurement(MESSAGE_TYPE_TEMPERATURE, temperature, "TX QUEUED Temperature"))
    {
        result = false;
    }
    if (!QueueMeasurement(MESSAGE_TYPE_PRESSURE, pressure, "TX QUEUED Pressure"))
    {
        result = false;
    }
    if (!QueueMeasurement(MESSAGE_TYPE_HUMIDITY, humidity, "TX QUEUED Humidity"))
    {
        result = false;
    }

    return result;
}

static void RestartI2CPeripheral(void)
{
    (void)HAL_I2C_DeInit(&hi2c1);
    osDelay(I2C_RESTART_DELAY_MS);
    MX_I2C1_Init();
}

static void ExternalDevicesPowerOn(void)
{
    /* Leave all bus pins high-impedance while the switched rail is off. */
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SPI1_NRF24_CS_GPIO_Port, SPI1_NRF24_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_DeInit(&hspi1);
    (void)HAL_I2C_DeInit(&hi2c1);

    HAL_GPIO_WritePin(DEV_SHUTDOWN_SW_GPIO_Port, DEV_SHUTDOWN_SW_Pin, GPIO_PIN_SET);
    Debug("External devices power ON\r\n");
    osDelay(DEVICE_POWER_STABILIZATION_MS);

    /* Restore MCU interfaces only after the external rail is stable. */
    MX_I2C1_Init();
    MX_SPI1_Init();
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SPI1_NRF24_CS_GPIO_Port, SPI1_NRF24_CS_Pin, GPIO_PIN_SET);
}

static void ExternalDevicesPowerOff(void)
{
    bh1750.initialized = false;
    bmp280.initialized = false;

    /* Prevent back-powering the unpowered nRF/sensors through MCU signal pins. */
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SPI1_NRF24_CS_GPIO_Port, SPI1_NRF24_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_DeInit(&hspi1);
    (void)HAL_I2C_DeInit(&hi2c1);

    HAL_GPIO_WritePin(DEV_SHUTDOWN_SW_GPIO_Port, DEV_SHUTDOWN_SW_Pin, GPIO_PIN_RESET);
    Debug("External devices power OFF\r\n");
}

static void LogCommunicationError(CommunicationStatus_t status)
{
    if (status == COMMUNICATION_PROCESS_LINK_ERROR)
    {
        DebugPrintf("Communication link error: %s\r\n",
                    CommunicationLink_StatusToString(
                        Communication_GetLastLinkStatus(&communication)));
    }
    else if (status == COMMUNICATION_SECURE_ERROR)
    {
        SecureTransportStatus_t secureStatus =
            Communication_GetLastSecureStatus(&communication);

        DebugPrintf("Secure communication error: %s",
                    SecureTransport_StatusToString(secureStatus));

        if (secureStatus == SECURE_TRANSPORT_PROTOCOL_ERROR)
        {
            DebugPrintf(" / %s",
                        SecureProtocol_StatusToString(
                            SecureCommunication_GetLastProtocolStatus(
                                &secureCommunication)));
        }

        Debug("\r\n");
    }
    else
    {
        DebugPrintf("Communication error: %s\r\n",
                    Communication_StatusToString(status));
    }
}

/* USER CODE END Application */

