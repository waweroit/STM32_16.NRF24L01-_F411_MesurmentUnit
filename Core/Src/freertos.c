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
#include "UsbDebug.h"
#include "SecureCommunication.h"
#include "Communication.h"
#include "CommunicationLink.h"
#include "NrfLink.h"
#include "CommunicationDevices.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
  uint32_t lastLedToggle = HAL_GetTick();
  (void)argument;

  DebugPrintf("Device 0x%02X started\r\n", currentDevice->deviceId);

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

  /* 1. Software/security layer: logical Device ID + key. */
  if (!SecureCommunication_Init(&secureCommunication,
                                currentDevice->deviceId,
                                currentDevice->deviceKey))
  {
    Debug("SecureCommunication init failed\r\n");
    for (;;)
    {
      osDelay(1000u);
    }
  }

  /* Authorize the selected peer with that device's individual key. */
  if (!SecureCommunication_AuthorizePeer(&secureCommunication,
										  rootDevice->deviceId,
										  rootDevice->deviceKey))
  {
    Debug("Secure peer authorization failed\r\n");
    for (;;)
    {
      osDelay(1000u);
    }
  }

  /* 2. Hardware/link layer: SPI + GPIO + physical nRF24 address only. */
  nrfStatus = NrfLink_Init(&nrfLink,
                           &hspi1,
                           NRF24_CE_GPIO_Port,
                           NRF24_CE_Pin,
                           SPI1_NRF24_CS_GPIO_Port,
                           SPI1_NRF24_CS_Pin,
                           currentDevice->nrfAddress);
  if (nrfStatus != NRF24_OK)
  {
    DebugPrintf("NRF init failed: %s\r\n", NRF24_StatusToString(nrfStatus));
    for (;;)
    {
      osDelay(1000u);
    }
  }

  /* 3. Generic routing joins SecureCommunication with the selected HW link. */
  linkInterface = NrfLink_AsCommunicationLink(&nrfLink);
  if (!Communication_Init(&communication, &secureCommunication, &linkInterface) ||
      !Communication_AddRoute(&communication,
                                rootDevice->deviceId,
                                rootDevice->nrfAddress,
                                NRF_LINK_ADDRESS_SIZE))
  {
    Debug("Communication routing init failed\r\n");
    for (;;)
    {
      osDelay(1000u);
    }
  }

  DebugPrintf("Communication initialized: logical ID=0x%02X\r\n",
              currentDevice->deviceId);

  for (;;)
  {
    CommunicationStatus_t status = Communication_Process(&communication);

    if (status != COMMUNICATION_OK && status != COMMUNICATION_IDLE)
    {
      LogCommunicationError(status);
    }

    while (Communication_TryReceive(&communication, &receivedMessage))
    {
      DebugMessage("RX",
                   receivedMessage.sourceId,
                   receivedMessage.destinationId,
                   (uint8_t)receivedMessage.messageType,
                   receivedMessage.payload,
                   receivedMessage.payloadLength);
    }

    osDelay(1u);
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
  static const char helloWorld[] = "Hello World";
  uint8_t counter = 0;
  char buffer[32];

  (void)argument;

  while (!Communication_IsInitialized(&communication))
  {
    osDelay(100u);
  }

  osDelay(1000u);

  for (;;)
  {
	snprintf(buffer, sizeof(buffer), " %.11s %d !", helloWorld, counter);

	if (Communication_Send(&communication,
						   rootDevice->deviceId,
						   MESSAGE_TYPE_HEARTBEAT,
						   buffer,
						   (uint16_t)strlen(buffer)))
	{
	  DebugMessage("TX QUEUED",
					currentDevice->deviceId,
					rootDevice->deviceId,
					(uint8_t)MESSAGE_TYPE_HEARTBEAT,
					(const uint8_t *)buffer,
					(uint16_t)strlen(buffer));

	  counter++;
	  if(counter == 100)
		  counter = 0;
	}
	else
	{
	  Debug("Hello World TX queue failed\r\n");
	}

	osDelay(5000u);
  }
  /* USER CODE END StartReadSensorsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

