/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    App/custom_app.c
 * @author  MCD Application Team
 * @brief   Custom Example Application (Server)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_includes.h"
#include "BLE/ble.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* UcServer */
  uint8_t               Ucc_Notification_Status;
  uint8_t               Ucc_Indication_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* UcServer */
static void Custom_Ucc_Update_Char(void);
static void Custom_Ucc_Send_Notification(void);
static void Custom_Ucc_Send_Indication(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */
  printf("Custom_STM_App_Notification: opcode=%d, length=%d\r\n", pNotification->Custom_Evt_Opcode, pNotification->DataTransfered.Length);

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* UcServer */
    case CUSTOM_STM_MM_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MM_READ_EVT */

      /* USER CODE END CUSTOM_STM_MM_READ_EVT */
      break;

    case CUSTOM_STM_MM_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MM_WRITE_NO_RESP_EVT */

      /* USER CODE END CUSTOM_STM_MM_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_MM_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MM_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_MM_WRITE_EVT */
      break;

    case CUSTOM_STM_BM_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BM_READ_EVT */

      /* USER CODE END CUSTOM_STM_BM_READ_EVT */
      break;

    case CUSTOM_STM_BM_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BM_WRITE_NO_RESP_EVT */

      /* USER CODE END CUSTOM_STM_BM_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_BM_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BM_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_BM_WRITE_EVT */
      break;

    case CUSTOM_STM_UCC_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_UCC_WRITE_NO_RESP_EVT */
              uint8_t *data_n = pNotification->DataTransfered.pPayload; 
        uint8_t size_n = pNotification->DataTransfered.Length;
        loadCommand((char *)data_n, size_n);

      /* USER CODE END CUSTOM_STM_UCC_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_UCC_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_UCC_WRITE_EVT */
              uint8_t *data = pNotification->DataTransfered.pPayload; 
        uint8_t size = pNotification->DataTransfered.Length;
        loadCommand((char *)data, size);

      /* USER CODE END CUSTOM_STM_UCC_WRITE_EVT */
      break;

    case CUSTOM_STM_UCC_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_UCC_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_UCC_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_UCC_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_UCC_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_UCC_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_UCC_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_UCC_INDICATE_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_UCC_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_UCC_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_UCC_INDICATE_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_UCC_INDICATE_DISABLED_EVT */
      break;

    case CUSTOM_STM_UCU_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_UCU_WRITE_NO_RESP_EVT */
      
      /* USER CODE END CUSTOM_STM_UCU_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_UCU_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_UCU_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_UCU_WRITE_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* UcServer */
__USED void Custom_Ucc_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ucc_UC_1*/

  /* USER CODE END Ucc_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_UCC, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Ucc_UC_Last*/

  /* USER CODE END Ucc_UC_Last*/
  return;
}

void Custom_Ucc_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ucc_NS_1*/

  /* USER CODE END Ucc_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_UCC, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Ucc_NS_Last*/

  /* USER CODE END Ucc_NS_Last*/

  return;
}

void Custom_Ucc_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ucc_IS_1*/

  /* USER CODE END Ucc_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_UCC, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Ucc_IS_Last*/

  /* USER CODE END Ucc_IS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
