/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service4_app.h
  * @author  MCD Application Team
  * @brief   Header for service4_app.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLS_APP_H
#define BLS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BLS_CONN_HANDLE_EVT,
  BLS_DISCON_HANDLE_EVT,

  /* USER CODE BEGIN Service2_OpcodeNotificationEvt_t */

  /* USER CODE END Service2_OpcodeNotificationEvt_t */

  BLS_LAST_EVT,
} BLS_APP_OpcodeNotificationEvt_t;

typedef struct
{
  BLS_APP_OpcodeNotificationEvt_t          EvtOpcode;
  uint16_t                                 ConnectionHandle;

  /* USER CODE BEGIN BLS_APP_ConnHandleNotEvt_t */

  /* USER CODE END BLS_APP_ConnHandleNotEvt_t */
} BLS_APP_ConnHandleNotEvt_t;
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void BLS_APP_Init(void);
void BLS_APP_EvtRx(BLS_APP_ConnHandleNotEvt_t *p_Notification);
/* USER CODE BEGIN EFP */
uint8_t BLS_APP_GetRACPCharacteristicIndicationEnabled(void);
void BLS_APP_BPFUpdate(void);
void BLS_APP_EPOCHUpdate(void);
uint16_t BLS_APP_ComputeCRC(uint8_t * pData, uint8_t dataLength);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*BLS_APP_H */
