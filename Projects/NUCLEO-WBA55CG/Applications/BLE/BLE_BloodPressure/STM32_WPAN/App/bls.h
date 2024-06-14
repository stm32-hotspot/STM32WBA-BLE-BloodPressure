/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service4.h
  * @author  MCD Application Team
  * @brief   Header for service4.c
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
#ifndef BLS_H
#define BLS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported defines ----------------------------------------------------------*/
/* USER CODE BEGIN ED */
/* New values defined on Core spec supplement (CSS v4), common profile and service error code
   It is used on PTS 5.3.0 */
#define BLS_ATT_ERROR_CODE_PROCEDURE_ALREADY_IN_PROGRESS                  (0xFE)
  
/* New values defined on Core spec supplement (CSS v4), common profile and service error code
   It is used on PTS 5.3.0 */
#define BLS_ATT_ERROR_CODE_CLIENT_CHAR_CONF_DESC_IMPROPERLY_CONFIGURED    (0xFD)
#define BLS_ATT_ERROR_CODE_MISSING_CRC                                    (0x80)
#define BLS_ATT_ERROR_CODE_INVALID_CRC                                    (0x81)
#define BLS_ATT_ERROR_CODE_OUT_OF_RANGE                                   (0xFF)
/* USER CODE END ED */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BLS_BPM,
  BLS_ICP,
  BLS_BPF,
  /* USER CODE BEGIN Service2_CharOpcode_t */

  /* USER CODE END Service2_CharOpcode_t */
  BLS_CHAROPCODE_LAST
} BLS_CharOpcode_t;

typedef enum
{
  BLS_BPM_INDICATE_ENABLED_EVT,
  BLS_BPM_INDICATE_DISABLED_EVT,
  BLS_ICP_NOTIFY_ENABLED_EVT,
  BLS_ICP_NOTIFY_DISABLED_EVT,
  BLS_BPF_READ_EVT,
  BLS_BPF_INDICATE_ENABLED_EVT,
  BLS_BPF_INDICATE_DISABLED_EVT,
  /* USER CODE BEGIN Service2_OpcodeEvt_t */

  /* USER CODE END Service2_OpcodeEvt_t */
  BLS_BOOT_REQUEST_EVT
} BLS_OpcodeEvt_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t Length;

  /* USER CODE BEGIN Service2_Data_t */

  /* USER CODE END Service2_Data_t */
} BLS_Data_t;

typedef struct
{
  BLS_OpcodeEvt_t       EvtOpcode;
  BLS_Data_t             DataTransfered;
  uint16_t                ConnectionHandle;
  uint16_t                AttributeHandle;
  uint8_t                 ServiceInstance;
  /* USER CODE BEGIN Service2_NotificationEvt_t */

  /* USER CODE END Service2_NotificationEvt_t */
} BLS_NotificationEvt_t;

/* USER CODE BEGIN ET */
typedef enum
{
  NO_FLAG = 0,
  VALUE_UNIT_KILO_PASCAL = (1<<0),                       /*0 -> Blood pressure systolic, diastolic & Mean values in units of mmHg - if 1 -> in units of kPa*/
  TIME_STAMP_PRESENT = (1<<1),
  PULSE_RATE_PRESENT = (1<<2),
  USER_ID_PRESENT = (1<<3),
  MEASUREMENT_STATUS_PRESENT = (1<<4),
  USER_FACING_TIME_PRESENT = (1<<5),
  EPOCH_START_TIME_2000_USED = (1<<6)
} BLS_Measurement_Flags_t;

typedef struct
{
  uint16_t  Year;
  uint8_t   Month;
  uint8_t   Day;
  uint8_t   Hours;
  uint8_t   Minutes;
  uint8_t   Seconds;
}BLS_TimeStamp_t;

typedef struct
{
  uint16_t MeasurementValue_Systolic;
  uint16_t MeasurementValue_Diastolic;
  uint16_t MeasurementValue_Mean;
  BLS_TimeStamp_t TimeStamp;
  uint16_t PulseRate;
  uint8_t UserID;
  uint16_t MeasurementStatus;
  uint8_t Flags;
}BLS_Value_t;

typedef struct
{
  uint8_t Flags;
  uint16_t MeasurementValue_Systolic;
  uint16_t MeasurementValue_Diastolic;
  uint16_t MeasurementValue_Mean;
  uint32_t TimeStamp;
  uint16_t PulseRate;
  uint8_t UserID;
  uint16_t MeasurementStatus;
  uint32_t UserFacingTime;
}BLS_Enhanced_Value_t;
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
void BLS_Init(void);
void BLS_Notification(BLS_NotificationEvt_t *p_Notification);
tBleStatus BLS_UpdateValue(BLS_CharOpcode_t CharOpcode, BLS_Data_t *pData);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*BLS_H */
