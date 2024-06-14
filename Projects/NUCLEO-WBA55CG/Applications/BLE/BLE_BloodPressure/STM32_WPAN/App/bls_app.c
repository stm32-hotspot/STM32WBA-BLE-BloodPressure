/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service4_app.c
  * @author  MCD Application Team
  * @brief   service4_app application definition.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "app_ble.h"
#include "ll_sys_if.h"
#include "dbg_trace.h"
#include "ble.h"
#include "bls_app.h"
#include "bls.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef enum
{
  Bpm_INDICATION_OFF,
  Bpm_INDICATION_ON,
  Icp_NOTIFICATION_OFF,
  Icp_NOTIFICATION_ON,
  Bpf_INDICATION_OFF,
  Bpf_INDICATION_ON,
  /* USER CODE BEGIN Service2_APP_SendInformation_t */

  /* USER CODE END Service2_APP_SendInformation_t */
  BLS_APP_SENDINFORMATION_LAST
} BLS_APP_SendInformation_t;

typedef struct
{
  BLS_APP_SendInformation_t     Bpm_Indication_Status;
  BLS_APP_SendInformation_t     Icp_Notification_Status;
  BLS_APP_SendInformation_t     Bpf_Indication_Status;
  /* USER CODE BEGIN Service2_APP_Context_t */
  uint8_t connHandleIndex;
  BLS_Value_t BloodPressureMeasurementChar;
  BLS_Value_t IntermediateCuffPressureChar;
  uint16_t BloodPressureFeatureChar;
  UTIL_TIMER_Object_t TimerBPMeasurement_Id;
  UTIL_TIMER_Object_t TimerIntCufPressure_Id;
  /* USER CODE END Service2_APP_Context_t */
  uint16_t              ConnectionHandle;
} BLS_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NOT_A_NUMBER                                                      0x07FF
#define DEFAULT_BLS_MEASUREMENTVALUE_SYSTOLIC                             0X0072
#define DEFAULT_BLS_MEASUREMENTVALUE_DIASTOLIC                            0X0072
#define DEFAULT_BLS_MEASUREMENTVALUE_MEAN                                 0X0072
#define DEFAULT_BLS_MEASUREMENTVALUE_PULSE_RATE                               80
#define DEFAULT_BLS_MEASUREMENTVALUE_USER_ID                                0X01
#define DEFAULT_BLS_TIME_STAMP_YEAR                                         2000
#define DEFAULT_BLS_TIME_STAMP_MONTH                                          01
#define DEFAULT_BLS_TIME_STAMP_DAY                                            01
#define DEFAULT_BLS_TIME_STAMP_HOURS                                          00
#define DEFAULT_BLS_TIME_STAMP_MINUTES                                        00
#define DEFAULT_BLS_TIME_STAMP_SECONDS                                        00
#define DEFAULT_BLS_MEASUREMENT_STATUS_BODY_MOVEMENT_DETECTION_FLAG         0X01
#define DEFAULT_BLS_MEASUREMENT_STATUS_CUFF_FIT_DETECTION_FLAG              0X01
#define DEFAULT_BLS_MEASUREMENT_STATUS_IRREGULAR_PULSE_DETECTION_FLAG       0X01
#define DEFAULT_BLS_MEASUREMENT_STATUS_PULSE_RATE_EXCEEDS_DETECTION_FLAG    0X01
#define DEFAULT_BLS_MEASUREMENT_STATUS_PULSE_RATE_LOWER_DETECTION_FLAG      0X01
#define DEFAULT_BLS_MEASUREMENT_STATUS_MEASUREMENT_POSITION_DETECTION_FLAG  0X01
#define DEFAULT_BLS_BODY_MOVEMENT_DETECTION_SUPPORT_BIT                     0x01
#define DEFAULT_BLS_CUFF_FIT_DETECTION_SUPPORT_BIT                          0x01
#define DEFAULT_BLS_IRREGULAR_PULSE_DETECTION_SUPPORT_BIT                   0x01
#define DEFAULT_BLS_PULSE_RATE_RANGE_DETECTION_SUPPORT_BIT                  0x01
#define DEFAULT_BLS_MEASUREMENT_POSITION_DETECTION_SUPPORT_BIT              0x01
#define DEFAULT_BLS_MULTIPLE_BOND_SUPPORT_BIT                               0x00
#define DEFAULT_FEATURES_SUPPORTED                                          0X1F
#define DEFAULT_INTERVAL                                                   (500)  /**< 500 ms */
#define BODY_MOVEMENT_DETECTION_SUPPORT_BIT                                    1
#define CUFF_FIT_DETECTION_SUPPORT_BIT                                         2
#define IRREGULAR_PULSE_DETECTION_SUPPORT_BIT                                  4
#define PULSE_RATE_RANGE_DETECTION_SUPPORT_BIT                                 8
#define MEASUREMENT_POSITION_DETECTION_SUPPORT_BIT                            16
#define MULTIPLE_BOND_SUPPORT_BIT                                             32
#define MEASUREMENT                                                            0
#define INT_CUFF_PRESS_MEASUREMENT                                             1
#define CCCD_IMPROPERLY_CONFIGURED                                          0xFD
/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern CRC_HandleTypeDef hcrc;
/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static BLS_APP_Context_t BLS_APP_Context;

uint8_t a_BLS_UpdateCharData[247];

/* USER CODE BEGIN PV */
static BLS_Value_t aBPMeasurement[100];
static int8_t BPMStoreIndex;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void BLS_Bpm_SendIndication(void);
static void BLS_Icp_SendNotification(void);
static void BLS_Bpf_SendIndication(void);

/* USER CODE BEGIN PFP */
static void BLS_APP_UpdateBPMeasurement_timCB( void *arg );
static void BLS_APP_UpdateBPIntCuffPressure_timCB( void *arg );
static void BLS_APP_UpdateBPMeasurement( void );
static void BLS_APP_UpdateBPIntCuffPressure( void );
static void BLS_APP_BPMStore(void);
static void BLS_APP_BPMSuppress(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void BLS_Notification(BLS_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service2_Notification_1 */

  /* USER CODE END Service2_Notification_1 */
  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service2_Notification_Service2_EvtOpcode */

    /* USER CODE END Service2_Notification_Service2_EvtOpcode */

    case BLS_BPM_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN Service2Char1_INDICATE_ENABLED_EVT */
      LOG_INFO_APP ("Blood Pressure Measurement indication enabled\n\r");
      BLS_APP_Context.Bpm_Indication_Status = Bpm_INDICATION_ON;
      /* USER CODE END Service2Char1_INDICATE_ENABLED_EVT */
      break;

    case BLS_BPM_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN Service2Char1_INDICATE_DISABLED_EVT */
      LOG_INFO_APP ("Blood Pressure Measurement indication disabled\n\r");
      BLS_APP_Context.Bpm_Indication_Status = Bpm_INDICATION_OFF;
      /* USER CODE END Service2Char1_INDICATE_DISABLED_EVT */
      break;

    case BLS_ICP_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service2Char2_NOTIFY_ENABLED_EVT */
      LOG_INFO_APP ("Intermediate Cuff Pressure notification enabled\n\r");
      BLS_APP_Context.Icp_Notification_Status = Icp_NOTIFICATION_ON;
      /* USER CODE END Service2Char2_NOTIFY_ENABLED_EVT */
      break;

    case BLS_ICP_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service2Char2_NOTIFY_DISABLED_EVT */
      LOG_INFO_APP ("Intermediate Cuff Pressure notification disabled\n\r");
      BLS_APP_Context.Icp_Notification_Status = Icp_NOTIFICATION_OFF;
      /* USER CODE END Service2Char2_NOTIFY_DISABLED_EVT */
      break;

    case BLS_BPF_READ_EVT:
      /* USER CODE BEGIN Service2Char3_READ_EVT */

      /* USER CODE END Service2Char3_READ_EVT */
      break;

    case BLS_BPF_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN Service2Char3_INDICATE_ENABLED_EVT */
      BLS_APP_Context.Bpf_Indication_Status = Bpf_INDICATION_ON;
      LOG_INFO_APP ("Blood Pressure Feature indication enabled\n\r");
      /* USER CODE END Service2Char3_INDICATE_ENABLED_EVT */
      break;

    case BLS_BPF_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN Service2Char3_INDICATE_DISABLED_EVT */
      BLS_APP_Context.Bpf_Indication_Status = Bpf_INDICATION_OFF;
      LOG_INFO_APP ("Blood Pressure Feature indication disabled\n\r");
      /* USER CODE END Service2Char3_INDICATE_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN Service2_Notification_default */

      /* USER CODE END Service2_Notification_default */
      break;
  }
  /* USER CODE BEGIN Service2_Notification_2 */

  /* USER CODE END Service2_Notification_2 */
  return;
}

void BLS_APP_EvtRx(BLS_APP_ConnHandleNotEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service2_APP_EvtRx_1 */

  /* USER CODE END Service2_APP_EvtRx_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service2_APP_EvtRx_Service2_EvtOpcode */

    /* USER CODE END Service2_APP_EvtRx_Service2_EvtOpcode */
    case BLS_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service2_APP_CONN_HANDLE_EVT */

      /* USER CODE END Service2_APP_CONN_HANDLE_EVT */
      break;

    case BLS_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN Service2_APP_DISCON_HANDLE_EVT */
      {
        BLS_APP_Context.Bpm_Indication_Status = Bpm_INDICATION_OFF;
        BLS_APP_Context.Icp_Notification_Status = Icp_NOTIFICATION_OFF;
      }
      /* USER CODE END Service2_APP_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service2_APP_EvtRx_default */

      /* USER CODE END Service2_APP_EvtRx_default */
      break;
  }

  /* USER CODE BEGIN Service2_APP_EvtRx_2 */

  /* USER CODE END Service2_APP_EvtRx_2 */

  return;
}

void BLS_APP_Init(void)
{
  UNUSED(BLS_APP_Context);
  BLS_Init();

  /* USER CODE BEGIN Service2_APP_Init */
  BLS_Data_t msg_conf;
  uint8_t length = 0;

  /* Register tasks for BP, BP Int Cuff measurements */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_BLS_BP_MEAS_REQ_ID, UTIL_SEQ_RFU, BLS_APP_UpdateBPMeasurement );
  UTIL_SEQ_RegTask( 1<< CFG_TASK_BLS_BP_INT_CUFF_PRESSURE_REQ_ID, UTIL_SEQ_RFU, BLS_APP_UpdateBPIntCuffPressure );

  /*
   * Initialise Flags
   */
  BLS_APP_Context.BloodPressureMeasurementChar.Flags = (uint8_t)NO_FLAG;
  BLS_APP_Context.IntermediateCuffPressureChar.Flags = (uint8_t)NO_FLAG;

  /**
   *
   */
  BLS_APP_Context.BloodPressureMeasurementChar.Flags |= TIME_STAMP_PRESENT;
  BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Year = DEFAULT_BLS_TIME_STAMP_YEAR;
  BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Month = DEFAULT_BLS_TIME_STAMP_MONTH;
  BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Day = DEFAULT_BLS_TIME_STAMP_DAY;
  BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Hours = DEFAULT_BLS_TIME_STAMP_HOURS;
  BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Minutes = DEFAULT_BLS_TIME_STAMP_MINUTES;
  BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Seconds = DEFAULT_BLS_TIME_STAMP_SECONDS;
  
  BPMStoreIndex = 0;
  BLS_APP_Context.IntermediateCuffPressureChar.Flags            |= TIME_STAMP_PRESENT;
  BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Year    = DEFAULT_BLS_TIME_STAMP_YEAR;
  BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Month   = DEFAULT_BLS_TIME_STAMP_MONTH;
  BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Day     = DEFAULT_BLS_TIME_STAMP_DAY;
  BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Hours   = DEFAULT_BLS_TIME_STAMP_HOURS;
  BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Minutes = DEFAULT_BLS_TIME_STAMP_MINUTES;
  BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Seconds = DEFAULT_BLS_TIME_STAMP_SECONDS;
  
  /**
   * Set initial Pulse Rate
   */
  BLS_APP_Context.BloodPressureMeasurementChar.Flags |= PULSE_RATE_PRESENT;
  BLS_APP_Context.BloodPressureMeasurementChar.PulseRate = DEFAULT_BLS_MEASUREMENTVALUE_PULSE_RATE;
  BLS_APP_Context.IntermediateCuffPressureChar.Flags |= PULSE_RATE_PRESENT;
  BLS_APP_Context.IntermediateCuffPressureChar.PulseRate = DEFAULT_BLS_MEASUREMENTVALUE_PULSE_RATE;
  /**
   * Set initial User Id
   */
  BLS_APP_Context.BloodPressureMeasurementChar.Flags |= USER_ID_PRESENT;
  BLS_APP_Context.BloodPressureMeasurementChar.UserID = DEFAULT_BLS_MEASUREMENTVALUE_USER_ID;
  BLS_APP_Context.IntermediateCuffPressureChar.Flags |= USER_ID_PRESENT;
  BLS_APP_Context.IntermediateCuffPressureChar.UserID    = DEFAULT_BLS_MEASUREMENTVALUE_USER_ID;
  /**
   * Set Measurement Status
   */
  BLS_APP_Context.BloodPressureMeasurementChar.Flags |= MEASUREMENT_STATUS_PRESENT;
  BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus = 0;
  BLS_APP_Context.IntermediateCuffPressureChar.Flags |= MEASUREMENT_STATUS_PRESENT;
  BLS_APP_Context.IntermediateCuffPressureChar.MeasurementStatus = 0;
  
  length = 0;
  a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.Flags;
  a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Systolic) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Systolic) >> 8) & 0xFF;
  a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Diastolic) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Diastolic) >> 8) & 0xFF;
  a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Mean) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Mean) >> 8) & 0xFF;

  if((BLS_APP_Context.IntermediateCuffPressureChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
  {
    a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Year) & 0xFF;
    a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Year) >> 8) & 0xFF;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Month;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Day;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Hours;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Minutes;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Seconds;
  }
  
  if((BLS_APP_Context.IntermediateCuffPressureChar.Flags & PULSE_RATE_PRESENT) == PULSE_RATE_PRESENT)
  {
    a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.PulseRate) & 0xFF;
    a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.PulseRate) >> 8) & 0xFF;
  }

  if((BLS_APP_Context.IntermediateCuffPressureChar.Flags & USER_ID_PRESENT) == USER_ID_PRESENT)
  {
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.UserID;
  }
  
  if((BLS_APP_Context.IntermediateCuffPressureChar.Flags & MEASUREMENT_STATUS_PRESENT) == MEASUREMENT_STATUS_PRESENT)
  {
    a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.MeasurementStatus) & 0xFF;
    a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.MeasurementStatus) >> 8) & 0xFF;
  }
  
  msg_conf.Length = length;
  msg_conf.p_Payload = a_BLS_UpdateCharData;
  BLS_UpdateValue(BLS_ICP, &msg_conf);

  length = 0;
  a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.Flags;
  a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic) >> 8) & 0xFF;
  a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic) >> 8) & 0xFF;
  a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean) >> 8) & 0xFF;

  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
  {
    a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Year) & 0xFF;
    a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Year) >> 8) & 0xFF;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Month;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Day;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Hours;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Minutes;
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Seconds;
  }
  
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & PULSE_RATE_PRESENT) == PULSE_RATE_PRESENT)
  {
    a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.PulseRate) & 0xFF;
    a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.PulseRate) >> 8) & 0xFF;
  }
  
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & USER_ID_PRESENT) == USER_ID_PRESENT)
  {
    a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.UserID;
  }
  
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & MEASUREMENT_STATUS_PRESENT) == MEASUREMENT_STATUS_PRESENT)
  {
    a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus) & 0xFF;
    a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus) >> 8) & 0xFF;
  }
  
  msg_conf.Length = length;
  msg_conf.p_Payload = a_BLS_UpdateCharData;
  BLS_UpdateValue(BLS_BPM, &msg_conf);

  BLS_APP_Context.BloodPressureFeatureChar = 0;
  
  BLS_APP_Context.BloodPressureFeatureChar = (BODY_MOVEMENT_DETECTION_SUPPORT_BIT        |
                                              CUFF_FIT_DETECTION_SUPPORT_BIT             |
                                              IRREGULAR_PULSE_DETECTION_SUPPORT_BIT      |
                                              PULSE_RATE_RANGE_DETECTION_SUPPORT_BIT     |
                                              MEASUREMENT_POSITION_DETECTION_SUPPORT_BIT);
  
  length = 0;
  a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureFeatureChar) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureFeatureChar) >> 8) & 0xFF;
  msg_conf.Length = length;
  msg_conf.p_Payload = a_BLS_UpdateCharData;
  BLS_UpdateValue(BLS_BPF, &msg_conf);

   /**
   * Create timer for Blood Pressure Measurement
   */
  UTIL_TIMER_Create(&(BLS_APP_Context.TimerBPMeasurement_Id),
                    DEFAULT_INTERVAL*4,
                    UTIL_TIMER_PERIODIC,
                    &BLS_APP_UpdateBPMeasurement_timCB, 0);

  BLS_APP_Context.Bpm_Indication_Status = Bpm_INDICATION_OFF;

  /*
   * Create timer for Intermediate Cuff Pressure Measurement
   */
  UTIL_TIMER_Create(&(BLS_APP_Context.TimerIntCufPressure_Id),
                    DEFAULT_INTERVAL,
                    UTIL_TIMER_PERIODIC,
                    &BLS_APP_UpdateBPIntCuffPressure_timCB, 0);

  BLS_APP_Context.Icp_Notification_Status = Icp_NOTIFICATION_OFF;

  UTIL_TIMER_StartWithPeriod(&(BLS_APP_Context.TimerBPMeasurement_Id), DEFAULT_INTERVAL*4);
  UTIL_TIMER_StartWithPeriod(&(BLS_APP_Context.TimerIntCufPressure_Id), DEFAULT_INTERVAL);
  /* USER CODE END Service2_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
/**
* @brief Feature update
* @param [in] None
* @retval None
*/
void BLS_APP_BPFUpdate(void)
{
  BLS_Data_t msg_conf;
  uint8_t length = 0;

  BLS_APP_Context.BloodPressureFeatureChar = (BODY_MOVEMENT_DETECTION_SUPPORT_BIT        |
                                              CUFF_FIT_DETECTION_SUPPORT_BIT             |
                                              IRREGULAR_PULSE_DETECTION_SUPPORT_BIT      |
                                              PULSE_RATE_RANGE_DETECTION_SUPPORT_BIT     |
                                              MEASUREMENT_POSITION_DETECTION_SUPPORT_BIT |
                                              MULTIPLE_BOND_SUPPORT_BIT);
  
  a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureFeatureChar) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureFeatureChar) >> 8) & 0xFF;
  msg_conf.Length = length;
  msg_conf.p_Payload = a_BLS_UpdateCharData;
  BLS_UpdateValue(BLS_BPF, &msg_conf);
  
  return;
} /* end of BLS_APP_BPFUpdate() */
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
__USED void BLS_Bpm_SendIndication(void) /* Property Indication */
{
  BLS_APP_SendInformation_t indication_on_off = Bpm_INDICATION_OFF;
  BLS_Data_t bls_indication_data;

  bls_indication_data.p_Payload = (uint8_t*)a_BLS_UpdateCharData;
  bls_indication_data.Length = 0;

  /* USER CODE BEGIN Service2Char1_IS_1*/

  /* USER CODE END Service2Char1_IS_1*/

  if (indication_on_off != Bpm_INDICATION_OFF)
  {
    BLS_UpdateValue(BLS_BPM, &bls_indication_data);
  }

  /* USER CODE BEGIN Service2Char1_IS_Last*/

  /* USER CODE END Service2Char1_IS_Last*/

  return;
}

__USED void BLS_Icp_SendNotification(void) /* Property Notification */
{
  BLS_APP_SendInformation_t notification_on_off = Icp_NOTIFICATION_OFF;
  BLS_Data_t bls_notification_data;

  bls_notification_data.p_Payload = (uint8_t*)a_BLS_UpdateCharData;
  bls_notification_data.Length = 0;

  /* USER CODE BEGIN Service2Char2_NS_1*/

  /* USER CODE END Service2Char2_NS_1*/

  if (notification_on_off != Icp_NOTIFICATION_OFF)
  {
    BLS_UpdateValue(BLS_ICP, &bls_notification_data);
  }

  /* USER CODE BEGIN Service2Char2_NS_Last*/

  /* USER CODE END Service2Char2_NS_Last*/

  return;
}

__USED void BLS_Bpf_SendIndication(void) /* Property Indication */
{
  BLS_APP_SendInformation_t indication_on_off = Bpf_INDICATION_OFF;
  BLS_Data_t bls_indication_data;

  bls_indication_data.p_Payload = (uint8_t*)a_BLS_UpdateCharData;
  bls_indication_data.Length = 0;

  /* USER CODE BEGIN Service2Char3_IS_1*/

  /* USER CODE END Service2Char3_IS_1*/

  if (indication_on_off != Bpf_INDICATION_OFF)
  {
    BLS_UpdateValue(BLS_BPF, &bls_indication_data);
  }

  /* USER CODE BEGIN Service2Char3_IS_Last*/

  /* USER CODE END Service2Char3_IS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
static void BLS_APP_UpdateBPMeasurement_timCB( void *arg )
{
  /**
   * The code shall be executed in the background as aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  UTIL_SEQ_SetTask( 1<<CFG_TASK_BLS_BP_MEAS_REQ_ID, CFG_SEQ_PRIO_0);

  return;
}

static void BLS_APP_UpdateBPIntCuffPressure_timCB( void *arg )
{
  /**
   * The code shall be executed in the background as aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  UTIL_SEQ_SetTask( 1<<CFG_TASK_BLS_BP_INT_CUFF_PRESSURE_REQ_ID, CFG_SEQ_PRIO_0);

  return;
}

static void BLS_APP_UpdateBPMeasurement(void)
{
  uint16_t diastolic_measurement;
  uint16_t mean_measurement;
  
  diastolic_measurement = (uint16_t)((rand()&0x1F)+DEFAULT_BLS_MEASUREMENTVALUE_DIASTOLIC);
  mean_measurement      = (uint16_t)((rand()&0x1F)+DEFAULT_BLS_MEASUREMENTVALUE_MEAN);
  
  BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic = diastolic_measurement;
  BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean      = mean_measurement;

  if(BLS_APP_Context.Icp_Notification_Status == Icp_NOTIFICATION_ON)
  {
    BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic  =
      BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Systolic;
    
    if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
    {
      BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Seconds =
        BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Seconds;
      BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Minutes =
        BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Minutes;
      BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Hours   =
        BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Hours;
      BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Day     =
        BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Day;
    }
  }
  else
  {
    uint16_t systolic_measurement;

    systolic_measurement  = (uint16_t)((rand()&0x1F)+DEFAULT_BLS_MEASUREMENTVALUE_SYSTOLIC);
    BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic  = systolic_measurement;
    BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic = diastolic_measurement;
    BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean      = mean_measurement;
    
    if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
    {
      BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Seconds += 4;
      if(BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Seconds > 59)
      {
        BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Seconds = 0;
        BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Minutes += 1;
        if(BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Minutes > 59)
        {
          BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Minutes = 0;
          BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Hours += 1;
          if(BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Hours > 23)
          {
            BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Hours = 0;
            BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Day += 1;
            if(BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Day > 31)
              BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Day = 1;
          }
        }
      }
    }
  }
  
#if 0
  if(APP_BLE_Get_Server_Connection_Status() == APP_BLE_CONNECTED_SERVER)
  {
    BLS_Data_t msg_conf;
    uint8_t length;

    if(BLS_APP_Context.Bpm_Indication_Status == Bpm_INDICATION_ON)
    {
      UTIL_TIMER_Stop(&(BLS_APP_Context.TimerBPMeasurement_Id));
      UTIL_TIMER_StartWithPeriod(&(BLS_APP_Context.TimerBPMeasurement_Id), DEFAULT_INTERVAL*4);

      length = 0;
      
      a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.Flags;
      a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic) & 0xFF;
      a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic) >> 8) & 0xFF;
      a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic) & 0xFF;
      a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic) >> 8) & 0xFF;
      a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean) & 0xFF;
      a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean) >> 8) & 0xFF;
      
      if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
      {
        a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Year) & 0xFF;
        a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Year) >> 8) & 0xFF;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Month;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Day;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Hours;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Minutes;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Seconds;
      }
      
      if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & PULSE_RATE_PRESENT) == PULSE_RATE_PRESENT)
      {
        a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.PulseRate) & 0xFF;
        a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.PulseRate) >> 8) & 0xFF;
      }
      
      if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & USER_ID_PRESENT) == USER_ID_PRESENT)
      {
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.UserID;
      }
      
      if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & MEASUREMENT_STATUS_PRESENT) == MEASUREMENT_STATUS_PRESENT)
      {
        a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus) & 0xFF;
        a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus) >> 8) & 0xFF;
      }
      
      msg_conf.Length = length;
      msg_conf.p_Payload = a_BLS_UpdateCharData;
      BLS_UpdateValue(BLS_BPM, &msg_conf);
    }
  }
#else
  if((APP_BLE_Get_Server_Connection_Status() == APP_BLE_CONNECTED_SERVER) &&
     (BLS_APP_Context.Bpm_Indication_Status == Bpm_INDICATION_ON))
  {
    if(BPMStoreIndex >= 0)
    {
      LOG_INFO_APP ("BLS_APP_UpdateBPMeasurement(): send stored measurement %d\n", BPMStoreIndex);
      BLS_APP_BPMSuppress();
      UTIL_TIMER_Stop(&(BLS_APP_Context.TimerBPMeasurement_Id));
      UTIL_TIMER_StartWithPeriod(&(BLS_APP_Context.TimerBPMeasurement_Id), DEFAULT_INTERVAL);
    }
    else
    {
      BLS_Data_t msg_conf;
      uint8_t length = 0;

      if(BLS_APP_Context.Bpm_Indication_Status == Bpm_INDICATION_ON)
      {
        UTIL_TIMER_Stop(&(BLS_APP_Context.TimerBPMeasurement_Id));
        UTIL_TIMER_StartWithPeriod(&(BLS_APP_Context.TimerBPMeasurement_Id), DEFAULT_INTERVAL*4);

        a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.Flags;
        a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic) & 0xFF;
        a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic) >> 8) & 0xFF;
        a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic) & 0xFF;
        a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic) >> 8) & 0xFF;
        a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean) & 0xFF;
        a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean) >> 8) & 0xFF;
        
        if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
        {
          a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Year) & 0xFF;
          a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Year) >> 8) & 0xFF;
          a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Month;
          a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Day;
          a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Hours;
          a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Minutes;
          a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp.Seconds;
        }
        
        if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & PULSE_RATE_PRESENT) == PULSE_RATE_PRESENT)
        {
          a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.PulseRate) & 0xFF;
          a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.PulseRate) >> 8) & 0xFF;
        }
        
        if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & USER_ID_PRESENT) == USER_ID_PRESENT)
        {
          a_BLS_UpdateCharData[length++] = BLS_APP_Context.BloodPressureMeasurementChar.UserID;
        }
        
        if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & MEASUREMENT_STATUS_PRESENT) == MEASUREMENT_STATUS_PRESENT)
        {
          a_BLS_UpdateCharData[length++] = (BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus) & 0xFF;
          a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus) >> 8) & 0xFF;
        }
      
        msg_conf.Length = length;
        msg_conf.p_Payload = a_BLS_UpdateCharData;
        BLS_UpdateValue(BLS_BPM, &msg_conf);
      }
    }
  }
  else
  {
    if(BPMStoreIndex < 0)
    {
      BPMStoreIndex = 0;
    }
    LOG_INFO_APP ("BLS_APP_UpdateBPMeasurement(): store BP measurement %d\n", BPMStoreIndex);
    BLS_APP_BPMStore();
  }
#endif
  return;
}

static void BLS_APP_UpdateBPIntCuffPressure(void)
{
  uint16_t systolic_measurement;
  uint16_t diastolic_measurement;
  uint16_t mean_measurement;

  systolic_measurement  = (uint16_t)((rand()&0x1F)+DEFAULT_BLS_MEASUREMENTVALUE_SYSTOLIC);
  diastolic_measurement = (uint16_t)(NOT_A_NUMBER);
  mean_measurement      = (uint16_t)(NOT_A_NUMBER);

  BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Systolic  = systolic_measurement;
  BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Diastolic = diastolic_measurement;
  BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Mean      = mean_measurement;
  
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
  {
    BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Seconds += 1;
    if(BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Seconds > 59)
    {
      BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Seconds = 0;
      BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Minutes += 1;
      if(BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Minutes > 59)
      {
        BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Minutes = 0;
        BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Hours += 1;
        if(BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Hours > 23)
        {
          BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Hours = 0;
          BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Day += 1;
          if(BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Day > 31)
            BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Day = 1;
        }
      }
    }
  }

  if(APP_BLE_Get_Server_Connection_Status() == APP_BLE_CONNECTED_SERVER)
  {
    BLS_Data_t msg_conf;
    uint8_t length;
    
    if(BLS_APP_Context.Icp_Notification_Status == Icp_NOTIFICATION_ON)
    {  
      length = 0;
      
      a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.Flags;
      a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Systolic) & 0xFF;
      a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Systolic) >> 8) & 0xFF;
      a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Diastolic) & 0xFF;
      a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Diastolic) >> 8) & 0xFF;
      a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Mean) & 0xFF;
      a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.MeasurementValue_Mean) >> 8) & 0xFF;
      
      if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
      {
        a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Year) & 0xFF;
        a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Year) >> 8) & 0xFF;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Month;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Day;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Hours;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Minutes;
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.TimeStamp.Seconds;
      }
      
      if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & PULSE_RATE_PRESENT) == PULSE_RATE_PRESENT)
      {
        a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.PulseRate) & 0xFF;
        a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.PulseRate) >> 8) & 0xFF;
      }
      
      if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & USER_ID_PRESENT) == USER_ID_PRESENT)
      {
        a_BLS_UpdateCharData[length++] = BLS_APP_Context.IntermediateCuffPressureChar.UserID;
      }
      
      if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & MEASUREMENT_STATUS_PRESENT) == MEASUREMENT_STATUS_PRESENT)
      {
        a_BLS_UpdateCharData[length++] = (BLS_APP_Context.IntermediateCuffPressureChar.MeasurementStatus) & 0xFF;
        a_BLS_UpdateCharData[length++] = ((BLS_APP_Context.IntermediateCuffPressureChar.MeasurementStatus) >> 8) & 0xFF;
      }

      msg_conf.Length = length;
      msg_conf.p_Payload = a_BLS_UpdateCharData;
      BLS_UpdateValue(BLS_ICP, &msg_conf);
    }
  }

  return;
}

static void BLS_APP_BPMStore(void)
{
  memcpy(&(aBPMeasurement[BPMStoreIndex].Flags),
         &(BLS_APP_Context.BloodPressureMeasurementChar.Flags),
         sizeof(BLS_APP_Context.BloodPressureMeasurementChar.Flags));
  memcpy(&(aBPMeasurement[BPMStoreIndex].MeasurementValue_Systolic),
         &(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic),
         sizeof(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic));
  memcpy(&(aBPMeasurement[BPMStoreIndex].MeasurementValue_Diastolic),
         &(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic),
         sizeof(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic));
  memcpy(&(aBPMeasurement[BPMStoreIndex].MeasurementValue_Mean),
         &(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean),
         sizeof(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean));
  
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
  {
    memcpy(&(aBPMeasurement[BPMStoreIndex].TimeStamp),
           &(BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp),
           sizeof(BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp));
  }
  
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & PULSE_RATE_PRESENT) == PULSE_RATE_PRESENT)
  {
    memcpy(&(aBPMeasurement[BPMStoreIndex].PulseRate),
           &(BLS_APP_Context.BloodPressureMeasurementChar.PulseRate),
           sizeof(BLS_APP_Context.BloodPressureMeasurementChar.PulseRate));
  }
  
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & USER_ID_PRESENT) == USER_ID_PRESENT)
  {
    memcpy(&(aBPMeasurement[BPMStoreIndex].UserID),
           &(BLS_APP_Context.BloodPressureMeasurementChar.UserID),
           sizeof(BLS_APP_Context.BloodPressureMeasurementChar.UserID));
  }
  
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & MEASUREMENT_STATUS_PRESENT) == MEASUREMENT_STATUS_PRESENT)
  {
    memcpy(&(aBPMeasurement[BPMStoreIndex].MeasurementStatus),
           &(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus),
           sizeof(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus));
  }
 
 BPMStoreIndex++;
 if(BPMStoreIndex == 100)
    BPMStoreIndex = 0;
}

static void BLS_APP_BPMSuppress(void)
{
  uint8_t i;
  BLS_Data_t msg_conf;
  uint8_t length = 0;

  a_BLS_UpdateCharData[length++] = aBPMeasurement[0].Flags;
  a_BLS_UpdateCharData[length++] = (aBPMeasurement[0].MeasurementValue_Systolic) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((aBPMeasurement[0].MeasurementValue_Systolic) >> 8) & 0xFF;
  a_BLS_UpdateCharData[length++] = (aBPMeasurement[0].MeasurementValue_Diastolic) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((aBPMeasurement[0].MeasurementValue_Diastolic) >> 8) & 0xFF;
  a_BLS_UpdateCharData[length++] =  (aBPMeasurement[0].MeasurementValue_Mean) & 0xFF;
  a_BLS_UpdateCharData[length++] = ((aBPMeasurement[0].MeasurementValue_Mean) >> 8) & 0xFF;
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
  {
    a_BLS_UpdateCharData[length++] =  (aBPMeasurement[0].TimeStamp.Year) & 0xFF;
    a_BLS_UpdateCharData[length++] = ((aBPMeasurement[0].TimeStamp.Year) >> 8) & 0xFF;
    a_BLS_UpdateCharData[length++] =   aBPMeasurement[0].TimeStamp.Month;
    a_BLS_UpdateCharData[length++] =   aBPMeasurement[0].TimeStamp.Day;
    a_BLS_UpdateCharData[length++] =   aBPMeasurement[0].TimeStamp.Hours;
    a_BLS_UpdateCharData[length++] =   aBPMeasurement[0].TimeStamp.Minutes;
    a_BLS_UpdateCharData[length++] =   aBPMeasurement[0].TimeStamp.Seconds;
  }
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & PULSE_RATE_PRESENT) == PULSE_RATE_PRESENT)
  {
    a_BLS_UpdateCharData[length++] =  (aBPMeasurement[0].PulseRate) & 0xFF;
    a_BLS_UpdateCharData[length++] = ((aBPMeasurement[0].PulseRate) >> 8) & 0xFF;
  }
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & USER_ID_PRESENT) == USER_ID_PRESENT)
  {
    a_BLS_UpdateCharData[length++] =   aBPMeasurement[0].UserID;
  }
  if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & MEASUREMENT_STATUS_PRESENT) == MEASUREMENT_STATUS_PRESENT)
  {
    a_BLS_UpdateCharData[length++] =  (aBPMeasurement[0].MeasurementStatus) & 0xFF;
    a_BLS_UpdateCharData[length++] = ((aBPMeasurement[0].MeasurementStatus) >> 8) & 0xFF;
  }
  msg_conf.Length = length;
  msg_conf.p_Payload = a_BLS_UpdateCharData;
  BLS_UpdateValue(BLS_BPM, &msg_conf);

  for(i = 1; i < BPMStoreIndex; i++)
  {
    memcpy(&(aBPMeasurement[i-1].Flags),
           &(aBPMeasurement[i].Flags),
           sizeof(BLS_APP_Context.BloodPressureMeasurementChar.Flags));
    memcpy(&(aBPMeasurement[i-1].MeasurementValue_Systolic),
           &(aBPMeasurement[i].MeasurementValue_Systolic),
           sizeof(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Systolic));
    memcpy(&(aBPMeasurement[i-1].MeasurementValue_Diastolic),
           &(aBPMeasurement[i].MeasurementValue_Diastolic),
           sizeof(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Diastolic));
    memcpy(&(aBPMeasurement[i-1].MeasurementValue_Mean),
           &(aBPMeasurement[i].MeasurementValue_Mean),
           sizeof(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementValue_Mean));
    
    if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & TIME_STAMP_PRESENT) == TIME_STAMP_PRESENT)
    {
      memcpy(&(aBPMeasurement[i-1].TimeStamp),
             &(aBPMeasurement[i].TimeStamp),
             sizeof(BLS_APP_Context.BloodPressureMeasurementChar.TimeStamp));
    }
    
    if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & PULSE_RATE_PRESENT) == PULSE_RATE_PRESENT)
    {
      memcpy(&(aBPMeasurement[i-1].PulseRate),
             &(aBPMeasurement[i].PulseRate),
             sizeof(BLS_APP_Context.BloodPressureMeasurementChar.PulseRate));
    }
    
    if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & USER_ID_PRESENT) == USER_ID_PRESENT)
    {
      memcpy(&(aBPMeasurement[i-1].UserID),
             &(aBPMeasurement[i].UserID),
             sizeof(BLS_APP_Context.BloodPressureMeasurementChar.UserID));
    }
    
    if((BLS_APP_Context.BloodPressureMeasurementChar.Flags & MEASUREMENT_STATUS_PRESENT) == MEASUREMENT_STATUS_PRESENT)
    {
      memcpy(&(aBPMeasurement[i-1].MeasurementStatus),
             &(aBPMeasurement[i].MeasurementStatus),
             sizeof(BLS_APP_Context.BloodPressureMeasurementChar.MeasurementStatus));
    }
  }
  BPMStoreIndex--;
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
