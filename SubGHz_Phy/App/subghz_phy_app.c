/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team - Modified version by MGG
  * @brief   RANGE TEST protocol based on the Application of the SubGHz_Phy Middleware
  * @device	 MASTER - OR - SLAVE: defined it manually
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "subghz_phy_version.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */
typedef enum
{
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE              3000
#define TX_TIMEOUT_VALUE              3000
/* PING string*/
#define PING "PING"
/* PONG string*/
#define PONG "PONG"
/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
/* wait for remote to be in Rx, before sending a Tx frame*/
#define RX_TIME_MARGIN                2000
/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH             83333
/* LED blink Period*/
#define LED_PERIOD_MS                 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;
/* USER CODE BEGIN PV */

/*Ping Pong FSM states */
static States_t State = RX;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;
/* Led Timers objects*/
static UTIL_TIMER_Object_t timerLed;
/*=============================================================================*/
/* MASTER or SLAVE															   */
/*=============================================================================*/
/* device state. Master: true, Slave: false*/
bool isMaster = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/**
  * @brief  Function executed on when led timer elapses
  * @param  context ptr of LED context
  */
static void OnledEvent(void *context);

/**
  * @brief PingPong state machine implementation
  */
static void PingPong_Process(void);
/* USER CODE END PFP */

/*=============================================================================*/
/* RANGE TEST																   */
/*=============================================================================*/

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

/*=============================================================================*/
/* WELCOME MESSAGE  														   */
/*=============================================================================*/
	APP_LOG(TS_OFF, VLEVEL_M, "\n\r RANGE TEST - WELCOME! \n\r");

  /* Led Timers*/
  UTIL_TIMER_Create(&timerLed, LED_PERIOD_MS, UTIL_TIMER_ONESHOT, OnledEvent, NULL);
  UTIL_TIMER_Start(&timerLed);

  /* USER CODE END SubghzApp_Init_1 ::::::::::::::::::::::::::::::::::::::::::::*/

 /*=============================================================================*/
 /* Initialize radio 													   */
 /*=============================================================================*/

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  /* current radio status ====================================================*/
  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* Radio Set frequency (already set to 868000000) ===========================*/
  Radio.SetChannel(RF_FREQUENCY);

  /*=============================================================================*/
  /* RADIO CONFIGURATION MESSAGE - BW & SF  							         */
  /*=============================================================================*/

  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_RF_FREQ=%d MHz\n\r",RF_FREQUENCY / 1000000);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);

  /*=============================================================================*/
  /* Radio configuration: LORA, TX_PWR, BW, SF, C/R, Preamble, Payload, Timeout */
  /*=============================================================================*/
  /* TX : Transmitter configuration												 */
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  /* RX : Receiver configuration 												 */
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  /*max payload length = 255 Bytes (previously defined) */
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

  /*fills tx buffer*/
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  /*=============================================================================*/
  /* RECEIVING MESSAGE  					  							         */
  /*=============================================================================*/
  /*node starts reception*/
  Radio.Rx(RX_TIMEOUT_VALUE);
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

  /*=============================================================================*/
  /* trigger "sequencer" for PingPong process  (see PingPong now)				 */
  /*=============================================================================*/
  /*register task to to be run in while(1) after Radio IT*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, PingPong_Process);

  /* USER CODE END SubghzApp_Init_2 :::::::::::::::::::::::::::::::::::::::::::::::*/
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");

  /* Update the State of the FSM*/
  State = TX;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
  /* Record payload Signal to noise ratio in Lora*/
  SnrValue = LoraSnr_FskCfo;

  /* Update the State of the FSM*/
  State = RX;
  /* Clear BufferRx*/
  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
  /* Record payload size*/
  RxBufferSize = size;
  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
  {
    memcpy(BufferRx, payload, RxBufferSize);
  }
  /* Record Received Signal Strength*/
  RssiValue = rssi;
  /* Record payload content*/
  APP_LOG(TS_ON, VLEVEL_H, "payload. size=%d \n\r", size);
  for (int i = 0; i < PAYLOAD_LEN; i++)
  {
    APP_LOG(TS_OFF, VLEVEL_H, "%02X", BufferRx[i]);
    if (i % 16 == 15)
    {
      APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
    }
  }
  APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout\n\r");
  /* Update the State of the FSM*/
  State = TX_TIMEOUT;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxTimeout\n\r");
  /* Update the State of the FSM*/
  State = RX_TIMEOUT;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");
  /* Update the State of the FSM*/
  State = RX_ERROR;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

/*=============================================================================*/
/* PING PONG 		    					  							       */
/*=============================================================================*/

static void PingPong_Process(void)
{
  /* for reducing energy consumption?*/
  Radio.Sleep();

  switch (State)
  {
    case RX:
      if (isMaster == true)
      /*=============================================================================*/
      /* MASTER        	    					  							         */
      /*=============================================================================*/
      {
        if (RxBufferSize > 0) /* 2 Bytes*/
        {
          /* if Master receives a PONG, it replies with a PING*/
          if (strncmp((const char *)BufferRx, PONG, sizeof(PONG) - 1) == 0)
          {
            UTIL_TIMER_Stop(&timerLed);
            /* switch off green led */
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); /* LED_GREEN */
            /* master toggles red led */
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED */
            /*=============================================================================*/
            /* MASTER Receiving		    					  				               */
            /*=============================================================================*/
            /* Add delay between RX and TX */
            HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
            APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
            /* master sends PING*/
            APP_LOG(TS_ON, VLEVEL_L, "..."
                    "PING  - MASTER"
                    "\n\r");
            /*=============================================================================*/
            /* MASTER Transmitting  		    					  				       */
            /*=============================================================================*/
            memcpy(BufferTx, PING, sizeof(PING) - 1);
            Radio.Send(BufferTx, PAYLOAD_LEN);
          }
          else if (strncmp((const char *)BufferRx, PING, sizeof(PING) - 1) == 0)
          {
        	/* if Master receives a PING - print an ERROR message*/
            APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
            APP_LOG(TS_ON, VLEVEL_L, "ERROR: Master receiving a PING from another Master?\n\r");
            Radio.Rx(RX_TIMEOUT_VALUE);
          }
          else /* valid reception but neither a PING or a PONG message */
          {
            /* invalid reception - print an ERROR message */
            APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
            APP_LOG(TS_ON, VLEVEL_L, "ERROR: Master receiving a invalid message\n\r");
            Radio.Rx(RX_TIMEOUT_VALUE);
          }
        }
      }
      /*=============================================================================*/
      /* SLAVE  		    					  							         */
      /*=============================================================================*/
      else
      {
        if (RxBufferSize > 0)
        {
           /* if slave receives a PING, it replies with a PONG*/
          if (strncmp((const char *)BufferRx, PING, sizeof(PING) - 1) == 0)
          {
            UTIL_TIMER_Stop(&timerLed);
            /* switch off red led */
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
            /* slave toggles green led */
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN */
            /*=============================================================================*/
            /* SLAVE Receiving		    					  				               */
            /*=============================================================================*/
            /* Add delay between RX and TX */
            HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
            APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
            /*slave sends PONG*/
            APP_LOG(TS_ON, VLEVEL_L, "..."
                    "PONG - SLAVE"
                    "\n\r");
            /*=============================================================================*/
            /* SLAVE Transmitting  		    					  				           */
            /*=============================================================================*/
            memcpy(BufferTx, PONG, sizeof(PONG) - 1);
            Radio.Send(BufferTx, PAYLOAD_LEN);
          }
          else /* valid reception but not a PING as expected */
          {
            /* invalid reception - print an ERROR message */
            APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
            APP_LOG(TS_ON, VLEVEL_L, "ERROR: Slave receiving a invalid message\n\r");
            Radio.Rx(RX_TIMEOUT_VALUE);
          }
        }
      }
      break;
    case TX:
      APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
      APP_LOG(TS_ON, VLEVEL_L, "... receiving \n\r");
      Radio.Rx(RX_TIMEOUT_VALUE);
      break;
    case RX_TIMEOUT:
      APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
      APP_LOG(TS_ON, VLEVEL_L, "... RX Timeout \n\r");
    case RX_ERROR:
      if (isMaster == true)
      {
        /*=============================================================================*/
        /* MASTER Receiving	(again)	    					  				           */
        /*=============================================================================*/
        /* Send the next PING frame */
        /* Add delay between RX and TX*/
        HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
        APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
        APP_LOG(TS_ON, VLEVEL_L, "RX ERROR: Master Re-transmission\n\r");
        /* master sends PING*/
        /*=============================================================================*/
        /* MASTER Transmitting  		    					  				       */
        /*=============================================================================*/
        memcpy(BufferTx, PING, sizeof(PING) - 1);
        Radio.Send(BufferTx, PAYLOAD_LEN);
      }
      else
      /*=============================================================================*/
      /* SLAVE  		    					  							         */
      /*=============================================================================*/
      {
       APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
       APP_LOG(TS_ON, VLEVEL_L, "RX ERROR: waiting for a re-transmission \n\r");
        Radio.Rx(RX_TIMEOUT_VALUE);
      }
      break;
    case TX_TIMEOUT:
       APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
       APP_LOG(TS_ON, VLEVEL_L, "TX Timeout: now starts the reception\n\r");
      Radio.Rx(RX_TIMEOUT_VALUE);
      break;
    default:
      break;
  }
}

static void OnledEvent(void *context)
{
  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN */
  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED */
  UTIL_TIMER_Start(&timerLed);
}

/* USER CODE END PrFD */
