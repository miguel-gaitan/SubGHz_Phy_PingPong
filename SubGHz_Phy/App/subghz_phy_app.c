/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  Miguel G. - adapted from original version from MCD Application Team
  * @brief   RANGE TEST - adapted from Application of the SubGHz_Phy Middleware
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
/*node ID*/
#define NODE_ID              		  00	/* 00 is master - otherwise is slave */

/*Timeout*/
#define RX_TIMEOUT_VALUE              3000
#define TX_TIMEOUT_VALUE              3000

/* CONFIG string*/
#define CONFIG "01"
#define OWN_ID "00"
/* PROBE string*/
#define PROBE "PROBE"
/* PROBE string*/
#define REPORT "REPORT"

/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
/* wait for remote to be in Rx, before sending a Tx frame*/
#define RX_TIME_MARGIN               1000

/* LED blink Period*/
#define LED_PERIOD_MS                200
/* USER CODE END PD */

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

/* device state. Master: true, Slave: false*/
#if NODE_ID > 00
bool isMaster = false;
#else
bool isMaster = true;
#endif

/* Probe ID - it is the ID of the node (could be own) to transmit the Probe*/
int8_t Probe_ID = 0;	/* 0: means not defined the probe yet */
/* Report ID - it is the ID of the node (could be own) to transmit the Report*/
int8_t Report_ID = 0;	/* 0: means not defined the report yet */

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


/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

/* WELCOME MESSAGE    */
  APP_LOG(TS_OFF, VLEVEL_M, "\n\r RANGE TEST - WELCOME! \n\r");

  /* Led Timers*/
  UTIL_TIMER_Create(&timerLed, LED_PERIOD_MS, UTIL_TIMER_ONESHOT, OnledEvent, NULL);
  UTIL_TIMER_Start(&timerLed);

  /* USER CODE END SubghzApp_Init_1 ::::::::::::::::::::::::::::::::::::::::::::*/

 /* Initialize radio   */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  /* current radio status */
  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* Radio Set frequency (already set to 868000000) */
  Radio.SetChannel(RF_FREQUENCY);

  /* RADIO CONFIGURATION MESSAGE - BW & SF  */
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "NODE_ID = %d\n\r", NODE_ID);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_RF_FREQ = %d MHz\n\r",RF_FREQUENCY / 1000000);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW = %d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF = %d\n\r", LORA_SPREADING_FACTOR);


  /* Radio configuration: LORA, TX_PWR, BW, SF, C/R, Preamble, Payload, Timeout */
  /* TX : Transmitter configuration---------------------------------------------*/
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  /* RX : Receiver configuration-------------------------------------------------*/
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  /*max payload length = 255 Bytes (previously defined) */
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

  /*fills Tx buffer with zeros*/
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  if (NODE_ID > 0)
  {/* If Slave ---------------------------------------------------------------------*/
	  //State = RX;	/*FSM is already in RX*/
	  APP_LOG(TS_OFF, VLEVEL_M, "--------SLAVE---------\n\r");
	  Radio.Rx(RX_TIMEOUT_VALUE);
  }
  else
  {/* If Master --------------------------------------------------------------------*/
	  //State = TX; 	/* Update the State of the FSM*/
	  APP_LOG(TS_OFF, VLEVEL_M, "--------MASTER---------\n\r");
	  Radio.Rx(RX_TIMEOUT_VALUE);
  }

  /*register task to to be run in while(1) after Radio IT*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, PingPong_Process);

  /* USER CODE END SubghzApp_Init_2 :::::::::::::::::::::::::::::::::::::::::::::::*/
}

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */

  /* Update the State of the FSM*/
  State = TX;

  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */

  /* Update the State of the FSM*/
  State = RX;

  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "OnRxDone \n\r");
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

  /* Record payload Signal to noise ratio in Lora*/
  SnrValue = LoraSnr_FskCfo;

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

  /* Update the State of the FSM*/
  State = TX_TIMEOUT;

  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout \n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */

  /* Update the State of the FSM*/
  State = RX_TIMEOUT;

  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "OnRxTimeout \n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{

  /* Update the State of the ~*/
  State = RX_ERROR;

  /* USER CODE BEGIN OnRxError */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");

  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */


static void PingPong_Process(void)
{
  Radio.Sleep();
  /* USER CODE BEGIN OnRxError */
  APP_LOG(TS_ON, VLEVEL_L, "radio sleep\n\r");

  switch (State)
  {
   case RX:
	   if (isMaster == true)
	   {
        if (RxBufferSize > 0) // it if has received something
        {
          /* Master receiving a REPORT */
          //if (strncmp((const char *)BufferRx, REPORT, sizeof(REPORT) - 1) == 0) // -1 to exclude terminating '\0
          if (strncmp((const char *)BufferRx, PROBE, sizeof(PROBE) - 1) == 0) // -1 to exclude terminating '\0
          {
      	  /* DELAY: wake-up time + giving time to the remote node to be in Rx*/
      	  HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);

    	  //APP_LOG(TS_ON, VLEVEL_L, "Receiving REPORT packet \n\r");
    	  APP_LOG(TS_ON, VLEVEL_L, "Receiving PROBE packet \n\r");
    	  // need to do something with report package here, save it
    	  // then nothing else I guess
    	  Radio.Rx(RX_TIMEOUT_VALUE);

          }/* Master receiving something else */
          else
          {
            /* invalid reception - print an ERROR message */
            APP_LOG(TS_ON, VLEVEL_L, "ERROR: Master receiving a invalid message\n\r");
            Radio.Rx(RX_TIMEOUT_VALUE);
            // this is just to check it can receive a PROBE an is not an error
          }
        }
        else{//
        	  APP_LOG(TS_ON, VLEVEL_L, "RxBuffer is zero - probably starting the experiment\n\r");
        }
      }
      /* SLAVE ---------------------------------------------------------------*/
      else
      {
        if (RxBufferSize > 0)
        {
          //  <== ADD COMPARISON WITH OWN NODE ID
          if (strncmp((const char *)BufferRx, OWN_ID, sizeof(OWN_ID) - 1) == 0)
          {
            UTIL_TIMER_Stop(&timerLed);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED - off */
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN - on */

            /* time wake-up from sleep + time the remote node needs to be in Rx (margin)*/
            //HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);

      	    APP_LOG(TS_ON, VLEVEL_L, "Sending a PROBE packet \n\r");

            memcpy(BufferTx, PROBE, sizeof(PROBE) - 1);
            Radio.Send(BufferTx, PAYLOAD_LEN);

            //Radio.Rx(RX_TIMEOUT_VALUE);
          }
          else if ((strncmp((const char *)BufferRx, PROBE, sizeof(PROBE) - 1) == 0))
		  {
              /* this value needs to be saved and reported */
        	  APP_LOG(TS_ON, VLEVEL_L, "Sending a REPORT packet \n\r");

              UTIL_TIMER_Stop(&timerLed);
              HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED - off */
              HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); /* LED_RED - off */
              HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); /* LED_BLUE - on */

              memcpy(BufferTx, REPORT, sizeof(REPORT) - 1);
              Radio.Send(BufferTx, PAYLOAD_LEN);

		  }
          else /* valid reception but not a CONFIG or PROBE, thus is a REPORT */
          {
            /* invalid reception - print an ERROR message */
      	    APP_LOG(TS_ON, VLEVEL_L, "report or something else, do nothing \n\r");
      	   Radio.Rx(RX_TIMEOUT_VALUE);
          }
        }
        else{
        	  APP_LOG(TS_ON, VLEVEL_L, "RxBuffer is zero - probably starting the experiment\n\r");
        }
      }
      break;
   case TX:
           if (isMaster == true)
           {
         	  HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);

         	  /* MASTER transmitting - red led "on"  */
               UTIL_TIMER_Stop(&timerLed);
               HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); /* LED_GREEN - off */
               HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED - on*/

         	  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
         	  APP_LOG(TS_ON, VLEVEL_L, "Sending CONFIG packet \n\r");

         	  // this puts a CONFIG message in the tx buffer and then sends it
         	  memcpy(BufferTx, CONFIG, sizeof(CONFIG) - 1);
         	  Radio.Send(BufferTx, PAYLOAD_LEN);

           }else // slaves transmit either a PROBE or a REPORT depending on what message they receive
           {
        	  /* DELAY: wake-up time + giving time to the remote node to be in Rx*/
        	  HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);

         	  /* SLAVE transmitting - red led "on"  */
               UTIL_TIMER_Stop(&timerLed);
               HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED - off */
               HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN - on */

         	   APP_LOG(TS_ON, VLEVEL_L, "A message was just sent by a slave node \n\r");

         	  // Transmissions of PROBE or REPORT will be made on the RX state?

         	  Radio.Rx(RX_TIMEOUT_VALUE);
           }
        break;
    case RX_TIMEOUT:
    case RX_ERROR:
      if (isMaster == true)
      {
    	APP_LOG(TS_ON, VLEVEL_L, "Why not sending a packet \n\r");
        /* master sends what is in the buffer_tx*/
        memcpy(BufferTx, CONFIG, sizeof(CONFIG) - 1);
        Radio.Send(BufferTx, PAYLOAD_LEN);
      }
      else // slave
      {
       // wait until the next rx time-out
       Radio.Rx(RX_TIMEOUT_VALUE);
      }
      break;
    case TX_TIMEOUT:
      // listen until the next rx time-out
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
