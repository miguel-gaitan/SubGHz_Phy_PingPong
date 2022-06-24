/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  Miguel G. - adapted from original version from MCD Application Team
  * @brief   RANGE TEST - adapted from Application of the SubGHz_Phy Middleware
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
  TX,
  TX_TIMEOUT,
} States_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* Configurations */
#define NODE_ID              		  7	/* 0 is master - otherwise is slave */

#define RX_TIMEOUT_VALUE              3000
#define TX_TIMEOUT_VALUE              3000

#define OWN_ID "7"
#define PROBE "PROBE"
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
/* App Rx & Tx Buffers*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packets: Rssi & SNR*/
int8_t RssiValue = 0;
int8_t SnrValue = 0;

/* Led Timers objects*/
static UTIL_TIMER_Object_t timerLed;

char SrcID = '0';

int8_t NofSlaves = 7;			// used by the counter
int8_t Counter = 1;				// counter
char ConfigCount[1] = "1";		// counter (in char format)
char OwnID[1] = "7";

int OwnID_int;
int SrcID_int;
int DelayFactor;

char ProbeMsg[2] = "PX";			// suffix of probe messages
char ReportMsg[3] = "RXY";			// suffix of probe messages

/* device state. Master: true, Slave: false*/
#if NODE_ID > 0
bool isMaster = false;
#else
bool isMaster = true;
#endif

bool isProbeRcvd = true;

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
  }
  else
  {/* If Master --------------------------------------------------------------------*/
	  //State = TX; 	/* Update the State of the FSM*/
	  APP_LOG(TS_OFF, VLEVEL_M, "--------MASTER---------\n\r");
  }

  Radio.Rx(RX_TIMEOUT_VALUE);
  /*register task to to be run in while(1) after Radio IT*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, PingPong_Process);

  /* USER CODE END SubghzApp_Init_2 :::::::::::::::::::::::::::::::::::::::::::::::*/
}

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */

  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone \n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

  /* Update the State of the FSM*/
  State = RX;

  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */

  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout \n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

  /* Update the State of the FSM*/
  State = TX;

  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxTimeout */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */

  /* Record payload RSSI & SNR*/
  /* MASTER ---------------------------------------------------------*/

	RssiValue = rssi;
	SnrValue = LoraSnr_FskCfo;

	RxBufferSize = size;						// size of the received packet (payload)
	memset(BufferRx, 0, MAX_APP_BUFFER_SIZE); 	// clear of the prior buffer
	if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
	{
		memcpy(BufferRx, payload, RxBufferSize); 	// copy payload into the BufferRx
		}

	if (isMaster == true)
	{
		if (strncmp((const char *)BufferRx, ReportMsg, 1) == 0)
		{
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
			APP_LOG(TS_ON, VLEVEL_L, "OnRxDone -- MASTER: REPORT \n\r");
			APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
			APP_LOG(TS_ON, VLEVEL_L, "BufferRx=%s \n\r", (char*) BufferRx);
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

			//here I need to do something with the REPORT

			State = RX;
		}
		else
		{
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
			APP_LOG(TS_ON, VLEVEL_L, "OnRxDone -- MASTER: something else \n\r");
			APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
			APP_LOG(TS_ON, VLEVEL_L, "BufferRx=%s \n\r", (char*) BufferRx);
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

			State = RX;
		}
	}
	/* SLAVE ---------------------------------------------------------*/
	else
	{
		if (strncmp((const char *)BufferRx, ProbeMsg, 1) == 0)
		{
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
			APP_LOG(TS_ON, VLEVEL_L, "OnRxDone -- SLAVE: PROBE %s \n\r",ProbeMsg);
			APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
			APP_LOG(TS_ON, VLEVEL_L, "BufferRx=%s \n\r", (char*) BufferRx);
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

			isProbeRcvd = true;
			SrcID = (char) BufferRx[1];
			State = TX;
		}
		else if (strncmp((const char *)BufferRx, OwnID, sizeof(OwnID)) == 0)
		{
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
			APP_LOG(TS_ON, VLEVEL_L, "OnRxDone -- SLAVE: CONFIG (OWN_ID=%s)\n\r",OWN_ID);
			APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
			APP_LOG(TS_ON, VLEVEL_L, "BufferRx =%s \n\r", (char*) BufferRx);
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

			isProbeRcvd = false;
			State = TX;
		}
		else
		{
			State = RX;
		}
	}

  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

  /* USER CODE END OnRxDone */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */

	APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
	APP_LOG(TS_ON, VLEVEL_L, "OnRxTimeout \n\r");
	APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

	if (isMaster == true)
	{
		State = TX;
	}
	else // slave
	{
		State = RX;
	}

  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxTimeout */
}

/* USER CODE BEGIN PrFD */

static void PingPong_Process(void)
{
  Radio.Sleep();

  switch (State)
  {
   case TX:
           if (isMaster == true)
           {
               UTIL_TIMER_Stop(&timerLed);
               HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
               HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED - on*/

               /* sending the CONFIG message*/
         	   memcpy(BufferTx, ConfigCount, sizeof(ConfigCount));
         	   Radio.Send(BufferTx, PAYLOAD_LEN);

         	   APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
         	   APP_LOG(TS_ON, VLEVEL_L, "CONFIG packet sent=%s\n\r", ConfigCount);

         	  /* update the CONFIG counter, i.e. next node to transmit the probe*/
         	   if (Counter < NofSlaves)
         	   {
         		   Counter = Counter + 1;
         	   }
         	   else
         	   {
         		   Counter = 1;
         	   }
         	   ConfigCount[0] = Counter+'0';
         	   APP_LOG(TS_ON, VLEVEL_L, "Next node to transmit=%s\n\r", ConfigCount);

           }else // SLAVE
           {
               if (isProbeRcvd == true)
               {
                   UTIL_TIMER_Stop(&timerLed);
                   HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
                   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
                   HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); /* LED_BLUE - on */

             	   APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
             	   APP_LOG(TS_ON, VLEVEL_L, "REPORT packet sent \n\r");

             	  //strcat(ReportMsg, (char*) OwnID);
             	  ReportMsg[1] = SrcID;
             	  ReportMsg[2] = OwnID[0];

                  memcpy(BufferTx, ReportMsg, sizeof(ReportMsg));

                  SrcID_int = SrcID -'0';
                  OwnID_int = OwnID[0] -'0';

                  /*--- DELAY ---*/
                  if (OwnID_int > SrcID_int)
				  {
                	  DelayFactor = OwnID_int - SrcID_int - 1;
                	  APP_LOG(TS_ON, VLEVEL_L, "Delay factor 1 = %d \n\r",DelayFactor);
				  }
                  else
                  {
                	  DelayFactor = NofSlaves - SrcID_int + OwnID_int - 1;
                	  APP_LOG(TS_ON, VLEVEL_L, "Delay factor 2 = %d \n\r",DelayFactor);
                  }
                  HAL_Delay(Radio.GetWakeupTime() + DelayFactor*RX_TIMEOUT_VALUE );
                  Radio.Send(BufferTx, PAYLOAD_LEN);
               }
               else // it was a CONFIG
               {
                   UTIL_TIMER_Stop(&timerLed);
                   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                   HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
                   HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN - on */

                   //strcat(ProbeMsg, OwnID);
                   ProbeMsg[1] = OwnID[0];
             	   APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
             	   APP_LOG(TS_ON, VLEVEL_L, "PROBE %s packet sent \n\r",ProbeMsg);

                   memcpy(BufferTx, ProbeMsg, sizeof(ProbeMsg));
                   Radio.Send(BufferTx, PAYLOAD_LEN);
               }

        	   /* DELAY: wake-up time + giving time to the remote node to be in Rx*/
        	  //HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
           }
        break;
   case RX:
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
