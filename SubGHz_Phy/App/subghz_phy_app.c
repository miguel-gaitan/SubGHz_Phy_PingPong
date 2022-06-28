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

#define NODE_ID              		  0	/* 0 is master - otherwise is slave */
#define N_OF_SLAVES					  3

#define RX_TIMEOUT_VALUE              1500
#define TX_TIMEOUT_VALUE              1500

/*Max size of the payload to be sent*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */

/* LED blink Period*/
#define LED_PERIOD_MS                200
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/

/* Radio events function pointer */
static RadioEvents_t RadioEvents;
/* USER CODE BEGIN PV */

static States_t State = RX;
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];

uint16_t RxBufferSize = 0;
int8_t RssiValue = 0;
int8_t SnrValue = 0;

/* Led Timers objects*/
static UTIL_TIMER_Object_t timerLed;

char SrcID = '0';
//int SrcID_int;
int8_t SrcID_int;

char OwnID[1];
//int OwnID_int;
int8_t OwnID_int;

char ConfigCount[1] = "1";		// counter (in char format)
int8_t CounterTx = 1;				// counter
int8_t CounterRx = 1;				// counter

//int DelayFactor;
int8_t DelayFactor;
int8_t NofSlaves;
//int NofSlaves;

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

  OwnID_int = NODE_ID;
  NofSlaves = N_OF_SLAVES;
  OwnID[0] = OwnID_int +'0';

  /* Radio Set frequency (already set to 868000000) */
  Radio.SetChannel(RF_FREQUENCY);

  /* RADIO CONFIGURATION MESSAGE - BW & SF  */
  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "NODE_ID = %s\n\r", OwnID);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_RF_FREQ = %d MHz\n\r",RF_FREQUENCY / 1000000);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW = %d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF = %d\n\r", LORA_SPREADING_FACTOR);

  /* Radio configuration: LORA, TX_PWR, BW, SF, C/R, Preamble, Payload, CRC off, Timeout */
  /* TX : Transmitter configuration---------------------------------------------*/
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  /* RX : Receiver configuration-------------------------------------------------*/
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, 0, 0, 0, LORA_IQ_INVERSION_ON, true);

  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  if (NODE_ID> 0)
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
			APP_LOG(TS_ON, VLEVEL_L, "Buffer size = %d \n\r", RxBufferSize);
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

			//here I need to do something with the REPORT

			if (CounterRx < NofSlaves)
			{
				APP_LOG(TS_ON, VLEVEL_L, "COUNTER RX = %d \n\r",CounterRx);
				CounterRx = CounterRx + 1;
				State = RX;
			}
			else
			{
				APP_LOG(TS_ON, VLEVEL_L, "COUNTER RX = %d (next TX) \n\r",CounterRx);
				CounterRx = 1;
				State = TX;
			}

			//State = RX;
		}
		else
		{
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
			APP_LOG(TS_ON, VLEVEL_L, "OnRxDone -- MASTER: something else \n\r");
			APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
			APP_LOG(TS_ON, VLEVEL_L, "BufferRx=%s \n\r", (char*) BufferRx);
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

			if (CounterRx < NofSlaves)
			{
				APP_LOG(TS_ON, VLEVEL_L, "COUNTER RX = %d \n\r",CounterRx);
				CounterRx = CounterRx + 1;
				State = RX;
			}
			else
			{
				APP_LOG(TS_ON, VLEVEL_L, "COUNTER RX = %d (next TX) \n\r",CounterRx);
				CounterRx = 1;
				State = TX;
			}

			//State = RX;
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
			APP_LOG(TS_ON, VLEVEL_L, "OnRxDone -- SLAVE: CONFIG (OwnID=%s)\n\r",OwnID);
			APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
			APP_LOG(TS_ON, VLEVEL_L, "BufferRx =%s \n\r", (char*) BufferRx);
			APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");

			isProbeRcvd = false;
			State = TX;
		}
		else
		{
			APP_LOG(TS_ON, VLEVEL_L, "Check: BufferRx =%s and ProbeMsg =%s \n\r", (const char*) BufferRx,ProbeMsg);
			APP_LOG(TS_ON, VLEVEL_L, "Buffer size = %d \n\r", RxBufferSize);

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
		if (CounterRx < NofSlaves)
		{
			APP_LOG(TS_ON, VLEVEL_L, "COUNTER RX = %d \n\r",CounterRx);
			CounterRx = CounterRx + 1;
			State = RX;
		}
		else
		{
			APP_LOG(TS_ON, VLEVEL_L, "COUNTER RX = %d (next TX) \n\r",CounterRx);
			CounterRx = 1;
			State = TX;
		}
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

         	   if (CounterTx < NofSlaves)
         	   {
         		   CounterTx = CounterTx + 1;
         	   }
         	   else
         	   {
         		   CounterTx = 1;
         	   }
         	   ConfigCount[0] = CounterTx+'0';
         	   APP_LOG(TS_ON, VLEVEL_L, "Next node to transmit=%s\n\r", ConfigCount);

           }else // SLAVE
           {
               if (isProbeRcvd == true)
               {

            	  ReportMsg[1] = SrcID;
             	  ReportMsg[2] = OwnID[0];

                  memcpy(BufferTx, ReportMsg, 3);

                  SrcID_int = SrcID -'0';
                  OwnID_int = OwnID[0] -'0';

                  /*--- DELAY ---*/
                  if (OwnID_int > SrcID_int)
				  {
                	  DelayFactor = OwnID_int - SrcID_int - 1;
                	  APP_LOG(TS_ON, VLEVEL_L, "Delay factor 1 = OwnID %d - SrcID %d - 1 = %d \n\r",OwnID_int,SrcID_int,DelayFactor);
				  }
                  else
                  {
                	  DelayFactor = NofSlaves - SrcID_int + OwnID_int - 1;
                	  APP_LOG(TS_ON, VLEVEL_L, "Delay factor 2 = NofSlaves %d - OwnID %d - SrcID %d - 1 = %d \n\r",NofSlaves, OwnID_int,SrcID_int,DelayFactor);
                  }
                  HAL_Delay(Radio.GetWakeupTime() + DelayFactor*RX_TIMEOUT_VALUE );
                  Radio.Send(BufferTx, PAYLOAD_LEN);

                  UTIL_TIMER_Stop(&timerLed);
                  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); /* LED_BLUE - on */

                  APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
                  APP_LOG(TS_ON, VLEVEL_L, "REPORT packet sent \n\r");
               }
               else // it was a CONFIG
               {

            	   ProbeMsg[1] = OwnID[0];
                   memcpy(BufferTx, ProbeMsg, 2);
                   Radio.Send(BufferTx, PAYLOAD_LEN);

                   UTIL_TIMER_Stop(&timerLed);
                   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                   HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
                   HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN - on */

             	   APP_LOG(TS_OFF, VLEVEL_M, "-----------------------\n\r");
             	   APP_LOG(TS_ON, VLEVEL_L, "PROBE %s packet sent \n\r",ProbeMsg);

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
