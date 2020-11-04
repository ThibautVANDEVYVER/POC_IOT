/**
  ******************************************************************************
  * @file           : gsm.c
  * @brief          : GSM driver
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gsm.h"

/* Define --------------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

osMessageQueueId_t MsgTcpRx;
osMessageQueueId_t MsgTcpTx;
osMessageQueueId_t MsgTcpStatus;

osThreadId_t gsmTaskHandle;
const osThreadAttr_t gsmTask_attributes =
{
  .name = "gsmTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
#define GSM_BUF_SIZE									(64)
#define GSM_TCP_SEND_NB								(8)

#define	GSM_IP_ADDRESS								("dpedesign.freeboxos.fr")	// Server IP address
#define	GSM_TCP_PORT									("23456")										// Server TCP port

// INIT
#define	GSM_POWER_OFF_TIME						(200)
#define	GSM_POWER_ON_TIME							(100)
#define	GSM_PRESS_TIME								(600)
#define	GSM_CK_STATUS_TIMEOUT					(10000)

// START
#define	GSM_START_RETRY								(20)

// TCP CONNECT
#define	GSM_TCP_CONNECT_TX_STR				("AT+QIOPEN=1,0,\"TCP\",\"%s\",%s,0,1\r\n")	// Open a Socket Service
#define	GSM_TCP_CONNECT_RX_STR				("+QIOPEN: 0,0\r\n")
#define	GSM_TCP_CONNECT_TIMEOUT				(5000)
#define GSM_TCP_CONNECT_RETRY					(5)

// TCP SEND
#define	GSM_TCP_SEND1_TX_STR					("AT+QISEND=0,%d\r\n")
#define	GSM_TCP_SEND1_RX_STR					("> ")
#define	GSM_TCP_SEND1_TIMEOUT					(3000)
#define	GSM_TCP_SEND2_RX_STR					("SEND OK\r\n")
#define	GSM_TCP_SEND2_TIMEOUT					(3000)
#define	GSM_TCP_SEND_RETRY						(10)

// TCP RECV
#define	GSM_TCP_RECV_RX_STR						("+QIURC: ")
#define	GSM_TCP_RECV_CLOSE_STR				("\"closed\",0")
#define	GSM_TCP_RECV_DATA_STR					("\"recv\",0,")
#define	GSM_TCP_RECV_TIMEOUT					(60000)							// Timeout after 1st byte received

// STOP
#define	GSM_STOP_RETRY								(1)

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	GSM_STATE_INIT = 0,
	GSM_STATE_CK_STATUS,
	GSM_STATE_START_INIT,
	GSM_STATE_START_TX,
	GSM_STATE_START_RX,
	GSM_STATE_REG_INIT,
	GSM_STATE_REG_TX,
	GSM_STATE_REG_RX,
	GSM_STATE_CONNECT_INIT,
	GSM_STATE_CONNECT_TX,
	GSM_STATE_CONNECT_RX,
	GSM_STATE_TCP_CONNECT_INIT,
	GSM_STATE_TCP_CONNECT_TX,
	GSM_STATE_TCP_CONNECT_RX,
	GSM_STATE_TCP_RECV_INIT,
	GSM_STATE_TCP_RECV_RX,
	GSM_STATE_TCP_SEND_INIT,
	GSM_STATE_TCP_SEND1_TX,
	GSM_STATE_TCP_SEND1_RX,
	GSM_STATE_TCP_SEND2_TX,
	GSM_STATE_TCP_SEND2_RX,
	GSM_STATE_TCP_STATUS_INIT,
	GSM_STATE_TCP_STATUS_TX,
	GSM_STATE_TCP_STATUS_RX,
	GSM_STATE_STOP_INIT,
	GSM_STATE_STOP_TX,
	GSM_STATE_STOP_RX,
} gsm_state_t;

typedef struct
{
	char txbuf[GSM_BUF_SIZE];
	char rxbuf1[GSM_BUF_SIZE];
	char rxbuf2[GSM_BUF_SIZE];
	uint32_t time;
	uint8_t skip;
} StructAT;

/* Constants -----------------------------------------------------------------*/
const StructAT sGsmATStart[] =
{
	{"AT\r\n"              									, ""						, "OK\r\n"	, 1000	, 0},	// Check UART communication
	{"ATE0\r\n"         										, ""						, "OK\r\n"	, 1000	, 0},	// Disable ECHO
//	{"ATI\r\n"         											, ""						, "OK\r\n"	, 1000	, 0},
//	{"AT+CMEE=2\r\n"        								, ""						, "OK\r\n"	, 1000	, 0},	// Enable Report Error
//	{"AT+CPIN?\r\n"         								, ""						, "OK\r\n"	, 1000	, 0},  // Report PIN state
//	{"AT+GSN\r\n"           								, ""						, "OK\r\n"	, 1000	, 0},	// Request International Mobile Equipment Identity (IMEI)
//	{"AT+QCCID\r\n"          								, ""						, "OK\r\n"	, 1000	, 0},	// Show ICCID
//	{"AT+CIMI\r\n"           								, ""						, "OK\r\n"	, 1000	, 0},	// Request International Mobile Subscriber Identity (IMSI)
//	{"AT+QGMR\r\n"													, ""						, "OK\r\n"	, 1000	, 0},	// Request Modem and Application Firmware Versions
//	{"AT+QCSCON=1\r\n"											, ""						, "OK\r\n"	, 1000}	, 0,	// Signaling Connection Status
	{"AT+CGDCONT=1,\"IP\",\"bicsapn\"\r\n"	, ""						, "OK\r\n"	, 1000	, 0},	// Define PDP Context
	{"AT+QICSGP=1,1,\"bicsapn\"\r\n"				, ""						, "OK\r\n"	, 1000	, 0},	// Configure Parameters of a TCP/IP Context
	{"AT+QPSMS=0\r\n"												, ""						, "OK\r\n"	, 1000	, 0},	// Power Saving Mode disabled
	{"AT+COPS=0\r\n"         								, ""						, "OK\r\n"	, 1000	, 0},	// Operator Selection = Automatic mode
	
	{"AT+CFUN=0\r\n"												, ""						, "OK\r\n"	, 1000	, 0},		// Set minimum functionnality
//	{"AT+QENG=\"servingcell\"\r\n"					, ""						, "OK\r\n"	, 1000	, 0},	// Query the information of serving cell
//	{"AT+QENG=\"neighbourcell\"\r\n"				, ""						, "OK\r\n"	, 1000	, 0},	// Query the information of neighbour cells
	{"AT+QCFG=\"servicedomain\",1,1\r\n"		, ""						, "OK\r\n"	, 1000	, 0},		// Service domain of UE : PS only
	{"AT+QCFG=\"nwscanseq\",020301,1\r\n"		, ""						, "OK\r\n"	, 1000	, 0},	// Set scanning network : LTE-M1 -> NB1 -> GSM
	{"AT+QCFG=\"nwscanmode\",0\r\n"      		, ""						, "OK\r\n"	, 1000	, 0},	// Configure RAT searched : Automatically
	{"AT+QCFG=\"iotopmode\",0,1\r\n"      	, ""						, "OK\r\n"	, 1000	, 0},	// Configure network category searched : LTE-M1	
	{"AT+QCFG=\"band\",0,80084,0,1\r\n"   	, ""						, "OK\r\n"	, 1000	, 0},	// Choose communication band : GSM no change / LTE-M1 B3/B8/B20 / NB1 no change
	{"AT+CFUN=1\r\n"												, ""						, "OK\r\n"	, 1000	, 0},	// Switch to full functionality
	
//	{"AT+CSQ\r\n"														, ""						, "OK\r\n"	, 1000	, 0},	// Signal Quality Report
//	{"AT+QCSQ\r\n"													, ""						, "OK\r\n"	, 1000	, 0},	// Query and Report Signal Strength
//	{"AT+QNWINFO\r\n"												, ""						, "OK\r\n"	, 1000	, 0},	// Query Network Information
//	{"AT+CGPADDR=1\r\n"											, ""						, "OK\r\n"	, 1000	, 0},	// Show PDP Address

	{"AT+COPS?\r\n"         								, "+COPS: 0,0"	, "\r\n"		, 1000	, 0},	// Query Operator

	{"AT+CEREG?\r\n"												, "+CEREG: 0,5" , "\r\n"		, 2000	, 5},	// Query LTE Network Registration Status => Roaming
	{"AT+CEREG?\r\n"												, "+CEREG: 0,1" , "\r\n"		, 2000	, 4},	// Query LTE Network Registration Status => Home network
	{"AT+CGREG?\r\n"												, "+CGREG: 0,5" , "\r\n"		, 2000	, 3},	// Query GPRS Network Registration Status => Roaming
	{"AT+CGREG?\r\n"												, "+CGREG: 0,1" , "\r\n"		, 2000	, 2},	// Query GPRS Network Registration Status => Home network
	{"AT+CREG?\r\n"													, "+CREG: 0,5"	, "\r\n"	  , 2000	, 1},	// Query GSM Network Registration Status => Roaming
	{"AT+CREG?\r\n"													, "+CREG: 0,1"	, "\r\n"	 	, 2000	, 0},	// Query GSM Network Registration Status => Home network
	
	{"AT+QIACT=1\r\n"												, ""						, "OK\r\n"	, 1000	, 0},	// Activate a PDP Context
//	{"AT+QIACT?\r\n"												, ""						, "OK\r\n"	, 1000},	// Query the PDP Context
};

const StructAT sGsmATStop[] =
{
	{"AT+QICLOSE=0\r\n"    									, "" , "OK\r\n"			, 1000	, 0},	// Close a Socket Service
	{"AT+QIDEACT=1\r\n"											, "" , "OK\r\n"	   	, 1000	, 0},	// Desactivate a PDP context
	{"AT+QPOWD=1\r\n"       								, "" , "OK\r\n"			, 1000	, 0},	// Power down
};

/* Private variables ---------------------------------------------------------*/
struct
{
	gsm_state_t state;
	gsm_status_t status;
	uint32_t idx;
	uint32_t time;
	uint32_t retry;
	uint32_t rxsize;
	uint8_t txbuf[GSM_BUF_SIZE + 1];
	uint8_t rxbuf[GSM_BUF_SIZE + 1];
	char ip_addr[32];
	char tcp_port[32];
} gsm;

static gsm_msg_tcp_t msg_tcp_rx;
static gsm_msg_tcp_t msg_tcp_tx;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief GSM init
  */
static void GSM_Init(void)
{
	// Init variables
	gsm.state = GSM_STATE_INIT;
	gsm.status = GSM_STATUS_STOP;
	gsm.idx = 0;
	gsm.time = 0;
	gsm.retry = 0;
	gsm.rxsize = 0;
	memset(gsm.txbuf, 0, sizeof(gsm.txbuf));
	memset(gsm.rxbuf, 0, sizeof(gsm.rxbuf));
	strcpy(gsm.ip_addr, GSM_IP_ADDRESS);
	strcpy(gsm.tcp_port, GSM_TCP_PORT);
	
	// Init UART
	MX_DMA_Init();
	MX_USART3_UART_Init();	
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  GSM task
  * @param  argument: Not used
  * @retval None
  */
void StartGsmTask(void *argument)
{
	// Init Messages
	MsgTcpRx = osMessageQueueNew(GSM_MSG_TCP_NB, sizeof(gsm_msg_tcp_t), NULL);
  MsgTcpTx = osMessageQueueNew(GSM_MSG_TCP_NB, sizeof(gsm_msg_tcp_t), NULL);
  MsgTcpStatus = osMessageQueueNew(1, sizeof(gsm_status_t), NULL);
	
	// Init state
	gsm.state = GSM_STATE_INIT;
	
  /* Infinite loop */
  while (1)
  {
		// Update received size
		uint32_t remain_size = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		gsm.rxsize = (GSM_BUF_SIZE > remain_size) ? (GSM_BUF_SIZE - remain_size) : 0;
	
		// State
		switch (gsm.state)
		{
			case GSM_STATE_INIT:
			{
				// Init GSM
				DPRINT("%04d| GSM : INIT\n", HAL_GetTick()/1000);
				GSM_Init();
				// Reset module
				DPRINT("%04d| GSM : POWER OFF\n", HAL_GetTick()/1000);
				GPIO_Clear(OUT_PWRKEY_GSM);
				GPIO_Set(OUT_RST_GSM);
				osDelay(GSM_POWER_OFF_TIME);
				DPRINT("%04d| GSM : POWER ON\n", HAL_GetTick()/1000);
				GPIO_Clear(OUT_PWRKEY_GSM);
				GPIO_Clear(OUT_RST_GSM);
				osDelay(GSM_POWER_ON_TIME);
				// Turn ON module
				DPRINT("%04d| GSM : PWRKEY PRESSED\n", HAL_GetTick()/1000);
				GPIO_Set(OUT_PWRKEY_GSM);
				osDelay(GSM_PRESS_TIME);
				DPRINT("%04d| GSM : PWRKEY RELEASED\n", HAL_GetTick()/1000);
				GPIO_Clear(OUT_PWRKEY_GSM);
				// Next state
				gsm.time = HAL_GetTick();
				gsm.state = GSM_STATE_CK_STATUS;
			}
			break;
			
			case GSM_STATE_CK_STATUS:
			{
				// Check status
				if (GPIO_Read(IN_STATUS_GSM) == GPIO_PIN_SET)
				{
					DPRINT("%04d| GSM : STATUS OK\n", HAL_GetTick()/1000);
					gsm.state = GSM_STATE_START_INIT;
				}
				// Check timeout
				if ((HAL_GetTick() - gsm.time) > GSM_CK_STATUS_TIMEOUT)
				{
					DPRINT("%04d| GSM : STATUS TIMEOUT\n", HAL_GetTick()/1000);
					gsm.state = GSM_STATE_INIT;
				}
			}			
			break;

			case GSM_STATE_START_INIT:
			{
				DPRINT("%04d| GSM : AT START\n", HAL_GetTick()/1000);
				gsm.idx = 0;
				gsm.retry = 0;
				gsm.state = GSM_STATE_START_TX;
			}
			break;
			case GSM_STATE_START_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(gsm.rxbuf, 0, sizeof(gsm.rxbuf));
				HAL_UART_Receive_DMA(&huart3, gsm.rxbuf, GSM_BUF_SIZE);
				// Send command
				memset(gsm.txbuf, 0, sizeof(gsm.txbuf));
				strcpy((char *)gsm.txbuf, sGsmATStart[gsm.idx].txbuf);
				DPRINT("%04d| GSM : AT START %d -> TX\n", HAL_GetTick()/1000, gsm.idx);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.txbuf);
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)gsm.txbuf, strlen((char *)gsm.txbuf));			
				gsm.time = HAL_GetTick();
				gsm.state = GSM_STATE_START_RX;
			}
			break;
			case GSM_STATE_START_RX:
			{
				// Check timeout
				if ((HAL_GetTick() - gsm.time) > sGsmATStart[gsm.idx].time)
				{
					DPRINT("%04d| GSM : AT START %d -> TIMEOUT\n", HAL_GetTick()/1000, gsm.idx);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.rxbuf);
#endif
					gsm.retry++;
					if (gsm.retry > GSM_START_RETRY)
					{
						if (sGsmATStart[gsm.idx].skip > 0)
						{
							gsm.idx++; 
							uint32_t nb_at = sizeof(sGsmATStart) / sizeof(sGsmATStart[0]);
							if (gsm.idx < nb_at) {gsm.state = GSM_STATE_START_TX;}
							else {gsm.state = GSM_STATE_TCP_CONNECT_INIT;}
						}
						else
						{
							gsm.state = GSM_STATE_INIT;
						}
					}
					else
					{
						gsm.state = GSM_STATE_START_TX;
					}
				}
				else
				{
					// Check received buffer
					char *ptr = strstr((char *)gsm.rxbuf, sGsmATStart[gsm.idx].rxbuf1);
					if (ptr != NULL)
					{
						ptr = strstr(ptr, sGsmATStart[gsm.idx].rxbuf2);
						if (ptr != NULL)
						{
							DPRINT("%04d| GSM : AT START %d -> RX OK\n", HAL_GetTick()/1000, gsm.idx);
#ifdef PRINT_GSM_RX
							DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.rxbuf);
#endif
							gsm.idx += 1 + sGsmATStart[gsm.idx].skip;
							uint32_t nb_at = sizeof(sGsmATStart) / sizeof(sGsmATStart[0]);
							if (gsm.idx < nb_at) {gsm.state = GSM_STATE_START_TX;}
							else {gsm.state = GSM_STATE_TCP_CONNECT_INIT;}
						}
					}
				}
			}
			break;
		
			case GSM_STATE_TCP_CONNECT_INIT:
			{
				DPRINT("%04d| GSM : TCP CONNECT\n", HAL_GetTick()/1000);
				gsm.retry = 0;
				gsm.state = GSM_STATE_TCP_CONNECT_TX;
			}
			break;
			case GSM_STATE_TCP_CONNECT_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(gsm.rxbuf, 0, sizeof(gsm.rxbuf));
				HAL_UART_Receive_DMA(&huart3, gsm.rxbuf, GSM_BUF_SIZE);
				// Send command
				sprintf((char *)gsm.txbuf, GSM_TCP_CONNECT_TX_STR, gsm.ip_addr, gsm.tcp_port);
				DPRINT("%04d| GSM : TCP CONNECT -> TX\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.txbuf);
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)gsm.txbuf, strlen((char *)gsm.txbuf));			
				gsm.time = HAL_GetTick();
				gsm.state = GSM_STATE_TCP_CONNECT_RX;
				break;
			case GSM_STATE_TCP_CONNECT_RX:
				// Check timeout
				if ((HAL_GetTick() - gsm.time) > GSM_TCP_CONNECT_TIMEOUT)
				{
					DPRINT("%04d| GSM : TCP CONNECT -> TIMEOUT\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.rxbuf);
#endif
					gsm.retry++;
					if (gsm.retry > GSM_TCP_CONNECT_RETRY) {gsm.state = GSM_STATE_STOP_INIT;}
					else {gsm.state = GSM_STATE_TCP_CONNECT_TX;}
				}
				else if (strstr((char *)gsm.rxbuf, GSM_TCP_CONNECT_RX_STR) != NULL)
				{
					DPRINT("%04d| GSM : TCP CONNECT -> RX OK\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.rxbuf);
#endif
					gsm.state = GSM_STATE_TCP_RECV_INIT;
				}
			}
			break;
			
			case GSM_STATE_TCP_RECV_INIT:
			{
				DPRINT("%04d| GSM : TCP RECV\n", HAL_GetTick()/1000);
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(gsm.rxbuf, 0, sizeof(gsm.rxbuf));
				HAL_UART_Receive_DMA(&huart3, gsm.rxbuf, GSM_BUF_SIZE);
				gsm.time = HAL_GetTick();
				gsm.state = GSM_STATE_TCP_RECV_RX;
			}
			break;
			case GSM_STATE_TCP_RECV_RX:
			{
				// Check data to send
				if (osMessageQueueGet(MsgTcpTx, &msg_tcp_tx, NULL, 0U) == osOK)
				{
					DPRINT("%04d| GSM : TCP MSG= %s\n", HAL_GetTick()/1000, (char *)msg_tcp_tx.data);
					gsm.state = GSM_STATE_TCP_SEND_INIT;
				}
				// Check received buffer
				else
				{
					char *ptr1;
					uint32_t idx = 0;
					while( ((ptr1 = strstr((char *)&gsm.rxbuf[idx], GSM_TCP_RECV_RX_STR)) == NULL) && (idx < GSM_BUF_SIZE) ) {idx++;}
					if (idx < GSM_BUF_SIZE)
					{
						ptr1 += strlen(GSM_TCP_RECV_RX_STR);
						// Check Connection Closed
						if (strstr(ptr1, GSM_TCP_RECV_CLOSE_STR) != NULL)
						{
							gsm.state = GSM_STATE_STOP_INIT;
						}
						// Check Incoming Data 
						else if (strstr(ptr1, GSM_TCP_RECV_DATA_STR) != NULL)
						{
							ptr1 += strlen(GSM_TCP_RECV_DATA_STR);
							// Check size of received buffer
							uint32_t data_size;
							if (sscanf(ptr1, "%d\r\n", &data_size) > 0)
							{
								if (data_size < GSM_BUF_SIZE)
								{
									if ((ptr1 = strstr(ptr1, "\r\n")) != NULL)
									{
										ptr1 += strlen("\r\n");
										uint32_t recv_size = (uint32_t)gsm.rxbuf + gsm.rxsize;
										recv_size = (recv_size > (uint32_t)ptr1) ? (recv_size - (uint32_t)ptr1) : 0;
										if (recv_size >= data_size)
										{
											DPRINT("%04d| GSM : TCP RECV -> RX OK\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
											char *ptr2 = ptr1; while( (isalnum(*ptr2) || ispunct(*ptr2) || isspace(*ptr2)) && (*ptr2 != '\0') ) {ptr2++;}
											if (*ptr2 == '\0') {DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, ptr1);}
											else {DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, hex2Str((uint8_t *)ptr1, data_size));}//strlen(ptr1)));}
#endif
											// Update received data
											memcpy(msg_tcp_rx.data, ptr1, data_size);
											msg_tcp_rx.size = data_size;
											osMessageQueuePut(MsgTcpRx, &msg_tcp_rx, NULL, 0U);
											// Relaunch reception
											gsm.state = GSM_STATE_TCP_RECV_INIT;
										}
									}
								}
							}
						}
					}
				}
				// Check timeout
				if ((HAL_GetTick() - gsm.time) > GSM_TCP_RECV_TIMEOUT)
				{
					// Relaunch reception
					gsm.state = GSM_STATE_TCP_RECV_INIT;
				}
			}
			break;
		
			case GSM_STATE_TCP_SEND_INIT:
			{
				DPRINT("%04d| GSM : TCP SEND\n", HAL_GetTick()/1000);
				gsm.retry = 0;
				gsm.state = GSM_STATE_TCP_SEND1_TX;
			}
			break;
			case GSM_STATE_TCP_SEND1_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(gsm.rxbuf, 0, sizeof(gsm.rxbuf));
				HAL_UART_Receive_DMA(&huart3, gsm.rxbuf, GSM_BUF_SIZE);
				// Send command
				sprintf((char *)gsm.txbuf, GSM_TCP_SEND1_TX_STR, msg_tcp_tx.size);
				DPRINT("%04d| GSM : TCP SEND 1 -> TX\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.txbuf);
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)gsm.txbuf, strlen((char *)gsm.txbuf));			
				gsm.time = HAL_GetTick();
				gsm.state = GSM_STATE_TCP_SEND1_RX;
			}
			break;
			case GSM_STATE_TCP_SEND1_RX:
			{
				// Check timeout
				if ((HAL_GetTick() - gsm.time) > GSM_TCP_SEND1_TIMEOUT)
				{
					DPRINT("%04d| GSM : TCP SEND 1 -> TIMEOUT\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.rxbuf);
#endif
					gsm.retry++;
					if (gsm.retry > GSM_TCP_SEND_RETRY) {gsm.state = GSM_STATE_STOP_INIT;}
					else {gsm.state = GSM_STATE_TCP_SEND1_TX;}
				}
				// Check received buffer
				else if (strstr((char *)gsm.rxbuf, GSM_TCP_SEND1_RX_STR) != NULL)
				{
					DPRINT("%04d| GSM : TCP SEND 1 -> RX OK\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.rxbuf);
#endif
					gsm.state = GSM_STATE_TCP_SEND2_TX;
				}
			}
			break;
			case GSM_STATE_TCP_SEND2_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(gsm.rxbuf, 0, sizeof(gsm.rxbuf));
				HAL_UART_Receive_DMA(&huart3, gsm.rxbuf, GSM_BUF_SIZE);
				// Send command
				memset(gsm.txbuf, 0, sizeof(gsm.txbuf));
				memcpy(gsm.txbuf, msg_tcp_tx.data, msg_tcp_tx.size);
				DPRINT("%04d| GSM : TCP SEND 2 -> TX\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, hex2Str(gsm.txbuf, msg_tcp_tx.size));
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)gsm.txbuf, msg_tcp_tx.size);
				gsm.time = HAL_GetTick();
				gsm.state = GSM_STATE_TCP_SEND2_RX;
			}
			break;
			case GSM_STATE_TCP_SEND2_RX:
			{
				// Check timeout
				if ((HAL_GetTick() - gsm.time) > GSM_TCP_SEND2_TIMEOUT)
				{
					DPRINT("%04d| GSM : TCP SEND 2 -> TIMEOUT\n", HAL_GetTick()/1000);
					gsm.retry++;
					if (gsm.retry > GSM_TCP_SEND_RETRY) {gsm.state = GSM_STATE_STOP_INIT;}
					else {gsm.state = GSM_STATE_TCP_SEND1_TX;}
				}
				// Check received buffer
				else
				{
					uint32_t idx = 0, idx_max = (GSM_BUF_SIZE > strlen(GSM_TCP_SEND2_RX_STR)) ? (GSM_BUF_SIZE - strlen(GSM_TCP_SEND2_RX_STR)) : 0;
					while( (memcmp(&gsm.rxbuf[idx], GSM_TCP_SEND2_RX_STR, strlen(GSM_TCP_SEND2_RX_STR)) != 0) && (idx < idx_max) ) {idx++;}
					if (idx < idx_max)
					{
						DPRINT("%04d| GSM : TCP SEND 2 -> RX OK\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
						DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.rxbuf);
#endif
						// Relaunch reception
						gsm.state = GSM_STATE_TCP_RECV_INIT;
					}
				}
			}
			break;

			case GSM_STATE_STOP_INIT:
			{
				DPRINT("%04d| GSM : AT STOP\n", HAL_GetTick()/1000);
				gsm.idx = 0;
				gsm.retry = 0;
				gsm.state = GSM_STATE_STOP_TX;
			}
			break;
			case GSM_STATE_STOP_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(gsm.rxbuf, 0, sizeof(gsm.rxbuf));
				HAL_UART_Receive_DMA(&huart3, gsm.rxbuf, GSM_BUF_SIZE);
				// Send command
				memset(gsm.txbuf, 0, sizeof(gsm.txbuf));
				strcpy((char *)gsm.txbuf, sGsmATStop[gsm.idx].txbuf);
				DPRINT("%04d| GSM : AT STOP %d -> TX\n", HAL_GetTick()/1000, gsm.idx);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.txbuf);
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)gsm.txbuf, strlen((char *)gsm.txbuf));			
				gsm.time = HAL_GetTick();
				gsm.state = GSM_STATE_STOP_RX;
			}
			break;
			case GSM_STATE_STOP_RX:
			{
				// Check timeout
				if ((HAL_GetTick() - gsm.time) > sGsmATStop[gsm.idx].time)
				{
					DPRINT("%04d| GSM : AT STOP %d -> TIMEOUT\n", HAL_GetTick()/1000, gsm.idx);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.rxbuf);
#endif
					gsm.retry++;
					if (gsm.retry > GSM_STOP_RETRY) {gsm.state = GSM_STATE_INIT;}
					else {gsm.state = GSM_STATE_STOP_TX;}
				}
				else
				{
					// Check received buffer
					char *ptr = strstr((char *)gsm.rxbuf, sGsmATStop[gsm.idx].rxbuf1);
					if (ptr != NULL)
					{
						ptr = strstr(ptr, sGsmATStop[gsm.idx].rxbuf2);
						if (ptr != NULL)
						{
							DPRINT("%04d| GSM : AT STOP %d -> RX OK\n", HAL_GetTick()/1000, gsm.idx);
#ifdef PRINT_GSM_RX
							DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, gsm.rxbuf);
#endif
							gsm.idx++;
							uint32_t nb_at = sizeof(sGsmATStop) / sizeof(sGsmATStop[0]);
							if (gsm.idx < nb_at) {gsm.state = GSM_STATE_STOP_TX;}
							else {gsm.state = GSM_STATE_INIT;}
						}
					}
				}
			}
			break;

			default: break;
		}
		
		// GSM status
		gsm_status_t status;
		if (gsm.state <= GSM_STATE_CK_STATUS) {status = GSM_STATUS_STOP;}
		else if (gsm.state < GSM_STATE_TCP_CONNECT_INIT) {status = GSM_STATUS_DISCONNECTED;}
		else if (gsm.state < GSM_STATE_TCP_RECV_INIT) {status = GSM_STATUS_CONNECTING;}
		else {status = GSM_STATUS_CONNECTED;}
		if (status != gsm.status)
		{
			gsm.status = status;
			osMessageQueueReset(MsgTcpStatus);
			osMessageQueuePut(MsgTcpStatus, &gsm.status, 0U, 0U);
		}
		
		// Delay
		osDelay(10);
	}
}
