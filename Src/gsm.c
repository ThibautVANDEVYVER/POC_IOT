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

#include "cmsis_os.h"
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gsm.h"


/* Public variables ----------------------------------------------------------*/

osMessageQueueId_t MsgTcpRx;
osMessageQueueId_t MsgTcpTx;

osThreadId_t gsmTaskHandle;
const osThreadAttr_t gsmTask_attributes =
{
  .name = "gsmTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};


/* Defines -------------------------------------------------------------------*/
#define GSM_BUF_LEN										(64)

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

// SOCKET
#define SOCKET_INIT_TIMEOUT						(10000)
#define SOCKET_OPEN_TIMEOUT						(10000)

#define GSM_HOSTNAME_LEN							(32)
#define GSM_TCP_PORT_LEN							(6)

#define GSM_MSG_TCP_NB	        			(8)
#define GSM_MSG_TCP_LEN   						(64)

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
	char tx_buf[GSM_BUF_LEN];
	char rx_buf1[GSM_BUF_LEN];
	char rx_buf2[GSM_BUF_LEN];
	uint32_t time;
	uint8_t skip;
} StructAT;

typedef enum
{
	GSM_STATUS_STOP				= 0,
	GSM_STATUS_INIT,
	GSM_STATUS_CONNECTING,
	GSM_STATUS_CONNECTED,
} gsm_status_t;

typedef struct
{
	uint8_t data[GSM_MSG_TCP_LEN];
	uint32_t len;
} gsm_msg_tcp_t;


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

static struct
{
	gsm_state_t state;
	uint32_t init				: 1;
	uint32_t connect		: 1;
	uint32_t close			: 1;
	uint32_t dummy			: 29;
	char hostname[GSM_HOSTNAME_LEN];
	char tcp_port[GSM_TCP_PORT_LEN];
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
	memset(&gsm, 0, sizeof(gsm));
	gsm.state = GSM_STATE_INIT;
	
	// Init UART
	MX_DMA_Init();
	MX_USART3_UART_Init();	
}

/* Extern functions ----------------------------------------------------------*/

/**
  * @brief  GSM task
  * @param  argument: Not used
  * @retval None
  */
void StartGsmTask(void *argument)
{
	uint32_t idx;
	uint32_t time;
	uint32_t retry;
	uint32_t rx_len;
	uint8_t tx_buf[GSM_BUF_LEN + 1];
	uint8_t rx_buf[GSM_BUF_LEN + 1];
	
	// Init Messages
	MsgTcpRx = osMessageQueueNew(GSM_MSG_TCP_NB, sizeof(gsm_msg_tcp_t), NULL);
  MsgTcpTx = osMessageQueueNew(GSM_MSG_TCP_NB, sizeof(gsm_msg_tcp_t), NULL);

	// Init GSM
	GSM_Init();
	
  /* Infinite loop */
  while (1)
  {
		// Update received size
		uint32_t remain_len = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		rx_len = (GSM_BUF_LEN > remain_len) ? (GSM_BUF_LEN - remain_len) : 0;
		
		// State
		switch (gsm.state)
		{
			case GSM_STATE_INIT:
			{
				// Reset module
				GPIO_Clear(OUT_PWRKEY_GSM);
				GPIO_Set(OUT_RST_GSM);
				// Wait for init
				if (gsm.init)
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
					time = HAL_GetTick();
					gsm.state = GSM_STATE_CK_STATUS;
				}
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
				if ((HAL_GetTick() - time) > GSM_CK_STATUS_TIMEOUT)
				{
					DPRINT("%04d| GSM : STATUS TIMEOUT\n", HAL_GetTick()/1000);
					gsm.state = GSM_STATE_INIT;
				}
			}			
			break;

			case GSM_STATE_START_INIT:
			{
				DPRINT("%04d| GSM : AT START\n", HAL_GetTick()/1000);
				idx = 0;
				retry = 0;
				gsm.state = GSM_STATE_START_TX;
			}
			break;
			case GSM_STATE_START_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(rx_buf, 0, sizeof(rx_buf));
				HAL_UART_Receive_DMA(&huart3, rx_buf, GSM_BUF_LEN);
				// Send command
				memset(tx_buf, 0, sizeof(tx_buf));
				strcpy((char *)tx_buf, sGsmATStart[idx].tx_buf);
				DPRINT("%04d| GSM : AT START %d -> TX\n", HAL_GetTick()/1000, idx);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, tx_buf);
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx_buf, strlen((char *)tx_buf));			
				time = HAL_GetTick();
				gsm.state = GSM_STATE_START_RX;
			}
			break;
			case GSM_STATE_START_RX:
			{
				// Check timeout
				if ((HAL_GetTick() - time) > sGsmATStart[idx].time)
				{
					DPRINT("%04d| GSM : AT START %d -> TIMEOUT\n", HAL_GetTick()/1000, idx);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, rx_buf);
#endif
					retry++;
					if (retry > GSM_START_RETRY)
					{
						if (sGsmATStart[idx].skip > 0)
						{
							idx++; 
							uint32_t nb_at = sizeof(sGsmATStart) / sizeof(sGsmATStart[0]);
							if (idx < nb_at) {gsm.state = GSM_STATE_START_TX;}
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
					char *ptr = strstr((char *)rx_buf, sGsmATStart[idx].rx_buf1);
					if (ptr != NULL)
					{
						ptr = strstr(ptr, sGsmATStart[idx].rx_buf2);
						if (ptr != NULL)
						{
							DPRINT("%04d| GSM : AT START %d -> RX OK\n", HAL_GetTick()/1000, idx);
#ifdef PRINT_GSM_RX
							DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, rx_buf);
#endif
							idx += 1 + sGsmATStart[idx].skip;
							uint32_t nb_at = sizeof(sGsmATStart) / sizeof(sGsmATStart[0]);
							if (idx < nb_at) {gsm.state = GSM_STATE_START_TX;}
							else {gsm.state = GSM_STATE_TCP_CONNECT_INIT;}
						}
					}
				}
			}
			break;
		
			case GSM_STATE_TCP_CONNECT_INIT:
			{
				if (gsm.connect)
				{
					DPRINT("%04d| GSM : TCP CONNECT\n", HAL_GetTick()/1000);
					retry = 0;
					gsm.state = GSM_STATE_TCP_CONNECT_TX;
				}
			}
			break;
			case GSM_STATE_TCP_CONNECT_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(rx_buf, 0, sizeof(rx_buf));
				HAL_UART_Receive_DMA(&huart3, rx_buf, GSM_BUF_LEN);
				// Send command
				sprintf((char *)tx_buf, GSM_TCP_CONNECT_TX_STR, gsm.hostname, gsm.tcp_port);
				DPRINT("%04d| GSM : TCP CONNECT -> TX\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, tx_buf);
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx_buf, strlen((char *)tx_buf));			
				time = HAL_GetTick();
				gsm.state = GSM_STATE_TCP_CONNECT_RX;
				break;
			case GSM_STATE_TCP_CONNECT_RX:
				// Check timeout
				if ((HAL_GetTick() - time) > GSM_TCP_CONNECT_TIMEOUT)
				{
					DPRINT("%04d| GSM : TCP CONNECT -> TIMEOUT\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, rx_buf);
#endif
					retry++;
					if (retry > GSM_TCP_CONNECT_RETRY) {gsm.state = GSM_STATE_STOP_INIT;}
					else {gsm.state = GSM_STATE_TCP_CONNECT_TX;}
				}
				else if (strstr((char *)rx_buf, GSM_TCP_CONNECT_RX_STR) != NULL)
				{
					DPRINT("%04d| GSM : TCP CONNECT -> RX OK\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, rx_buf);
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
				memset(rx_buf, 0, sizeof(rx_buf));
				HAL_UART_Receive_DMA(&huart3, rx_buf, GSM_BUF_LEN);
				time = HAL_GetTick();
				gsm.state = GSM_STATE_TCP_RECV_RX;
			}
			break;
			case GSM_STATE_TCP_RECV_RX:
			{
				// Check data to send
				if (osMessageQueueGet(MsgTcpTx, &msg_tcp_tx, NULL, 0U) == osOK)
				{
					msg_tcp_tx.data[msg_tcp_tx.len] = '\0';
					DPRINT("%04d| GSM : TCP MSG= %s\n", HAL_GetTick()/1000, (char *)msg_tcp_tx.data);
					gsm.state = GSM_STATE_TCP_SEND_INIT;
				}
				// Check received buffer
				else
				{
					char *ptr1;
					uint32_t idx = 0;
					while( ((ptr1 = strstr((char *)&rx_buf[idx], GSM_TCP_RECV_RX_STR)) == NULL) && (idx < GSM_BUF_LEN) ) {idx++;}
					if (idx < GSM_BUF_LEN)
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
							uint32_t data_len;
							if (sscanf(ptr1, "%d\r\n", &data_len) > 0)
							{
								if (data_len < GSM_BUF_LEN)
								{
									if ((ptr1 = strstr(ptr1, "\r\n")) != NULL)
									{
										ptr1 += strlen("\r\n");
										uint32_t recv_len = (uint32_t)rx_buf + rx_len;
										recv_len = (recv_len > (uint32_t)ptr1) ? (recv_len - (uint32_t)ptr1) : 0;
										if (recv_len >= data_len)
										{
											DPRINT("%04d| GSM : TCP RECV -> RX OK\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
											ptr1[data_len] = '\0';
//											DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, ptr1);
#endif
											// Update received data
											memcpy(msg_tcp_rx.data, ptr1, data_len);
											msg_tcp_rx.len = data_len;
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
				if ((HAL_GetTick() - time) > GSM_TCP_RECV_TIMEOUT)
				{
					// Relaunch reception
					gsm.state = GSM_STATE_TCP_RECV_INIT;
				}
			}
			break;
		
			case GSM_STATE_TCP_SEND_INIT:
			{
				DPRINT("%04d| GSM : TCP SEND\n", HAL_GetTick()/1000);
				retry = 0;
				gsm.state = GSM_STATE_TCP_SEND1_TX;
			}
			break;
			case GSM_STATE_TCP_SEND1_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(rx_buf, 0, sizeof(rx_buf));
				HAL_UART_Receive_DMA(&huart3, rx_buf, GSM_BUF_LEN);
				// Send command
				sprintf((char *)tx_buf, GSM_TCP_SEND1_TX_STR, msg_tcp_tx.len);
				DPRINT("%04d| GSM : TCP SEND 1 -> TX\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, tx_buf);
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx_buf, strlen((char *)tx_buf));			
				time = HAL_GetTick();
				gsm.state = GSM_STATE_TCP_SEND1_RX;
			}
			break;
			case GSM_STATE_TCP_SEND1_RX:
			{
				// Check timeout
				if ((HAL_GetTick() - time) > GSM_TCP_SEND1_TIMEOUT)
				{
					DPRINT("%04d| GSM : TCP SEND 1 -> TIMEOUT\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, rx_buf);
#endif
					retry++;
					if (retry > GSM_TCP_SEND_RETRY) {gsm.state = GSM_STATE_STOP_INIT;}
					else {gsm.state = GSM_STATE_TCP_SEND1_TX;}
				}
				// Check received buffer
				else if (strstr((char *)rx_buf, GSM_TCP_SEND1_RX_STR) != NULL)
				{
					DPRINT("%04d| GSM : TCP SEND 1 -> RX OK\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, rx_buf);
#endif
					gsm.state = GSM_STATE_TCP_SEND2_TX;
				}
			}
			break;
			case GSM_STATE_TCP_SEND2_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(rx_buf, 0, sizeof(rx_buf));
				HAL_UART_Receive_DMA(&huart3, rx_buf, GSM_BUF_LEN);
				// Send command
				memset(tx_buf, 0, sizeof(tx_buf));
				memcpy(tx_buf, msg_tcp_tx.data, msg_tcp_tx.len);
				DPRINT("%04d| GSM : TCP SEND 2 -> TX\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, tx_buf);
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx_buf, msg_tcp_tx.len);
				time = HAL_GetTick();
				gsm.state = GSM_STATE_TCP_SEND2_RX;
			}
			break;
			case GSM_STATE_TCP_SEND2_RX:
			{
				// Check timeout
				if ((HAL_GetTick() - time) > GSM_TCP_SEND2_TIMEOUT)
				{
					DPRINT("%04d| GSM : TCP SEND 2 -> TIMEOUT\n", HAL_GetTick()/1000);
					retry++;
					if (retry > GSM_TCP_SEND_RETRY) {gsm.state = GSM_STATE_STOP_INIT;}
					else {gsm.state = GSM_STATE_TCP_SEND1_TX;}
				}
				// Check received buffer
				else
				{
					uint32_t idx = 0, idx_max = (GSM_BUF_LEN > strlen(GSM_TCP_SEND2_RX_STR)) ? (GSM_BUF_LEN - strlen(GSM_TCP_SEND2_RX_STR)) : 0;
					while( (memcmp(&rx_buf[idx], GSM_TCP_SEND2_RX_STR, strlen(GSM_TCP_SEND2_RX_STR)) != 0) && (idx < idx_max) ) {idx++;}
					if (idx < idx_max)
					{
						DPRINT("%04d| GSM : TCP SEND 2 -> RX OK\n", HAL_GetTick()/1000);
#ifdef PRINT_GSM_RX
						DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, rx_buf);
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
				idx = 0;
				retry = 0;
				gsm.state = GSM_STATE_STOP_TX;
			}
			break;
			case GSM_STATE_STOP_TX:
			{
				// Flush rx buffer & relaunch reception
				HAL_UART_Abort(&huart3);
				memset(rx_buf, 0, sizeof(rx_buf));
				HAL_UART_Receive_DMA(&huart3, rx_buf, GSM_BUF_LEN);
				// Send command
				memset(tx_buf, 0, sizeof(tx_buf));
				strcpy((char *)tx_buf, sGsmATStop[idx].tx_buf);
				DPRINT("%04d| GSM : AT STOP %d -> TX\n", HAL_GetTick()/1000, idx);
#ifdef PRINT_GSM_TX
				DPRINT("%04d| TX BUFFER -> %s\n", HAL_GetTick()/1000, tx_buf);
#endif
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx_buf, strlen((char *)tx_buf));			
				time = HAL_GetTick();
				gsm.state = GSM_STATE_STOP_RX;
			}
			break;
			case GSM_STATE_STOP_RX:
			{
				// Check timeout
				if ((HAL_GetTick() - time) > sGsmATStop[idx].time)
				{
					DPRINT("%04d| GSM : AT STOP %d -> TIMEOUT\n", HAL_GetTick()/1000, idx);
#ifdef PRINT_GSM_RX
					DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, rx_buf);
#endif
					retry++;
					if (retry > GSM_STOP_RETRY) {gsm.state = GSM_STATE_INIT;}
					else {gsm.state = GSM_STATE_STOP_TX;}
				}
				else
				{
					// Check received buffer
					char *ptr = strstr((char *)rx_buf, sGsmATStop[idx].rx_buf1);
					if (ptr != NULL)
					{
						ptr = strstr(ptr, sGsmATStop[idx].rx_buf2);
						if (ptr != NULL)
						{
							DPRINT("%04d| GSM : AT STOP %d -> RX OK\n", HAL_GetTick()/1000, idx);
#ifdef PRINT_GSM_RX
							DPRINT("%04d| RX BUFFER -> %s\n", HAL_GetTick()/1000, rx_buf);
#endif
							idx++;
							uint32_t nb_at = sizeof(sGsmATStop) / sizeof(sGsmATStop[0]);
							if (idx < nb_at) {gsm.state = GSM_STATE_STOP_TX;}
							else {gsm.state = GSM_STATE_INIT;}
						}
					}
				}
			}
			break;

			default: break;
		}
	
		// Socket close management
		if( (gsm.close) && (GSM_STATE_INIT < gsm.state) && (gsm.state < GSM_STATE_STOP_INIT) )
		{
			if (gsm.state <= GSM_STATE_TCP_CONNECT_INIT) {gsm.state = GSM_STATE_STOP_INIT;}
			else {gsm.state = GSM_STATE_INIT;}
		}
		
		// Delay
		osDelay(10);
	}
}


/**
  * @brief  Socket status
  * @param  argument: Not used
  * @retval Socket status
  */
static gsm_status_t gsm_get_status(void)
{
	gsm_status_t status;
	if (gsm.state <= GSM_STATE_INIT) {status = GSM_STATUS_STOP;}
	else if (gsm.state < GSM_STATE_TCP_CONNECT_INIT) {status = GSM_STATUS_INIT;}
	else if (gsm.state < GSM_STATE_TCP_RECV_INIT) {status = GSM_STATUS_CONNECTING;}
	else {status = GSM_STATUS_CONNECTED;}
	return status;
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Socket init
  * @param  argument: Not used
  * @retval Status
  */
status_t socket_init(void)
{
	// Init socket
	uint32_t time = HAL_GetTick();
	gsm.close = FALSE;
	gsm.init = TRUE;
	while (gsm_get_status() <= GSM_STATUS_INIT)
	{
		if ((HAL_GetTick() - time) > SOCKET_INIT_TIMEOUT) {return STATUS_ERROR;}
		osDelay(10);
	}
	
	// return
	return STATUS_SUCCESS;
}

/**
  * @brief  Socket open
  * @param  argument: Not used
  * @retval Status
  */
status_t socket_open(char *hostname, uint16_t port)
{
	// Store IP address and TCP port
	if (strlen(hostname) > GSM_HOSTNAME_LEN) {return STATUS_ERROR;}
	strcpy(gsm.hostname, hostname);
	sprintf(gsm.tcp_port, "%u", port);
	if (strlen(gsm.tcp_port) > GSM_TCP_PORT_LEN) {return STATUS_ERROR;}
	
	// Connect socket
	uint32_t time = HAL_GetTick();
	gsm.connect = TRUE;
	while (gsm_get_status() <= GSM_STATUS_CONNECTING)
	{
		if ((HAL_GetTick() - time) > SOCKET_OPEN_TIMEOUT) {return STATUS_ERROR;}
		osDelay(10);
	}
	
	// return
	return STATUS_SUCCESS;
}

/**
  * @brief  Socket close
  * @param  argument: Not used
  * @retval Status
  */
status_t socket_close(void)
{
	// Connect socket
	uint32_t time = HAL_GetTick();
	gsm.init = FALSE;
	gsm.connect = FALSE;
	gsm.close = TRUE;
	while (gsm_get_status() != GSM_STATUS_STOP)
	{
		if ((HAL_GetTick() - time) > SOCKET_INIT_TIMEOUT) {return STATUS_ERROR;}
		osDelay(10);
	}
	
	// return
	return STATUS_SUCCESS;
}

/**
  * @brief  Socket send
  * @param  argument: data, len
  * @retval Status
  */
status_t socket_send(void *data, uint32_t len)
{
	// Check limits
	if (len > GSM_MSG_TCP_LEN) {return STATUS_ERROR;}
	if (gsm_get_status() != GSM_STATUS_CONNECTED) {return STATUS_ERROR;}
	
	// Send message
	gsm_msg_tcp_t msg_tx;
	memcpy(msg_tx.data, data, len);
	msg_tx.len = len;
	if (osMessageQueuePut(MsgTcpTx, &msg_tx, 0U, 0U) != osOK) {return STATUS_ERROR;}
	//DPRINT("%04d| SEND MSG -> %s\n", HAL_GetTick()/1000, (char *)msg_tx.data);

	// return
	return STATUS_SUCCESS;
}

/**
  * @brief  Socket receive
  * @param  argument: data, len
  * @retval Status
  */
status_t socket_recv(void *data, uint32_t *len)
{
	// Check limits
	if (gsm_get_status() != GSM_STATUS_CONNECTED) {return STATUS_ERROR;}
	
	// Check message
	gsm_msg_tcp_t msg_rx;
	if (osMessageQueueGet(MsgTcpRx, &msg_rx, NULL, 0U) != osOK) {return STATUS_ERROR;}
	memcpy(data, msg_rx.data, msg_rx.len);
	*len = msg_rx.len;
	//DPRINT("%04d| RECV MSG -> %s\n", HAL_GetTick()/1000, (char *)msg_rx.data);
	
	// return
	return STATUS_SUCCESS;
}
