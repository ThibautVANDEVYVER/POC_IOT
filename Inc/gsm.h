/**
  ******************************************************************************
  * @file           : gsm.h
  * @brief          : Header for gsm.c file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GSM_H
#define __GSM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported typedefs ---------------------------------------------------------*/
typedef enum
{
	GSM_STATUS_STOP					= 0,
	GSM_STATUS_DISCONNECTED	= 1,
	GSM_STATUS_CONNECTING		= 2,
	GSM_STATUS_CONNECTED		= 3,
} gsm_status_t;

#define GSM_MSG_TCP_NB	        (4UL)
#define GSM_MSG_TCP_SIZE   			(64UL)
typedef struct
{
	uint8_t data[GSM_MSG_TCP_SIZE];
	uint32_t size;
} gsm_msg_tcp_t;

/* Exported variables constants ----------------------------------------------*/
extern osThreadId_t gsmTaskHandle;

/* Exported constants --------------------------------------------------------*/
extern const osThreadAttr_t gsmTask_attributes;

/* Exported functions prototypes ---------------------------------------------*/
void StartGsmTask(void *argument);


#ifdef __cplusplus
}
#endif

#endif /* __GSM_H */
