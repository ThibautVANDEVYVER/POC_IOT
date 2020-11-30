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
	STATUS_SUCCESS		= 0,
	STATUS_ERROR,
} status_t;

/* Exported variables constants ----------------------------------------------*/
extern osThreadId_t gsmTaskHandle;

/* Exported constants --------------------------------------------------------*/
extern const osThreadAttr_t gsmTask_attributes;

/* Exported functions prototypes ---------------------------------------------*/
void StartGsmTask(void *argument);

/* Public functions prototypes -----------------------------------------------*/
status_t socket_init(void);
status_t socket_open(char *hostname, uint16_t port);
status_t socket_close(void);
status_t socket_send(void *data, uint32_t len);
status_t socket_recv(void *data, uint32_t *len);


#ifdef __cplusplus
}
#endif

#endif /* __GSM_H */
