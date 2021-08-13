/**
  ******************************************************************************
  * @file    uart.h
  * @author  John Vedder
  * @brief   Header file for My UART Driver module
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "main.h"


#define UART_RX_FIFO_LENGTH 32


/* Exported types ------------------------------------------------------------*/

 /**
   * @brief  My UART handle Structure definition
   */
typedef struct UART_Handle_struct
{
	/* UART register base address */
	USART_TypeDef            *Instance;

	/* UART RX FIFO */
	volatile uint8_t 		rx_fifo[UART_RX_FIFO_LENGTH];
	volatile uint16_t 		rx_fifo_in;
	volatile uint16_t 		rx_fifo_out;
	volatile bool			rx_overflow;
	volatile uint32_t 		rx_errors;
	volatile uint32_t 		tx_errors;
} UART_Handle_t;

extern UART_Handle_t huart1;
extern UART_Handle_t huart2;

/* Exported functions --------------------------------------------------------*/
void UART_Init( void );
int16_t UART_Get( UART_Handle_t *huart );
void UART_Put( UART_Handle_t *huart, const char C );
void UART_Send( UART_Handle_t *huart, const char * msg );
void UART_SendHex( UART_Handle_t *huart, uint32_t Data, uint16_t Digits );
bool UART_IsRxOverflow( UART_Handle_t *huart );
void UART_IRQHandler(UART_Handle_t *huart);



#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
