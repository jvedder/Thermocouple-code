/**
 ******************************************************************************
 * @file    uart.h
 * @author  John Vedder
 * @brief   Header file for My UART Driver module
 ******************************************************************************
 */
/**
 ******************************************************************************
 * MIT License
 *
 * Copyright (c) 2021 John Vedder
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this hardware, software, and associated documentation files (the "Product"),
 * to deal in the Product without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Product, and to permit persons to whom the Product is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Product.
 *
 * THE PRODUCT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE PRODUCT OR THE USE OR OTHER DEALINGS IN THE
 * PRODUCT.
 *
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
typedef struct UART_FIFO_Handle_struct
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
} UART_FIFO_Handle_t;

extern UART_FIFO_Handle_t huart1;
extern UART_FIFO_Handle_t huart2;

/* Exported functions --------------------------------------------------------*/
void UART_Init( void );
int16_t UART_Get( UART_FIFO_Handle_t *huart );
void UART_Put( UART_FIFO_Handle_t *huart, const char C );
void UART_Send( UART_FIFO_Handle_t *huart, const char * msg );
void UART_SendHex( UART_FIFO_Handle_t *huart, uint32_t Data, uint16_t Digits );
bool UART_IsRxOverflow( UART_FIFO_Handle_t *huart );
void UART_FIFO_IRQHandler(UART_FIFO_Handle_t *huart);



#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
