/**
  ******************************************************************************
  * @file    uart.c
  * @author  John Vedder
  * @brief   My UART Driver module. Replaces stm32f0xx_hal_uart.c/.h
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include "uart.h"
#include "main.h"
//#include "stm32f0xx_hal.h"
//#include "stm32f0xx_hal_uart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *uart_handle = NULL;
UART_Handle_t huart1;
UART_Handle_t huart2;

/* Private function prototypes -----------------------------------------------*/

/* Private functions --------------------------------------------------------*/

/**
  * @brief Initialize the UART specified by the UART handle to 115.2K baud,
  * 1-start, 8-bits, 1-stop, No-parity.  Inits the RX fifo bufferand enabled
  * the RXNEIE interrupt.
  * @param huart UART handle.
  * @retval none
  */
static void UART_Init_Regs( UART_Handle_t *huart )
{
	uint32_t reg;

	assert_param(huart != NULL);

	/* Disable UART to set config */
	CLEAR_BIT(huart->Instance->CR1, USART_CR1_UE);

	/* USART CR1 Config Register */
	reg = (uint32_t)UART_WORDLENGTH_8B | UART_PARITY_NONE | UART_MODE_TX_RX | UART_OVERSAMPLING_16 | USART_CR1_RXNEIE;
	WRITE_REG(huart->Instance->CR1, reg);

	/*USART CR2 Config register */
	/* Configure the UART Stop Bits: Set STOP[13:12] bits according
	* to huart->Init.StopBits value */
	reg = (uint32_t) UART_STOPBITS_1;
	WRITE_REG(huart->Instance->CR2, reg);

	/* USART CR3 Config register */
	reg = (uint32_t)UART_HWCONTROL_NONE | UART_ONE_BIT_SAMPLE_DISABLE ;
	WRITE_REG(huart->Instance->CR3, reg);

	/* USART BRR Baud Rate Register */
	/* 48MHz / 115.2K = 416.667 rounds to 417 */
	reg = (uint32_t) 417;
	WRITE_REG(huart->Instance->BRR, reg);

	/* Reset the RX FIFO */
	huart->rx_fifo_in = 0;
	huart->rx_fifo_out = 0;
	huart->rx_overflow = false;
	huart->rx_errors = 0;
	huart->tx_errors = 0;

	/* Enable UART with new config */
	SET_BIT(huart->Instance->CR1, USART_CR1_UE);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief Initialize and enable the UART1 and UART2 hardware
  * @retval None
  */
void UART_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* USART1 clock enable */
	__HAL_RCC_USART1_CLK_ENABLE();

	/**
     * UART1 GPIO Configuration
     * PB6 --> USART1_TX
     * PB7 --> USART1_RX
    */
    GPIO_InitStruct.Pin = UART1_TX_Pin|UART1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configure USART1 as UART1 */
	huart1.Instance = USART1;
	UART_Init_Regs(&huart1);


	/* USART2 clock enable */
	__HAL_RCC_USART2_CLK_ENABLE();

	/**
	 * USART2 GPIO Configuration
	 * PA2  --> USART2_TX
	 * PA3  --> USART2_RX
    */
    GPIO_InitStruct.Pin = UART2_TX_Pin|UART2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Configure USART2 as UART2 */
	huart1.Instance = USART2;
	UART_Init_Regs(&huart2);
}


/**
  * @brief Send a null terminated string in blocking mode.
  * @param huart UART handle.
  * @param msg Pointer to string to transmit. Must be null terminated
  * @retval none
  */
void UART_Send( UART_Handle_t *huart, const char * msg )
{
	assert_param(huart != NULL);
	assert_param(msg != NULL);

	uint32_t timeout;

	while(*msg)
	{
		/* Start USART Tx transmission */
		huart->Instance->TDR = *msg;

		/* TODO: Wait for TXE set instead of TC.
		 * TXE clears on the next write to TDR.
		 * Eliminates waiting for last byte to Tx,
		 * but need to check no Tx in process on entry.
		 * Unclear if TXE will be set on startup.
		 */

		/* Wait for Tx transfer complete up to timeout */
		timeout = 50000L;
		while ( ((huart->Instance->ISR & USART_ISR_TC) == 0) && (timeout > 0))
		{
			timeout--;
		}

		if (timeout == 0)
		{
			huart-> tx_errors++;
		}

		/* Clear Tx transfer complete flag */
		huart->Instance->ICR |= USART_ICR_TCCF;

		/* move to the next character */
		msg++;
	}
}

/**
  * @brief Get one character from the UART receive buffer or -1 if it is empty. Does not block.
  * @param huart UART handle.
  * @retval The next character or -1
  */
int16_t UART_Get( UART_Handle_t *huart )
{
	int16_t c = -1;

	/* get a byte from the fifo if not empty */
	if (huart->rx_fifo_out != huart->rx_fifo_out)
	{
		c = (int16_t) huart->rx_fifo[huart->rx_fifo_out];
		huart->rx_fifo_out = (huart->rx_fifo_out+1) % UART_RX_FIFO_LENGTH;
	}
	return c;
}



bool UART_IsRxOverflow( UART_Handle_t *huart )
{
	/* Capture the value before clearing it */
	bool ret = huart->rx_overflow;
	huart->rx_overflow = false;
	return ret;
}

/**
  * @brief Handle UART interrupt request.
  * @param huart UART handle.
  * @retval None
  */
void UART_IRQHandler(UART_Handle_t *huart)
{
	/* get ISR and CR1 flags */
	uint32_t isrflags   = READ_REG(huart->Instance->ISR);

	/* UART should always be in Receive Mode */
	if( (isrflags & USART_ISR_RXNE) != RESET)
	{
		/* Receive UART data */
		/* NOTE: This clears the USART_ISR_RXNE bit */
		uint8_t data = (uint8_t) READ_REG(huart->Instance->RDR);

		/* add data to the fifo */
		huart->rx_fifo[huart->rx_fifo_in] = data;
		huart->rx_fifo_in = (huart->rx_fifo_in + 1) % UART_RX_FIFO_LENGTH;

		if (huart->rx_fifo_in == huart->rx_fifo_out)
		{
			huart->rx_overflow = true;
		}
	}

	/* flag and clear any errors */
	uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
	if (errorflags != 0)
	{
		huart->rx_errors++;
		WRITE_REG(huart->Instance->ICR, errorflags);
	}

	return; /* from interrupt */
}
