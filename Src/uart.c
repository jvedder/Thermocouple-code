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
#include "main.h"
#include "gpio.h"
#include "uart.h"

//#include "stm32f0xx_hal.h"
//#include "stm32f0xx_hal_uart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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

	/* USART CR1 Config Register
	 * [28] M[1]   = 00: 1 Start bit, 8 data bits, n stop bits
	 * [27] EOBIE  = 0: End of Block interrupt Disabled
	 * [26] RTOIE  = 0: Receiver timeout interrupt Disabled
	 * [25:21] DEAT[4:0] = 0: Drive Enable Assertion Time
	 * [20:16] DEDT[4:0] = 0: Driver Enable De-assertion time
	 * [15] OVER8  = 0: Over-sampling by 16
	 * [14] CMIE   = 0: Character match interrupt disabled
	 * [13] MME    = 0 Receiver in active mode (unmuted)
	 * [12] M[0]   ==> See M[1] above
	 * [11] WAKE:  = 0: Receiver wakeup from mute on idle line
	 * [10] PCE    = 0: Parity Control Disabled
	 * [9]  PS     = 0: Even Parity
	 * [8]  PEIE:  = 0: Parity Error (PE) interrupt Disabled
	 * [7]  TXEIE  = 0: Tx Empty (TXE) interrupt Disabled
	 * [6]  TCIE   = 0: Tx complete interrupt Disabled
	 * [5]  RXNEIE = 1: Rx Not Empty (RXNE) Interrupt Enabled
	 * [4]  IDLEIE = 0: IDLE Interrupt Disabled
	 * [3]  TE     = 1: Transmitter Enabled
	 * [2]  RE     = 1; Receiver Enabled
	 * [1]  UESM   = 0: USART not able to wake up the MCU from Stop mode
	 * [0]  UE     = 1: USART enable
	 * -----       -----
	 * [31:0]      = 0x002C Disabled
	 * [31:0]      = 0x002D Enabled
	 *
	 * */
	reg = (uint32_t) USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	WRITE_REG(huart->Instance->CR1, reg);


	/* USART CR2 Config register
	 * [13:12] STOP[1:0] = 00; 1 Stop Bit
	 * */
	WRITE_REG(huart->Instance->CR2, 0x0000);

	/* USART CR3 Config register
	 * [14] DEM   = 0: Driver enable mode disabled
	 * [10] CTSIE = 0: CTS interrupt disabled
	 * [9]  CTSE  = 0: CTS disabled
	 * [8]  RTSE  = 0: RTS disabled
	 * [0]  EIE   = 0: Error interrupt disabled
	 * */
	WRITE_REG(huart->Instance->CR3, 0x0000);

	/* USART BRR Baud Rate Register
	 * 48MHz / 115.2K = 416.667 rounds to 417
	 * */
	reg = (uint32_t) 417;
	WRITE_REG(huart->Instance->BRR, reg);


	/* USART ICR Interrupt Clear Flag register
	 * Clear all flags
	 */
	WRITE_REG(huart->Instance->ICR, 0xFFFF);

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
	huart2.Instance = USART2;
	UART_Init_Regs(&huart2);
}


/**
  * @brief Get one character from the UART receive buffer or -1 if it is empty. Does not block.
  * @param huart UART handle.
  * @retval The next character or -1
  */
int16_t UART_Get( UART_Handle_t *huart )
{
	assert_param(huart != NULL);

	int16_t c = -1;

	/* get a byte from the fifo if not empty */
	if (huart->rx_fifo_in != huart->rx_fifo_out)
	{
		c = (int16_t) huart->rx_fifo[huart->rx_fifo_out];
		huart->rx_fifo_out = (huart->rx_fifo_out+1) % UART_RX_FIFO_LENGTH;
	}
	return c;
}

/**
  * @brief Send a null terminated string in blocking mode.
  * @param huart UART handle.
  * @param msg Pointer to string to transmit. Must be null terminated
  * @retval none
  */
void UART_Put( UART_Handle_t *huart, const char Ch )
{
	assert_param(huart != NULL);

	/* TODO: Consider this change:
	 * Wait for TXE set instead of TC.
	 * TXE clears on the next write to TDR.
	 * Eliminates waiting for last byte to Tx,
	 * but need to check no Tx in process on entry.
	 * Unclear if TXE will be set on startup.
	 */

	/* Start USART Tx transmission */
	huart->Instance->TDR = Ch;

	/* Wait for Tx transfer complete up to timeout */
	uint16_t timeout = 50000L;
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

	while(*msg)
	{
		UART_Put(huart, *msg);
		msg++;
	}
}

#if 0
//
// Does not work correctly
//
void UART_SendHex( UART_Handle_t *huart, uint32_t Data, uint16_t Digits )
{
	assert_param(huart != NULL);
	assert_param( (0 < Digits) && (Digits < 17) );

	while (Digits)
	{
		/* Grab the most significant nibble */
		char c = (Data >> ((Digits-1) * 4)) & 0x000F;

		/* convert to hex and send it */
		c += ( (c < 10) ? '0' : ('A'-1) );
		UART_Put( huart, c);

		/* shift data by a nibble */
		Data >>= 4;

		/* move index towards lsb */
		Digits--;
	}
}
#endif



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
