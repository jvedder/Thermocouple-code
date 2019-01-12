
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
#define MSG_SIZE 64
static char msg[MSG_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
// static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t timeout;
	uint32_t n;
	uint32_t err;
	uint8_t tx;
	uint8_t rx;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_ADC_Init();
  //MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  //  Local Loopback on Tx and Rx on #1
  //  HAL_GPIO_WritePin(UART2_DE1_GPIO_Port, UART2_DE1_Pin, GPIO_PIN_SET);
  //  HAL_GPIO_WritePin(UART2_RE1_N_GPIO_Port, UART2_RE1_N_Pin, GPIO_PIN_RESET);
  //  HAL_GPIO_WritePin(UART2_DE2_GPIO_Port, UART2_DE2_Pin, GPIO_PIN_RESET);
  //  HAL_GPIO_WritePin(UART2_RE2_N_GPIO_Port, UART2_RE2_N_Pin, GPIO_PIN_SET);

  //  Remote Loopback Tx on #1 and Rx on #2
  HAL_GPIO_WritePin(UART2_DE1_GPIO_Port, UART2_DE1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(UART2_RE1_N_GPIO_Port, UART2_RE1_N_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(UART2_DE2_GPIO_Port, UART2_DE2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(UART2_RE2_N_GPIO_Port, UART2_RE2_N_Pin, GPIO_PIN_RESET);

  MX_USART2_UART_Init();

  //MX_USB_PCD_Init();

  /* Initialize interrupts */
  //MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

    sprintf(msg, "\r\nHello World.\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 1000);

    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  n = 0;
  err = 0;
  tx = 0;
  while (1)
  {
	  n++;
	  tx++;
	  //if (tx == 0) tx = 1;

  	  /* turn on Green LED */
	  HAL_GPIO_WritePin(LED_GRN_GPIO_Port, LED_GRN_Pin, GPIO_PIN_SET);


	  SPI_ReadMax31855();

	  if ((n & 0xFFFF) == 0)
	  {
		  sprintf(msg, "n=%lu err=%lu\r\n", n, err);
		  HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 1000);
	  }

	  /* Start USART Tx transmission */
  	  USART2->TDR = tx;

  	  /* Wait for Tx transfer complete */
  	  timeout = 500000L;
  	  while ( ((USART2->ISR & USART_ISR_TC) == 0) && (timeout > 0))
  	  {
  		  timeout--;
  	  }

  	  if (timeout == 0)
  	  {
  		sprintf(msg, "tx timeout\r\n");
  		HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 1000);
  		err++;
  	  }

  	  /* Clear Tx transfer complete flag */
  	  USART2->ICR |= USART_ICR_TCCF;

  	  /* turn off Green LED */
	  HAL_GPIO_WritePin(LED_GRN_GPIO_Port, LED_GRN_Pin, GPIO_PIN_RESET);

  	  /* wait for receive flag */
  	  timeout = 500000L;
  	  while ( ((USART2->ISR & USART_ISR_RXNE) == 0) && (timeout > 0))
  	  {
  		timeout--;
  	  }

  	  /* Read the received byte */
  	  rx = (uint8_t)(USART2->RDR); /* Receive data, clear flag */

  	  if (timeout == 0)
  	  {
  		//Rx Timeout
  		sprintf(msg, "rx timeout\r\n");
    	HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 1000);
    	err++;
  	  }
  	  else
  	  {
		  if (tx != rx)
		  {
			sprintf(msg, "mismatch: tx %d, rx %d\r\n", (int) tx, (int) rx);
			HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 1000);
			err++;
		  }
  	  }

  	  /* turn on Red LED if error */
  	  if (err > 0)
  	  {
  		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  	  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

#if 0

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

#endif

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
