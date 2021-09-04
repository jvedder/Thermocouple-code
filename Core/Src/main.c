/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
//#include "app_usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "eeprom.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define MSG_SIZE 64
//static char msg[MSG_SIZE];
#define BUFFER_SIZE 16
uint8_t buffer[BUFFER_SIZE];
#define LOBYTE(x)  (uint16_t)((x) & 0xFFFF)
#define HIBYTE(x)  (uint16_t)((x) >> 16)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config( void );
static void MX_NVIC_Init( void );
/* USER CODE BEGIN PFP */
void Delay_ms(uint32_t delay_ms);
void PrintQuarter( int16_t value );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main( void )
{
    /* USER CODE BEGIN 1 */
    int16_t i, j;				// general purpose counter
    uint16_t chan;				// SPI1 Channel #
    uint32_t n;					// raw value read from SPI MAX321855
    int16_t lo, hi;		        // high and low words of raw value (note: signed)
    uint16_t err[4];   			// error flags
    int16_t tc_temp[4];			// thermocouple temperature in deg C with 1 LSB = 1/4 dec C
    int16_t ref_temp[4];		// reference temperature in deg C with 1 LSB = 1/16 deg C
    uint32_t count;				// sample count
    HAL_StatusTypeDef status;	// return status

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init( );

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config( );

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_USART2_UART_Init();
    MX_GPIO_Init( );
    MX_ADC_Init( );
    MX_I2C1_Init( );
    MX_SPI1_Init( );
    MX_USART1_UART_Init();
 
    //  Local Loopback on Tx and Rx on #1
    //  HAL_GPIO_WritePin(UART2_DE1_GPIO_Port, UART2_DE1_Pin, GPIO_PIN_SET);
    //  HAL_GPIO_WritePin(UART2_RE1_N_GPIO_Port, UART2_RE1_N_Pin, GPIO_PIN_RESET);
    //  HAL_GPIO_WritePin(UART2_DE2_GPIO_Port, UART2_DE2_Pin, GPIO_PIN_RESET);
    //  HAL_GPIO_WritePin(UART2_RE2_N_GPIO_Port, UART2_RE2_N_Pin, GPIO_PIN_SET);

    //  Remote Loopback Tx on #1 and Rx on #2
    HAL_GPIO_WritePin( UART2_DE1_GPIO_Port, UART2_DE1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( UART2_RE1_N_GPIO_Port, UART2_RE1_N_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( UART2_DE2_GPIO_Port, UART2_DE2_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( UART2_RE2_N_GPIO_Port, UART2_RE2_N_Pin, GPIO_PIN_RESET );

    //MX_USB_PCD_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    HAL_TIM_Base_Start(&htim2);

    /* Initialize interrupts */
    MX_NVIC_Init( );

    /* USER CODE BEGIN 2 */

    printf( "\r\nHello World.\r\n" );

    printf( "Build: %s %s\r\n", __DATE__, __TIME__ );

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    /* turn on Red LED */
    HAL_GPIO_WritePin( LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET );

    /* Display the MAC Address from the MAC48 EEPROM */
    EEPROM_ReadMac48( );
    printf("MAC: " );
    for ( i = 0; i < 6; i++ )
    {
        if ( i ) printf(":");
        printf( "%02X", EEPROM_buffer[i] );
    }
    printf("\r\n" );

    /* Display the 128-bit Serial Number from MAC48 EEPROM */
    EEPROM_ReadSN128( );
    printf( "SN: " );
    for ( i = 0; i < 16; i++ )
    {
        printf( "%02X", EEPROM_buffer[i] );
    }
    printf( "\r\n" );

#if 0
	/* Display the MCU's unique ID as hex */
	printf( "MCU ID: ");
	for (i=11; i>=0; i--)
	{
		uint8_t reg = READ_REG(*(   (uint8_t *)(UID_BASE + i) ) );
		printf( "%02X", (int)reg );
	}
	printf("\r\n");


	/* Display the MCU's unique ID as text */
	printf("MCU ID: '");
	for (i=11; i>=0; i--)
	{
		uint8_t reg = READ_REG(*( (uint8_t *)(UID_BASE + i) ));
		if (i > 4)
		{
			printf("%c", (char)reg );
		}
		else
		{
			printf("%02X", (int)reg );
		}
	}
	printf("'\r\n");
#endif

    /* Display the MCU's unique ID as hex string */
    printf( "MCU ID: %08lX%08lX%08lX\r\n", HAL_GetUIDw2( ), HAL_GetUIDw1( ),
            HAL_GetUIDw0( ) );

    /* display blocks 0 to 15 */
    for ( j = 0; j < 15; j++ )
    {
        status = EEPROM_ReadBlock( j );
        if ( HAL_OK != status )
        {
            printf( "Read Status: %d\r\n", (int) status );
        }
        printf( "BLK[%02X]: ", j );
        for ( i = 0; i < 16; i++ )
        {
            printf( "%02X", EEPROM_buffer[i] );
        }
        printf( "\r\n" );
    }

    /* turn off Red LED */
    HAL_GPIO_WritePin( LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET );

    count = 0;
    while ( 1 )
    {
        count++;

        /* turn on Green LED */
        HAL_GPIO_WritePin( LED_GRN_GPIO_Port, LED_GRN_Pin, GPIO_PIN_SET );


        uint32_t start = htim2.Instance->CNT;

        for ( chan = 0; chan < 4; chan++ )
        {
            n = SPI_Read32( chan );

            lo = n & 0xFFFF;
            hi = n >> 16;

            err[chan] = n & 0x07;
            if ( err[chan] )
            {
                tc_temp[chan] = 0;
            }
            else
            {
                /* Note: use hi/4, not hi>>2, because >> may not sign extend */
                tc_temp[chan] = ((int16_t)hi / 4);  // 14-bit,  1 LSB = 1/4 degree C
            }

            /* Note: use lo/16, not lo>>4, because >> may not sign extend */
            ref_temp[chan] = ( (int16_t)lo /16 );   // 12-bit, 1 LSB = 1/16 degree C
        }

        uint32_t finish = htim2.Instance->CNT;

#if 1
        printf( "Start: %lu\r\n", start);
        printf( "End:   %lu\r\n", finish);
        printf( "Delta: %lu\r\n", finish - start);

        /*
         * This loop takes 3608 ticks of 48MHz = 75.167 uSec
         */

        for ( chan = 0; chan < 4; chan++ )
        {
		  //sprintf(msg, "%d: %04X %04X %1d %d %d\r\n", chan, hi, lo, err[chan], ref_temp[chan], tc_temp[chan]);
          printf("%d: %04X_%04X,%1d,", chan, hi, lo, err[chan]);
          PrintQuarter(ref_temp[chan]/4);
          printf(",");
          PrintQuarter(tc_temp[chan]);
          if (chan == 3) printf("\r\n");
        }
#endif

#if 0
        printf( "%06ld,", count );

        for ( chan = 0; chan < 4; chan++ )
        {
            printf( "%1d,", err[chan] );
        }

        for ( chan = 0; chan < 4; chan++ )
        {
            printf( "%d,", ref_temp[chan] );
        }

        for ( chan = 0; chan < 4; chan++ )
        {
            printf( "%6d,", tc_temp[chan] );
        }

        printf( "\r\n" );
#endif

        /* turn off Green LED */
        HAL_GPIO_WritePin( LED_GRN_GPIO_Port, LED_GRN_Pin, GPIO_PIN_RESET );

        /* wait a little bit */
        Delay_ms(2000);

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config( void )
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        Error_Handler( );
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        Error_Handler( );
    }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

    if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        Error_Handler( );
    }
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init( void )
{
    /* USART2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( USART2_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( USART2_IRQn );
    /* USART1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( USART1_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( USART1_IRQn );
}

/* USER CODE BEGIN 4 */

/**
 * Delay using a spin wait for the specified number of milliseconds.
 * Timing is based off the SysTick timer.
 */
void Delay_ms(uint32_t delay_ms)
{
    /*  Handles SysTick roll-over: https://stackoverflow.com/questions/61443/  */
    uint32_t start_time_ms = HAL_GetTick();
    while ( (HAL_GetTick() - start_time_ms) < delay_ms)
    {
        // spin wait
    }

    return;
}

/**
 * Prints an integer value divided by 4.
 */
void PrintQuarter( int16_t value )
{
    // TODO: This probably does not work for negative values

    printf( "%d", (uint16_t) (value/4) );
    switch (value & 3)
    {
        case 0:
            printf( ".00" );
            break;
        case 1:
            printf( ".25" );
            break;
        case 2:
            printf( ".50" );
            break;
        case 3:
            printf( ".75" );
            break;
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler( void )
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq( );
    while ( 1 )
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
