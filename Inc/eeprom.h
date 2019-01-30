/**
  ******************************************************************************
  * @file    eeprom.h
  * @author  John Vedder
  * @brief   Header file for AT24MAC402 EEPROM with MAC (EUI-48)
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __eeprom_H
#define __eeprom_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"

 /* Exported defines ---------------------------------------------------------*/
#define EEPROM_BUFFER_SIZE 			16
#define EEPROM_MAC48_BYTE_LENGTH	6
#define EEPROM_SN128_BYTE_LENGTH	16

 /* Exported variables -------------------------------------------------------*/
extern uint8_t EEPROM_buffer[];

 /* Exported functions -------------------------------------------------------*/
HAL_StatusTypeDef EEPROM_ReadMac48( void );
HAL_StatusTypeDef EEPROM_ReadSN128( void );
HAL_StatusTypeDef EEPROM_ReadBlock( uint8_t BlockIndex );
HAL_StatusTypeDef EEPROM_WriteBlock( uint8_t BlockIndex );

#ifdef __cplusplus
}
#endif
#endif /*__eeprom_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
