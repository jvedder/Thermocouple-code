/**
  ******************************************************************************
  * @file    eeprom.h
  * @author  John Vedder
  * @brief   Header file for AT24MAC402 EEPROM with MAC (EUI-48)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "i2c.h"
#include "gpio.h"
#include "eeprom.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t EEPROM_buffer[EEPROM_BUFFER_SIZE];

#define EEPROM_I2C_ADDR			0xA0	/* 1010000x */
#define EEPROM_EXT_I2C_ADDR		0xB0	/* 1011000x */

#define EEPROM_MAC48_BYTE_ADDR	0x9A
#define EEPROM_SN128_BYTE_ADDR	0x80

#define EEPROM_BLOCK_SIZE 		16

#define EEPROM_TIMEOUT 1000

#if 0
HAL_StatusTypeDef EEPROM_ReadMac48_Manual( void )
{
	HAL_StatusTypeDef status;

	/* Send Byte address to extended EEPROM */
	EEPROM_buffer[0] = EEPROM_MAC48_BYTE_ADDR;
	status = HAL_I2C_Master_Transmit(&hi2c1, EEPROM_EXT_I2C_ADDR, EEPROM_buffer, 1, EEPROM_TIMEOUT);

	/* clear buffer */
	memset(EEPROM_buffer, 0, EEPROM_BUFFER_SIZE);

	/* read the MAC48 */
	if (HAL_OK == status)
	{
		status = HAL_I2C_Master_Receive(&hi2c1, EEPROM_EXT_I2C_ADDR, EEPROM_buffer, EEPROM_MAC48_BYTE_LENGTH, EEPROM_TIMEOUT);
	}

	//HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)();
	//HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)

	return status;
}
#endif


//HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)();
//HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)


HAL_StatusTypeDef EEPROM_ReadMac48( void )
{
	/* zero out buffer */
	memset(EEPROM_buffer, 0, EEPROM_BUFFER_SIZE);

	/* read MAC from extened EEPROM into buffer */
	return HAL_I2C_Mem_Read(&hi2c1, EEPROM_EXT_I2C_ADDR, EEPROM_MAC48_BYTE_ADDR, I2C_MEMADD_SIZE_8BIT, EEPROM_buffer, EEPROM_MAC48_BYTE_LENGTH, EEPROM_TIMEOUT);
}

HAL_StatusTypeDef EEPROM_ReadSN128( void )
{
	/* zero out buffer */
	memset(EEPROM_buffer, 0, EEPROM_BUFFER_SIZE);

	/* read MAC from extened EEPROM into buffer */
	return HAL_I2C_Mem_Read(&hi2c1, EEPROM_EXT_I2C_ADDR, EEPROM_SN128_BYTE_ADDR, I2C_MEMADD_SIZE_8BIT, EEPROM_buffer, EEPROM_SN128_BYTE_LENGTH, EEPROM_TIMEOUT);
}

/**
 * Reads a 16-byte block from the 2KB EEPROM
 */
HAL_StatusTypeDef EEPROM_ReadBlock( uint8_t BlockIndex )
{
	uint16_t ByteAddr = BlockIndex * 16;

	/* zero out buffer */
	memset(EEPROM_buffer, 0, EEPROM_BUFFER_SIZE);

	/* read from EEPROM into buffer */
	return HAL_I2C_Mem_Read(&hi2c1, EEPROM_I2C_ADDR, ByteAddr, I2C_MEMADD_SIZE_8BIT, EEPROM_buffer, EEPROM_BUFFER_SIZE, EEPROM_TIMEOUT);
}

/**
 * Writes a 16-byte block from the 2KB EEPROM
 */
HAL_StatusTypeDef EEPROM_WriteBlock( uint8_t BlockIndex )
{
	uint16_t ByteAddr = BlockIndex * 16;

	/* write from Buffer into EEPROM */
	return HAL_I2C_Mem_Write(&hi2c1, EEPROM_I2C_ADDR, ByteAddr, I2C_MEMADD_SIZE_8BIT, EEPROM_buffer, EEPROM_BUFFER_SIZE, EEPROM_TIMEOUT);
}


