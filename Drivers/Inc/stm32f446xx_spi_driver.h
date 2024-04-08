/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Apr 8, 2024
 *      Author: paul.contis
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

/*************************************************************************************************************************************************/
#include "stm32f446xx.h"
/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t	SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;				/*!<This holds the base address of SPIx peripheral					*/
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/*************************************************************************************************************************************************
 *					                 						APIs supported by this driver
 *											For more information about the APIs check the function definitions
 *************************************************************************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(GPIO_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(uint8_t *pHandle);

/*
 * Other Peripheral Control APIs
 */


/*************************************************************************************************************************************************/


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
