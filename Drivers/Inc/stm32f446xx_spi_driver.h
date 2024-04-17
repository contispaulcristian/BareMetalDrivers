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
	uint8_t SPI_DeviceMode;					/*!< Master or slave																				 */
	uint8_t SPI_BusConfig;					/*!< full duplex, half duplex, simplex																 */
	uint8_t	SPI_SclkSpeed;					/*!< 																								 */
	uint8_t SPI_DFF;						/*!< Data frame format: 8 bits or 16 bits															 */
	uint8_t SPI_CPOL;						/*!< Clock polarity: CPOL 0 - default state LOW;
										   	   	   	   	   	   	 CPOL 1 - default state HIGH 													 */
	uint8_t	SPI_CPHA;						/*!< Clock phase: CPHA 0 - the first edge on the SCK pin captures the first data bit transacted
	 	 	 	 	 	 	 	 	 	 	                  CPHA 1 - the second edge on the SCK pin captures the first data bit transacted     */
	uint8_t	SPI_SSM;						/*!< Slave select management: software or hardware													 */
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;					/*!<This holds the base address of SPIx peripheral													 */
	SPI_Config_t SPI_Config;
}SPI_Handle_t;

/*
 * @SPI_DeviceMode macros
 */
#define SPI_DEVICE_MODE_MASTER				1	/* CR1 register - bit 2 */
#define SPI_DEVICE_MODE_SLAVE				0	/* default mode */

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1	/* Ful duplex: CR1 register - bit 15 LOW unidirectional mode, bit 10 should be LOW */
#define SPI_BUS_CONFIG_HD					2	/* Half duplex: CR1 register - bit 15 HIGH bidirectional mode, bit 10 should be LOW
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	   bit 14 LOW receive only, HIGH transmit only */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3	/* Simplex Rx only: CR1 register - bit 10 LOW transmit and receive
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	   bit 10 HIGH receive only, bit 15 should be LOW */

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLCK_SPEED_DIV2 				0  /* Baud rate control: CR1 register - bit 3,4,5  */
#define SPI_SCLCK_SPEED_DIV4				1
#define SPI_SCLCK_SPEED_DIV8				2
#define SPI_SCLCK_SPEED_DIV16				3
#define SPI_SCLCK_SPEED_DIV32				4
#define SPI_SCLCK_SPEED_DIV64				5
#define SPI_SCLCK_SPEED_DIV128				6
#define SPI_SCLCK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS						0	/* CR1 register - bit 11 */
#define SPI_DFF_16BITS						1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH						1	/* CR1 register - bit 1 */
#define SPI_CPOL_LOW						0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH						1	/* CR1 register - bit 0 */
#define SPI_CPHA_LOW						0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN							1	/* CR1 register - bit 9 */
#define SPI_SSM_DI							0   /* default state */

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG						( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG						( 1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG						( 1 << SPI_SR_BSY )

/*************************************************************************************************************************************************
 *					                 						APIs supported by this driver
 *											For more information about the APIs check the function definitions
 *************************************************************************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
/*************************************************************************************************************************************************/


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
