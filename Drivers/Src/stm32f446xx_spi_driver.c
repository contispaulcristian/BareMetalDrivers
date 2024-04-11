/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Apr 8, 2024
 *      Author: paul.contis
 */

/*************************************************************************************************************************************************/
#include "stm32f446xx_spi_driver.h"
/**************************************************************************************************************************************************
 * @fn 							- SPI_PeriClockControl
 * @brief 						- This function enables or disables peripheral clock for given SPI
 *
 * @param pSPIx[in]				- Base address of the spi peripheral
 * @param EnorDi[in]			- ENABLE or BISABLE macros
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void SPI_PeriClockControl (SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
    {
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}

    }
	else
    {
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
    }
}

/**************************************************************************************************************************************************
 * @fn 							- SPI_Init
 * @brief 						- Init
 *
 * @param pSPIHandle			-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void SPI_Init (SPI_Handle_t *pSPIHandle)
{


}

/**************************************************************************************************************************************************
 * @fn 							- SPI_DeInit
 * @brief 						- De-init
 *
 * @param pSPIx					-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void SPI_DeInit (SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx == SPI3)
    {
    	SPI3_REG_RESET();
    }
    else if (pSPIx == SPI4)
    {
        SPI4_REG_RESET();
    }
}
/**
 * @fn void SPI_SendData(SPI_RegDef_t*, uint8_t*, uint32_t)
 * @brief
 *
 * @param pSPIx
 * @param pTxBuffer
 * @param Length
 */
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length)
{

}
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length)
{

}
/**
 * @fn void SPI_IRQInterruptConfig(uint8_t, uint8_t)
 * @brief
 *
 * @param IRQNumber
 * @param EnorDi
 */
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
{

}
/**
 * @fn void SPI_IRQPriorityConfig(uint8_t, uint32_t)
 * @brief
 *
 * @param IRQNumber
 * @param IRQPriority
 */
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{

}
/**
 * @fn void SPI_IRQHandling(uint8_t*)
 * @brief
 *
 * @param pHandle
 */
void SPI_IRQHandling (uint8_t *pHandle)
{

}

