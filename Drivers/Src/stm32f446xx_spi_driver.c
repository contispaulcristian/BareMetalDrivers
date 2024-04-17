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
	/* Configure the SPI_CR1 register */
	uint32_t tempreg = 0;
	/* Enable the peripheral clock */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	/* Configure the device mode */
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;
	/* Configure the bus config */
	if ( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD )
	{
		/* bidi mode should be cleared */
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
	}
	else if ( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD )
	{
		/* bidi mode should be set */
		tempreg |= ( 1 << SPI_CR1_BIDIMODE );
	}
	else if ( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY )
	{
		/* bidi mode should be cleared */
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
		/* and rxonly bit must be set */
		tempreg |= ( 1 << SPI_CR1_RXONLY );
	}
	/* Configure the spi serial clock speed (baud rate) */
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	/* Configure the DFF */
	tempreg |=pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	/* Configure the CPOL */
	tempreg |=pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/* Configure the CPHA */
	tempreg |=pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	/* Save values in CR1 register */
	pSPIHandle->pSPIx->CR1 = tempreg;
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

/**************************************************************************************************************************************************
 * @fn 							- SPI_SendData
 * @brief 						- Tx
 *
 * @param pSPIx					-
 * @param pTxBuffer				-
 * @param Length				-
 *
 * @return						- none
 *
 * @note						- This is a blocking call
 *************************************************************************************************************************************************/
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length)
{
	while( Length > 0 )
	{
		/* Wait until TXE is set */
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		/* Check the DFF bit in CR1 */
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) )
		{
			/* 16 bit DFF*/
			/* load the data into the DR */
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Length--;
			Length--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			/* 8 bit DFF */
			pSPIx->DR = *pTxBuffer;
			Length--;
			pTxBuffer++;
		}

	}
}

/**************************************************************************************************************************************************
 * @fn 							- SPI_ReceiveData
 * @brief 						- Rx
 *
 * @param pSPIx					-
 * @param pRxBuffer				-
 * @param Length				-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length)
{
	while( Length > 0 )
	{
		/* Wait until RXNE is set */
		while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
		/* Check the DFF bit in CR1 */
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) )
		{
			/* 16 bit DFF*/
			/* load the data from DR to RxBuffer address */
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Length--;
			Length--;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			/* 8 bit DFF */
			*pRxBuffer = pSPIx->DR;
			Length--;
			pRxBuffer++;
		}
	}
}

/**************************************************************************************************************************************************
 * @fn 							- SPI_IRQInterruptConfig
 * @brief 						-
 *
 * @param IRQNumber				-
 * @param EnorDi				-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
{

}

/**************************************************************************************************************************************************
 * @fn 							- SPI_IRQPriorityConfig
 * @brief 						-
 *
 * @param IRQNumber				-
 * @param IRQPriority			-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/**************************************************************************************************************************************************
 * @fn 							- SPI_IRQHandling
 * @brief 						-
 *
 * @param pHandle				-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void SPI_IRQHandling (uint8_t *pHandle)
{

}

/**************************************************************************************************************************************************
 * @fn 							- SPI_GetFlagStatus
 * @brief 						-
 *
 * @param pSPIx					-
 * @param FlagName				-
 *
 * @return						- none
 *
 * @note						- local function
 *************************************************************************************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if( pSPIx->SR == FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;


}

/**************************************************************************************************************************************************
 * @fn 							- SPI_PeripheralControl
 * @brief 						-
 *
 * @param pSPIx					-
 * @param EnOrDi				- ENABLE or DISABLE
 *
 * @return						- none
 *
 * @note						-
 *************************************************************************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE );
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE );
	}
}

/**************************************************************************************************************************************************
 * @fn 							- SPI_SSIConfig
 * @brief 						-
 *
 * @param pSPIx					-
 * @param EnOrDi				- ENABLE or DISABLE
 *
 * @return						- none
 *
 * @note						-
 *************************************************************************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI );
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI );
	}
}

/**************************************************************************************************************************************************
 * @fn 							- SPI_SSOEConfig
 * @brief 						-
 *
 * @param pSPIx					-
 * @param EnOrDi				- ENABLE or DISABLE
 *
 * @return						- none
 *
 * @note						-
 *************************************************************************************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE );
	}
	else
	{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE );
	}
}
