/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Apr 8, 2024
 *      Author: paul.contis
 */

/*************************************************************************************************************************************************/
#include "stm32f446xx_spi_driver.h"


/* private helper functions for @SPI_IRQHandling*/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
 * @fn 							- SPI_SendDataIT
 * @brief 						- Tx
 *
 * @param pSPIHandle			-
 * @param pTxBuffer				-
 * @param Length				-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->TxState;
	if( state != SPI_BUSY_IN_RX )
	{
		/* Save the Tx buffer address and Length information in some global variables */
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLength = Length;

		/* Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		/* Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= ( 1<<SPI_CR2_TXEIE );
	}

	/* Data transmission will be handled by the ISR code */
	//TODO

	return state;
}

/**************************************************************************************************************************************************
 * @fn 							- SPI_ReceiveDataIT
 * @brief 						- Rx
 *
 * @param pSPIHandle			-
 * @param pRxBuffer				-
 * @param Length				-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->TxState;

	if( state != SPI_BUSY_IN_RX )
	{
		/* Save the Tx buffer address and Length information in some global variables */
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLength = Length;

		/* Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		/* Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= ( 1<<SPI_CR2_RXNEIE );
	}

	/* Data receiving will be handled by the ISR code */
	//TODO

	return state;
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
	/* processor side NVIC*/
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			/* program ISER0 register */
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if ((IRQNumber > 31) && (IRQNumber < 64))
		{
			/* program ISER1 register */
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if ((IRQNumber >= 64) && (IRQNumber < 96))
		{
			/* program ISER2 register */
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			/* program ICER0 register */
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if ((IRQNumber > 31) && (IRQNumber < 64))
		{
			/* program ICER1 register */
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if ((IRQNumber >= 64) && (IRQNumber < 96))
		{
			/* program ICER2 register */
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
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
	/* find out the ipr register */
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;
		uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
		*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
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
void SPI_IRQHandling (SPI_Handle_t *pHandle)
{
	uint8_t temp1;
	uint8_t temp2;

	/* check for TXE */
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

	if( temp1 && temp2 )
	{
		/* handle TXE */
		spi_txe_interrupt_handle(pHandle);
	}

	/* check for RXNE */
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);

	if( temp1 && temp2 )
	{
		/* handle RXNE */
		spi_rxne_interrupt_handle(pHandle);
	}

	/* check for ovr flag */
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);

	if( temp1 && temp2 )
	{
		/* handle ovr */
		spi_ovr_err_interrupt_handle(pHandle);
	}
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
	if( pSPIx->SR & FlagName)
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


/*helper functions @SPI_IRQHandling*/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) )
	{
		/* 16 bit DFF*/
		/* load the data into the DR */
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLength--;
		pSPIHandle->TxLength--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		/* 8 bit DFF */
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLength--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLength)
	{
		/* TxLength is zero, close the spi transmission and inform the application */

		/* this prevent interrupts from setiing up TXE flag */
		SPI_CloseTransmission(pSPIHandle);
		SPI_AppEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) )
	{
		/* 16 bit DFF*/
		/* load the data into the DR */
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLength -= 2;
		pSPIHandle->RxLength--;
		pSPIHandle->RxLength--;
	}
	else
	{
		/* 8 bit DFF */
		*pSPIHandle->pRxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLength--;
		pSPIHandle->pRxBuffer--;
	}
	if(!pSPIHandle->RxLength)
	{
		/* close the spi reception and inform the application */
		SPI_CloseReception(pSPIHandle);
		SPI_AppEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	/* clear ovr flag */
	if( pSPIHandle->TxState != SPI_BUSY_IN_TX )
	{
		temp=pSPIHandle->pSPIx->DR;
		temp=pSPIHandle->pSPIx->SR;
	}

	(void)temp;

	/* inform the application */
	SPI_AppEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


/**************************************************************************************************************************************************
 * @fn 							- SPI_CloseTransmission
 * @brief 						-
 *
 * @param pSPIHandle			-
 *
 * @return						- none
 *
 * @note						-
 *************************************************************************************************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLength = 0;
	pSPIHandle->TxState = SPI_READY;

}

/**************************************************************************************************************************************************
 * @fn 							- SPI_CloseReception
 * @brief 						-
 *
 * @param pSPIHandle			-
 *
 * @return						- none
 *
 * @note						-
 *************************************************************************************************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLength = 0;
	pSPIHandle->RxState = SPI_READY;
}

/**************************************************************************************************************************************************
 * @fn 							- SPI_ClearOVRFlag
 * @brief 						-
 *
 * @param pSPIx			        -
 *
 * @return						- none
 *
 * @note						-
 *************************************************************************************************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	temp=pSPIx->DR;
	temp=pSPIx->SR;

	(void)temp;
}

__weak void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	/* This is a weak implementation. The application may override the function */

}


