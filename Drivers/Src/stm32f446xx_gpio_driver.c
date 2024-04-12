/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Mar 28, 2024
 *      Author: paul.contis
 */

/*************************************************************************************************************************************************/
#include "stm32f446xx_gpio_driver.h"
/**************************************************************************************************************************************************
 * @fn 							- GPIO_PeriClockControl
 * @brief 						- This function enables or disables peripheral clock for given GPIO port
 *
 * @param pGPIOx[in]			- Base address of the gpio peripheral
 * @param EnorDi[in]			- ENABLE or BISABLE macros
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void GPIO_PeriClockControl (GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
        	GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
        	GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
        	GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
       	{
        	GPIOD_PCLK_EN();
       	}
        else if (pGPIOx == GPIOE)
        {
        	GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
        	GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
        	GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
        	GPIOH_PCLK_EN();
        }
    }
	else
    {
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
    }
}
/**************************************************************************************************************************************************
 * @fn 							- GPIO_Init
 * @brief 						- Init
 *
 * @param pGPIOHandle			-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void GPIO_Init (GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; //temp. register
	uint8_t temp1 = 0;
	uint8_t temp2 = 0;
	uint8_t portcode = 0;

	/* Enable the peripheral clock */
	GPIO_PeriClockControl((pGPIOHandle->pGPIOx), ENABLE);
	/* configure the mode of gpio pin*/
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
		/* The non interrupt mode */
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /* clearing bits before setting it*/
      	pGPIOHandle->pGPIOx->MODER |= temp; /* setting the bit*/
    }
	else
    {

		/* The interrupt mode */
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			/* Configure the FTSR */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear the coresponding RTSR bit*/
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			/* Configure the RTSR */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear the coresponding RTSR bit*/
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			/* Configure both RFSR and RTSR */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		/* configure the GPIO port selection in SYSCFG_EXTICR */
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << temp2 * 4;
		/* enable the exti interrupt delivery using IMR */
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    }
	/* configure the speed*/
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	/* configure the pupd settings*/
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	/* configure the optype*/ //TODO: configure only in output mode
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	/* configure the alt functionality*/
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
		uint8_t temp1;
		uint8_t temp2;
		/* selecting the register AFR[0] (AFRL) or AFR[1] (AFRH) */
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		/* selecting the bit position*/
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}
/**************************************************************************************************************************************************
 * @fn 							- GPIO_DeInit
 * @brief 						- De-init
 *
 * @param pGPIOx				-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
    {
		GPIOA_REG_RESET();
    }
	else if (pGPIOx == GPIOB)
    {
		GPIOB_REG_RESET();
    }
	else if (pGPIOx == GPIOC)
    {
		GPIOC_REG_RESET();
    }
	else if (pGPIOx == GPIOD)
    {
		GPIOD_REG_RESET();
    }
	else if (pGPIOx == GPIOE)
    {
		GPIOE_REG_RESET();
    }
	else if (pGPIOx == GPIOF)
    {
		GPIOF_REG_RESET();
    }
	else if (pGPIOx == GPIOG)
    {
		GPIOG_REG_RESET();
    }
	else if (pGPIOx == GPIOH)
    {
		GPIOH_REG_RESET();
    }
}
/**************************************************************************************************************************************************
 * @fn 							- GPIO_ReadFromInputPin
 * @brief 						- Data read
 *
 * @param pGPIOx				-
 * @param PinNumber				-
 *
 * @return						- 0 or 1
 *
 * @note						- none
 *************************************************************************************************************************************************/
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
/**************************************************************************************************************************************************
 * @fn 							- GPIO_ReadFromInputPort
 * @brief 						- Data read
 *
 * @param pGPIOx				-
 *
 * @return						-
 *
 * @note						- none
 *************************************************************************************************************************************************/
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);
	return value;
}
/**************************************************************************************************************************************************
 * @fn 							- GPIO_WriteToOutputPin
 * @brief 						- Data write
 *
 * @param pGPIOx				-
 * @param PinNumber				-
 * @param Value					-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
    {
		/*write 1 to the output data register at the bit field corresponding to the pin number*/
		pGPIOx->ODR |= (1 << PinNumber);
    }
	else
    {
		/*write 0 to the output data register at the bit field corresponding to the pin number*/
		pGPIOx->ODR &= ~(1 << PinNumber);
    }
}
/**************************************************************************************************************************************************
 * @fn 							- GPIO_WriteToOutputPort
 * @brief 						- Data write
 *
 * @param pGPIOx				-
 * @param Value					-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= Value;
}
/**************************************************************************************************************************************************
 * @fn 							- GPIO_ToggleOutputPin
 * @brief 						-
 *
 * @param pGPIOx				-
 * @param PinNumber				-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}
/**************************************************************************************************************************************************
 * @fn 							- GPIO_IRQInterruptConfig
 * @brief 						- IRQ Configuration and ISR handling
 *
 * @param IRQNumber				-
 * @param EnorDi				-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void GPIO_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn							- void GPIO_IRQPriorityConfig(uint8_t)
 * @brief						-
 *
 * @param IRQNumber				-
 * @param IRQPriority			-
 *
 * @return 						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	/* find out the ipr register */
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/**************************************************************************************************************************************************
 * @fn 							- GPIO_IRQHandling
 * @brief 						-
 *
 * @param PinNumber				-
 *
 * @return						- none
 *
 * @note						- none
 *************************************************************************************************************************************************/
void GPIO_IRQHandling (uint8_t PinNumber)
{
	/* Clear the EXTI PR register corresponding to the pin number */
	if ( EXTI->PR & (1 << PinNumber))
    {
		/* clear */
		EXTI->PR |= (1 << PinNumber);
    }
}

