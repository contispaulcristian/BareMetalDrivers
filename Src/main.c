/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Paul Contis
 * @brief          : Main program body   V1.1
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Paul Contis.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "stm32f446xx.h"
#include "string.h"

/*************************************************************************************************************************************************/
#define LOW				0
#define BTN_PRESSED		LOW
/*************************************************************************************************************************************************/
void delay(void);
/*************************************************************************************************************************************************/



int main(void)
{
	/* This section is allocated for testing the driver functionality using interrupt mode. */
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioBtn;

	/* initialize structures */
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));

	/* led gpio configuration */
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	/* button gpio configuration */
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	/* IRQ configurations */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

    /* Loop forever */
	while(1);
		/*{
		;
			if((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}*/

	return 0;
}
/*************************************************************************************************************************************************/

void delay(void)
{
	for(uint32_t count = 0 ; count < 500000/2; count++);
}

/*************************************************************************************************************************************************/

void EXTI15_10_IRQHandler(void)
{
	delay();
	/* handle the interrupt */
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}

