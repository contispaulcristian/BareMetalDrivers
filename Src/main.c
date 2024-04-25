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
#include<stdio.h>
/*************************************************************************************************************************************************/
#define LOW				0
#define BTN_PRESSED		LOW
#define LED_ON					1
#define LED_OFF					0

#define MAX_LEN 500

/* arduino led */
#define LED_PIN					9

SPI_Handle_t SPI2Handle;

char RcvBuff[MAX_LEN];

volatile char ReadByte;

volatile uint8_t rcvStop = 0;

volatile uint8_t dataAvailable = 0; /* This flag will be set in the interrupt handler of the Arduino interrupt GPIO */





/*************************************************************************************************************************************************/
void delay(void);
void SPI2_GPIOInits(void);
void SPI2_Inits(void);
/*void GPIO_ButtonInit(void);
void GPIO_LedInit(void);*/
uint8_t SPI_VerifyResponse(uint8_t ackbyte);
void Slave_GPIO_InterruptPinInit(void);
/*************************************************************************************************************************************************/

/*	This is a simple application to test SPI communication using polling mode.
	ALT function mode: 5
	SPI2_NSS  -> PB12
	SPI2_SCK  -> PB13
	SPI2_MISO -> PB14
	SPI2_MOSI -> PB15 */

int main(void)
{
	uint8_t dummy = 0xff;

	/*GPIO_ButtonInit();*/
	/*GPIO_LedInit();*/

	SPI2_GPIOInits();	/* This function is used to initialize the GPIO pins to behave as SPI2 pins*/

	Slave_GPIO_InterruptPinInit();

	SPI2_Inits();		/* This function is used to initialize the SPI2 peripheral parameters */

	SPI_SSOEConfig(SPI2,  ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while(1)
	{
		rcvStop = 0;

		while( !dataAvailable ); /* wait till data available interrupt from transmitter device */

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, DISABLE);

		SPI_PeripheralControl(SPI2, ENABLE);

		while( !rcvStop )
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while( SPI_SendDataIT(&SPI2Handle, &dummy, 1) == SPI_BUSY_IN_TX );
			while( SPI_ReceiveDataIT(&SPI2Handle, &ReadByte, 1) == SPI_BUSY_IN_RX );
		}

		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) ); /* confirm SPI is not busy */

		/* disable the SPI2 peripheral */
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Rcvd data = %s\n", RcvBuff);

		dataAvailable = 0;
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	}
	return 0;
}
/*************************************************************************************************************************************************/

void delay(void)
{
	for(uint32_t count = 0 ; count < 500000/2; count++);
}

/*************************************************************************************************************************************************/

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode	= GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	/* SCLK */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	/* MOSI */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	/* MISO */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	/* NSS */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}
/*************************************************************************************************************************************************/

void SPI2_Inits()
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLCK_SPEED_DIV8; /* generates sclk of 2MHz, fclk= 16MHz */
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; /* hardware slave management enabled for NSS pin */

	SPI_Init(&SPI2handle);

}

/*************************************************************************************************************************************************/
/*void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;
	 button gpio configuration
		GpioBtn.pGPIOx = GPIOC;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GpioBtn);
}*/

/*************************************************************************************************************************************************/

/*void GPIO_LedInit(void)
{
	GPIO_Handle_t GpioLed;
	 led gpio configuration
		GpioLed.pGPIOx = GPIOA;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GpioLed);
}*/

/*************************************************************************************************************************************************/
uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if( ackbyte == 0xF5)
	{
		/* ack */
		return 1;
	}
	return 0;
}


/*************************************************************************************************************************************************/

void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin,0,sizeof(spiIntPin));

	/* this is configuration */
	spiIntPin.pGPIOx = GPIOC;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&spiIntPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

}


void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI2Handle);
}


void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	static uint32_t i=0;
	/* In the RX complete event, copy data into rcv buffer. '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || (i == MAX_LEN))
		{
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i=0;
		}
	}
}

/* Slave data available interrupt handler */
void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	dataAvailable = 1;
}


