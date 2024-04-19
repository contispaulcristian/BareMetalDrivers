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

/* command codes */
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x51

#define LED_ON					1
#define LED_OFF					0

/* arduino analog pins */
#define ANALOG_PIN0				0

/* arduino led */
#define LED_PIN					9
/*************************************************************************************************************************************************/
void delay(void);
void SPI2_GPIOInits(void);
void SPI2_Inits(void);
void GPIO_ButtonInit(void);
uint8_t SPI_VerifyResponse(uint8_t ackbyte);
/*************************************************************************************************************************************************/

/*	This is a simple application to test SPI communication using polling mode.
	ALT function mode: 5
	SPI2_NSS  -> PB12
	SPI2_SCK  -> PB13
	SPI2_MISO -> PB14
	SPI2_MOSI -> PB15 */

int main(void)
{
	//char user_data[]= "Hello World";
	//uint8_t dataLength = strlen(user_data);
	uint8_t dummy_write = 0xff;
	uint8_t cmd_code = COMMAND_LED_CTRL;
	uint8_t ackbyte;
	uint8_t args[2];
	uint8_t dummy_read;

	GPIO_ButtonInit();

	SPI2_GPIOInits();	/* This function is used to initialize the GPIO pins to behave as SPI2 pins*/

	SPI2_Inits();		/* This function is used to initialize the SPI2 peripheral parameters */

	SPI_SSOEConfig(SPI2,  ENABLE);

	while(1)
	{
		while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		delay();

		/* enable the SPI2 peripheral */
		SPI_PeripheralControl(SPI2, ENABLE);

		/* CMD_LED_CTRL <pin no(1)> <value(1)> */
		SPI_SendData(SPI2, &cmd_code, 1);

		/* do dummy read to clear off the RXNE*/
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		/* send some dummy bits (1byte) to fetch the response from the slave. */
		SPI_SendData(SPI2, &dummy_write, 1);

		/* read the ack byte received */
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if ( SPI_VerifyResponse(ackbyte) )
		{
			/* send arguments */
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
		}

		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

		/* disable the SPI2 peripheral */
		SPI_PeripheralControl(SPI2, DISABLE);
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
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;
	/* button gpio configuration */
		GpioBtn.pGPIOx = GPIOC;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GpioBtn);
}

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




