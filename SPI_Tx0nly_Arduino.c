/*
 * SPI_Tx0nly_Arduino.c
 *
 *  Created on: Jan 25, 2023
 *      Author: rifat
 */
// PB15 --> MOSI, PB14 --> MISO, PB13 --> SCK, PB12 -->NSS, ALT function mode 5

#include <string.h>
#include "_hal_f401xx.h"
#include "gpio_driver.h"
#include "spi_driver.h"

void delay(void){
	for(uint32_t i = 0; i < 1000000; i++);
}

void GPIO_ButtonInit(void){
	GPIO_handle_t button;

	button.pGPIOx = GPIOA;
	button.GPIO_pinConfig.pinNumber = PIN_0;
	button.GPIO_pinConfig.pinMode = Input_mode;
//	button.GPIO_pinConfig.pinOutputType = Output_push_pull;
	button.GPIO_pinConfig.pinSpeed = High_speed;
	button.GPIO_pinConfig.pinPUPD = Pull_up;

	GPIO_Init(&button);
}

void SPI2_GPIOInit(void){
	GPIO_handle_t SPIpins;

	SPIpins.pGPIOx=GPIOB;
	SPIpins.GPIO_pinConfig.pinMode = Alternate_function_mode;
	SPIpins.GPIO_pinConfig.pinAltFunction = 5;
	SPIpins.GPIO_pinConfig.pinOutputType = Output_push_pull;
	SPIpins.GPIO_pinConfig.pinPUPD = No_pull_up_pull_down;
	SPIpins.GPIO_pinConfig.pinSpeed = Very_high_speed;

	SPIpins.GPIO_pinConfig.pinNumber = PIN_12; //NSS
	GPIO_Init(&SPIpins);

	SPIpins.GPIO_pinConfig.pinNumber = PIN_13; //SCK
	GPIO_Init(&SPIpins);

//	SPIpins.GPIO_pinConfig.pinNumber = PIN_14; //MISO
//	GPIO_Init(&SPIpins);

	SPIpins.GPIO_pinConfig.pinNumber = PIN_15; //MOSI
	GPIO_Init(&SPIpins);
}

void SPI2_Init(void){
	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_FullDuplex;
	SPI2_Handle.SPIConfig.SPI_DeviceMode = SPI_MASTER;
	SPI2_Handle.SPIConfig.SPI_ClockSpeed = SPI_Speed_Div8;
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8B;
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;				//Hardeare slave management enabled

	SPI_Init(&SPI2_Handle);
}

int main (void){
	char user_data[]= "Hello world";

	GPIO_ButtonInit();
	SPI2_GPIOInit();
	SPI2_Init();
	// Enabling SSI bits to avoid mode fault
//	SPI_SSIConfig(SPI2, ENABLE); // Seta NSS signal internally high and avoid modef error
	//Enable SSOE bit to automate slave seclect
	SPI_SSOEonfig(SPI2, ENABLE);

	while(1){
		while(GPIO_ReadPin(GPIOA, PIN_0));
		delay();
		// Enable the SPI2 peripheral to start communication
		SPI_PeripheralControl(SPI2, ENABLE);
		//Sending length information
		uint8_t datalen = strlen(user_data);
		SPI_TransmitData(SPI2, &datalen, 1);
		SPI_TransmitData(SPI2, (uint8_t*)user_data, strlen(user_data));
		while( SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY) );
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}


