/*
 * SPI_cmd_handling.c
 *
 *  Created on: Jan 27, 2023
 *      Author: rifat
 */

/*
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * PB13 -> SPI2_CLK
 * PB12 -> SPI2_NSS
 * ALT function mode 5
 * */
#include <string.h>
#include "_hal_f401xx.h"
//#include "gpio_driver.h"
//#include "spi_driver.h"

void SPI2_GPIO_Init(void){
	GPIO_handle_t SPI_pins;
	SPI_pins.pGPIOx = GPIOB;
	SPI_pins.GPIO_pinConfig.pinMode = Alternate_function_mode;
	SPI_pins.GPIO_pinConfig.pinAltFunction = 5;
	SPI_pins.GPIO_pinConfig.pinOutputType = Output_push_pull;
	SPI_pins.GPIO_pinConfig.pinPUPD = No_pull_up_pull_down;
	SPI_pins.GPIO_pinConfig.pinSpeed = High_speed;

//	SCLK
	SPI_pins.GPIO_pinConfig.pinNumber = PIN_13;
	GPIO_Init(&SPI_pins);
//	MOSI
	SPI_pins.GPIO_pinConfig.pinNumber = PIN_15;
	GPIO_Init(&SPI_pins);
//	MISO
	SPI_pins.GPIO_pinConfig.pinNumber = PIN_14;
	GPIO_Init(&SPI_pins);
//	NSS
	SPI_pins.GPIO_pinConfig.pinNumber = PIN_12;
	GPIO_Init(&SPI_pins);
}

void SPI2_Init(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_FullDuplex;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_MASTER;
	SPI2handle.SPIConfig.SPI_ClockSpeed = SPI_Speed_Div2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8B;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}

int main(void)
{
	char userdata[]= "hello world";
	SPI2_GPIO_Init();
	SPI2_Init();
	SPI_SSIConfig(SPI2,ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);
	SPI_Transmit(SPI2, (uint8_t*)userdata, strlen(userdata));
	SPI_PeripheralControl(SPI2, DISABLE);
	while(1);
	return 0;
}
