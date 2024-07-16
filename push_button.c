/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Rifat
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "_hal_f401xx.h"

void delay(void){
	for(uint32_t i = 0; i < 1000000; i++);
}

int main(void)
{
	GPIO_handle_t led, button;
 	memset(&led, 0, sizeof(led));
	memset(&button, 0, sizeof(button));

	led.pGPIOx = GPIOC;
	led.GPIO_pinConfig.pinNumber = PIN_13;
	led.GPIO_pinConfig.pinMode = Output_mode;
	led.GPIO_pinConfig.pinSpeed = High_speed;
	led.GPIO_pinConfig.pinOutputType = Output_push_pull;
	led.GPIO_pinConfig.pinPUPD = No_pull_up_pull_down;

//	GPIO_ClkControl(led.pGPIOx, ENABLE);
	GPIO_Init(&led);

	button.pGPIOx = GPIOA;
	button.GPIO_pinConfig.pinNumber = PIN_0;
	button.GPIO_pinConfig.pinMode = Input_mode;
	button.GPIO_pinConfig.pinSpeed = High_speed;
	button.GPIO_pinConfig.pinPUPD = Pull_up;

//	GPIO_ClkControl(button.pGPIOx, ENABLE);
	GPIO_Init(&button);

	/* Loop forever */
	while(1)
	{
		if(!GPIO_ReadPin(button.pGPIOx, PIN_0)){
		GPIO_TogglePin(led.pGPIOx, PIN_13);
//		GPIO_WritePin(led.pGPIOx, PIN_13, SET);
//		delay();
//		GPIO_WritePin(led.pGPIOx, PIN_13, RESET);
		delay();
		}
	}
	return 0;
}
