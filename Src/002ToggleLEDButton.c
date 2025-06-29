/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"


void delay(void){
	for (uint32_t i=0; i<500000/2; i++);
}

int main(void)
{
	/*1. Intialise the LED.
	 * 2.toggle API
	 */

	Gpio_PinConfig_Struct PinConfig;

			PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_5;
			PinConfig.GPIO_PinMode=PIN_MODE_OUT;
			PinConfig.GPIO_PinOPType=OTYPE_PP;
			PinConfig.GPIO_PinSpeed=PIN_SPEED_LOW;

			PinConfig.GPIO_PinPuPdControl=PIN_NO_PU_PD;


	Gpio_Handle_t gpio_handle;
			gpio_handle.pGPIOx=GPIOA;
			gpio_handle.GPIO_PinConfig=PinConfig;


	Gpio_PinConfig_Struct PinConfig2;
	PinConfig2.GPIO_PinNumber=GPIO_PIN_NUM_13;
	PinConfig2.GPIO_PinMode=PIN_MODE_IN;
	PinConfig2.GPIO_PinPuPdControl=PIN_NO_PU_PD;


	Gpio_Handle_t gpio_handle2;
	gpio_handle2.pGPIOx=GPIOC;
	gpio_handle2.GPIO_PinConfig=PinConfig2;


	//0. Plk enable
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC,ENABLE);

	//1. Initialisation
	GPIO_Init(&gpio_handle);
	GPIO_Init(&gpio_handle2);



	for(;;)
	{

		//2. toggling
		//since PC13 will be HIGH BY DEFAULT
		if (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM_13)==0)
		{
		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		}
	}

}
