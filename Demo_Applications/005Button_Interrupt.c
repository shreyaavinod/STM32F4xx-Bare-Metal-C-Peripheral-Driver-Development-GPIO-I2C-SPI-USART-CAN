/*
 * 005Button_Interrupt.c
 *
 *  Created on: Jul 3, 2025
 *      Author: uma_r
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include <string.h>

void delay(void)
{
	for (int i=0;i<500000/2;i++);
}

void GPIO_button_config(void)

{

	Gpio_Handle_t gpio_handle_button;

	memset(&gpio_handle_button,0,sizeof(gpio_handle_button));
		gpio_handle_button.pGPIOx=GPIOC;
		gpio_handle_button.GPIO_PinConfig.GPIO_PinMode=PIN_MODE_FT;
		gpio_handle_button.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_5;
		gpio_handle_button.GPIO_PinConfig.GPIO_PinPuPdControl=PIN_PU;
	GPIO_Init(&gpio_handle_button);
}

void LED_Config(void)
{
	Gpio_Handle_t gpio_handle_led;
	memset(&gpio_handle_led,0,sizeof(gpio_handle_led));
			gpio_handle_led.pGPIOx=GPIOA;
			gpio_handle_led.GPIO_PinConfig.GPIO_PinMode=PIN_MODE_OUT;
			gpio_handle_led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_5;
			gpio_handle_led.GPIO_PinConfig.GPIO_PinPuPdControl=PIN_NO_PU_PD;
			gpio_handle_led.GPIO_PinConfig.GPIO_PinOPType=OTYPE_PP;
			gpio_handle_led.GPIO_PinConfig.GPIO_PinSpeed=PIN_SPEED_FAST;

		GPIO_Init(&gpio_handle_led);

}
void EXTI9_5_IRQHandler (void)
	{
		delay();
		GPIO_IRQHandler(GPIO_PIN_NUM_5);

		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);


	}


int main()
{


	GPIO_button_config();

	LED_Config();

	GPIO_IRQConifg(IRQ_NUM_EXTI9_5, ENABLE);

	IRQ_Priority_Config(IRQ_NUM_EXTI9_5, 255);


	while(1);


}
