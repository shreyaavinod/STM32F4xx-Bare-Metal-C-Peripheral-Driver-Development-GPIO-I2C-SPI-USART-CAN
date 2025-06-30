/*
 * 004Spi_Send_Data_test.c
 *
 *  Created on: Jun 30, 2025
 *      Author: uma_r
 */

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"
#include<string.h>




/*
 * SPI GPIO PINS
 *
 * PB12   - NSS     AF5 MODE FOR ALL
 * PB13	  - SCLK
 * PB14   - MISO
 * PB15   - MOSI
 *
 */
//1. SPI PIN Config function
	void GPIO_SPIconfig(void)
	{
		//2.handle struct
			Gpio_Handle_t gpio_handle;

			gpio_handle.pGPIOx=GPIOB;
			gpio_handle.GPIO_PinConfig.GPIO_PinMode=PIN_MODE_AF;
			gpio_handle.GPIO_PinConfig.GPIO_PinAlternateFunctionMode=GPIO_AF_5;
			gpio_handle.GPIO_PinConfig.GPIO_PinOPType=OTYPE_PP;
			gpio_handle.GPIO_PinConfig.GPIO_PinSpeed=PIN_SPEED_FAST;

			// peripheral clock enable
			GPIO_PeriClockControl(gpio_handle.pGPIOx, ENABLE);


			//PIN 12
			gpio_handle.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_12;
			GPIO_Init(&gpio_handle);

			gpio_handle.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_13;
			GPIO_Init(&gpio_handle);

			gpio_handle.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_14;
			GPIO_Init(&gpio_handle);


			gpio_handle.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_15;
			GPIO_Init(&gpio_handle);

	}

 void SPI_config(void)
 {


	 SPI_handle_t SPI_handle;
	 SPI_handle.pSPIx=SPI2;
	 SPI_handle.SPI_Config.SPI_Mode=SPI_MODE_MASTER;
	 SPI_handle.SPI_Config.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	 SPI_handle.SPI_Config.SPI_SCLKSpeed=SPI_SCLK_SPEED_DIV2;
	 SPI_handle.SPI_Config.SPI_DFF=SPI_DFF_8BITS;
	 SPI_handle.SPI_Config.SPI_SSM=SPI_SSM_EN;
	 SPI_handle.SPI_Config.SPI_CPOL=SPI_CPOL_LOW;
	 SPI_handle.SPI_Config.SPI_CPHA=SPI_CPHA_LOW;

	 SPI_PeriClockControl(SPI_handle.pSPIx, ENABLE);


	 SPI_Init(&SPI_handle);
 }



int main()
{

	char array[]="hello world";

	GPIO_SPIconfig();

	SPI_config();

	SPI_SSI_Conifg(SPI2,ENABLE);

	SPI_PeriEnable(SPI2, ENABLE);

	SPI_Transmitdata(SPI2, ( uint8_t *)array, strlen(array));

	SPI_PeriEnable(SPI2, DISABLE);

	while (1);

}
