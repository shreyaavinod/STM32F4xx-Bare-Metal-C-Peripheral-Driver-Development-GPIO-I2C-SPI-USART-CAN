
/*
 * Header files inclusion
 */

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx.h"


/* GPIO PIN CONFIG STRUCT
    typedef struct {
	uint8_t GPIO_PinNumber ;    <possible values @GPIO_PinNum>
	uint8_t GPIO_PinMode ;	     <possible values @GPIO_PinMode>
	uint8_t GPIO_PinSpeed ;		 <possible values @GPIO_PinSpeed>
	uint8_t GPIO_PinOPType ;	 <possible values @GPIO_PinOPType>
	uint8_t GPIO_PinPuPdControl ; <possible values @GPIO_PinPUPD>
	uint8_t GPIO_PinAlternateFunctionMode ; <possible values @GPIO_PinAF>


}Gpio_PinConfig_Struct;

//GPIO HANDLE STRUCT
typedef struct {
	Gpio_struct *pGPIOx; // This is a pointer to the base address of the Gpio_struct
	Gpio_PinConfig_Struct GPIO_PinConfig;
}Gpio_Handle_t;*/

/* ---------------APIs PROVIDED BY THE DRIVER----------------*/

//1. Peripheral clock enable

void GPIO_PeriClockControl(Gpio_struct *pGPIOx, uint8_t EnOrDis)

{
    if (EnOrDis == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
        	GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB)
        {
        	GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC)
                {
                	GPIOC_PCLK_EN();
                }
        else if(pGPIOx == GPIOD)
                {
                	GPIOD_PCLK_EN();
                }
        else if(pGPIOx == GPIOE)
                {
                	GPIOE_PCLK_EN();
                }
        else if(pGPIOx == GPIOF)
                {
                	GPIOF_PCLK_EN();
                }
        else if(pGPIOx == GPIOG)
                {
                	GPIOG_PCLK_EN();
                }
        else if(pGPIOx == GPIOH)
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
    	        else if(pGPIOx == GPIOB)
    	        {
    	        	GPIOB_PCLK_DI();
    	        }
    	        else if(pGPIOx == GPIOC)
    	                {
    	                	GPIOC_PCLK_DI();
    	                }
    	        else if(pGPIOx == GPIOD)
    	                {
    	                	GPIOD_PCLK_DI();
    	                }
    	        else if(pGPIOx == GPIOE)
    	                {
    	                	GPIOE_PCLK_DI();
    	                }
    	        else if(pGPIOx == GPIOF)
    	                {
    	                	GPIOF_PCLK_DI();
    	                }
    	        else if(pGPIOx == GPIOG)
    	                {
    	                	GPIOG_PCLK_DI();
    	                }
    	        else if(pGPIOx == GPIOH)
    	                {
    	                	GPIOH_PCLK_DI();
    	                }
    }

}
//2. Initialisation and Deinitialisation

void GPIO_Init(Gpio_Handle_t *pGpioHandle)
{
	GPIO_PeriClockControl(pGpioHandle->pGPIOx, ENABLE);
	uint32_t temp;
	uint32_t mask;
	if (pGpioHandle->GPIO_PinConfig.GPIO_PinMode < PIN_MODE_ANALOG)
	{
			//1. MODER
			temp= pGpioHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			mask= (0x3) << (2* pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);

			pGpioHandle->pGPIOx->MODER &=~mask;
			pGpioHandle->pGPIOx->MODER |=temp;
			temp=0;
			mask=0;
			//2. SPEEDR
			temp= pGpioHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			mask= (0x3) << (2* pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGpioHandle->pGPIOx->OSPEEDER &=~mask;
			pGpioHandle->pGPIOx->OSPEEDER|=temp;
			temp=0;
			mask=0;
			//3. PUPDR
			temp= pGpioHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			mask= (0x3) << (2* pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGpioHandle->pGPIOx->PUPDR &=~mask;
			pGpioHandle->pGPIOx->PUPDR |=temp;
			temp=0;
			mask=0;
			//4. OUTPUT TYPER
			temp= pGpioHandle->GPIO_PinConfig.GPIO_PinOPType << (pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			mask=(0x1)<<(pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGpioHandle->pGPIOx->OTYPER &=~mask;
			pGpioHandle->pGPIOx->OTYPER |=temp;
			temp=0;
			mask=0;

			//Alternate function mode in init
			if (pGpioHandle->GPIO_PinConfig.GPIO_PinMode==PIN_MODE_AF)
			{
				//Configure AF register
				uint8_t reg_select= (pGpioHandle->GPIO_PinConfig.GPIO_PinNumber/8);
				uint8_t pin_select=pGpioHandle->GPIO_PinConfig.GPIO_PinNumber%8;
				temp=pGpioHandle->GPIO_PinConfig.GPIO_PinAlternateFunctionMode<<(4*pin_select);
				mask= 0xF <<(4*pin_select);
				if (reg_select==1)
				{
					pGpioHandle->pGPIOx->AFR[1] &=~(mask);
					pGpioHandle->pGPIOx->AFR[1] |=temp;

				}
				else
				{
					pGpioHandle->pGPIOx->AFR[0] &=~(mask);
					pGpioHandle->pGPIOx->AFR[0] |=temp;

				}
				temp=0;
				mask=0;

			}
	}
	else
	{
		// SET PIN IN INPUT MODE
		temp= (PIN_MODE_IN)<<(2*(pGpioHandle->GPIO_PinConfig.GPIO_PinNumber));
		mask= 0x3<<(2*pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGpioHandle->pGPIOx->MODER &=~(mask);
		pGpioHandle->pGPIOx->MODER |=temp;
		temp=0;
		mask=0;

		//3. PUPDR
					temp= pGpioHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
					mask= (0x3) << (2* pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
					pGpioHandle->pGPIOx->PUPDR &=~mask;
					pGpioHandle->pGPIOx->PUPDR |=temp;
					temp=0;
					mask=0;

		//interrupts
		if (pGpioHandle->GPIO_PinConfig.GPIO_PinMode==PIN_MODE_RT)
		{

			EXTI->RTSR|=(0x1<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR&=~(0X1<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGpioHandle->GPIO_PinConfig.GPIO_PinMode==PIN_MODE_FT)
		{
			EXTI->FTSR|=(0x1<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR&=~(0X1<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGpioHandle->GPIO_PinConfig.GPIO_PinMode==PIN_MODE_RFT)
		{
			EXTI->FTSR|=(0x1<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR|=(0X1<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//SYSCFG EXTI
		//1. syscfg peripheral clk enable
		//2. syscfg exti
		SYSCFG_PCLK_EN();

		uint16_t portnum = BASEADDR_TO_PORTNUM(pGpioHandle->pGPIOx);
		uint8_t regselect1= pGpioHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t pinselect1=pGpioHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		temp= portnum<<(pinselect1*4);
		mask=0xF<<(pinselect1*4);
		SYSCFG->EXTICR[regselect1]&=~(mask);
		SYSCFG->EXTICR[regselect1]|=temp;
		temp=0;
		mask=0;

		//TO ACCEPT INTERRUPT IN THAT PIN
		EXTI->IMR|=(0X1)<<pGpioHandle->GPIO_PinConfig.GPIO_PinNumber;


	}


}
void GPIO_Deinit(Gpio_struct *pGPIOx){

		if (pGPIOx == GPIOA)
	        		{
				GPIOA_REG_RST();
	        		}
	        else if(pGPIOx == GPIOB)
	        		{
	        	GPIOB_REG_RST();
	        		}
	        else if(pGPIOx == GPIOC)
	                {
	        	GPIOC_REG_RST();
	                }
	        else if(pGPIOx == GPIOD)
	                {
	        	GPIOD_REG_RST();
	                }
	        else if(pGPIOx == GPIOE)
	                {
	        	GPIOE_REG_RST();
	                }
	        else if(pGPIOx == GPIOF)
	                {
	        	GPIOF_REG_RST();
	                }
	        else if(pGPIOx == GPIOG)
	                {
	        	GPIOG_REG_RST();
	                }
	        else if(pGPIOx == GPIOH)
	                {
	        	GPIOH_REG_RST();
	                }

}

//3. Data Read and Write

uint8_t GPIO_ReadFromInputPin(Gpio_struct *pGPIOx,uint8_t PinNumber){
	uint8_t value = (((uint16_t)(pGPIOx->IDR) >> PinNumber)&(0x1));
	return value;

}
uint16_t GPIO_ReadFromInputPort(Gpio_struct *pGPIOx){

	uint16_t value2= ((uint16_t)pGPIOx->IDR);
	return value2;

}

void GPIO_WriteToOutputPin(Gpio_struct *pGPIOx,uint8_t PinNumber, uint8_t val){
	if (val==GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (0x1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(0x1<<PinNumber);
	}

}
void GPIO_WriteToOutputPort(Gpio_struct *pGPIOx,uint16_t val){

	pGPIOx->ODR = val;

}

void GPIO_ToggleOutputPin(Gpio_struct *pGPIOx,uint8_t PinNumber){

	pGPIOx->ODR^=(1<<PinNumber);

}



//4. IRQ HANDLER AND CONFIG

void GPIO_IRQConifg(uint8_t IRQNum,uint8_t EnOrDis ){

	uint8_t reg_select2= IRQNum/32;
	uint8_t pin_select2= IRQNum%32;
	if (EnOrDis==ENABLE)
	{
	 NVIC->ISER[reg_select2]|=((0x1)<<pin_select2);
	}
	else
	{
	 NVIC->ICER[reg_select2]|=((0x1)<<pin_select2);
	}

}

void IRQ_Priority_Config(uint8_t IRQNum,uint8_t IRQPriority)
{
	uint8_t reg_select3=IRQNum/4;
	uint8_t pin_select3=IRQNum%4;


	uint32_t mask=(0xFF<<(pin_select3*8));
	uint32_t temp=IRQPriority<<((pin_select3*8));

	NVIC->IPR[reg_select3]&=~(mask);
	NVIC->IPR[reg_select3]|=temp;
}


void GPIO_IRQHandler(uint8_t PinNumber){

	if (EXTI->PR&(1<<PinNumber))
	{
	EXTI->PR|=(0X1<<PinNumber);
	}


}








