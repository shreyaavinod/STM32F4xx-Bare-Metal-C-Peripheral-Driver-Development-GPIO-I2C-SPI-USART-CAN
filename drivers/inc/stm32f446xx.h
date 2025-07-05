/*
 * stm32f446xx.h
 *
 *  Created on: Jun 25, 2025
 *      Author: SHREYAA VINOD
 */

#include<stdint.h>

#ifndef DRIVERS_INC_STM32F446XX_H_
#define DRIVERS_INC_STM32F446XX_H_
#define __vo volatile
#define __weak __attribute((weak))

//irq number MACROS
#define IRQ_NUM_EXTI0  	6
#define IRQ_NUM_EXTI1  	7
#define IRQ_NUM_EXTI2  	8
#define IRQ_NUM_EXTI3  	9
#define IRQ_NUM_EXTI4 	10
#define IRQ_NUM_EXTI9_5 	23
#define IRQ_NUM_EXTI15_10 	40
#define IRQ_NUM_SPI1  	35
#define IRQ_NUM_SPI2	36
#define IRQ_NUM_SPI3 	51
#define IRQ_NUM_SPI4    84





//1. DEFINING BASE ADDRESSES OF FLASH AND SRAM
#define FLASH_BASEADDR			 0x08000000U
#define SYS_MEM_BASEADDR		 0x1FFF0000U
#define SRAM1_BASEADDR			 0x20000000U
#define SRAM_BASEADDR			 SRAM1_BASEADDR
#define SRAM2_BASEADDR			 0x2001C000U


//2. DEFINING BASE ADDRESSES OF BUS DOMAINS

#define PERIPH_BASEADDR		    0X40000000U
#define APB1_PERIPH_BASEADDR	PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR	0x40010000U
#define AHB1_PERIPH_BASEADDR    0x40020000U
#define AHB2_PERIPH_BASEADDR	0x50000000U

//3. DEFINING BASE ADDRESSES OF PERIPHERALS IN AHB1 BUS
#define GPIOA_PERIPH_BASEADDR	(AHB1_PERIPH_BASEADDR + 0x0000)
#define GPIOB_PERIPH_BASEADDR	(AHB1_PERIPH_BASEADDR + 0x0400)
#define GPIOC_PERIPH_BASEADDR	(AHB1_PERIPH_BASEADDR + 0x0800)
#define GPIOD_PERIPH_BASEADDR  	(AHB1_PERIPH_BASEADDR + 0x0C00)
#define GPIOE_PERIPH_BASEADDR	(AHB1_PERIPH_BASEADDR + 0x1000)
#define GPIOF_PERIPH_BASEADDR	(AHB1_PERIPH_BASEADDR + 0x1400)
#define GPIOG_PERIPH_BASEADDR	(AHB1_PERIPH_BASEADDR + 0x1800)
#define GPIOH_PERIPH_BASEADDR	(AHB1_PERIPH_BASEADDR + 0x1C00)
#define RCC_PERIPH_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x3800)

//4. DEFINING BASE ADDRESSES OF PERIPHERALS IN APB1 BUS
#define SPI2_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x3800)
#define SPI3_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x3C00)
#define USART2_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x4400)
#define USART3_PERIPH_BASEADDR  (APB1_PERIPH_BASEADDR + 0x4800)
#define UART4_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x4C00)
#define UART5_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x5000)
#define I2C1_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x5400)
#define I2C2_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x5800)
#define I2C3_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x5C00)
#define CAN1_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x6400)
#define CAN2_PERIPH_BASEADDR	(APB1_PERIPH_BASEADDR + 0x6800)

//5. DEFINING BASE ADDRESSES OF PERIPHERALS IN APB2 BUS
#define USART1_PERIPH_BASEADDR	(APB2_PERIPH_BASEADDR + 0x1000)
#define USART6_PERIPH_BASEADDR  (APB2_PERIPH_BASEADDR + 0x1400)
#define SPI1_PERIPH_BASEADDR	(APB2_PERIPH_BASEADDR + 0x3000)  //0x40013000
#define SPI4_PERIPH_BASEADDR	(APB2_PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR  			(APB2_PERIPH_BASEADDR + 0x3C00)
#define NVIC_BASE_ADDR			0xE000E100U

//6. GPIO PERIPHERAL REGISTERS STRUCTURE DEFINITION

/*  USECASE : Gpio_struct * gpio_base_ptr = (Gpio_struct *)GPIOA_PERIPH_BASEADDR;
To access MODER
gpio_base_ptr->MODER = 25;
*/


typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDER;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];  					/* AFR[0] - Alternate function low register  AFR[1] - Alternate function high register */


} Gpio_struct;


typedef struct {

		__vo uint16_t CR1 ;
		__vo uint16_t CR2;
		__vo uint16_t SR;
		__vo uint16_t DR;
		__vo uint16_t CRCPR;
		__vo uint16_t RXCRCCR;
		__vo uint16_t TXCRCR;
		__vo uint16_t I2SCFGR;
		__vo uint16_t I2SPR;



}SPI_struct;

/* RCC PERIPHERAL REGISTER STRUCT */
typedef struct {
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_PLL_CFGR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_AHB1RSTR;
	__vo uint32_t RCC_AHB2RSTR;
	__vo uint32_t RCC_AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t RCC_APB1_RSTR;
	__vo uint32_t RCC_APB2_RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t RCC_AHB1ENR;
	__vo uint32_t RCC_AHB2ENR;
	__vo uint32_t RCC_AHB3ENR;
	__vo uint32_t RESERVED3;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_APB2ENR;
	__vo uint32_t RESERVED4[2];
	__vo uint32_t RCC_AHB1LPENR;
	__vo uint32_t RCC_AHB2LPENR;
	__vo uint32_t RCC_AHB3LPENR;
	__vo uint32_t RESERVED6;
	__vo uint32_t RCC_APB1LPENR;
	__vo uint32_t RCC_APB2LPENR;
	__vo uint32_t RESERVED7[2];
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
	__vo uint32_t RESERVED9[2];
	__vo uint32_t RCC_SSCGR;
	__vo uint32_t RCC_PLLI2SCFGR;
	__vo uint32_t RCC_PLLSAICFGR;
	__vo uint32_t RCC_DCKCFGR;
	__vo uint32_t RCC_CKGATENR;
	__vo uint32_t RCC_DCKCFHR2;


}RCC_struct;

typedef struct {
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RTSR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;

}EXTI_struct;


typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED0[2];
    __vo uint32_t CMPCR;
	uint32_t RESERVED1[3];
	__vo uint32_t CFGR;

}SYSCFG_struct;

typedef struct {
	__vo uint32_t ISER[8];
	uint32_t RESERVED0[24];
	__vo uint32_t ICER[8];
	uint32_t RESERVED1[24];
	__vo uint32_t ISPR[8];
	uint32_t RESERVED2[24];
	__vo uint32_t ICPR[8];
	uint32_t RESERVED3[24];
	__vo uint32_t IABR[8];
	uint32_t RESERVED4[56];
	__vo uint32_t IPR[60];
	__vo uint32_t STIR;

} NVIC_struct;



/* Creating the macros for peripheral base addresses typecasted to (Gpio_struct*) */

#define GPIOA     ((Gpio_struct *)GPIOA_PERIPH_BASEADDR)
#define GPIOB     ((Gpio_struct *)GPIOB_PERIPH_BASEADDR)
#define GPIOC     ((Gpio_struct *)GPIOC_PERIPH_BASEADDR)
#define GPIOD     ((Gpio_struct *)GPIOD_PERIPH_BASEADDR)
#define GPIOE     ((Gpio_struct *)GPIOE_PERIPH_BASEADDR)
#define GPIOF     ((Gpio_struct *)GPIOF_PERIPH_BASEADDR)
#define GPIOG     ((Gpio_struct *)GPIOG_PERIPH_BASEADDR)
#define GPIOH     ((Gpio_struct *)GPIOH_PERIPH_BASEADDR)
#define RCC		  ((RCC_struct *)RCC_PERIPH_BASEADDR)
#define EXTI 	  ((EXTI_struct*)EXTI_BASEADDR)
#define SYSCFG    ((SYSCFG_struct*)SYSCFG_BASEADDR)
#define NVIC 	  ((NVIC_struct*)NVIC_BASE_ADDR)
#define SPI1	  ((SPI_struct*)SPI1_PERIPH_BASEADDR)
#define SPI2	  ((SPI_struct*)SPI2_PERIPH_BASEADDR)
#define SPI3	  ((SPI_struct*)SPI3_PERIPH_BASEADDR)
#define SPI4	  ((SPI_struct*)SPI4_PERIPH_BASEADDR)


/*clock enabling macros
 *
 */
//GPIO
#define GPIOA_PCLK_EN()   (RCC->RCC_AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()   (RCC->RCC_AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()   (RCC->RCC_AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()   (RCC->RCC_AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()   (RCC->RCC_AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()   (RCC->RCC_AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()   (RCC->RCC_AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()   (RCC->RCC_AHB1ENR |= (1<<7))

//I2C

#define I2C1_PCLK_EN()	    (RCC->RCC_APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->RCC_APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->RCC_APB1ENR |= (1<<23))

//SPI
#define SPI1_PCLK_EN()	    (RCC->RCC_APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()	    (RCC->RCC_APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()	    (RCC->RCC_APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()	    (RCC->RCC_APB2ENR |= (1<<13))

//USART/UART
#define USART1_PCLK_EN()	    (RCC->RCC_APB2ENR |= (1<<4))
#define USART2_PCLK_EN()	    (RCC->RCC_APB1ENR |= (1<<17))
#define USART3_PCLK_EN()	    (RCC->RCC_APB1ENR |= (1<<18))
#define UART4_PCLK_EN()	        (RCC->RCC_APB1ENR |= (1<<19))
#define UART5_PCLK_EN()	        (RCC->RCC_APB1ENR |= (1<<20))
#define USART6_PCLK_EN()	    (RCC->RCC_APB2ENR |= (1<<5))

//SYSCFG

#define SYSCFG_PCLK_EN()		(RCC->RCC_APB2ENR |=(1<<14))


//DISABLE MACROS

//GPIO
#define GPIOA_PCLK_DI()   (RCC->RCC_AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()   (RCC->RCC_AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()   (RCC->RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()   (RCC->RCC_AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()   (RCC->RCC_AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()   (RCC->RCC_AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()   (RCC->RCC_AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()   (RCC->RCC_AHB1ENR &= ~(1<<7))

//I2C

#define I2C1_PCLK_DI()	    (RCC->RCC_APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1<<23))

//SPI
#define SPI1_PCLK_DI()	    (RCC->RCC_APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()	    (RCC->RCC_APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()	    (RCC->RCC_APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()	    (RCC->RCC_APB2ENR &= ~(1<<13))

//USART/UART
#define USART1_PCLK_DI()	    (RCC->RCC_APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()	    (RCC->RCC_APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()	    (RCC->RCC_APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()	        (RCC->RCC_APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()	        (RCC->RCC_APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()	    (RCC->RCC_APB2ENR &= ~(1<<5))

//SYSCFG

#define SYSCFG_PCLK_DI()		(RCC-> RCC_APB2ENR &=~(1<<14))

//Peripheral reset macros
#define GPIOA_REG_RST()   do {RCC->RCC_AHB1RSTR |= (1<<0); RCC->RCC_AHB1RSTR &= ~(1<<0);}while (0)
#define GPIOB_REG_RST()   do {RCC->RCC_AHB1RSTR |= (1<<1); RCC->RCC_AHB1RSTR &= ~(1<<1);}while (0)
#define GPIOC_REG_RST()   do {RCC->RCC_AHB1RSTR |= (1<<2); RCC->RCC_AHB1RSTR &= ~(1<<2);}while (0)
#define GPIOD_REG_RST()   do {RCC->RCC_AHB1RSTR |= (1<<3); RCC->RCC_AHB1RSTR &= ~(1<<3);}while (0)
#define GPIOE_REG_RST()   do {RCC->RCC_AHB1RSTR |= (1<<4); RCC->RCC_AHB1RSTR &= ~(1<<4);}while (0)
#define GPIOF_REG_RST()   do {RCC->RCC_AHB1RSTR |= (1<<5); RCC->RCC_AHB1RSTR &= ~(1<<5);}while (0)
#define GPIOG_REG_RST()   do {RCC->RCC_AHB1RSTR |= (1<<6); RCC->RCC_AHB1RSTR &= ~(1<<6);}while (0)
#define GPIOH_REG_RST()   do {RCC->RCC_AHB1RSTR |= (1<<7); RCC->RCC_AHB1RSTR &= ~(1<<7);}while (0)


////PORTNUM
#define BASEADDR_TO_PORTNUM(x)		((x==GPIOA)?0:  \
									(x==GPIOB)?1:  \
									(x==GPIOC)?2:  \
									(x==GPIOD)?3:  \
									(x==GPIOE)?4:  \
									(x==GPIOF)?5:  \
									(x==GPIOG)?6:  \
									(x==GPIOH)?7:-1  )




#define ENABLE 1
#define DISABLE 0
#define GPIO_PIN_SET ENABLE
#define GPIO_PI_RESET DISABLE
#define SET   ENABLE
#define RESET DISABLE
#define FLAG_RESET DISABLE
#define FLAG_SET ENABLE


#endif /* DRIVERS_INC_STM32F446XX_H_ */
