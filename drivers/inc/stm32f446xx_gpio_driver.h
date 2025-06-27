/*
 * stm32f446xx_gpio_Driver.h
 *
 *  Created on: Jun 26, 2025
 *      Author: uma_r
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include"stm32f446xx.h"
#include<stdint.h>

//GPIO PIN CONFIG STRUCT
typedef struct {
	uint8_t GPIO_PinNumber ;    /* <possible values @GPIO_PinNum> */
	uint8_t GPIO_PinMode ;		/* <possible values @GPIO_PinMode> */
	uint8_t GPIO_PinSpeed ;		/* <possible values @GPIO_PinSpeed> */
	uint8_t GPIO_PinOPType ;	/* <possible values @GPIO_PinOPType> */
	uint8_t GPIO_PinPuPdControl ;/* <possible values @GPIO_PinPUPD> */
	uint8_t GPIO_PinAlternateFunctionMode ;/* <possible values @GPIO_PinAF> */


}Gpio_PinConfig_Struct;

//GPIO HANDLE STRUCT
typedef struct {
	Gpio_struct *pGPIOx; // This is a pointer to the base address of the Gpio_struct
	Gpio_PinConfig_Struct GPIO_PinConfig;
}Gpio_Handle_t;

//define MACROS for GPIO initialisation

//uint8_t GPIO_PinNumber ;
//@GPIO_PinNum
#define GPIO_PIN_NUM_0  0
#define GPIO_PIN_NUM_1  1
#define GPIO_PIN_NUM_2  2
#define GPIO_PIN_NUM_3  3
#define GPIO_PIN_NUM_4  4
#define GPIO_PIN_NUM_5  5
#define GPIO_PIN_NUM_6  6
#define GPIO_PIN_NUM_7  7
#define GPIO_PIN_NUM_8  8
#define GPIO_PIN_NUM_9	9
#define GPIO_PIN_NUM_10 10
#define GPIO_PIN_NUM_11 11
#define GPIO_PIN_NUM_12 12
#define GPIO_PIN_NUM_13 13
#define GPIO_PIN_NUM_14 14
#define GPIO_PIN_NUM_15	15

//uint8_t GPIO_PinMode ;
//@GPIO_PinMode
#define PIN_MODE_IN    	0
#define PIN_MODE_OUT    1
#define PIN_MODE_AF   	2
#define PIN_MODE_ANALOG 3
#define PIN_MODE_RT    	4
#define PIN_MODE_FT    	5
#define PIN_MODE_RFT   	6

//uint8_t GPIO_PinSpeed
//@GPIO_PinSpeed
#define PIN_SPEED_LOW		0
#define PIN_SPEED_MEDIUM 	1
#define PIN_SPEED_FAST		2
#define PIN_SPEED_HIGH		3

//uint8_t GPIO_PinOPType ;
//@GPIO_PinOPType
#define OTYPE_OPEND			0
#define OTYPE_PP			1

//uint8_t GPIO_PinPuPdControl ;
//@GPIO_PinPUPD

#define PIN_NO_PU_PD 		0
#define PIN_PU				1
#define PIN_PD 				2


//uint8_t GPIO_PinAlternateFunctionMode ;
//@GPIO_PinAF


/*APIs PROVIDED BY GPIO DRIVER
 * init
 * deinit
 *
 * pclkenable
 *
 * irqconfig
 * irqhandler
 *
 * readfrom pin
 * readfromport
 *
 * writetopin
 * writetoport
 * togglepin
 */

//Peripheral clock enable

void GPIO_PeriClockControl(Gpio_struct *pGPIOx, uint8_t EnOrDis);

//Initialisation and Deinitialisation

void GPIO_Init(Gpio_Handle_t *pGpioHandle);
void GPIO_Deinit(Gpio_struct *pGPIOx);

//Data Read and Write

uint8_t GPIO_ReadFromInputPin(Gpio_struct *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(Gpio_struct *pGPIOx);

void GPIO_WriteToOutputPin(Gpio_struct *pGPIOx,uint8_t PinNumber, uint8_t val);
void GPIO_WriteToOutputPort(Gpio_struct *pGPIOx,uint16_t val);

void GPIO_ToggleOutputPin(Gpio_struct *pGPIOx,uint8_t PinNumber);

//IRQ HANDLER AND CONFIG

void GPIO_IRQConifg(uint8_t IRQNum, uint8_t IRQPriority,uint8_t EnOrDis );
void GPIO_IRQHandler(uint8_t PinNumber);








#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
