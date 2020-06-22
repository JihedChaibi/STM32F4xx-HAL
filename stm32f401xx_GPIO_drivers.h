/*
 * stm32f401xx_GPIO_drivers.h
 *
 *  Created on: Jun 5, 2020
 *      Author: hs
 */

#ifndef DRIVERS_INC_STM32F401XX_GPIO_DRIVERS_H_
#define DRIVERS_INC_STM32F401XX_GPIO_DRIVERS_H_

#include "stm32f401xx.h"


typedef struct{

	uint8_t GPIO_PinNumber; /* possible values: ref @GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode;   /* possible values: ref @GPIO_MODES */
	uint8_t GPIO_PinSpeed;  /* possible values: ref @GPIO_OutputSPEED */
	uint8_t GPIO_PinPuPd;   /* possible values: ref @GPIO_PUPD */
	uint8_t GPIO_PinOType;  /* possible values: ref @GPIO_OType */
	uint8_t GPIO_PinAltFun; /* possible values: ref @GPIO_ALT_FUN */

}GPIO_PinConfig_t;



typedef struct{

	GPIO_Reg_t * pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handler_t;



/* @GPIO_PIN_NUMBERS
 * GPIP pin numbers Macros
 */
#define GPIO_PIN_0          0
#define GPIO_PIN_1          1
#define GPIO_PIN_2          2
#define GPIO_PIN_3          3
#define GPIO_PIN_4          4
#define GPIO_PIN_5          5
#define GPIO_PIN_6          6
#define GPIO_PIN_7          7
#define GPIO_PIN_8          8
#define GPIO_PIN_9          9
#define GPIO_PIN_10         10
#define GPIO_PIN_11         11
#define GPIO_PIN_12         12
#define GPIO_PIN_13         13
#define GPIO_PIN_14         14
#define GPIO_PIN_15         15



/* @GPIO_MODES
 * GPIO Modes (MODER register) Macros */
#define GPIO_MODE_INPUT     0
#define GPIO_MODE_OUTPUT    1
#define GPIO_MODE_ALTFUN    2
#define GPIO_MODE_ANALOG    3
#define GPIO_IT_FET         4 // Interrupt Falling Edge Trigger
#define GPIO_IT_RET         5 // Interrupt Rising Edge Trigger
#define GPIO_IT_FRET        6 // Interrupt Falling/Rising Edge Trigger


/*  @GPIO_OutputSPEED
 *  GPIO Output Speed (OSPEEDR register) Macros*/
#define GPIO_SPEED_LOW           0
#define GPIO_SPEED_MED           1
#define GPIO_SPEED_HIGH          2
#define GPIO_SPEED_VHIGH         3


/*  @GPIO_PUPD
 *  GPIO PU/PD Config (PUPDR register) Macros*/
#define GPIO_NO_PUPD             0
#define GPIO_PIN_PullUp          1
#define GPIO_PIN_PullDown        2


/*  @GPIO_OType
 *  GPIO Output Type Config  (OTYPER register) Macros*/
#define GPIO_Output_PushPull     0
#define GPIO_Output_OpenDrain    1


/* @GPIO_ALT_FUN
 * GPIO Alternate Functions (AFRL register) Macros*/




/**********************************************************************************************************
 * 				                           SUPPORTED APIs
 *         			          Check Functions Definitions For More Info
**********************************************************************************************************/



void GPIO_CLKControl(GPIO_Reg_t* pGPIO, uint8_t ENA_DIS);


void GPIO_Init(GPIO_Handler_t* pGPIO_Handler);
void GPIO_DeInit(GPIO_Reg_t *pGPIO);


uint8_t GPIO_ReadInputPin(GPIO_Reg_t* pGPIO, uint8_t pinNumber);
uint16_t GPIO_ReadPort(GPIO_Reg_t* pGPIO);

void GPIO_WriteOutputPin(GPIO_Reg_t* pGPIO,uint8_t pinNumber, uint8_t value);
void GPIO_WriteOutputPort(GPIO_Reg_t* pGPIO, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_Reg_t* pGPIO,uint8_t pinNumber);



void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t ENA_DIS);
void GPIO_IRQPriorityConfig(uint8_t IRQ_Number,uint32_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* DRIVERS_INC_STM32F401XX_GPIO_DRIVERS_H_ */
