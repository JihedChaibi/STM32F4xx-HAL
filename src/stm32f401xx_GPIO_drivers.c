/*
 * stm32f401xx_GPIO_drivers.c
 *
 *  Created on: Jun 5, 2020
 *      Author: hs
 */


#include "stm32f401xx_GPIO_drivers.h"




/**********************************************************************************************************
 * 				                           SUPPORTED APIs
 *         			                   Functions Definitions
**********************************************************************************************************/




/*********************************************************************
 * @fn      		  - GPIO_CLKControl
 *
 * @brief             - This function enables or disables peripheral clock for a GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_CLKControl(GPIO_Reg_t* pGPIO, uint8_t ENA_DIS)
{
	if (ENA_DIS == ENABLE)
	{

		if 		(pGPIO == GPIOA) GPIOA_CLK_EN();
		else if (pGPIO == GPIOB) GPIOB_CLK_EN();
		else if (pGPIO == GPIOC) GPIOC_CLK_EN();
		else if (pGPIO == GPIOD) GPIOD_CLK_EN();
		else if (pGPIO == GPIOE) GPIOE_CLK_EN();
		else if (pGPIO == GPIOH) GPIOH_CLK_EN();

	}

	else
	{

		if 		(pGPIO == GPIOA) GPIOA_CLK_DI();
		else if (pGPIO == GPIOB) GPIOB_CLK_DI();
		else if (pGPIO == GPIOC) GPIOC_CLK_DI();
		else if (pGPIO == GPIOD) GPIOD_CLK_DI();
		else if (pGPIO == GPIOE) GPIOE_CLK_DI();
		else if (pGPIO == GPIOH) GPIOH_CLK_DI();

	}
}




/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initialize the GPIO peripheral
 *
 * @param[in]         - base address of the GPIO peripheral handler
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_Init(GPIO_Handler_t* pGPIO_Handler)
{

	uint32_t temp=0;


    // 1. Pin Mode
	if (pGPIO_Handler->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		  //NON Interrupt Mode

		  //Each pin MODER has 2 bits
          temp = (pGPIO_Handler->GPIO_PinConfig.GPIO_PinMode << (2*pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber));
          pGPIO_Handler->pGPIOx->MODER &= ~(0x3 << (2*pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber)); // CLEARING
          pGPIO_Handler->pGPIOx->MODER |= temp; // SETTING

	}else
	{ //Interrupt Mode

		 // 1.  Config Trigger mode

         if(pGPIO_Handler->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_FET)
         {
              // ENABLE FALLING TRIGGER (AND CLEAR THE OTHER)
              EXTI->FTSR |= (1<< pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber);
              EXTI->RTSR &= ~(1<< pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber);


         }else if(pGPIO_Handler->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RET)
         {
             // ENABLE FALLING TRIGGER (AND CLEAR THE OTHER)
             EXTI->RTSR |= (1<< pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber);
             EXTI->FTSR &= ~(1<< pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber);

         }else if(pGPIO_Handler->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_FRET)
         {

             // ENABLE BOTH (FALLING AND RISING) TRIGGER
             EXTI->RTSR |= (1<< pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber);
             EXTI->FTSR &= ~(1<< pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber);

         }

          // 2.  Config GPIO port selection in SYSCFG_EXTICR
             uint32_t temp1 = pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber / 4;
             uint32_t temp2 = pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber % 4;
             uint8_t portcode = GPIO_To_Code(pGPIO_Handler->pGPIOx);
             SYSCFG_CLK_EN();
             SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);




          // 3.  Enable the EXTI interrupt using IMR (Interrupt mask register)
             EXTI->IMR|= (1<< pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber);



	}



      // 2. Pin Speed, Each pin OSPEEDR has 2 bits

	  temp=0;
      temp = (pGPIO_Handler->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber));

      pGPIO_Handler->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber)); // CLEARING
      pGPIO_Handler->pGPIOx->OSPEEDR |= temp;  // SETTING


      // 3. PU/PD Settings, Each pin PUPDR has 2 bits
	  temp=0;
      temp = (pGPIO_Handler->GPIO_PinConfig.GPIO_PinPuPd << (2*pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber));

      pGPIO_Handler->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber)); // CLEARING
      pGPIO_Handler->pGPIOx->PUPDR |= temp; // SETTING


      // 4. OType Config, Each pin PUPDR has 1 bit
	  temp=0;
      temp = (pGPIO_Handler->GPIO_PinConfig.GPIO_PinOType << (1*pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber));

      pGPIO_Handler->pGPIOx->OTYPER &= ~(0x1 << pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber); // CLEARING
      pGPIO_Handler->pGPIOx->OTYPER |= temp; // SETTING
      temp=0;



      // 5. Alternate Function Config
      if(pGPIO_Handler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
      {
    	  // Config alternate function
    	  uint8_t temp1,temp2;

    	  temp1 = pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber / 8;
    	  temp2 = pGPIO_Handler->GPIO_PinConfig.GPIO_PinNumber % 8;

    	  pGPIO_Handler->pGPIOx->AFR[temp1] &= ~(0x0F << (4 * temp2));  // SETTING
    	  pGPIO_Handler->pGPIOx->AFR[temp1] |= (pGPIO_Handler->GPIO_PinConfig.GPIO_PinAltFun << (4 * temp2));  // SETTING


      }


}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initialize the GPIO peripheral
 *                      to its reset values
 *
 * @param[in]         - base address of the GPIO peripheral handler
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_Reg_t *pGPIO)
{

	if 		(pGPIO == GPIOA) GPIOA_REG_RESET();
	else if (pGPIO == GPIOB) GPIOB_REG_RESET();
	else if (pGPIO == GPIOC) GPIOC_REG_RESET();
	else if (pGPIO == GPIOD) GPIOD_REG_RESET();
	else if (pGPIO == GPIOE) GPIOE_REG_RESET();
	else if (pGPIO == GPIOH) GPIOH_REG_RESET();


}



/*********************************************************************
 * @fn      		  - GPIO_ReadInputPin
 *
 * @brief             - This function reads a GPIO pin value
 *
 * @param[in]         -  Base address of the GPIO peripheral handler
 * @param[in]         -  Pin number
 *
 * @return            -  Pin value ( 0 or 1 )
 *
 * @Note              -  none
 */
uint8_t GPIO_ReadInputPin(GPIO_Reg_t* pGPIO, uint8_t pinNumber)
{
	uint32_t value;
	value = (uint32_t)(pGPIO->IDR >> pinNumber) & (0x01);
	return (value);
}


/*********************************************************************
 * @fn      		  - GPIO_ReadPortPin
 *
 * @brief             - This function reads a GPIO port value
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         -
 *
 * @return            -  Port value
 *
 * @Note              -  none
 */
uint16_t GPIO_ReadPort(GPIO_Reg_t* pGPIO)
{

	uint16_t value;
	value = (uint16_t)(pGPIO->IDR);
	return (value);

}


/*********************************************************************
 * @fn      		  - GPIO_WriteOutputPin
 *
 * @brief             - This function writes to GPIO pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number
 * @param[in]         - value (SET/RESET)
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteOutputPin(GPIO_Reg_t* pGPIO, uint8_t pinNumber, uint8_t value)
{
	if (value == SET)           pGPIO->ODR |= (1 << pinNumber);
	else if (value == RESET)    pGPIO->ODR &= ~(1 << pinNumber);

}


/*********************************************************************
 * @fn      		  - GPIO_WriteOutputPort
 *
 * @brief             - This function writes to a GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - value
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteOutputPort(GPIO_Reg_t* pGPIO, uint16_t value)
{
	pGPIO->ODR = value;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteOutputPin
 *
 * @brief             - This function toggles a GPIO pin value
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_ToggleOutputPin(GPIO_Reg_t* pGPIO,uint8_t pinNumber)
{
	 pGPIO->ODR ^= (1 << pinNumber);
}



/*********************************************************************
 * @fn      		  - GPIO_WriteOutputPin
 *
 * @brief             - This function enables/disables GPIO interrupts
 *
 * @param[in]         - IRQ number
 * @param[in]         - enable/disable interrupt (ENABLE/DISABLE)
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t ENA_DIS)
{

	if(ENA_DIS == ENABLE)
		{
			if(IRQ_Number <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQ_Number );

			}else if(IRQ_Number > 31 && IRQ_Number < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQ_Number % 32) );
			}
			else if(IRQ_Number >= 64 && IRQ_Number < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER2 |= ( 1 << (IRQ_Number % 64) );
			}
		}else
		{
			if(IRQ_Number <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQ_Number );
			}else if(IRQ_Number > 31 && IRQ_Number < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQ_Number % 32) );
			}
			else if(IRQ_Number >= 64 && IRQ_Number < 96 )
			{
				//program ICER2 register
				*NVIC_ICER2 |= ( 1 << (IRQ_Number % 64) );
			}
		}

}


/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         - IRQ Number
 * @param[in]         - IRQ Priority
 *
 * @return            - None
 *
 *
 * @Note              -
 */
void GPIO_IRQPriorityConfig(uint8_t IRQ_Number,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQ_Number / 4;
	uint8_t iprx_section  = IRQ_Number %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}




/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function handles a GPIO interrupt
 *
 * @param[in]         - GPIO pin number
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
	   EXTI->PR |= ( 1 << PinNumber);
	}

}



