/*
 * stm32f401xx.h
 *
 *  Created on: Jun 5, 2020
 *      Author: hs
 */

#ifndef DRIVERS_INC_STM32F401XX_H_
#define DRIVERS_INC_STM32F401XX_H_



#include <stdint.h>
#include<stddef.h>

#define __weak __attribute__((weak))

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (volatile uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2  		((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((volatile uint32_t*)0xE000E400)



/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4



/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15

/*
 * SOME MACROS
 */

#define ENABLE            1
#define DISABLE           0
#define SET               ENABLE
#define RESET             DISABLE
#define FLAG_RESET        RESET
#define FLAG_SET 	      SET

/*
 * Base addresses of FLASH, RAM and System memory (ROM)
*/

#define FLASH_BASE_ADDR    (0x08000000U)
#define SRAM1_BASE_ADDR    (0x20000000U)
#define SYS_MEM_ADDR       (0x1FFF0000U)
#define RAM                (SRAM1_BASE_ADDR)

/*
 * Base addresses of AHBx and APBx peripheral
*/


#define AHB1_BASE_ADDR           (0x40020000U)
#define AHB2_BASE_ADDR           (0x50000000U)
#define APB1_BASE_ADDR           (0x40000000U)
#define APB2_BASE_ADDR           (0x40010000U)


/*
 * Base addresses of peripherals hanging on AHB1
*/

#define GPIOA_BASE_ADDR          (AHB1_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR          (AHB1_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR          (AHB1_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR          (AHB1_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR          (AHB1_BASE_ADDR + 0x1000)
#define GPIOH_BASE_ADDR          (AHB1_BASE_ADDR + 0x1C00)

#define CRC_BASE_ADDR            (AHB1_BASE_ADDR + 0x3000)
#define RCC_BASE_ADDR            (AHB1_BASE_ADDR + 0x3800)

#define FLASH_INTERF_BASE_ADDR   (AHB1_BASE_ADDR + 0x6400)

#define DMA1_BASE_ADDR           (AHB1_BASE_ADDR + 0x6400)
#define DMA2_BASE_ADDR           (AHB1_BASE_ADDR + 0x6000)



/*
 * Base addresses of peripherals hanging on AHB2
*/

#define USB_OTG_BASE_ADDR        (AHB2_BASE_ADDR + 0x0000)


/*
 * Base addresses of peripherals hanging on APB1
*/

#define TIM2_BASE_ADDR           (APB1_BASE_ADDR + 0x0000)
#define TIM3_BASE_ADDR           (APB1_BASE_ADDR + 0x0400)
#define TIM4_BASE_ADDR           (APB1_BASE_ADDR + 0x0800)
#define TIM5_BASE_ADDR           (APB1_BASE_ADDR + 0x0C00)

#define RTC_BKP_BASE_ADDR        (APB1_BASE_ADDR + 0x2800)
#define WWDG_BASE_ADDR           (APB1_BASE_ADDR + 0x2C00)
#define IWDG_BASE_ADDR           (APB1_BASE_ADDR + 0x3000)

#define I2S2EXT_BASE_ADDR        (APB1_BASE_ADDR + 0x3400)

#define SPI2_I2S2_BASE_ADDR      (APB1_BASE_ADDR + 0x3800)
#define SPI3_I2S3_BASE_ADDR      (APB1_BASE_ADDR + 0x3C00)

#define I2S3EXT_BASE_ADDR        (APB1_BASE_ADDR + 0x4000)

#define USART2_BASE_ADDR         (APB1_BASE_ADDR + 0x4400)

#define I2C1_BASE_ADDR           (APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR           (APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR           (APB1_BASE_ADDR + 0x5C00)

#define PWR_BASE_ADDR            (APB1_BASE_ADDR + 0x7000)



/*
 * Base addresses of peripherals hanging on APB2
*/

#define TIM1_BASE_ADDR           (APB2_BASE_ADDR + 0x0000)
#define USART1_BASE_ADDR         (APB2_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR         (APB2_BASE_ADDR + 0x1400)
#define ADC1_BASE_ADDR           (APB2_BASE_ADDR + 0x2000)
#define SDIO_BASE_ADDR           (APB2_BASE_ADDR + 0x2C00)
#define SPI1_BASE_ADDR           (APB2_BASE_ADDR + 0x3000)
#define SPI4_BASE_ADDR           (APB2_BASE_ADDR + 0x3400)

#define I2C1_BASE_ADDR           (APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR           (APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR           (APB1_BASE_ADDR + 0x5C00)


#define SYSCFG_BASE_ADDR         (APB2_BASE_ADDR + 0x3800)
#define EXTI_BASE_ADDR           (APB2_BASE_ADDR + 0x3C00)
#define TIM9_BASE_ADDR           (APB2_BASE_ADDR + 0x4000)
#define TIM10_BASE_ADDR          (APB2_BASE_ADDR + 0x4400)
#define TIM11_BASE_ADDR          (APB2_BASE_ADDR + 0x4800)


/*
 * GPIO structure definition
 */

typedef struct{

	volatile uint32_t MODER;          /* MODE REGISTER */
	volatile uint32_t OTYPER;         /* OUTPUT TYPE REGISTER */
	volatile uint32_t OSPEEDR;        /* OUTPUT SPEED REGISTER */
	volatile uint32_t PUPDR;          /* PULLUP / PULLDOWN REGISTER */
	volatile uint32_t IDR;            /* INPUT DATA REGISTER */
	volatile uint32_t ODR;            /* OUTPUT DATA REGISTER */
	volatile uint32_t BSRR;           /* BIT SET/RESET REGISTER */
	volatile uint32_t LCKR;           /* LOCK GPIO REGISTER */
	volatile uint32_t AFR[2];         /* ALTERNATE FUNCTION REGISTER AFR[0] : AFRL , AFR[1] : AFRH */

}GPIO_Reg_t;



/*
 * RCC structure definition
 */

typedef struct{

    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
             uint32_t RESERVERD1[2];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
             uint32_t RESERVERD2[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
             uint32_t RESERVERD3[2];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
             uint32_t RESERVERD4[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
             uint32_t RESERVERD5[2];
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
             uint32_t RESERVERD6[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
             uint32_t RESERVERD7[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;


}RCC_Reg_t;




/*
 * EXTI structure definition
 */

typedef struct{

	volatile uint32_t IMR;    //Interrupt mask register
	volatile uint32_t EMR;    //Event mask register
	volatile uint32_t RTSR;   //Rising trigger selection register
	volatile uint32_t FTSR;   //Falling trigger selection register
	volatile uint32_t SWIER;  //Software interrupt event register
	volatile uint32_t PR;     //Pending register

}EXTI_Reg_t;



/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	volatile uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	volatile uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	volatile uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	volatile uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	volatile uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	volatile uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	volatile uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	volatile uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	volatile uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;




/*
 * peripheral register definition structure for I2C
 */

typedef struct{

	volatile uint32_t CR1;    // Control register 1
	volatile uint32_t CR2;    // Control register 2
	volatile uint32_t OAR1;   // Own address register 1
	volatile uint32_t OAR2;   // Own address register 2
	volatile uint32_t DR;     // Data Register
	volatile uint32_t SR1;    // Status register 1
	volatile uint32_t SR2;    // Status register 2
	volatile uint32_t CCR;    // Clock control register
	volatile uint32_t TRISE;  // TODO
	volatile uint32_t FLTR;   // TODO

}I2C_RegDef_t;


/*
 * SYSCFG structure definition
 */

typedef struct{

	volatile uint32_t MEMRMP;        //Memory remap register
	volatile uint32_t PMC;           //Peripheral mode configuration register
	volatile uint32_t EXTICR[4];     //External interrupt configuration registers
	volatile uint32_t RESERVED[2];   //Unused region
	volatile uint32_t CMPCR;         //Compensation cell control register

}SYSCFG_Reg_t;



/*
 * Peripheral definitions
 */



#define GPIOA  ((GPIO_Reg_t *)GPIOA_BASE_ADDR)
#define GPIOB  ((GPIO_Reg_t *)GPIOB_BASE_ADDR)
#define GPIOC  ((GPIO_Reg_t *)GPIOC_BASE_ADDR)
#define GPIOD  ((GPIO_Reg_t *)GPIOD_BASE_ADDR)
#define GPIOE  ((GPIO_Reg_t *)GPIOE_BASE_ADDR)
#define GPIOH  ((GPIO_Reg_t *)GPIOH_BASE_ADDR)
#define RCC    ((RCC_Reg_t*)RCC_BASE_ADDR)
#define EXTI   ((EXTI_Reg_t*)EXTI_BASE_ADDR)
#define SYSCFG ((SYSCFG_Reg_t*)SYSCFG_BASE_ADDR)

#define SPI1 ((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_I2S2_BASE_ADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_I2S3_BASE_ADDR)
#define SPI4 ((SPI_RegDef_t*)SPI4_BASE_ADDR)


#define I2C1 ((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2 ((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define I2C3 ((I2C_RegDef_t*)I2C3_BASE_ADDR)



/*
 * Converts GPIO to Port Code
 */


#define GPIO_To_Code(x)   (  (x == GPIOA)?0: \
		                     (x == GPIOB)?1: \
		                     (x == GPIOC)?2: \
		                     (x == GPIOD)?3: \
		                     (x == GPIOE)?4: \
		                     (x == GPIOH)?5:0 )





/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8




/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


/*
 * IRQ Numbers of STM32F401xx
 */

#define IRQ_NO_EXTI0     6
#define IRQ_NO_EXTI1     7
#define IRQ_NO_EXTI2     8
#define IRQ_NO_EXTI3     9
#define IRQ_NO_EXTI4     10
#define IRQ_NO_EXTI5_9   23
#define IRQ_NO_EXTI10_15 40

#define IRQ_NO_SPI1	     35
#define IRQ_NO_SPI2      36
#define IRQ_NO_SPI3      51
#define IRQ_NO_SPI4      84


#define IRQ_NO_I2C1_EV   31
#define IRQ_NO_I2C1_ER   32
#define IRQ_NO_I2C2_EV   33
#define IRQ_NO_I2C2_ER   34
#define IRQ_NO_I2C3_EV   72
#define IRQ_NO_I2C3_ER   73

#define IRQ_NO_USART1	 37
#define IRQ_NO_USART2	 38

/*
 *   RCC Enable/Disable Macros of GPIOx peripherals
 */

#define GPIOA_CLK_EN() ( RCC -> AHB1ENR |= (1 << 0) )
#define GPIOB_CLK_EN() ( RCC -> AHB1ENR |= (1 << 1) )
#define GPIOC_CLK_EN() ( RCC->  AHB1ENR |= (1 << 2) )
#define GPIOD_CLK_EN() ( RCC -> AHB1ENR |= (1 << 3) )
#define GPIOE_CLK_EN() ( RCC -> AHB1ENR |= (1 << 4) )
#define GPIOH_CLK_EN() ( RCC -> AHB1ENR |= (1 << 7) )


#define GPIOA_CLK_DI() ( RCC -> AHB1ENR &= ~(1 << 0) )
#define GPIOB_CLK_DI() ( RCC -> AHB1ENR &= ~(1 << 1) )
#define GPIOC_CLK_DI() ( RCC -> AHB1ENR &= ~(1 << 2) )
#define GPIOD_CLK_DI() ( RCC -> AHB1ENR &= ~(1 << 3) )
#define GPIOE_CLK_DI() ( RCC -> AHB1ENR &= ~(1 << 4) )
#define GPIOH_CLK_DI() ( RCC -> AHB1ENR &= ~(1 << 7) )

/*
 *   RCC Enable/Disable Macros of I2Cx peripherals
 */

#define I2C1_CLK_EN() ( RCC -> APB1ENR |= (1 << 21))
#define I2C2_CLK_EN() ( RCC -> APB1ENR |= (1 << 22))
#define I2C3_CLK_EN() ( RCC -> APB1ENR |= (1 << 23))

#define I2C1_CLK_DI() ( RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI() ( RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DI() ( RCC -> APB1ENR &= ~(1 << 23))


/*
 *   RCC Enable/Disable Macros of SPIx peripherals
 */

#define SPI1_CLK_EN() ( RCC -> APB2ENR |= (1 << 12))
#define SPI2_CLK_EN() ( RCC -> APB1ENR |= (1 << 14))
#define SPI3_CLK_EN() ( RCC -> APB1ENR |= (1 << 15))
#define SPI4_CLK_EN() ( RCC -> APB2ENR |= (1 << 13))


#define SPI1_CLK_DI() ( RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DI() ( RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI() ( RCC -> APB1ENR &= ~(1 << 15))
#define SPI4_CLK_DI() ( RCC -> APB2ENR &= ~(1 << 13))

/*
 *   RCC Enable/Disable Macros of USARTx peripherals
 */

#define USART1_CLK_EN() ( RCC -> APB2ENR |= (1 << 4))
#define USART2_CLK_EN() ( RCC -> APB2ENR |= (1 << 17))
#define USART6_CLK_EN() ( RCC -> APB2ENR |= (1 << 5))

#define USART1_CLK_DI() ( RCC -> APB2ENR &= ~(1 << 4))
#define USART2_CLK_DI() ( RCC -> APB2ENR &= ~(1 << 17))
#define USART6_CLK_DI() ( RCC -> APB2ENR &= ~(1 << 5))

/*
 *   RCC Enable/Disable Macros of SYSCFG peripherals
 */

#define SYSCFG_CLK_EN() ( RCC -> APB2ENR |= (1 << 14))

#define SYSCFG_CLK_DI() ( RCC -> APB2ENR &= ~(1 << 14))

/*
 *   Reset GPIO peripherals Macros
 */

#define GPIOA_REG_RESET() do{ (RCC -> AHB1RSTR |= (1 << 0));  (RCC -> AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() do{ (RCC -> AHB1RSTR |= (1 << 1));  (RCC -> AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() do{ (RCC -> AHB1RSTR |= (1 << 2));  (RCC -> AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() do{ (RCC -> AHB1RSTR |= (1 << 3));  (RCC -> AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET() do{ (RCC -> AHB1RSTR |= (1 << 3));  (RCC -> AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOH_REG_RESET() do{ (RCC -> AHB1RSTR |= (1 << 3));  (RCC -> AHB1RSTR &= ~(1 << 7)); } while(0)


/*
 *   Reset SPI peripherals Macros
 */

#define SPI1_REG_RESET() do{ (RCC -> APB2RSTR |= (1 << 12));  (RCC -> APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET() do{ (RCC -> APB1RSTR |= (1 << 14));  (RCC -> APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET() do{ (RCC -> APB1RSTR |= (1 << 15));  (RCC -> APB1RSTR &= ~(1 << 15)); } while(0)
#define SPI4_REG_RESET() do{ (RCC -> APB2RSTR |= (1 << 13));  (RCC -> APB2RSTR &= ~(1 << 13)); } while(0)



/*
 *   Reset I2C peripherals Macros
 */

#define I2C1_REG_RESET() do{ (RCC -> APB1RSTR |= (1 << 21));  (RCC -> APB1RSTR &= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET() do{ (RCC -> APB1RSTR |= (1 << 22));  (RCC -> APB1RSTR &= ~(1 << 22)); } while(0)
#define I2C3_REG_RESET() do{ (RCC -> APB1RSTR |= (1 << 23));  (RCC -> APB1RSTR &= ~(1 << 23)); } while(0)




#endif /* DRIVERS_INC_STM32F401XX_H_ */
