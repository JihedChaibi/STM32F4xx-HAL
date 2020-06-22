/*
 * stm32f401xx_I2C_drivers.h
 *
 *  Created on: Jun 14, 2020
 *      Author: hs
 */

#ifndef DRIVERS_INC_STM32F401XX_I2C_DRIVERS_H_
#define DRIVERS_INC_STM32F401XX_I2C_DRIVERS_H_


#include "stm32f401xx.h"


/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

}I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
    uint32_t        RxSize;		/* !< To store Rx size  > */
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;




/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000


/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE        1
#define I2C_ACK_DISABLE       0


/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2        0
#define I2C_FM_DUTY_16_9     1





/**********************************************************************************************************
 * 				                           SUPPORTED APIs
 *         			          Check Functions Definitions For More Info
**********************************************************************************************************/



/*********************************************************************
 * @fn      		  - I2C_Control
 *
 * @brief             - This function enables or disables the I2Cx Peripheral
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void I2C_Control(I2C_RegDef_t* pI2C, uint8_t ENA_DIS){

	if (ENA_DIS == ENABLE)
	{
		pI2C->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2C->CR1 &= ~(1 << I2C_CR1_PE);
	}
}



void I2C_CLKControl(I2C_RegDef_t* pI2C, uint8_t ENA_DIS);


void I2C_Init(I2C_Handle_t *pI2CHandler);
void I2C_DeInit(I2C_RegDef_t *pI2C);


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);



/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);




/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);




#endif /* DRIVERS_INC_STM32F401XX_I2C_DRIVERS_H_ */
