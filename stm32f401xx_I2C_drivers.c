/*
 * stm32f401xx_I2C_drivers.c
 *
 *  Created on: Jun 14, 2020
 *      Author: hs
 */


#include "stm32f401xx_I2C_drivers.h"


/*********************************************************************
 * @fn      		  - I2C_CLKControl
 *
 * @brief             - This function enables or disables peripheral clock for I2C
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_CLKControl(I2C_RegDef_t* pI2Cx, uint8_t ENA_DIS){



	if(ENA_DIS == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_CLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_CLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_CLK_EN();
		}

	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_CLK_DI();
		}else if (pI2Cx == I2C2)
		{
			I2C2_CLK_DI();
		}else if (pI2Cx == I2C3)
		{
			I2C3_CLK_DI();
		}

	}

}


/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - This function initialize the SOI peripheral
 *
 * @param[in]         - base address of the I2C handler
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_Init(I2C_Handle_t *pI2CHandler){


}

/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - This function de-initialize the I2C
 *                      to its reset values
 *
 * @param[in]         - base address of the I2C peripheral handler
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_DeInit(I2C_RegDef_t *pI2C){

	if 		(pI2C == I2C1) I2C1_REG_RESET();
	else if (pI2C == I2C2) I2C2_REG_RESET();
	else if (pI2C == I2C3) I2C3_REG_RESET();

}
