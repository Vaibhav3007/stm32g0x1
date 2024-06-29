/*
 * stm32g0b1re_gpio_driver.c
 *
 *  Created on: June 10, 2024
 *      Author: vaibhav
 */

#include "stm32g0b1re.h"
#include "stm32g0b1re_gpio_driver.h"

/*********************************************************************
 * @function 		- GPIO_PClkCtrl
 *
 * @brief 			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t E_D)
{
	if (E_D == ENABLE)
	{
		if(pGPIOx == GPIOA)			{	GPIOA_CLK_EN();	}
		else if(pGPIOx == GPIOB)	{	GPIOB_CLK_EN();	}
		else if(pGPIOx == GPIOC)	{	GPIOC_CLK_EN();	}
		else if(pGPIOx == GPIOD)	{	GPIOD_CLK_EN();	}
		else if(pGPIOx == GPIOE)	{	GPIOE_CLK_EN();	}
		else if(pGPIOx == GPIOF)	{	GPIOF_CLK_EN();	}
	}
	else
	{
		if(pGPIOx == GPIOA)			{	GPIOA_CLK_DI();	}
		else if(pGPIOx == GPIOB)	{	GPIOB_CLK_DI();	}
		else if(pGPIOx == GPIOC)	{	GPIOC_CLK_DI();	}
		else if(pGPIOx == GPIOD)	{	GPIOD_CLK_DI();	}
		else if(pGPIOx == GPIOE)	{	GPIOE_CLK_DI();	}
		else if(pGPIOx == GPIOF)	{	GPIOF_CLK_DI();	}
	}
}

/*********************************************************************
 * @function 		- GPIO_Init
 *
 * @brief 			- This function in initializes a GPIO with its mode, output type, speed,
 * 					- pull up-down configuration,alternate function
 *
 * @param[in] 		- base address of the GPIO peripheral
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_PIN_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->GPIOx_MODER &= ~(3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->GPIOx_MODER |= temp;
		temp = 0;
	}
	else
	{
		//For G0 MCU, 00 is input mode
		pGPIOHandle->pGPIOx->GPIOx_MODER &= ~(3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PIN_MODE_INT_FL)
		{
			//Configure FTSR
			EXTI->EXTI_RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PIN_MODE_INT_RI)
		{
			//Configure RTSR
			EXTI->EXTI_FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PIN_MODE_INT_RF)
		{
			//Configure RTSR and FTSR
			EXTI->EXTI_RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//configure EXTI_EXTICRx
		uint8_t temp1, temp2, port_value;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		port_value = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		EXTI->EXTI_EXTICR[temp1] |= port_value << (8*temp2);
		//enable EXTI interrupt delivery using IMR
		EXTI->EXTI_IMR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType));
	pGPIOHandle->pGPIOx->GPIOx_OTYPER &= ~(0x1 << pGPIOHandle->pGPIOx->GPIOx_OTYPER);
	pGPIOHandle->pGPIOx->GPIOx_OTYPER |= (temp);
	temp = 0;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->GPIOx_OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIOx_OSPEEDR |= temp;
	temp = 0;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd));
	pGPIOHandle->pGPIOx->GPIOx_PUPDR &= ~(3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIOx_PUPDR |= temp;
	temp = 0 ;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PIN_MODE_ALT)
	{
		uint8_t temp1, temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->GPIOx_AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->GPIOx_AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFn << (4 * temp2));
	}
}

/*********************************************************************
 * @function 		- GPIO_Reset
 *
 * @brief 			- This function resets a GPIOs configuration
 *
 * @param[in] 		- base address of the GPIO peripheral
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_Reset(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA) {	GPIOA_REG_RESET();	}
	else if (pGPIOx == GPIOB) {	GPIOB_REG_RESET();	}
	else if (pGPIOx == GPIOC) {	GPIOC_REG_RESET();	}
	else if (pGPIOx == GPIOD) {	GPIOD_REG_RESET();	}
	else if (pGPIOx == GPIOE) {	GPIOE_REG_RESET();	}
	else if (pGPIOx == GPIOF) {	GPIOF_REG_RESET();	}
}

/*********************************************************************
 * @function 		- GPIO_ReadFromPin
 *
 * @brief 			- This function reads a specific pin of  GPIO
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- pin number to be read
 *
 * @return 			- value of read pin
 *
 * @Note			- none
 */
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Pin_Read_Value;
	Pin_Read_Value = (uint8_t)((pGPIOx->GPIOx_IDR >> PinNumber) & 0x00000001);
	return Pin_Read_Value;
}

/*********************************************************************
 * @function 		- GPIO_ReadFromPort
 *
 * @brief 			- This function reads from a port
 *
 * @param[in] 		- base address of the GPIO peripheral
 *
 * @return 			- read value of specified port
 *
 * @Note			- none
 */
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Port_Read_Value;
	Port_Read_Value = (uint16_t)(pGPIOx->GPIOx_IDR);
	return Port_Read_Value;
}

/*********************************************************************
 * @function 		- GPIO_WriteToPin
 *
 * @brief 			- This function writes user value to specified pin
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- pin number of specified port
 * @param[in]		- value to be written
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == SET)	{	pGPIOx->GPIOx_ODR |= (1 << PinNumber);	}
	else				{	pGPIOx->GPIOx_ODR &= ~(1 << PinNumber);	}
}

/*********************************************************************
 * @function 		- GPIO_WriteToPort
 *
 * @brief 			- This function writes user value to specified port
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- value to be written
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->GPIOx_ODR |= (value);
}

/*********************************************************************
 * @function 		- GPIO_TogglePin
 *
 * @brief 			- This function toggles value of specified pin
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- pin number
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->GPIOx_ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @function 		- GPIO_IRQConfig
 *
 * @brief 			- This function sets/resets NVIC in the processor
 *
 * @param[in] 		- IRQ number
 * @param[in]		- Enable/Disable
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t E_D)
{
	if(E_D == ENABLE)
	{
		*NVIC_ISER |= (1 << IRQNumber);
	}
	else
	{
		*NVIC_ICER |= (1 << IRQNumber);
	}
}

/*********************************************************************
 * @function 		- GPIO_IRQPriorityConfig
 *
 * @brief 			- This function sets priority of interrupt
 *
 * @param[in] 		- IRQ number
 * @param[in]		- IRQ priority (0,1,2,3)
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_sec = IRQNumber % 4;
	uint8_t shift_value = (IPRx_sec * 8) + 6;//Six comes from IPR implementation of cortex-m0+ processor
	*(NVIC_IPR + IPRx) |= (IRQPriority << shift_value);
}

/*********************************************************************
 * @function 		- GPIO_IRQHandling
 *
 * @brief 			- This function handles IRQ by acknowledging interrupt and resetting pending register
 *
 * @param[in] 		- Pin number of EXTI
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->EXTI_RPR1 & (1 << PinNumber))
	{
		EXTI->EXTI_RPR1 |= (1 << PinNumber);
	}
	else if (EXTI->EXTI_FPR1 & (1 << PinNumber))
	{
		EXTI->EXTI_FPR1 |= (1 << PinNumber);
	}
}




