/*
 * stm32g0b1re_gpio_driver.h
 *
 *  Created on: Jun 8, 2024
 *      Author: vaibhav
 */

#ifndef INC_STM32G0B1RE_GPIO_DRIVER_H_
#define INC_STM32G0B1RE_GPIO_DRIVER_H_

#include "stm32g0b1re.h"

/*
 * GPIO constants
 */
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15

#define GPIO_PIN_MODE_IN		0
#define GPIO_PIN_MODE_OUT		1
#define GPIO_PIN_MODE_ALT		2
#define GPIO_PIN_MODE_ANALOG	3
#define GPIO_PIN_MODE_INT_RI	4
#define GPIO_PIN_MODE_INT_FL	5
#define GPIO_PIN_MODE_INT_RF	6

#define GPIO_OUT_TYPE_PP		0
#define GPIO_OUT_TYPE_OD		1

#define GPIO_OUT_SPEED_VLOW		0
#define GPIO_OUT_SPEED_LOW		1
#define GPIO_OUT_SPEED_HI		2
#define GPIO_OUT_SPEED_VHI		3

#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2

#define GPIO_AF0				0
#define GPIO_AF1				1
#define GPIO_AF2				2
#define GPIO_AF3				3
#define GPIO_AF4				4
#define GPIO_AF5				5
#define GPIO_AF6				6
#define GPIO_AF7				7

/*
 * Pin configuration structure
 */
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinOutType;
	uint8_t GPIO_PinOutSpeed;
	uint8_t GPIO_PinPuPd;
	uint8_t GPIO_PinAltFn;
}GPIO_PinConfig_t;

/*
 * GPIO handle structure
 */
typedef struct{
	GPIO_RegDef_t 		*pGPIOx; //Holds GPIO base address to which pin belong
	GPIO_PinConfig_t	GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * GPIO control APIs type definitions
 */
void GPIO_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t E_D);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Reset(GPIO_RegDef_t *pGPIOx);
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t E_D);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32G0B1RE_GPIO_DRIVER_H_ */
