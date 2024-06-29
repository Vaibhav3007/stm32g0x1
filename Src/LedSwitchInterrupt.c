/*
 * LedSwitchInterrupt.c
 *
 *  Created on: Jun 23, 2024
 *      Author: vaibhav
 *
 */

#include <stdio.h>
#include <string.h>
#include "stm32g0b1re_gpio_driver.h"

int main()
{
	GPIO_Handle_t LedPin;
	memset(&LedPin,0,sizeof(LedPin));
	LedPin.pGPIOx = GPIOA;
	LedPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	LedPin.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_OUT;
	LedPin.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_PP;
	LedPin.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;
	GPIO_ClkCtrl(GPIOA, ENABLE);
	GPIO_Reset(GPIOA);
	GPIO_Init(&LedPin);

	GPIO_Handle_t UsrPin;
	memset(&UsrPin,0,sizeof(UsrPin));
	UsrPin.pGPIOx = GPIOC;
	UsrPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	UsrPin.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_INT_FL;
	UsrPin.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;
	GPIO_ClkCtrl(GPIOC, ENABLE);
	GPIO_Reset(GPIOC);
	GPIO_Init(&UsrPin);

	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI4_15, PRIORITY_1);
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI4_15, ENABLE);


	while(1);
}

void EXTI4_15_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_13);
	GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}


