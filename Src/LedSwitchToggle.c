/*
 * LedSwitchToggle.c
 *
 *  Created on: Jun 21, 2024
 *      Author: vaibhav
 */

#include <stdint.h>
#include "stm32g0b1re_gpio_driver.h"

void user_delay()
{
	for (uint32_t i = 0; 1 <= 300000/2; i++);
}

int main()
{
	GPIO_Handle_t LedPin;
	GPIO_Handle_t BtPin;

	LedPin.pGPIOx = GPIOA;
	LedPin.GPIO_PinConfig.GPIO_PinNumber 	= GPIO_PIN_5;
	LedPin.GPIO_PinConfig.GPIO_PinMode 		= GPIO_PIN_MODE_OUT;
	LedPin.GPIO_PinConfig.GPIO_PinOutType 	= GPIO_OUT_TYPE_PP;
	LedPin.GPIO_PinConfig.GPIO_PinPuPd 		= GPIO_NO_PUPD;
	GPIO_ClkCtrl(GPIOA, ENABLE);
	GPIO_Reset(GPIOA);
	GPIO_Init(&LedPin);

	BtPin.pGPIOx = GPIOC;
	BtPin.GPIO_PinConfig.GPIO_PinNumber 	= GPIO_PIN_13;
	BtPin.GPIO_PinConfig.GPIO_PinMode 		= GPIO_PIN_MODE_IN;
	BtPin.GPIO_PinConfig.GPIO_PinPuPd 		= GPIO_NO_PUPD;
	GPIO_ClkCtrl(GPIOC, ENABLE);
	GPIO_Reset(GPIOC);
	GPIO_Init(&BtPin);
	while(1)
	{
		if (!GPIO_ReadFromPin(GPIOC, GPIO_PIN_13))
		{
			GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			user_delay();
		}
	}
	return 0;
}

