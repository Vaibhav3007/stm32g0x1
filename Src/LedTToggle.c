/*
 * LedTToggle.c
 *
 *  Created on: Jun 20, 2024
 *      Author: vaibhav
 */

#include <stdint.h>
#include "stm32g0b1re_gpio_driver.h"

void delay();

int main()
{

	GPIO_Handle_t LedGpio;
	LedGpio.pGPIOx = GPIOA;
	LedGpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	LedGpio.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_OUT;
	LedGpio.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_PP;
	LedGpio.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_Reset(GPIOA);
	GPIO_ClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&LedGpio);

	while(1)
	{
		GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		delay();
	}

	return 0;
}

void delay()
{
	for (uint32_t tick = 0; tick <= 300000; tick ++);
}
