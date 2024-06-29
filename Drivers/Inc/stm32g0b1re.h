/*
 * stm32g0b1re_driver.h
 *
 *  Created on: Jun 8, 2024
 *      Author: vaibhav
 */

#ifndef INC_STM32G0B1RE_H_
#define INC_STM32G0B1RE_H_

#include <stdint.h>

#define __vo						volatile
#define SET							1
#define RESET						0
#define ENABLE						SET
#define DISABLE						RESET

/*
 * ARM Cortex-M0+ specific registers
 */
#define NVIC_ISER			((__vo uint32_t *)0xE000E100)
#define NVIC_ICER			((__vo uint32_t *)0xE000E180)
#define NVIC_IPR			((__vo uint32_t *)0xE000E400)

/*
 * Base addresses of system buses
 */
#define FLASH_BASE_ADDR				0x08000000U
#define SYSTEM_MEMORY_BASE_ADDR		0x1FFF0000U
#define RAM_BASE_ADDR				0x20000000U
#define PERIPHERAL_BASE				0x40000000U
#define CORTEX_M_INT_PERIPHERAL		0xE0000000U

/*
 * Peripheral bus' base address
 */
#define APB_BUS_BASE_ADDR			PERIPHERAL_BASE
#define AHB_BUS_BASE_ADDR			(PERIPHERAL_BASE+0x20000)
#define IOPORT_BASE_ADDR			(PERIPHERAL_BASE+0x10000000)

/*
 * IOPORT peripheral base addresses
 */
#define GPIOA_BASE_ADDR				(IOPORT_BASE_ADDR)
#define GPIOB_BASE_ADDR				(IOPORT_BASE_ADDR+0x0400)
#define GPIOC_BASE_ADDR				(IOPORT_BASE_ADDR+0x0800)
#define GPIOD_BASE_ADDR				(IOPORT_BASE_ADDR+0x0C00)
#define GPIOE_BASE_ADDR				(IOPORT_BASE_ADDR+0x1000)
#define GPIOF_BASE_ADDR				(IOPORT_BASE_ADDR+0x1400)

/*
 * AHB peripheral base addresses
 */
#define RCC_BASE_ADDR				(AHB_BUS_BASE_ADDR+0x1000)
#define EXTI_BASE_ADDR				(AHB_BUS_BASE_ADDR+0x1800)

/*
 * APB peripheral base addresses
 */
#define SYSCFG_BASE_ADDR			(APB_BUS_BASE_ADDR+0x10000)

/*
 * RCC Structure Definition
 */
typedef struct{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_ICSR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t reserved;
	__vo uint32_t RCC_PLLCFGR;
	__vo uint32_t RCC_CRRCR;
	__vo uint32_t RCC_CIER;
	__vo uint32_t RCC_CIFR;
	__vo uint32_t RCC_CICR;
	__vo uint32_t RCC_IOPRSTR;
	__vo uint32_t RCC_AHBRSTR;
	__vo uint32_t RCC_APBRSTR1;
	__vo uint32_t RCC_APBRSTR2;
	__vo uint32_t RCC_IOPENR;
	__vo uint32_t RCC_AHBENR;
	__vo uint32_t RCC_APBENR1;
	__vo uint32_t RCC_APBENR2;
	__vo uint32_t RCC_IOPSMENR;
	__vo uint32_t RCC_AHBSMENR;
	__vo uint32_t RCC_APBSMENR1;
	__vo uint32_t RCC_APBSMENR2;
	__vo uint32_t RCC_CCIPR1;
	__vo uint32_t RCC_CCIPR2;
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
}RCC_RegDef_t;

/*
 * SYSCFG structure definition
 */
typedef struct{
	__vo uint32_t SYSCFG_CFGR1;
}SYSCFG_RegDef_t;

/*
 * GPIOx structure definition
 */
typedef struct{
	__vo uint32_t GPIOx_MODER;
	__vo uint32_t GPIOx_OTYPER;
	__vo uint32_t GPIOx_OSPEEDR;
	__vo uint32_t GPIOx_PUPDR;
	__vo uint32_t GPIOx_IDR;
	__vo uint32_t GPIOx_ODR;
	__vo uint32_t GPIOx_BSRR;
	__vo uint32_t GPIOx_LCKR;
	__vo uint32_t GPIOx_AFR[2];
	__vo uint32_t GPIOx_BRR;
}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t EXTI_RTSR1;
	__vo uint32_t EXTI_FTSR1;
	__vo uint32_t EXTI_SWIER1;
	__vo uint32_t EXTI_RPR1;
	__vo uint32_t EXTI_FPR1;
	__vo uint32_t Reserved_0[5];
	__vo uint32_t EXTI_RTSR2;
	__vo uint32_t EXTI_FTSR2;
	__vo uint32_t EXTI_SWIER2;
	__vo uint32_t EXTI_RPR2;
	__vo uint32_t EXTI_FPR2;
	__vo uint32_t Reserved_1[9];
	__vo uint32_t EXTI_EXTICR[4];
	__vo uint32_t Reserved_2[4];
	__vo uint32_t EXTI_IMR1;
	__vo uint32_t EXTI_EMR1;
	__vo uint32_t EXTI_IMR2;
	__vo uint32_t EXTI_EMR2;
}EXTI_RegDef_t;

/*
 * member elements for register definition structures
 */
#define GPIOA				((GPIO_RegDef_t *)GPIOA_BASE_ADDR)
#define GPIOB				((GPIO_RegDef_t *)GPIOB_BASE_ADDR)
#define GPIOC				((GPIO_RegDef_t *)GPIOC_BASE_ADDR)
#define GPIOD				((GPIO_RegDef_t *)GPIOD_BASE_ADDR)
#define GPIOE				((GPIO_RegDef_t *)GPIOE_BASE_ADDR)
#define GPIOF				((GPIO_RegDef_t *)GPIOF_BASE_ADDR)
#define RCC					((RCC_RegDef_t *)RCC_BASE_ADDR)
#define EXTI				((EXTI_RegDef_t *)EXTI_BASE_ADDR)

/*
 * clock enable functions
 */
#define GPIOA_CLK_EN()			(RCC->RCC_IOPENR |= (1 << 0))
#define GPIOB_CLK_EN()			(RCC->RCC_IOPENR |= (1 << 1))
#define GPIOC_CLK_EN()			(RCC->RCC_IOPENR |= (1 << 2))
#define GPIOD_CLK_EN()			(RCC->RCC_IOPENR |= (1 << 3))
#define GPIOE_CLK_EN()			(RCC->RCC_IOPENR |= (1 << 4))
#define GPIOF_CLK_EN()			(RCC->RCC_IOPENR |= (1 << 5))

/*
 * clock disable functions
 */
#define GPIOA_CLK_DI()			(RCC->RCC_IOPENR &= ~(1 << 0))
#define GPIOB_CLK_DI()			(RCC->RCC_IOPENR &= ~(1 << 1))
#define GPIOC_CLK_DI()			(RCC->RCC_IOPENR &= ~(1 << 2))
#define GPIOD_CLK_DI()			(RCC->RCC_IOPENR &= ~(1 << 3))
#define GPIOE_CLK_DI()			(RCC->RCC_IOPENR &= ~(1 << 4))
#define GPIOF_CLK_DI()			(RCC->RCC_IOPENR &= ~(1 << 5))

/*
 * IO port reset functions
 */
#define GPIOA_REG_RESET()		do {(RCC->RCC_IOPRSTR |= (1 << 0)); (RCC->RCC_IOPRSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()		do {(RCC->RCC_IOPRSTR |= (1 << 1)); (RCC->RCC_IOPRSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()		do {(RCC->RCC_IOPRSTR |= (1 << 2)); (RCC->RCC_IOPRSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()		do {(RCC->RCC_IOPRSTR |= (1 << 3)); (RCC->RCC_IOPRSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()		do {(RCC->RCC_IOPRSTR |= (1 << 4)); (RCC->RCC_IOPRSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()		do {(RCC->RCC_IOPRSTR |= (1 << 5)); (RCC->RCC_IOPRSTR &= ~(1 << 5));} while(0)

/*
 * Macro to return port code based on user selected gpio
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)?0x00:\
									 (x == GPIOB)?0x01:\
									 (x == GPIOC)?0x02:\
									 (x == GPIOD)?0x03:\
									 (x == GPIOF)?0x05:0)

/*
 * IRQ number for stm32g0b1re
 */
#define IRQ_NUM_EXTI0_1			5
#define IRQ_NUM_EXTI2_3			6
#define IRQ_NUM_EXTI4_15		7

/*
 * Programmable priority levels
 */
#define PRIORITY_0				0
#define PRIORITY_1				1
#define PRIORITY_2				2
#define PRIORITY_3				3

#endif /* INC_STM32G0B1RE_H_ */





