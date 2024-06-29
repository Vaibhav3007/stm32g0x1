################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32g0b1re_gpio_driver.c 

OBJS += \
./Drivers/Src/stm32g0b1re_gpio_driver.o 

C_DEPS += \
./Drivers/Src/stm32g0b1re_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32G0 -DNUCLEO_G0B1RE -DSTM32G0B1RETx -c -I../Inc -I"E:/study/Embedded_C/MCU1/CodeSpace/stm32g0x1driver/Drivers" -I"E:/study/Embedded_C/MCU1/CodeSpace/stm32g0x1driver/Drivers/Inc" -I"E:/study/Embedded_C/MCU1/CodeSpace/stm32g0x1driver/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32g0b1re_gpio_driver.cyclo ./Drivers/Src/stm32g0b1re_gpio_driver.d ./Drivers/Src/stm32g0b1re_gpio_driver.o ./Drivers/Src/stm32g0b1re_gpio_driver.su

.PHONY: clean-Drivers-2f-Src

