################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/LedSwitchInterrupt.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/LedSwitchInterrupt.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/LedSwitchInterrupt.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32G0 -DNUCLEO_G0B1RE -DSTM32G0B1RETx -c -I../Inc -I"E:/study/Embedded_C/MCU1/CodeSpace/stm32g0x1driver/Drivers" -I"E:/study/Embedded_C/MCU1/CodeSpace/stm32g0x1driver/Drivers/Inc" -I"E:/study/Embedded_C/MCU1/CodeSpace/stm32g0x1driver/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/LedSwitchInterrupt.cyclo ./Src/LedSwitchInterrupt.d ./Src/LedSwitchInterrupt.o ./Src/LedSwitchInterrupt.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

