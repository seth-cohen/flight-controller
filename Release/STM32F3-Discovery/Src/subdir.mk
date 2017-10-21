################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32F3-Discovery/Src/stm32f3_discovery.c \
../STM32F3-Discovery/Src/stm32f3_discovery_accelerometer.c \
../STM32F3-Discovery/Src/stm32f3_discovery_gyroscope.c 

OBJS += \
./STM32F3-Discovery/Src/stm32f3_discovery.o \
./STM32F3-Discovery/Src/stm32f3_discovery_accelerometer.o \
./STM32F3-Discovery/Src/stm32f3_discovery_gyroscope.o 

C_DEPS += \
./STM32F3-Discovery/Src/stm32f3_discovery.d \
./STM32F3-Discovery/Src/stm32f3_discovery_accelerometer.d \
./STM32F3-Discovery/Src/stm32f3_discovery_gyroscope.d 


# Each subdirectory must supply rules for building sources it contributes
STM32F3-Discovery/Src/%.o: ../STM32F3-Discovery/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F303xC -I"/Users/sethcohen/Documents/openSTM32/workspace/Flight_Controller/Inc" -I"/Users/sethcohen/Documents/openSTM32/workspace/Flight_Controller/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/Users/sethcohen/Documents/openSTM32/workspace/Flight_Controller/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/Users/sethcohen/Documents/openSTM32/workspace/Flight_Controller/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/Users/sethcohen/Documents/openSTM32/workspace/Flight_Controller/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/Users/sethcohen/Documents/openSTM32/workspace/Flight_Controller/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/Users/sethcohen/Documents/openSTM32/workspace/Flight_Controller/Drivers/CMSIS/Include" -I"/Users/sethcohen/Documents/openSTM32/workspace/Flight_Controller/STM32F3-Discovery/Inc" -I"/Users/sethcohen/Documents/openSTM32/workspace/Flight_Controller/STM32F3-Discovery/Components"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


