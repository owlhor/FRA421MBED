################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/stk/ST7735_SF.c 

OBJS += \
./Core/Src/stk/ST7735_SF.o 

C_DEPS += \
./Core/Src/stk/ST7735_SF.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/stk/%.o Core/Src/stk/%.su: ../Core/Src/stk/%.c Core/Src/stk/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-stk

clean-Core-2f-Src-2f-stk:
	-$(RM) ./Core/Src/stk/ST7735_SF.d ./Core/Src/stk/ST7735_SF.o ./Core/Src/stk/ST7735_SF.su

.PHONY: clean-Core-2f-Src-2f-stk

