################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LCDrivers/stm32h7xx/lcd_io_gpio8.c 

OBJS += \
./Drivers/LCDrivers/stm32h7xx/lcd_io_gpio8.o 

C_DEPS += \
./Drivers/LCDrivers/stm32h7xx/lcd_io_gpio8.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LCDrivers/stm32h7xx/%.o Drivers/LCDrivers/stm32h7xx/%.su: ../Drivers/LCDrivers/stm32h7xx/%.c Drivers/LCDrivers/stm32h7xx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LCDrivers-2f-stm32h7xx

clean-Drivers-2f-LCDrivers-2f-stm32h7xx:
	-$(RM) ./Drivers/LCDrivers/stm32h7xx/lcd_io_gpio8.d ./Drivers/LCDrivers/stm32h7xx/lcd_io_gpio8.o ./Drivers/LCDrivers/stm32h7xx/lcd_io_gpio8.su

.PHONY: clean-Drivers-2f-LCDrivers-2f-stm32h7xx

