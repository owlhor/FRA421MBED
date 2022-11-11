################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LCDrivers/stm32_adafruit_lcd.c \
../Drivers/LCDrivers/stm32_adafruit_ts.c 

OBJS += \
./Drivers/LCDrivers/stm32_adafruit_lcd.o \
./Drivers/LCDrivers/stm32_adafruit_ts.o 

C_DEPS += \
./Drivers/LCDrivers/stm32_adafruit_lcd.d \
./Drivers/LCDrivers/stm32_adafruit_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LCDrivers/%.o Drivers/LCDrivers/%.su: ../Drivers/LCDrivers/%.c Drivers/LCDrivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LCDrivers

clean-Drivers-2f-LCDrivers:
	-$(RM) ./Drivers/LCDrivers/stm32_adafruit_lcd.d ./Drivers/LCDrivers/stm32_adafruit_lcd.o ./Drivers/LCDrivers/stm32_adafruit_lcd.su ./Drivers/LCDrivers/stm32_adafruit_ts.d ./Drivers/LCDrivers/stm32_adafruit_ts.o ./Drivers/LCDrivers/stm32_adafruit_ts.su

.PHONY: clean-Drivers-2f-LCDrivers

