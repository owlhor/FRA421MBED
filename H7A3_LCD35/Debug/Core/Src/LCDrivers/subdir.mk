################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LCDrivers/ili9486.c \
../Core/Src/LCDrivers/lcd_io_gpio8.c \
../Core/Src/LCDrivers/stm32_adafruit_lcd.c \
../Core/Src/LCDrivers/stm32_adafruit_ts.c 

OBJS += \
./Core/Src/LCDrivers/ili9486.o \
./Core/Src/LCDrivers/lcd_io_gpio8.o \
./Core/Src/LCDrivers/stm32_adafruit_lcd.o \
./Core/Src/LCDrivers/stm32_adafruit_ts.o 

C_DEPS += \
./Core/Src/LCDrivers/ili9486.d \
./Core/Src/LCDrivers/lcd_io_gpio8.d \
./Core/Src/LCDrivers/stm32_adafruit_lcd.d \
./Core/Src/LCDrivers/stm32_adafruit_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LCDrivers/%.o Core/Src/LCDrivers/%.su: ../Core/Src/LCDrivers/%.c Core/Src/LCDrivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H7A3xxQ -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LCDrivers

clean-Core-2f-Src-2f-LCDrivers:
	-$(RM) ./Core/Src/LCDrivers/ili9486.d ./Core/Src/LCDrivers/ili9486.o ./Core/Src/LCDrivers/ili9486.su ./Core/Src/LCDrivers/lcd_io_gpio8.d ./Core/Src/LCDrivers/lcd_io_gpio8.o ./Core/Src/LCDrivers/lcd_io_gpio8.su ./Core/Src/LCDrivers/stm32_adafruit_lcd.d ./Core/Src/LCDrivers/stm32_adafruit_lcd.o ./Core/Src/LCDrivers/stm32_adafruit_lcd.su ./Core/Src/LCDrivers/stm32_adafruit_ts.d ./Core/Src/LCDrivers/stm32_adafruit_ts.o ./Core/Src/LCDrivers/stm32_adafruit_ts.su

.PHONY: clean-Core-2f-Src-2f-LCDrivers

