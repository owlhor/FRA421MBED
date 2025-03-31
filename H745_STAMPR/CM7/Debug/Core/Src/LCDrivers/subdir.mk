################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LCDrivers/ili9486.c \
../Core/Src/LCDrivers/lcd_io_gpio8.c 

OBJS += \
./Core/Src/LCDrivers/ili9486.o \
./Core/Src/LCDrivers/lcd_io_gpio8.o 

C_DEPS += \
./Core/Src/LCDrivers/ili9486.d \
./Core/Src/LCDrivers/lcd_io_gpio8.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LCDrivers/%.o Core/Src/LCDrivers/%.su Core/Src/LCDrivers/%.cyclo: ../Core/Src/LCDrivers/%.c Core/Src/LCDrivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../LIBJPEG/App -I../LIBJPEG/Target -I../../Middlewares/Third_Party/LibJPEG/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LCDrivers

clean-Core-2f-Src-2f-LCDrivers:
	-$(RM) ./Core/Src/LCDrivers/ili9486.cyclo ./Core/Src/LCDrivers/ili9486.d ./Core/Src/LCDrivers/ili9486.o ./Core/Src/LCDrivers/ili9486.su ./Core/Src/LCDrivers/lcd_io_gpio8.cyclo ./Core/Src/LCDrivers/lcd_io_gpio8.d ./Core/Src/LCDrivers/lcd_io_gpio8.o ./Core/Src/LCDrivers/lcd_io_gpio8.su

.PHONY: clean-Core-2f-Src-2f-LCDrivers

