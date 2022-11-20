################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LCDrivers/lcd/ili9486/ili9486.c 

OBJS += \
./Drivers/LCDrivers/lcd/ili9486/ili9486.o 

C_DEPS += \
./Drivers/LCDrivers/lcd/ili9486/ili9486.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LCDrivers/lcd/ili9486/%.o Drivers/LCDrivers/lcd/ili9486/%.su: ../Drivers/LCDrivers/lcd/ili9486/%.c Drivers/LCDrivers/lcd/ili9486/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../LIBJPEG/App -I../LIBJPEG/Target -I../../Middlewares/Third_Party/LibJPEG/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LCDrivers-2f-lcd-2f-ili9486

clean-Drivers-2f-LCDrivers-2f-lcd-2f-ili9486:
	-$(RM) ./Drivers/LCDrivers/lcd/ili9486/ili9486.d ./Drivers/LCDrivers/lcd/ili9486/ili9486.o ./Drivers/LCDrivers/lcd/ili9486/ili9486.su

.PHONY: clean-Drivers-2f-LCDrivers-2f-lcd-2f-ili9486

