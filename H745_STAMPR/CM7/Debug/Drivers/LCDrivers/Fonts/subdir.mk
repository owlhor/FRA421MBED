################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LCDrivers/Fonts/font12.c \
../Drivers/LCDrivers/Fonts/font16.c \
../Drivers/LCDrivers/Fonts/font20.c \
../Drivers/LCDrivers/Fonts/font24.c \
../Drivers/LCDrivers/Fonts/font8.c 

OBJS += \
./Drivers/LCDrivers/Fonts/font12.o \
./Drivers/LCDrivers/Fonts/font16.o \
./Drivers/LCDrivers/Fonts/font20.o \
./Drivers/LCDrivers/Fonts/font24.o \
./Drivers/LCDrivers/Fonts/font8.o 

C_DEPS += \
./Drivers/LCDrivers/Fonts/font12.d \
./Drivers/LCDrivers/Fonts/font16.d \
./Drivers/LCDrivers/Fonts/font20.d \
./Drivers/LCDrivers/Fonts/font24.d \
./Drivers/LCDrivers/Fonts/font8.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LCDrivers/Fonts/%.o Drivers/LCDrivers/Fonts/%.su: ../Drivers/LCDrivers/Fonts/%.c Drivers/LCDrivers/Fonts/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LCDrivers-2f-Fonts

clean-Drivers-2f-LCDrivers-2f-Fonts:
	-$(RM) ./Drivers/LCDrivers/Fonts/font12.d ./Drivers/LCDrivers/Fonts/font12.o ./Drivers/LCDrivers/Fonts/font12.su ./Drivers/LCDrivers/Fonts/font16.d ./Drivers/LCDrivers/Fonts/font16.o ./Drivers/LCDrivers/Fonts/font16.su ./Drivers/LCDrivers/Fonts/font20.d ./Drivers/LCDrivers/Fonts/font20.o ./Drivers/LCDrivers/Fonts/font20.su ./Drivers/LCDrivers/Fonts/font24.d ./Drivers/LCDrivers/Fonts/font24.o ./Drivers/LCDrivers/Fonts/font24.su ./Drivers/LCDrivers/Fonts/font8.d ./Drivers/LCDrivers/Fonts/font8.o ./Drivers/LCDrivers/Fonts/font8.su

.PHONY: clean-Drivers-2f-LCDrivers-2f-Fonts

