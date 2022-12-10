################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LCDrivers/Fonts/font12.c \
../Core/Src/LCDrivers/Fonts/font16.c \
../Core/Src/LCDrivers/Fonts/font20.c \
../Core/Src/LCDrivers/Fonts/font24.c \
../Core/Src/LCDrivers/Fonts/font8.c 

OBJS += \
./Core/Src/LCDrivers/Fonts/font12.o \
./Core/Src/LCDrivers/Fonts/font16.o \
./Core/Src/LCDrivers/Fonts/font20.o \
./Core/Src/LCDrivers/Fonts/font24.o \
./Core/Src/LCDrivers/Fonts/font8.o 

C_DEPS += \
./Core/Src/LCDrivers/Fonts/font12.d \
./Core/Src/LCDrivers/Fonts/font16.d \
./Core/Src/LCDrivers/Fonts/font20.d \
./Core/Src/LCDrivers/Fonts/font24.d \
./Core/Src/LCDrivers/Fonts/font8.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LCDrivers/Fonts/%.o Core/Src/LCDrivers/Fonts/%.su: ../Core/Src/LCDrivers/Fonts/%.c Core/Src/LCDrivers/Fonts/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../LIBJPEG/App -I../LIBJPEG/Target -I../../Middlewares/Third_Party/LibJPEG/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LCDrivers-2f-Fonts

clean-Core-2f-Src-2f-LCDrivers-2f-Fonts:
	-$(RM) ./Core/Src/LCDrivers/Fonts/font12.d ./Core/Src/LCDrivers/Fonts/font12.o ./Core/Src/LCDrivers/Fonts/font12.su ./Core/Src/LCDrivers/Fonts/font16.d ./Core/Src/LCDrivers/Fonts/font16.o ./Core/Src/LCDrivers/Fonts/font16.su ./Core/Src/LCDrivers/Fonts/font20.d ./Core/Src/LCDrivers/Fonts/font20.o ./Core/Src/LCDrivers/Fonts/font20.su ./Core/Src/LCDrivers/Fonts/font24.d ./Core/Src/LCDrivers/Fonts/font24.o ./Core/Src/LCDrivers/Fonts/font24.su ./Core/Src/LCDrivers/Fonts/font8.d ./Core/Src/LCDrivers/Fonts/font8.o ./Core/Src/LCDrivers/Fonts/font8.su

.PHONY: clean-Core-2f-Src-2f-LCDrivers-2f-Fonts

