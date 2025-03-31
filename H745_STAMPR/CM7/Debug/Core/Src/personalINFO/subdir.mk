################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/personalINFO/persona_1.c 

OBJS += \
./Core/Src/personalINFO/persona_1.o 

C_DEPS += \
./Core/Src/personalINFO/persona_1.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/personalINFO/%.o Core/Src/personalINFO/%.su Core/Src/personalINFO/%.cyclo: ../Core/Src/personalINFO/%.c Core/Src/personalINFO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../LIBJPEG/App -I../LIBJPEG/Target -I../../Middlewares/Third_Party/LibJPEG/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-personalINFO

clean-Core-2f-Src-2f-personalINFO:
	-$(RM) ./Core/Src/personalINFO/persona_1.cyclo ./Core/Src/personalINFO/persona_1.d ./Core/Src/personalINFO/persona_1.o ./Core/Src/personalINFO/persona_1.su

.PHONY: clean-Core-2f-Src-2f-personalINFO

