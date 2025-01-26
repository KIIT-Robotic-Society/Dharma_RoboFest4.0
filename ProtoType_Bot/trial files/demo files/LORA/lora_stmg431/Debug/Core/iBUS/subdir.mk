################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/iBUS/ibus.c 

OBJS += \
./Core/iBUS/ibus.o 

C_DEPS += \
./Core/iBUS/ibus.d 


# Each subdirectory must supply rules for building sources it contributes
Core/iBUS/%.o Core/iBUS/%.su Core/iBUS/%.cyclo: ../Core/iBUS/%.c Core/iBUS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-iBUS

clean-Core-2f-iBUS:
	-$(RM) ./Core/iBUS/ibus.cyclo ./Core/iBUS/ibus.d ./Core/iBUS/ibus.o ./Core/iBUS/ibus.su

.PHONY: clean-Core-2f-iBUS

