################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/cmsismodule.c \
../Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/fftinit.c 

OBJS += \
./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/cmsismodule.o \
./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/fftinit.o 

C_DEPS += \
./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/cmsismodule.d \
./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/fftinit.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/%.o Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/%.su Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/%.cyclo: ../Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/%.c Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-PythonWrapper-2f-cmsisdsp_pkg-2f-src

clean-Drivers-2f-CMSIS-2f-DSP-2f-PythonWrapper-2f-cmsisdsp_pkg-2f-src:
	-$(RM) ./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/cmsismodule.cyclo ./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/cmsismodule.d ./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/cmsismodule.o ./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/cmsismodule.su ./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/fftinit.cyclo ./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/fftinit.d ./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/fftinit.o ./Drivers/CMSIS/DSP/PythonWrapper/cmsisdsp_pkg/src/fftinit.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-PythonWrapper-2f-cmsisdsp_pkg-2f-src

