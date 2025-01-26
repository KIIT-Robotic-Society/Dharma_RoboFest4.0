################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/main.c 

OBJS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/main.o 

C_DEPS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/main.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/%.o Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/%.su Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/%.cyclo: ../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/%.c Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-DspLibTest_FVP_A5

clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-DspLibTest_FVP_A5:
	-$(RM) ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/main.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/main.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/main.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/DspLibTest_FVP_A5/main.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-DspLibTest_FVP_A5

