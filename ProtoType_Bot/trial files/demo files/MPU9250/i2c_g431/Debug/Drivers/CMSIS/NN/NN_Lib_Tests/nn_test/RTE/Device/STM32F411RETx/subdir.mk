################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/startup_stm32f411xe.s 

C_SRCS += \
../Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/system_stm32f4xx.c 

OBJS += \
./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/startup_stm32f411xe.o \
./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/system_stm32f4xx.o 

S_DEPS += \
./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/startup_stm32f411xe.d 

C_DEPS += \
./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/%.o: ../Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/%.s Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/%.o Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/%.su Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/%.cyclo: ../Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/%.c Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-NN-2f-NN_Lib_Tests-2f-nn_test-2f-RTE-2f-Device-2f-STM32F411RETx

clean-Drivers-2f-CMSIS-2f-NN-2f-NN_Lib_Tests-2f-nn_test-2f-RTE-2f-Device-2f-STM32F411RETx:
	-$(RM) ./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/startup_stm32f411xe.d ./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/startup_stm32f411xe.o ./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/system_stm32f4xx.cyclo ./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/system_stm32f4xx.d ./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/system_stm32f4xx.o ./Drivers/CMSIS/NN/NN_Lib_Tests/nn_test/RTE/Device/STM32F411RETx/system_stm32f4xx.su

.PHONY: clean-Drivers-2f-CMSIS-2f-NN-2f-NN_Lib_Tests-2f-nn_test-2f-RTE-2f-Device-2f-STM32F411RETx

