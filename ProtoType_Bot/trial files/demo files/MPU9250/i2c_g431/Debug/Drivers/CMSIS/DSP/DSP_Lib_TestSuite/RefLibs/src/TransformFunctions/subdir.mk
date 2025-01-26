################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/TransformFunctions.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/bitreversal.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/cfft.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/dct4.c \
../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/rfft.c 

OBJS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/TransformFunctions.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/bitreversal.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/cfft.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/dct4.o \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/rfft.o 

C_DEPS += \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/TransformFunctions.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/bitreversal.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/cfft.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/dct4.d \
./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/rfft.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/%.o Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/%.su Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/%.cyclo: ../Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/%.c Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-RefLibs-2f-src-2f-TransformFunctions

clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-RefLibs-2f-src-2f-TransformFunctions:
	-$(RM) ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/TransformFunctions.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/TransformFunctions.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/TransformFunctions.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/TransformFunctions.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/bitreversal.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/bitreversal.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/bitreversal.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/bitreversal.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/cfft.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/cfft.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/cfft.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/cfft.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/dct4.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/dct4.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/dct4.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/dct4.su ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/rfft.cyclo ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/rfft.d ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/rfft.o ./Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/src/TransformFunctions/rfft.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-DSP_Lib_TestSuite-2f-RefLibs-2f-src-2f-TransformFunctions

