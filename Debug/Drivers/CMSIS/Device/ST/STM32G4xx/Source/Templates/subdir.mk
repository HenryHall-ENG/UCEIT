################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c 

C_DEPS += \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.d 

OBJS += \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/%.o Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/%.su Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/%.cyclo: ../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/%.c Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DARM_MATH_CM4=ARM_MATH_CM4 -c -I../Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -v -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32G4xx-2f-Source-2f-Templates

clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32G4xx-2f-Source-2f-Templates:
	-$(RM) ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.cyclo ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.su

.PHONY: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32G4xx-2f-Source-2f-Templates
