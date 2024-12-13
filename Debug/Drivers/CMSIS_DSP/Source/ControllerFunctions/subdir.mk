################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.c 

C_DEPS += \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.d 

OBJS += \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Source/ControllerFunctions/%.o Drivers/CMSIS_DSP/Source/ControllerFunctions/%.su Drivers/CMSIS_DSP/Source/ControllerFunctions/%.cyclo: ../Drivers/CMSIS_DSP/Source/ControllerFunctions/%.c Drivers/CMSIS_DSP/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-ControllerFunctions

clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-ControllerFunctions:
	-$(RM) ./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-ControllerFunctions

