################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctions.c \
../Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctionsF16.c \
../Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.c \
../Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.c 

C_DEPS += \
./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctions.d \
./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctionsF16.d \
./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.d \
./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.d 

OBJS += \
./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctions.o \
./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctionsF16.o \
./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.o \
./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Source/BayesFunctions/%.o Drivers/CMSIS_DSP/Source/BayesFunctions/%.su Drivers/CMSIS_DSP/Source/BayesFunctions/%.cyclo: ../Drivers/CMSIS_DSP/Source/BayesFunctions/%.c Drivers/CMSIS_DSP/Source/BayesFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-BayesFunctions

clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-BayesFunctions:
	-$(RM) ./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctions.cyclo ./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctions.d ./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctions.o ./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctions.su ./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctionsF16.cyclo ./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctionsF16.d ./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctionsF16.o ./Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctionsF16.su ./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.cyclo ./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.d ./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.o ./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.su ./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.cyclo ./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.d ./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.o ./Drivers/CMSIS_DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-BayesFunctions

