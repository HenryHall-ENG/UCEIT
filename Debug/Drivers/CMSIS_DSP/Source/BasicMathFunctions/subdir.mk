################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctions.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f64.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f64.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u8.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f64.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f64.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f64.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u8.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f64.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u8.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f64.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f64.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q15.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q31.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q7.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u16.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u32.c \
../Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u8.c 

C_DEPS += \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctions.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f64.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f64.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u8.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f64.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f64.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f64.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u8.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f64.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u8.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f64.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f64.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q15.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q31.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q7.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u16.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u32.d \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u8.d 

OBJS += \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctions.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f64.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f64.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u8.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f64.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f64.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f64.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u8.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f64.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u8.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f64.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f64.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q15.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q31.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q7.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u16.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u32.o \
./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u8.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Source/BasicMathFunctions/%.o Drivers/CMSIS_DSP/Source/BasicMathFunctions/%.su Drivers/CMSIS_DSP/Source/BasicMathFunctions/%.cyclo: ../Drivers/CMSIS_DSP/Source/BasicMathFunctions/%.c Drivers/CMSIS_DSP/Source/BasicMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-BasicMathFunctions

clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-BasicMathFunctions:
	-$(RM) ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctions.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctions.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctions.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctions.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f64.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f64.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f64.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_f64.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q31.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_abs_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f64.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f64.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f64.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_f64.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q31.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_add_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u8.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u8.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u8.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_and_u8.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_f32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q31.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_clip_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f32.su
	-$(RM) ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f64.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f64.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f64.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_f64.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q31.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_dot_prod_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f64.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f64.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f64.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_f64.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q31.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_mult_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f64.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f64.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f64.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_f64.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q31.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_negate_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u8.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u8.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u8.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_not_u8.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f64.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f64.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f64.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_f64.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q31.d
	-$(RM) ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_offset_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u8.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u8.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u8.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_or_u8.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f64.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f64.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f64.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_f64.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q31.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_scale_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q31.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_shift_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f64.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f64.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f64.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_f64.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q15.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q15.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q15.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q15.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q31.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q31.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q31.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q31.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q7.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q7.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q7.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_sub_q7.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u16.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u16.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u16.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u16.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u32.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u32.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u32.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u32.su ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u8.cyclo ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u8.d ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u8.o ./Drivers/CMSIS_DSP/Source/BasicMathFunctions/arm_xor_u8.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-BasicMathFunctions
