################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctions.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctionsF16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q7.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f64.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q7.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.c \
../Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.c 

C_DEPS += \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctions.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctionsF16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q7.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f64.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q7.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.d \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.d 

OBJS += \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctions.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctionsF16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q7.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f64.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q7.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.o \
./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Source/MatrixFunctions/%.o Drivers/CMSIS_DSP/Source/MatrixFunctions/%.su Drivers/CMSIS_DSP/Source/MatrixFunctions/%.cyclo: ../Drivers/CMSIS_DSP/Source/MatrixFunctions/%.c Drivers/CMSIS_DSP/Source/MatrixFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -IC:/Users/henri/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-MatrixFunctions

clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-MatrixFunctions:
	-$(RM) ./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctions.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctions.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctions.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctions.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctionsF16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctionsF16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctionsF16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctionsF16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_householder_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_add_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f64.d
	-$(RM) ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_init_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_inverse_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_opt_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q7.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q7.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q7.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_mult_q7.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_qr_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q31.o
	-$(RM) ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_scale_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_sub_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f64.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f64.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f64.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_f64.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q7.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q7.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q7.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_trans_q7.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.cyclo ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.su ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.cyclo
	-$(RM) ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.d ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.o ./Drivers/CMSIS_DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-MatrixFunctions

