################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g431xx.s \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g441xx.s \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g471xx.s \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g473xx.s \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g474xx.s \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g483xx.s \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g484xx.s \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g491xx.s \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g4a1xx.s \
../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32gbk1cb.s 

S_DEPS += \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g431xx.d \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g441xx.d \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g471xx.d \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g473xx.d \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g474xx.d \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g483xx.d \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g484xx.d \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g491xx.d \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g4a1xx.d \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32gbk1cb.d 

OBJS += \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g431xx.o \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g441xx.o \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g471xx.o \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g473xx.o \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g474xx.o \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g483xx.o \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g484xx.o \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g491xx.o \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g4a1xx.o \
./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32gbk1cb.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/%.o: ../Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/%.s Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG '-DARM_MATH_CM4 =ARM_MATH_CM4' -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32G4xx-2f-Source-2f-Templates-2f-iar

clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32G4xx-2f-Source-2f-Templates-2f-iar:
	-$(RM) ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g431xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g431xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g441xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g441xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g471xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g471xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g473xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g473xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g474xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g474xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g483xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g483xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g484xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g484xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g491xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g491xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g4a1xx.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32g4a1xx.o ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32gbk1cb.d ./Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/iar/startup_stm32gbk1cb.o

.PHONY: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32G4xx-2f-Source-2f-Templates-2f-iar
