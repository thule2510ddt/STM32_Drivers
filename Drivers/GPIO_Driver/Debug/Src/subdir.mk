################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/1_Led_Toggle.c \
../Src/2_Led_Button.c \
../Src/3_Led_Button_EXTI.c \
../Src/4_GPIO_Freq.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/1_Led_Toggle.o \
./Src/2_Led_Button.o \
./Src/3_Led_Button_EXTI.o \
./Src/4_GPIO_Freq.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/1_Led_Toggle.d \
./Src/2_Led_Button.d \
./Src/3_Led_Button_EXTI.d \
./Src/4_GPIO_Freq.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -c -I../Inc -I"H:/STM32_Drivers/Drivers/GPIO_Driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/1_Led_Toggle.cyclo ./Src/1_Led_Toggle.d ./Src/1_Led_Toggle.o ./Src/1_Led_Toggle.su ./Src/2_Led_Button.cyclo ./Src/2_Led_Button.d ./Src/2_Led_Button.o ./Src/2_Led_Button.su ./Src/3_Led_Button_EXTI.cyclo ./Src/3_Led_Button_EXTI.d ./Src/3_Led_Button_EXTI.o ./Src/3_Led_Button_EXTI.su ./Src/4_GPIO_Freq.cyclo ./Src/4_GPIO_Freq.d ./Src/4_GPIO_Freq.o ./Src/4_GPIO_Freq.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

