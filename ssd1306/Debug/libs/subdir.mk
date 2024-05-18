################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libs/button_detect.c \
../libs/fonts.c \
../libs/ssd1306.c 

OBJS += \
./libs/button_detect.o \
./libs/fonts.o \
./libs/ssd1306.o 

C_DEPS += \
./libs/button_detect.d \
./libs/fonts.d \
./libs/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
libs/%.o libs/%.su libs/%.cyclo: ../libs/%.c libs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/phamh/STM32CubeIDE/workspace_1.13.2/ssd1306/libs" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-libs

clean-libs:
	-$(RM) ./libs/button_detect.cyclo ./libs/button_detect.d ./libs/button_detect.o ./libs/button_detect.su ./libs/fonts.cyclo ./libs/fonts.d ./libs/fonts.o ./libs/fonts.su ./libs/ssd1306.cyclo ./libs/ssd1306.d ./libs/ssd1306.o ./libs/ssd1306.su

.PHONY: clean-libs

