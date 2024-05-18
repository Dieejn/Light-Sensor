################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/button_detect.c \
../lib/fonts.c \
../lib/ssd1306.c 

OBJS += \
./lib/button_detect.o \
./lib/fonts.o \
./lib/ssd1306.o 

C_DEPS += \
./lib/button_detect.d \
./lib/fonts.d \
./lib/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
lib/%.o lib/%.su lib/%.cyclo: ../lib/%.c lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/phamh/STM32CubeIDE/workspace_1.13.2/ssd1306/libs" -I"C:/Users/vutoa/Documents/Code/STM/ssd1306/lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-lib

clean-lib:
	-$(RM) ./lib/button_detect.cyclo ./lib/button_detect.d ./lib/button_detect.o ./lib/button_detect.su ./lib/fonts.cyclo ./lib/fonts.d ./lib/fonts.o ./lib/fonts.su ./lib/ssd1306.cyclo ./lib/ssd1306.d ./lib/ssd1306.o ./lib/ssd1306.su

.PHONY: clean-lib

