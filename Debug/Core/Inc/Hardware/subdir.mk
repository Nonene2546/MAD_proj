################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Hardware/ILI9341_GFX.c \
../Core/Inc/Hardware/ILI9341_STM32_Driver.c \
../Core/Inc/Hardware/ILI9341_Touchscreen.c 

OBJS += \
./Core/Inc/Hardware/ILI9341_GFX.o \
./Core/Inc/Hardware/ILI9341_STM32_Driver.o \
./Core/Inc/Hardware/ILI9341_Touchscreen.o 

C_DEPS += \
./Core/Inc/Hardware/ILI9341_GFX.d \
./Core/Inc/Hardware/ILI9341_STM32_Driver.d \
./Core/Inc/Hardware/ILI9341_Touchscreen.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Hardware/%.o Core/Inc/Hardware/%.su Core/Inc/Hardware/%.cyclo: ../Core/Inc/Hardware/%.c Core/Inc/Hardware/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I"D:/MAD_Lab/test-camera-2/Core/Inc/Hardware" -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Hardware

clean-Core-2f-Inc-2f-Hardware:
	-$(RM) ./Core/Inc/Hardware/ILI9341_GFX.cyclo ./Core/Inc/Hardware/ILI9341_GFX.d ./Core/Inc/Hardware/ILI9341_GFX.o ./Core/Inc/Hardware/ILI9341_GFX.su ./Core/Inc/Hardware/ILI9341_STM32_Driver.cyclo ./Core/Inc/Hardware/ILI9341_STM32_Driver.d ./Core/Inc/Hardware/ILI9341_STM32_Driver.o ./Core/Inc/Hardware/ILI9341_STM32_Driver.su ./Core/Inc/Hardware/ILI9341_Touchscreen.cyclo ./Core/Inc/Hardware/ILI9341_Touchscreen.d ./Core/Inc/Hardware/ILI9341_Touchscreen.o ./Core/Inc/Hardware/ILI9341_Touchscreen.su

.PHONY: clean-Core-2f-Inc-2f-Hardware

