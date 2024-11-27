################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Hardware/ILI9341_GFX.c \
../Core/Src/Hardware/ILI9341_STM32_Driver.c \
../Core/Src/Hardware/ILI9341_Touchscreen.c \
../Core/Src/Hardware/OV7670.c 

OBJS += \
./Core/Src/Hardware/ILI9341_GFX.o \
./Core/Src/Hardware/ILI9341_STM32_Driver.o \
./Core/Src/Hardware/ILI9341_Touchscreen.o \
./Core/Src/Hardware/OV7670.o 

C_DEPS += \
./Core/Src/Hardware/ILI9341_GFX.d \
./Core/Src/Hardware/ILI9341_STM32_Driver.d \
./Core/Src/Hardware/ILI9341_Touchscreen.d \
./Core/Src/Hardware/OV7670.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Hardware/%.o Core/Src/Hardware/%.su Core/Src/Hardware/%.cyclo: ../Core/Src/Hardware/%.c Core/Src/Hardware/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I"C:/Users/jirap/OneDrive/Desktop/test_camera_5/Core/Inc/Hardware" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/AI/Inc -I../X-CUBE-AI/App -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Hardware

clean-Core-2f-Src-2f-Hardware:
	-$(RM) ./Core/Src/Hardware/ILI9341_GFX.cyclo ./Core/Src/Hardware/ILI9341_GFX.d ./Core/Src/Hardware/ILI9341_GFX.o ./Core/Src/Hardware/ILI9341_GFX.su ./Core/Src/Hardware/ILI9341_STM32_Driver.cyclo ./Core/Src/Hardware/ILI9341_STM32_Driver.d ./Core/Src/Hardware/ILI9341_STM32_Driver.o ./Core/Src/Hardware/ILI9341_STM32_Driver.su ./Core/Src/Hardware/ILI9341_Touchscreen.cyclo ./Core/Src/Hardware/ILI9341_Touchscreen.d ./Core/Src/Hardware/ILI9341_Touchscreen.o ./Core/Src/Hardware/ILI9341_Touchscreen.su ./Core/Src/Hardware/OV7670.cyclo ./Core/Src/Hardware/OV7670.d ./Core/Src/Hardware/OV7670.o ./Core/Src/Hardware/OV7670.su

.PHONY: clean-Core-2f-Src-2f-Hardware

