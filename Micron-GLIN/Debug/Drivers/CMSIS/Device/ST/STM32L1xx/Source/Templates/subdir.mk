################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/Device/ST/STM32L1xx/Source/Templates/system_stm32l1xx.c 

OBJS += \
./Drivers/CMSIS/Device/ST/STM32L1xx/Source/Templates/system_stm32l1xx.o 

C_DEPS += \
./Drivers/CMSIS/Device/ST/STM32L1xx/Source/Templates/system_stm32l1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Device/ST/STM32L1xx/Source/Templates/%.o: ../Drivers/CMSIS/Device/ST/STM32L1xx/Source/Templates/%.c Drivers/CMSIS/Device/ST/STM32L1xx/Source/Templates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xB -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"D:/Micron-GLIN soft/Micron-GLIN/USB_DEVICE/App" -I"D:/Micron-GLIN soft/Micron-GLIN/USB_DEVICE/Target" -I"D:/Micron-GLIN soft/Micron-GLIN/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32L1xx-2f-Source-2f-Templates

clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32L1xx-2f-Source-2f-Templates:
	-$(RM) ./Drivers/CMSIS/Device/ST/STM32L1xx/Source/Templates/system_stm32l1xx.d ./Drivers/CMSIS/Device/ST/STM32L1xx/Source/Templates/system_stm32l1xx.o

.PHONY: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32L1xx-2f-Source-2f-Templates

