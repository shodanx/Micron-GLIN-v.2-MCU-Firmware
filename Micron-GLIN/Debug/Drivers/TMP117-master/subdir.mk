################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/TMP117-master/tmp117.c 

C_DEPS += \
./Drivers/TMP117-master/tmp117.d 

OBJS += \
./Drivers/TMP117-master/tmp117.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/TMP117-master/%.o Drivers/TMP117-master/%.su: ../Drivers/TMP117-master/%.c Drivers/TMP117-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xB -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"D:/Micron-GLIN soft/Micron-GLIN/USB_DEVICE/App" -I"D:/Micron-GLIN soft/Micron-GLIN/USB_DEVICE/Target" -I"D:/Micron-GLIN soft/Micron-GLIN/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I"D:/Micron-GLIN soft/Micron-GLIN/Circular-buffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-TMP117-2d-master

clean-Drivers-2f-TMP117-2d-master:
	-$(RM) ./Drivers/TMP117-master/tmp117.d ./Drivers/TMP117-master/tmp117.o ./Drivers/TMP117-master/tmp117.su

.PHONY: clean-Drivers-2f-TMP117-2d-master

