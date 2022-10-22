################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/PeripheralDriver/Src/IMU.c \
../Middlewares/PeripheralDriver/Src/oled.c \
../Middlewares/PeripheralDriver/Src/tm_stm32_ahrs_imu.c 

OBJS += \
./Middlewares/PeripheralDriver/Src/IMU.o \
./Middlewares/PeripheralDriver/Src/oled.o \
./Middlewares/PeripheralDriver/Src/tm_stm32_ahrs_imu.o 

C_DEPS += \
./Middlewares/PeripheralDriver/Src/IMU.d \
./Middlewares/PeripheralDriver/Src/oled.d \
./Middlewares/PeripheralDriver/Src/tm_stm32_ahrs_imu.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/PeripheralDriver/Src/%.o Middlewares/PeripheralDriver/Src/%.su: ../Middlewares/PeripheralDriver/Src/%.c Middlewares/PeripheralDriver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-PeripheralDriver-2f-Src

clean-Middlewares-2f-PeripheralDriver-2f-Src:
	-$(RM) ./Middlewares/PeripheralDriver/Src/IMU.d ./Middlewares/PeripheralDriver/Src/IMU.o ./Middlewares/PeripheralDriver/Src/IMU.su ./Middlewares/PeripheralDriver/Src/oled.d ./Middlewares/PeripheralDriver/Src/oled.o ./Middlewares/PeripheralDriver/Src/oled.su ./Middlewares/PeripheralDriver/Src/tm_stm32_ahrs_imu.d ./Middlewares/PeripheralDriver/Src/tm_stm32_ahrs_imu.o ./Middlewares/PeripheralDriver/Src/tm_stm32_ahrs_imu.su

.PHONY: clean-Middlewares-2f-PeripheralDriver-2f-Src

