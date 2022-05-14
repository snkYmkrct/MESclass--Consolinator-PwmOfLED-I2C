################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Sensors/aiq_PMSA003I_i2c.c \
../Core/Src/Sensors/co2_scd40_i2c.c \
../Core/Src/Sensors/scd4x_i2c.c \
../Core/Src/Sensors/sensirion_common.c \
../Core/Src/Sensors/sensirion_i2c.c \
../Core/Src/Sensors/sensirion_i2c_hal.c 

OBJS += \
./Core/Src/Sensors/aiq_PMSA003I_i2c.o \
./Core/Src/Sensors/co2_scd40_i2c.o \
./Core/Src/Sensors/scd4x_i2c.o \
./Core/Src/Sensors/sensirion_common.o \
./Core/Src/Sensors/sensirion_i2c.o \
./Core/Src/Sensors/sensirion_i2c_hal.o 

C_DEPS += \
./Core/Src/Sensors/aiq_PMSA003I_i2c.d \
./Core/Src/Sensors/co2_scd40_i2c.d \
./Core/Src/Sensors/scd4x_i2c.d \
./Core/Src/Sensors/sensirion_common.d \
./Core/Src/Sensors/sensirion_i2c.d \
./Core/Src/Sensors/sensirion_i2c_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Sensors/%.o Core/Src/Sensors/%.su: ../Core/Src/Sensors/%.c Core/Src/Sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I"C:/Users/silvi/STM32CubeIDE/workspace_1.9.0/Week 5 homework/Core/Src/Sensors" -I"C:/Users/silvi/STM32CubeIDE/workspace_1.9.0/Week 5 homework/Core/Src/Consolinator" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Sensors

clean-Core-2f-Src-2f-Sensors:
	-$(RM) ./Core/Src/Sensors/aiq_PMSA003I_i2c.d ./Core/Src/Sensors/aiq_PMSA003I_i2c.o ./Core/Src/Sensors/aiq_PMSA003I_i2c.su ./Core/Src/Sensors/co2_scd40_i2c.d ./Core/Src/Sensors/co2_scd40_i2c.o ./Core/Src/Sensors/co2_scd40_i2c.su ./Core/Src/Sensors/scd4x_i2c.d ./Core/Src/Sensors/scd4x_i2c.o ./Core/Src/Sensors/scd4x_i2c.su ./Core/Src/Sensors/sensirion_common.d ./Core/Src/Sensors/sensirion_common.o ./Core/Src/Sensors/sensirion_common.su ./Core/Src/Sensors/sensirion_i2c.d ./Core/Src/Sensors/sensirion_i2c.o ./Core/Src/Sensors/sensirion_i2c.su ./Core/Src/Sensors/sensirion_i2c_hal.d ./Core/Src/Sensors/sensirion_i2c_hal.o ./Core/Src/Sensors/sensirion_i2c_hal.su

.PHONY: clean-Core-2f-Src-2f-Sensors

