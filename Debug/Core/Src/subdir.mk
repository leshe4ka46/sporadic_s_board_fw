################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adxl345.c \
../Core/Src/ahrs.c \
../Core/Src/bmp180.c \
../Core/Src/debug.c \
../Core/Src/gy801.c \
../Core/Src/l3g4200d.c \
../Core/Src/lsm303dlhc.c \
../Core/Src/main.c \
../Core/Src/nrf.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/adxl345.o \
./Core/Src/ahrs.o \
./Core/Src/bmp180.o \
./Core/Src/debug.o \
./Core/Src/gy801.o \
./Core/Src/l3g4200d.o \
./Core/Src/lsm303dlhc.o \
./Core/Src/main.o \
./Core/Src/nrf.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/adxl345.d \
./Core/Src/ahrs.d \
./Core/Src/bmp180.d \
./Core/Src/debug.d \
./Core/Src/gy801.d \
./Core/Src/l3g4200d.d \
./Core/Src/lsm303dlhc.d \
./Core/Src/main.d \
./Core/Src/nrf.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adxl345.cyclo ./Core/Src/adxl345.d ./Core/Src/adxl345.o ./Core/Src/adxl345.su ./Core/Src/ahrs.cyclo ./Core/Src/ahrs.d ./Core/Src/ahrs.o ./Core/Src/ahrs.su ./Core/Src/bmp180.cyclo ./Core/Src/bmp180.d ./Core/Src/bmp180.o ./Core/Src/bmp180.su ./Core/Src/debug.cyclo ./Core/Src/debug.d ./Core/Src/debug.o ./Core/Src/debug.su ./Core/Src/gy801.cyclo ./Core/Src/gy801.d ./Core/Src/gy801.o ./Core/Src/gy801.su ./Core/Src/l3g4200d.cyclo ./Core/Src/l3g4200d.d ./Core/Src/l3g4200d.o ./Core/Src/l3g4200d.su ./Core/Src/lsm303dlhc.cyclo ./Core/Src/lsm303dlhc.d ./Core/Src/lsm303dlhc.o ./Core/Src/lsm303dlhc.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/nrf.cyclo ./Core/Src/nrf.d ./Core/Src/nrf.o ./Core/Src/nrf.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

