################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/lsm303agr/lsm303agr_reg.c 

OBJS += \
./Drivers/BSP/lsm303agr/lsm303agr_reg.o 

C_DEPS += \
./Drivers/BSP/lsm303agr/lsm303agr_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/lsm303agr/%.o Drivers/BSP/lsm303agr/%.su: ../Drivers/BSP/lsm303agr/%.c Drivers/BSP/lsm303agr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"/home/dnk067/workspace/I2C_BASIC/Drivers/BSP/lsm303agr" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-lsm303agr

clean-Drivers-2f-BSP-2f-lsm303agr:
	-$(RM) ./Drivers/BSP/lsm303agr/lsm303agr_reg.d ./Drivers/BSP/lsm303agr/lsm303agr_reg.o ./Drivers/BSP/lsm303agr/lsm303agr_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-lsm303agr

