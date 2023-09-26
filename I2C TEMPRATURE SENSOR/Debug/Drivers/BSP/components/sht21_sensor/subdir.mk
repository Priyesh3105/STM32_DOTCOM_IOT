################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/components/sht21_sensor/sht21.c 

OBJS += \
./Drivers/BSP/components/sht21_sensor/sht21.o 

C_DEPS += \
./Drivers/BSP/components/sht21_sensor/sht21.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/components/sht21_sensor/%.o Drivers/BSP/components/sht21_sensor/%.su Drivers/BSP/components/sht21_sensor/%.cyclo: ../Drivers/BSP/components/sht21_sensor/%.c Drivers/BSP/components/sht21_sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/DNK124/Desktop/Priyesh/STM32/I2C TEMPRATURE SENSOR/Drivers/BSP/components/sht21_sensor" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-components-2f-sht21_sensor

clean-Drivers-2f-BSP-2f-components-2f-sht21_sensor:
	-$(RM) ./Drivers/BSP/components/sht21_sensor/sht21.cyclo ./Drivers/BSP/components/sht21_sensor/sht21.d ./Drivers/BSP/components/sht21_sensor/sht21.o ./Drivers/BSP/components/sht21_sensor/sht21.su

.PHONY: clean-Drivers-2f-BSP-2f-components-2f-sht21_sensor

