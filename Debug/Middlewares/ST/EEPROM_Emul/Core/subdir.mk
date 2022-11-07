################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/EEPROM_Emul/Core/eeprom_emul.c 

OBJS += \
./Middlewares/ST/EEPROM_Emul/Core/eeprom_emul.o 

C_DEPS += \
./Middlewares/ST/EEPROM_Emul/Core/eeprom_emul.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/EEPROM_Emul/Core/%.o Middlewares/ST/EEPROM_Emul/Core/%.su: ../Middlewares/ST/EEPROM_Emul/Core/%.c Middlewares/ST/EEPROM_Emul/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4A6xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Middlewares/ST/EEPROM_Emul/Porting/STM32L4 -I../Middlewares/ST/EEPROM_Emul/Core -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-EEPROM_Emul-2f-Core

clean-Middlewares-2f-ST-2f-EEPROM_Emul-2f-Core:
	-$(RM) ./Middlewares/ST/EEPROM_Emul/Core/eeprom_emul.d ./Middlewares/ST/EEPROM_Emul/Core/eeprom_emul.o ./Middlewares/ST/EEPROM_Emul/Core/eeprom_emul.su

.PHONY: clean-Middlewares-2f-ST-2f-EEPROM_Emul-2f-Core

