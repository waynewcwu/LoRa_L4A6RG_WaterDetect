################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/ST/EEPROM_Emul/Porting/STM32L4/subdir.mk
-include Middlewares/ST/EEPROM_Emul/Core/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := LoRa_L4A6RG_WaterDetect_V1q4
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LoRa_L4A6RG_WaterDetect_V1q4.elf \

MAP_FILES += \
LoRa_L4A6RG_WaterDetect_V1q4.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LoRa_L4A6RG_WaterDetect_V1q4.list \


# All Target
all: main-build

# Main-build Target
main-build: LoRa_L4A6RG_WaterDetect_V1q4.elf secondary-outputs

# Tool invocations
LoRa_L4A6RG_WaterDetect_V1q4.elf LoRa_L4A6RG_WaterDetect_V1q4.map: $(OBJS) $(USER_OBJS) C:\Users\USER\STM32CubeIDE\workspace_1.1.0\LoRa_L4A6RG_WaterDetect_V1q4\STM32L4A6RGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "LoRa_L4A6RG_WaterDetect_V1q4.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\USER\STM32CubeIDE\workspace_1.1.0\LoRa_L4A6RG_WaterDetect_V1q4\STM32L4A6RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LoRa_L4A6RG_WaterDetect_V1q4.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LoRa_L4A6RG_WaterDetect_V1q4.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LoRa_L4A6RG_WaterDetect_V1q4.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) LoRa_L4A6RG_WaterDetect_V1q4.elf LoRa_L4A6RG_WaterDetect_V1q4.list LoRa_L4A6RG_WaterDetect_V1q4.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
