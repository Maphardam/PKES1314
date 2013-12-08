################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
PKES1.lss \

FLASH_IMAGE += \
PKES1.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: PKES1.elf secondary-outputs

# Tool invocations
PKES1.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,PKES1.map -mmcu=atmega2560 -o "PKES1.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

PKES1.lss: PKES1.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S PKES1.elf  >"PKES1.lss"
	@echo 'Finished building: $@'
	@echo ' '

PKES1.hex: PKES1.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex PKES1.elf  "PKES1.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: PKES1.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega2560 PKES1.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) PKES1.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
