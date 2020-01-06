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
ifneq ($(strip $(I_DEPS)),)
-include $(I_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(M_DEPS)),)
-include $(M_DEPS)
endif
ifneq ($(strip $(MI_DEPS)),)
-include $(MI_DEPS)
endif
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

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Volvo-KLine-Interface.lss \

FLASH_IMAGE += \
Volvo-KLine-Interface.hex \

EEPROM_IMAGE += \
Volvo-KLine-Interface.eep \

SIZEDUMMY += \
sizedummy \

AVRDUDEDUMMY += \
avrdudedummy \


# All Target
all: Volvo-KLine-Interface.elf secondary-outputs

# Tool invocations
Volvo-KLine-Interface.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Volvo-KLine-Interface.map -mmcu=atmega2560 -o "Volvo-KLine-Interface.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Volvo-KLine-Interface.lss: Volvo-KLine-Interface.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Volvo-KLine-Interface.elf  >"Volvo-KLine-Interface.lss"
	@echo 'Finished building: $@'
	@echo ' '

Volvo-KLine-Interface.hex: Volvo-KLine-Interface.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Volvo-KLine-Interface.elf  "Volvo-KLine-Interface.hex"
	@echo 'Finished building: $@'
	@echo ' '

Volvo-KLine-Interface.eep: Volvo-KLine-Interface.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Volvo-KLine-Interface.elf  "Volvo-KLine-Interface.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Volvo-KLine-Interface.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega2560 Volvo-KLine-Interface.elf
	@echo 'Finished building: $@'
	@echo ' '

avrdudedummy: Volvo-KLine-Interface.elf
	@echo 'Invoking: AVRDude'
	/usr/bin/avrdude -pm2560 -cwiring -P/dev/ttyACM0 -b115200 -D -Uflash:w:Volvo-KLine-Interface.hex:a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(M_DEPS)$(MI_DEPS)$(AVRDUDEDUMMY)$(S_DEPS)$(LSS)$(C_DEPS)$(FLASH_IMAGE)$(ELFS)$(I_DEPS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(SIZEDUMMY)$(S_UPPER_DEPS) Volvo-KLine-Interface.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY) $(AVRDUDEDUMMY)

.PHONY: all clean dependents

-include ../makefile.targets
