################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/User/subdir.mk
-include User/USB_Async/subdir.mk
-include User/SW4STM32/subdir.mk
-include Middlewares/USBD_Library/Core/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/arm\ math/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/STM32F4-Discovery/subdir.mk
-include Drivers/BSP/Components/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: USB_buffer_DAC.elf

# Tool invocations
USB_buffer_DAC.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -L"C:\Users\JMF\Documents\STM32_New\STM32Cube_FW_F4_V1.13.0\Middlewares\ST\STM32_Audio\Addons\PDM" -L"C:\Users\JMF\Documents\STM32_New\STM32Cube_FW_F4_V1.13.0\Drivers\CMSIS\Lib\GCC"  -specs=nosys.specs -specs=nano.specs -T"../STM32F407VGTx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -lm -o "USB_buffer_DAC.elf" @"objects.list" $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "USB_buffer_DAC.elf" "USB_buffer_DAC.bin"
	arm-none-eabi-size "USB_buffer_DAC.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
