PROJECT := dfcom_example
BUILD_DIR := build

CROSS_COMPILE ?= arm-none-eabi-
CC := $(CROSS_COMPILE)gcc
AS := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE := $(CROSS_COMPILE)size

TARGET_MCU ?= STM32F103C8
FLASH_ADDR ?= 0x08000000
OPENOCD_INTERFACE ?= interface/stlink.cfg
OPENOCD_TARGET ?= target/stm32f1x.cfg

CPU_FLAGS := -mcpu=cortex-m3 -mthumb
DEFS := -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000

INCLUDES := \
  -IUSER \
  -ICORE \
  -IHARDWARE \
  -ISYSTEM/sys \
  -ISYSTEM/usart \
  -ISYSTEM/delay \
  -ISTM32F10x_FWLib/inc

C_SOURCES := \
  USER/main.c \
  USER/stm32f10x_it.c \
  USER/system_stm32f10x.c \
  SYSTEM/usart/usart.c \
  SYSTEM/delay/delay.c \
  HARDWARE/DFCom_Tx.c \
  HARDWARE/DFCom_Rx.c \
  HARDWARE/DFCom_Print.c \
  GCC/syscalls.c \
  $(wildcard STM32F10x_FWLib/src/*.c)

ASM_SOURCES := GCC/startup_stm32f103xb.s
LD_SCRIPT := GCC/STM32F103C8Tx_FLASH.ld

OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))

CFLAGS := $(CPU_FLAGS) $(DEFS) $(INCLUDES) -std=gnu99 -Os -ffunction-sections -fdata-sections -Wall -Wextra
ASFLAGS := $(CPU_FLAGS) -x assembler-with-cpp
LDFLAGS := $(CPU_FLAGS) -T$(LD_SCRIPT) -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map,--gc-sections -specs=nano.specs -specs=nosys.specs -u _printf_float

.PHONY: all clean flash flash-openocd size help

all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).bin $(BUILD_DIR)/$(PROJECT).hex size

$(BUILD_DIR)/$(PROJECT).elf: $(OBJECTS) $(LD_SCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

size: $(BUILD_DIR)/$(PROJECT).elf
	$(SIZE) $<

flash: $(BUILD_DIR)/$(PROJECT).bin
	st-flash --reset write $< $(FLASH_ADDR)

flash-openocd: $(BUILD_DIR)/$(PROJECT).elf
	openocd -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) \
	  -c "program $< verify reset exit"

clean:
	rm -rf $(BUILD_DIR)

help:
	@printf "Targets:\n"
	@printf "  make              Build ELF/BIN/HEX\n"
	@printf "  make flash        Flash with st-flash (ST-Link)\n"
	@printf "  make flash-openocd Flash with OpenOCD (ST-Link by default)\n"
	@printf "  make clean        Remove build output\n"
	@printf "\nUseful overrides:\n"
	@printf "  make OPENOCD_INTERFACE=interface/cmsis-dap.cfg flash-openocd\n"
	@printf "  make CROSS_COMPILE=/path/to/arm-none-eabi- flash\n"

-include $(OBJECTS:.o=.d)
