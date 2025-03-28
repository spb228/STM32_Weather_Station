# Project Name
TARGET = stm32_weather_station

# Directories
BUILD_DIR = build
SRC_DIR = src

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)as
LD = $(PREFIX)ld
OBJCOPY = $(PREFIX)objcopy
SIZE = $(PREFIX)size

# MCU Config
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Source Files
C_SOURCES = \
$(SRC_DIR)/main.c \
$(SRC_DIR)/startup.c \
$(SRC_DIR)/common/led/led.c \
$(SRC_DIR)/common/clock/system_clock.c \
$(SRC_DIR)/common/systick/systick.c \
$(SRC_DIR)/common/uart/uart.c

# Includes
C_INCLUDES = \
-I$(SRC_DIR) \
-I$(SRC_DIR)/common \
-I$(SRC_DIR)/common/led \
-I$(SRC_DIR)/common/clock \
-I$(SRC_DIR)/common/systick \
-I$(SRC_DIR)/common/uart

# Compiler Flags
C_DEFS = \
-DSTM32F401xE

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) \
-Wall -Wextra \
-Os \
-ffunction-sections \
-fdata-sections \
-ffreestanding \
-nostdlib \
-g -gdwarf-2

# Linker
LDSCRIPT = linkerscript.ld
LDFLAGS = $(MCU) -nostartfiles -T$(LDSCRIPT) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# Build Rules
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR):
	if not exist $@ mkdir $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	if not exist "$(dir $@)" mkdir "$(dir $@)"
	$(CC) -c $(CFLAGS) -o $@ $<

$(BUILD_DIR)/$(TARGET).elf: $(C_SOURCES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)
	$(CC) $^ $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rmdir /s /q build 2>nul || exit 0

.PHONY: all clean