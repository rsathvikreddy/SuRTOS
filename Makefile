# Target executable name (without extension)
TARGET = qemu_cortexm3_test

# MCU architecture details
MCU_ARCH = cortex-m3

# Toolchain prefix
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)as
LD = $(PREFIX)gcc # Using gcc as linker driver is common
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
GDB = $(PREFIX)gdb
SIZE = $(PREFIX)size

# Source files
# C source files
SRCS_C = src/main.c
# Assembly source files
SRCS_S = startup/startup_cortexm3.S

# Object files generated from C sources
OBJS_C = $(SRCS_C:.c=.o)
# Object files generated from Assembly sources
OBJS_S = $(SRCS_S:.S=.o)
# All object files
OBJS = $(OBJS_C) $(OBJS_S)

# Linker script
LDSCRIPT = linker/linker_cortexm3.ld

# Build directory (optional, keeps .o files out of source dirs)
# BUILD_DIR = build
# OBJS_C = $(addprefix $(BUILD_DIR)/, $(notdir $(SRCS_C:.c=.o)))
# OBJS_S = $(addprefix $(BUILD_DIR)/, $(notdir $(SRCS_S:.S=.o)))
# OBJS = $(OBJS_C) $(OBJS_S)


# Compiler flags
# -mcpu: Specify the target CPU
# -mthumb: Generate Thumb-2 instructions (standard for Cortex-M)
# -g: Generate debugging information
# -O0: No optimization (best for initial debugging)
# -Wall: Enable all common warnings
# -ffreestanding: Indicate that we are in a bare-metal environment (no standard libraries assumed)
# -fdata-sections, -ffunction-sections: Place each function/data item in its own section. Useful for linker garbage collection (if enabled).
CFLAGS = -mcpu=$(MCU_ARCH) -mthumb -g -O0 -Wall -ffreestanding
CFLAGS += -fdata-sections -ffunction-sections
# CFLAGS += -I./src # Add include paths if necessary, e.g., for header files in src/

# Assembler flags (GCC can assemble .S files, so CFLAGS often suffice for basic assembly)
# If using 'as' directly, ASFLAGS would be used. For .S files compiled with gcc:
ASFLAGS = -mcpu=$(MCU_ARCH) -mthumb -g

# Linker flags
# -T: Specify the linker script
# -nostdlib: Do NOT link with standard C libraries. Crucial for bare-metal.
# -Wl,: Pass options to the linker itself
#   -Map=$(TARGET).map: Generate a map file (useful for debugging memory layout)
#   --cref: Add cross-reference information to the map file
#   --gc-sections: Remove unused sections (requires -fdata-sections/-ffunction-sections in CFLAGS)
LFLAGS = -T $(LDSCRIPT) -nostdlib
LFLAGS += -Wl,-Map=$(TARGET).map,--cref #,--gc-sections

# --- Build Rules ---

# Default target: build the .elf file
all: $(TARGET).elf

# Rule to link the .elf file
$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo "Linking target: $@"
	$(LD) $(CFLAGS) $(OBJS) $(LFLAGS) -o $@
	@echo "--- Size of $(TARGET).elf ---"
	$(SIZE) $@
	@echo "--------------------------"

# Rule to compile C source files to object files
# $(BUILD_DIR)/%.o: %.c
# 	@mkdir -p $(@D)
# 	@echo "Compiling C: $<"
# 	$(CC) $(CFLAGS) -c $< -o $@
%.o: %.c $(LDSCRIPT) # Depend on linker script in case it defines symbols C code uses
	@echo "Compiling C: $<"
	$(CC) $(CFLAGS) -c $< -o $@


# Rule to assemble .S source files to object files
# $(BUILD_DIR)/%.o: %.S
# 	@mkdir -p $(@D)
# 	@echo "Assembling: $<"
# 	$(CC) $(ASFLAGS) -c $< -o $@ # Using GCC to assemble
%.o: %.S $(LDSCRIPT)
	@echo "Assembling: $<"
	$(CC) $(ASFLAGS) -c $< -o $@ # Using GCC to assemble

# --- Utility Rules ---

# Clean up build artifacts
clean:
	@echo "Cleaning up..."
	-if exist "$(TARGET).elf" del /F /Q "$(TARGET).elf"
	-if exist "$(TARGET).map" del /F /Q "$(TARGET).map"
	-if exist src if exist src\main.o del /F /Q src\main.o
	-if exist startup if exist startup\startup_cortexm3.o del /F /Q startup\startup_cortexm3.o
	@echo "Clean complete."



# Rule to run the .elf file in QEMU
# -M lm3s6965evb: Specify the emulated machine
# -kernel $(TARGET).elf: Load the .elf file as the kernel
# -nographic: Do not open a QEMU GUI window (output to console)
# -serial stdio: Redirect the emulated serial port (UART0 for lm3s6965evb) to standard input/output
qemu: $(TARGET).elf
	@echo "Starting QEMU with $(TARGET).elf..."
	qemu-system-arm -M lm3s6965evb -kernel $(TARGET).elf -nographic -serial mon:stdio

# Rule to start QEMU in debug mode (paused, waiting for GDB)
# -S: Start QEMU paused
# -s: Shorthand for -gdb tcp::1234 (listen for GDB on TCP port 1234)
debug-qemu: $(TARGET).elf
	@echo "Starting QEMU in debug mode (waiting for GDB on port 1234)..."
	qemu-system-arm -M lm3s6965evb -kernel $(TARGET).elf -nographic -serial mon:stdio -S -s

# To use GDB:
# 1. Run 'make debug-qemu' in one terminal.
# 2. In another terminal, navigate to this project directory and run:
#    arm-none-eabi-gdb $(TARGET).elf
# 3. In GDB, type:
#    target remote localhost:1234
#    load
#    break main
#    continue

.PHONY: all clean qemu debug-qemu # Declare phony targets