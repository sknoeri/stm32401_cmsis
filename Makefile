
TARGET = stm32401_CMSIS
######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0
#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/stm32f4xx_it.c \
Drivers/CMSIS/Device/ST/STM32F4xx/Source/system_stm32f4xx.c \
startup_stm32f401xe.c
#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4
# fpu
FPU = -mfpu=fpv4-sp-d16
# float-abi
FLOAT-ABI = -mfloat-abi=hard
# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
# macros for gcc
# C defines
C_DEFS =  \
-DSTM32F401xE
# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include
# compile gcc flags
CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F401RETx_FLASH.ld
# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
# default action: build all
# Der : nach etwasem schaut ob das was dem : steht existiert
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
# macht aus den C sourcen build/name.o
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
# vpath teillt make mit wo der quellcode name.c zu finden ist
vpath %.c $(sort $(dir $(C_SOURCES)))
# schlieslich wird jdes .c file mit %.c aufgerufen und compliliert
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
# Asemby file complier stuff not needed animore
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
# Build elf stuf
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
$(BUILD_DIR):
	mkdir $@		
#######################################
# flash
#######################################
flash_openocd: all
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
# flsh with j flash
device = STM32F401RE 
\n = & echo
$(BUILD_DIR)/jflash: $(BUILD_DIR)/$(TARGET).bin
	@echo $@
	@echo device $(device) > $@
	@echo si 1 >> $@
	@echo speed 4000 >> $@
	@echo loadbin $< 0x8000000 >> $@
	@echo r >> $@
	@echo g  >> $@
	@echo qc >> $@

jflash: $(BUILD_DIR)/jflash
	JLink -commanderscript $<
stflash: $(BUILD_DIR)/$(TARGET).bin
	st-flash --reset write $< 0x8000000
#######################################
# clean up
#######################################
clean: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	-rm -fR  $(BUILD_DIR)
#######################################
# dependencies -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***