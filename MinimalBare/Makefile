# STM32F407 最小裸机工程 Makefile

#------------------------------------------------------------------------------
# 工具链配置
#------------------------------------------------------------------------------
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc
LD = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE = $(PREFIX)size

#------------------------------------------------------------------------------
# 目标文件
#------------------------------------------------------------------------------
TARGET = main
BUILD_DIR = build

#------------------------------------------------------------------------------
# 源文件
#------------------------------------------------------------------------------
C_SOURCES = main.c
ASM_SOURCES = startup.s

#------------------------------------------------------------------------------
# 编译选项
#------------------------------------------------------------------------------
# CPU选项
CPU = -mcpu=cortex-m4
FPU = -mfloat-abi=soft
MCU = $(CPU) -mthumb $(FPU)

# C编译选项
CFLAGS = $(MCU)
CFLAGS += -Wall -Wextra              # 开启警告
CFLAGS += -std=c99                   # C99标准
CFLAGS += -ffunction-sections        # 每个函数放独立段（便于优化）
CFLAGS += -fdata-sections            # 每个数据放独立段
CFLAGS += -Os                        # 优化大小
CFLAGS += -g                         # 调试信息

# 汇编选项
ASFLAGS = $(MCU)
ASFLAGS += -Wall

# 链接选项
LDSCRIPT = linker.ld
LDFLAGS = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections         # 删除未使用的段
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map  # 生成map文件
LDFLAGS += --specs=nosys.specs       # 不使用系统调用
LDFLAGS += -nostartfiles             # 不使用标准启动文件

#------------------------------------------------------------------------------
# 构建规则
#------------------------------------------------------------------------------
# 目标文件
OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))

# 默认目标
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

# 创建目录
$(BUILD_DIR):
	mkdir -p $@

# 编译C文件
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# 编译汇编文件
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

# 链接
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	@echo "LD $@"
	@$(LD) $(OBJECTS) $(LDFLAGS) -o $@

# 生成HEX文件
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	@echo "OBJCOPY $@"
	@$(OBJCOPY) -O ihex $< $@

# 生成BIN文件
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo "OBJCOPY $@"
	@$(OBJCOPY) -O binary $< $@

# 显示大小
size: $(BUILD_DIR)/$(TARGET).elf
	@echo ""
	@echo "======== 内存使用情况 ========"
	@$(SIZE) $<

# 反汇编（学习用）
disasm: $(BUILD_DIR)/$(TARGET).elf
	$(OBJDUMP) -d -S $< > $(BUILD_DIR)/$(TARGET).dis
	@echo "反汇编文件: $(BUILD_DIR)/$(TARGET).dis"

# 清理
clean:
	rm -rf $(BUILD_DIR)

# 烧录（使用CMSIS-DAP）
flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
		-c "program $< verify reset exit"

.PHONY: all clean flash flash-stlink size disasm
