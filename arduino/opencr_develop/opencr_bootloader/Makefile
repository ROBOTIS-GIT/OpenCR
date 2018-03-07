CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size


BRD_NAME = opencr

PRJ_NAME = bin/opencr_boot
OBJ_DIR  = bin/obj
BIN_DIR  = bin



INC_DIRS  = src/
INC_DIRS += common/bsp/$(BRD_NAME)/
INC_DIRS += common/bsp/$(BRD_NAME)/include/
INC_DIRS += common/bsp/$(BRD_NAME)/cfg/
INC_DIRS += common/hal/
INC_DIRS += common/lib/STM32F7xx_HAL_Driver/Inc/
INC_DIRS += common/lib/STM32_USB_Device_Library/Core/Inc/




PRJ_SRC  = src/
BSP_SRC  = common/bsp/$(BRD_NAME)/
CFG_SRC  = common/bsp/$(BRD_NAME)/cfg/
HAL_SRC  = common/hal/
STM_SRC  = common/lib/STM32F7xx_HAL_Driver/Src/
USB_SRC  = common/lib/STM32_USB_Device_Library/Core/Src/
LNK_SRC  = common/bsp/$(BRD_NAME)/ld/opencr_flash.ld



vpath %.c $(PRJ_SRC)
vpath %.c $(BSP_SRC)
vpath %.c $(CFG_SRC)
vpath %.c $(HAL_SRC)
vpath %.c $(STM_SRC)
vpath %.c $(USB_SRC)
vpath %.s $(CFG_SRC)


PRJ_SRCS = $(notdir $(wildcard $(PRJ_SRC)*.c ))
BSP_SRCS = $(notdir $(wildcard $(BSP_SRC)*.c ))
CFG_SRCS = $(notdir $(wildcard $(CFG_SRC)*.c ))
HAL_SRCS = $(notdir $(wildcard $(HAL_SRC)*.c ))
STM_SRCS = $(notdir $(wildcard $(STM_SRC)*.c ))
USB_SRCS = $(notdir $(wildcard $(USB_SRC)*.c ))
SRT_SRCS = $(notdir $(wildcard $(CFG_SRC)*.s ))


SRCS  = main.c




INCLUDE = $(addprefix -I,$(INC_DIRS))

DEFS = -DSTM32F746xx

CFLAGS  = -ggdb -O2 -ffunction-sections -fdata-sections -std=gnu99
CFLAGS += -mthumb 
CFLAGS += -mcpu=cortex-m7 
CFLAGS += -mfloat-abi=softfp -mfpu=fpv5-sp-d16
CFLAGS += -Wl,--gc-sections
WFLAGS += -Wall -Wextra -Warray-bounds -Wno-unused-parameter
LFLAGS  = -T$(LNK_SRC) -Wl,-Map=$(PRJ_NAME).map


TARGET_OBJS	  = $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(PRJ_SRCS))))
TARGET_OBJS	 += $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(BSP_SRCS))))
TARGET_OBJS	 += $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(CFG_SRCS))))
TARGET_OBJS	 += $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(HAL_SRCS))))
TARGET_OBJS	 += $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(STM_SRCS))))
TARGET_OBJS	 += $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(USB_SRCS))))
TARGET_OBJS	 += $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(SRT_SRCS))))



all: $(PRJ_NAME)



$(PRJ_NAME): $(PRJ_NAME).elf




$(PRJ_NAME).elf: $(SRCS) $(TARGET_OBJS)
	$(CC) $(INCLUDE) $(DEFS) $(CFLAGS) $(WFLAGS) $(LFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(PRJ_NAME).elf   $(PRJ_NAME).hex
	$(OBJCOPY) -O binary $(PRJ_NAME).elf $(PRJ_NAME).bin
	$(SIZE) $(PRJ_NAME).elf



$(OBJ_DIR)/%.o: %.c
	$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^

$(OBJ_DIR)/%.o: %.s
	$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^
		

clean:
	rm -f $(OBJ_DIR)/*.* $(BIN_DIR)/*.*