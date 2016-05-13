CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size



PRJ_NAME = bin/opencr_boot
OBJ_DIR  = bin/obj
BIN_DIR  = bin



INC_DIRS  = src/
INC_DIRS += common/bsp/opencr/
INC_DIRS += common/bsp/opencr/Include/
INC_DIRS += common/lib/STM32F7xx_HAL_Driver/Inc/
INC_DIRS += common/lib/STM32_USB_Device_Library/Core/Inc/
INC_DIRS += common/lib/STM32_USB_Device_Library/Class/CDC/Inc/



PRJ_SRC  = src/
BSP_SRC  = common/bsp/opencr/
HAL_SRC  = common/lib/STM32F7xx_HAL_Driver/Src/
USB_SRC  = common/lib/STM32_USB_Device_Library/Core/Src/
LNK_SRC  = common/bsp/opencr/ld/opencr_flash.ld



vpath %.c $(PRJ_SRC)
vpath %.c $(BSP_SRC)
vpath %.c $(HAL_SRC)
vpath %.c $(USB_SRC)


PRJ_SRCS = $(notdir $(wildcard $(PRJ_SRC)*.c ))
BSP_SRCS = $(notdir $(wildcard $(BSP_SRC)*.c ))
HAL_SRCS = $(notdir $(wildcard $(HAL_SRC)*.c ))
USB_SRCS = $(notdir $(wildcard $(USB_SRC)*.c ))



SRCS = main.c
SRCS += common/bsp/opencr/startup_stm32f746xx.s



INCLUDE = $(addprefix -I,$(INC_DIRS))

DEFS = -DSTM32F746xx

CFLAGS  = -ggdb -O0 -std=gnu99
CFLAGS += -mthumb 
CFLAGS += -mcpu=cortex-m7 
CFLAGS += -Wl,--gc-sections
WFLAGS += -Wall -Wextra -Warray-bounds -Wno-unused-parameter
LFLAGS  = -T$(LNK_SRC)


TARGET_OBJS	  = $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(PRJ_SRCS))))
TARGET_OBJS	 += $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(BSP_SRCS))))
TARGET_OBJS	 += $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(HAL_SRCS))))
TARGET_OBJS	 += $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(USB_SRCS))))


all: $(PRJ_NAME)



$(PRJ_NAME): $(PRJ_NAME).elf




$(PRJ_NAME).elf: $(SRCS) $(TARGET_OBJS)
	$(CC) $(INCLUDE) $(DEFS) $(CFLAGS) $(WFLAGS) $(LFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(PRJ_NAME).elf   $(PRJ_NAME).hex
	$(OBJCOPY) -O binary $(PRJ_NAME).elf $(PRJ_NAME).bin
	$(SIZE) $(PRJ_NAME).elf



$(OBJ_DIR)/%.o: %.c
	$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^

		

clean:
	rm -f $(OBJ_DIR)/*.* $(BIN_DIR)/*.*