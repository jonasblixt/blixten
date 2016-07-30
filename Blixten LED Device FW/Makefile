# Blixten Makefile

RTX = CMSIS-SP-00300-r4p5-00rel0/CMSIS/RTOS/RTX/SRC
DSPLIB = CMSIS-SP-00300-r4p5-00rel0/CMSIS/DSP_Lib/Source

SRCS = $(RTX)/HAL_CM.c $(RTX)/rt_CMSIS.c $(RTX)/rt_Event.c $(RTX)/rt_List.c $(RTX)/rt_Mailbox.c $(RTX)/rt_MemBox.c $(RTX)/rt_Memory.c $(RTX)/rt_Mutex.c $(RTX)/rt_Robin.c $(RTX)/rt_Semaphore.c $(RTX)/rt_System.c $(RTX)/rt_Task.c $(RTX)/rt_Time.c $(RTX)/rt_Timer.c 
SRCS += $(DSPLIB)/FastMathFunctions/arm_sin_f32.c $(DSPLIB)/CommonTables/arm_common_tables.c
SRCS += main.c syscalls.c usart1.c timers.c neopixel_rgbw.c cc2520.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJ_NAME=main


CC=arm-none-eabi-gcc
GDB=arm-none-eabi-gdb
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size

# Main compiler options
CFLAGS  = -Wall -ggdb3 -std=c99 -O0  -D__CORTEX_M4 -mlittle-endian -mcpu=cortex-m4 -mthumb -march=armv7e-m
# RTX Options
CFLAGS += -ICMSIS-SP-00300-r4p5-00rel0/CMSIS/Include/ -ICMSIS-SP-00300-r4p5-00rel0/CMSIS/RTOS/RTX/INC -ICMSIS-SP-00300-r4p5-00rel0/CMSIS/RTOS/RTX/SRC   -D__CMSIS_RTOS -funwind-tables -DOS_SYSTICK=1
# Cortex M4 FPU options
CFLAGS += -mfloat-abi=hard -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfpu=fpv5-sp-d16 -fsingle-precision-constant
# Misc options
CFLAGS += -ffunction-sections -fdata-sections --specs=nano.specs --specs=nosys.specs 

#Linker options
LDFLAGS +=  -LCMSIS-SP-00300-r4p5-00rel0/CMSIS/Lib/GCC/ -Wl,--gc-sections -Wl,-Map=$(PROJ_NAME).map 

ROOT=$(shell pwd)

#STARTUP = startup_stm32f30x.s # add startup file to build
STARTUP = $(RTX)/GCC/HAL_CM4.s startup_stm32f30x.s $(RTX)/GCC/SVC_Table.S

OBJS = $(addprefix .objs/,$(SRCS:.c=.o))
DEPS = $(addprefix .deps/,$(SRCS:.c=.d))

###################################################

.PHONY: all proj clean

all:  proj

-include $(DEPS)


proj: 	$(PROJ_NAME).elf

dirs:
	@mkdir -p .deps .objs

.objs/%.o : %.c dirs
	@mkdir -p $(@D)
	@$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF .deps/$(*F).d
	@echo CC $<

$(PROJ_NAME).elf: $(OBJS)
	@$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(STARTUP) -Tstm32f302k8.ld
	@echo LINK $@
	@$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	@$(SIZE) $(PROJ_NAME).elf

clean:
	find ./ -name '*~' | xargs rm -f	
	rm -rf .objs/
	rm -rf .deps/
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).map
	rm -f $(PROJ_NAME).lst
