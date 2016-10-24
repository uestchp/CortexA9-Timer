NAME = FreeRTOSDemo

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

DEFINES = -DPRINTF_FLOAT_SUPPORT

LIBS = -lm

SOURCE_DIR = .
BUILD_DIR = Build

C_OPTS =	-fmessage-length=0 \
		-mcpu=cortex-a9 \
		-g \
		-std=c99

C_FILES =	handlers.c
S_FILES =	startup.S

C_OBJS = $(C_FILES:%.c=$(BUILD_DIR)/%.o)

S_OBJS = $(S_FILES:%.S=$(BUILD_DIR)/%.o)

AUTODEPENDENCY_CFLAGS=-MMD -MF$(@:.o=.d) -MT$@

ALL_CFLAGS = $(C_OPTS) $(DEFINES) $(CFLAGS) $(AUTODEPENDENCY_CFLAGS)

ALL_LDFLAGS_BASE =	$(LD_FLAGS) \
			-nostartfiles \
			-mcpu=cortex-a9

ALL_LDFLAGS = $(ALL_LDFLAGS_BASE) -Wl,-T,plain.ld

AUTODEPENDENCY_CFLAGS=-MMD -MF$(@:.o=.d) -MT$@

MACHINEV := versatilepb
MACHINER := realview-pbx-a9

.SUFFIXES: .o .c .bin

all: $(NAME).uimg

clean:
	rm -rf $(BUILD_DIR) *.elf *.bin *.uimg

qemu: $(NAME).uimg
	qemu-system-arm -M $(MACHINER) -nographic -kernel $(NAME).elf -s -S

qemu-run: $(NAME).uimg
	qemu-system-arm -M $(MACHINER) -nographic -kernel $(NAME).elf

$(NAME).uimg: $(NAME).bin
	mkimage -A arm -O linux -T kernel -C none -a 0x0010000 -e 0x3010000 -d $< -n FreeRTOS.O $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

$(NAME).elf: $(C_OBJS) $(S_OBJS)
	$(LD) $(ALL_LDFLAGS) -o $@ $^ $(LIBS)

$(BUILD_DIR)/%.o: $(SOURCE_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) $(AUTODEPENDENCY_CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(SOURCE_DIR)/%.S
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) $(AUTODEPENDENCY_CFLAGS) -c $< -o $@

-include $(C_OBJS:.o=.d)

