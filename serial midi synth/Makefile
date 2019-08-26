CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
DIRS=-I. 
DIRS+=-I./cmsis 
DIRS+=-I./cmsis_boot 
DIRS+=-I./cmsis_boot/startup 
CFLAGS=-mcpu=cortex-m3 -mthumb -Wall -ffunction-sections -g -O0 -DSTM32F103CB -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -D__ASSEMBLY__ $(DIRS)
OUTPUTFOLDER=build/
LDFLAGS=-Tlink.ld -Wl,-Map,$(OUTPUTFOLDER)main.map -Wl,--gc-sections
TARGET=main
SRCS=main.c

all:
	$(CC) $(CFLAGS) -o $(OUTPUTFOLDER)$(TARGET).o -c $(SRCS)
	$(CC) $(CFLAGS) -o $(OUTPUTFOLDER)system_stm32f10x.o -c cmsis_boot/system_stm32f10x.c
	$(CC) $(CFLAGS) -o $(OUTPUTFOLDER)startup_stm32f10x_md.o -c cmsis_boot/startup/startup_stm32f10x_md.c
	$(CC) $(CFLAGS) $(LDFLAGS) $(OUTPUTFOLDER)startup_stm32f10x_md.o $(OUTPUTFOLDER)system_stm32f10x.o  $(OUTPUTFOLDER)$(TARGET).o -lm -o $(OUTPUTFOLDER)$(TARGET).elf
	$(OBJCOPY) -O binary $(OUTPUTFOLDER)$(TARGET).elf $(OUTPUTFOLDER)$(TARGET).bin

flash:
	make all
	"C:\Program Files\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility/ST-LINK_CLI.exe" -c SWD UR LPM -ME -p $(OUTPUTFOLDER)$(TARGET).bin 0x08000000 -Rst

clean:
	rm -f $(OUTPUTFOLDER)*.bin $(OUTPUTFOLDER)*.elf $(OUTPUTFOLDER)*.o $(OUTPUTFOLDER)*.map