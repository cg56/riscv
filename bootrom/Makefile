BIN := /usr/bin/riscv64-unknown-elf-
CFLAGS := -Os -march=rv32ima -mabi=ilp32 -static -static-libgcc -fno-stack-protector -fdata-sections -ffunction-sections -Wno-address-of-packed-member

all:    bootrom.b64 bin2rom sim
	#./bin2rom bootrom.bin
	ls -l bootrom.bin

bootrom.b64:  bootrom.bin
	base64 bootrom.bin > bootrom.b64

bootrom.bin:  bootrom.elf
	$(BIN)objcopy $^ -O binary $@

bootrom.elf: bootrom.o bootrom.S
	$(BIN)gcc -o $@ bootrom.o bootrom.S $(CFLAGS) -T bootrom.ld -nostdlib -Wl,--gc-sections

bootrom.o: bootrom.c
	$(BIN)gcc -c -o $@ bootrom.c $(CFLAGS) -nostdlib -Wl,--gc-sections

sim: bootrom.c
	gcc -o $@ bootrom.c -g -DSIM -Wno-address-of-packed-member

bin2rom: bin2rom.c
	gcc bin2rom.c -o bin2rom

clean:
	rm -f bin2rom sim bootrom.o bootrom.elf bootrom.bin bootrom.b64

