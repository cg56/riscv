# Bootrom

The boot rom instructions are hard-coded into the verilog source code for the
risk-v processor. When the processor starts up, it will execute this code. The
boot rom will then load a file "Image.bin" from the root directory of the
inserted CF card into the ram and start executing it from address 0x80000000.

## Install risk-v compiler

On Ubuntu, run `sudo apt install gcc-riscv64-unknown-elf`

## Build the bootrom

Run `make & ./bin2rom bootrom.bin`, then copy the output into the BootRom.vh verilog
file.
