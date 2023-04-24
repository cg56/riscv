# Bootrom

## Install risk-v compiler

Run `sudo apt install gcc-riscv64-unknown-elf`

## Build the bootrom

Run `make & ./bin2rom bootrom.bin`, then copy the output into the rom.v verilog file.
