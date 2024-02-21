`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 
// Decodes the memory address space.
//
// 0x00xxxxxx = ROM
// 0x8xxxxxxx = RAM
//
// Memory-mapped I/O
// 0x10xxxxxx = UART registers
// 0x11xxxxxx = CLINT registers
// 0x12xxxxxx = SPI registers
// 0x13xxxxxx = SW (switches) register
//
// Copyright Colm Gavin, 2024. All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////////

module address_decoder(
    input[7:0] address,     // High byte of the address bus
    output ram_enable,
    output rom_enable,
    output uart_enable,
    output clint_enable,
    output spi_enable,
    output sw_enable
    );


    assign ram_enable   = (address[7:3] == 5'b10000);
    assign rom_enable   = (address[7:0] == 8'b00000000);
    assign uart_enable  = (address[7:0] == 8'b00010000);
    assign clint_enable = (address[7:0] == 8'b00010001);
    assign spi_enable   = (address[7:0] == 8'b00010010);
    assign sw_enable    = (address[7:0] == 8'b00010011);
endmodule
