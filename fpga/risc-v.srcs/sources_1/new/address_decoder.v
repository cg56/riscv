`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/23/2023 09:31:27 PM
// Design Name: 
// Module Name: address_decoder
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module address_decoder(
    input[7:0] address,     // High byte of the address bus
    output ram_enable,
    output rom_enable,
    output uart_enable,
    output clint_enable,
    output spi_enable
    );
    
    // Memory-mapped I/O
    // 0x1000xxxx = UART registers
    // 0x1100xxxx = CLINT registers
    // 0x1200xxxx = SPI registers

    assign ram_enable   = (address[7:3] == 5'b10000);
    assign rom_enable   = (address[7:0] == 8'b00000000);
    assign uart_enable  = (address[7:0] == 8'b00010000);
    assign clint_enable = (address[7:0] == 8'b00010001);
    assign spi_enable   = (address[7:0] == 8'b00010010);

endmodule
