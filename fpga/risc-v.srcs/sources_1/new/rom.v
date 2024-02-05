`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Interface between the CPU bus and a boot rom.
//
// CPU reads must be alligned. E.g: 32-bit words can only be accessed
// on 4 byte boundaries.
//
// Copyright (c) Colm Gavin, 2024
// 
//////////////////////////////////////////////////////////////////////////////////

module rom(
    input clock,
    input reset,            // Active high reset 
    input[12:0] address,    // 2k x 32-bit words (8Kbyte)
    inout[31:0] data_io,    // Only data out
    input enable,           // Enable the memory for reading
    input read,             // Request to read from the memory
    output reg complete     // The read request is complete
    );

    // Tri-state bus
    
    reg[31:0] data_out;
    assign data_io = (read & enable) ? data_out : 32'bZ;
    
    // Select the appropriate 32-bit word
    
    reg[31:0] word;
    
    always @(posedge clock) begin
        if (reset) begin
            word     <= 32'h0;
            complete <= 0;    
        end else begin
            if (read & enable) begin
                case (address[12:2])
                `include "BootRom.vh"   

                default: word <= 32'h00000000;
                endcase
             
                complete <= 1;
            end else
                complete <= 0;
        end
    end
    
    // Shift to get the appropriate short/byte. We don't care about width,
    // so we may return extra higher bits.
    
    always @(address or word) begin
        case (address[1:0])
        0: data_out = word[31:0];
        1: data_out = { 8'h00,word[31:8]};
        2: data_out = {16'h00,word[31:16]};
        3: data_out = {24'h00,word[31:24]};
        endcase
    end
endmodule
