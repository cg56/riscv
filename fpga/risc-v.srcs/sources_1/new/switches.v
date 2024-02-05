`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Memory mapped I/O to read the lower 8 switches.
//
// Copyright Colm Gavin, 2024. All rights reserved.
// 
//////////////////////////////////////////////////////////////////////////////////

module switches(
    input [7:0] switches,
    input clock,            // 100Mhz clock
    inout[31:0] data_io,    // Only supports reading
    input enable,           // Enable the memory mapped I/O
    input read,             // Request to read
    output reg complete     // The read or write request is complete
    );
    
    // Tri-state bus
    
    reg[31:0] data_out;
    assign data_io = (read & enable) ? data_out : 32'bZ;
    
    // Read the values of the switches
    
    always @(posedge clock) begin
        if (read & enable) begin
            data_out = {24'h00,switches[7:0]};
            complete <= 1;
        end else
            complete <= 0;
    end
endmodule
