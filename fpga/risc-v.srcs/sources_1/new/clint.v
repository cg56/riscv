`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Core-Local Interrupt.
//
// Implements a 64-bit usec timer with interrupt.
// See: https://github.com/riscv/riscv-aclint/blob/main/riscv-aclint.adoc
// The addresses impelemted by this device are:
//
// timermatchl = 11004000
// timermatchh = 11004004
// timerl = 1100bff8
// timerh = 1100bffc
//
// Copyright (c) Colm Gavin, 2024 
//
//////////////////////////////////////////////////////////////////////////////////

module clint(
    input clock,            // 100MHz input clock
    input reset,            // Active high reset 
    input[15:0] address,
    inout[31:0] data_io,    // Only supports 32-bit reads and writes
    input enable,           // Enable for reading or writing
    input read,             // Request to read from the I/O device
    input write,            // Request to write to the I/O device
    output reg complete,    // The read or write request is complete
    output interrupt_req    // Timer interrupt request
    );
    
    // Tri-state bus
    
    reg[31:0] data_out;
    assign data_io = (read & enable) ? data_out : 32'bZ;
   
   // Local registers
   
    reg[7:0]  divider;
    reg[63:0] timer;
    reg[63:0] timermatch;
    
    // Interrupt request signal
    
    assign interrupt_req = ((timermatch != 0) && (timer >= timermatch)) ? 1 : 0;
    
    // The logic
    
    always @(posedge clock) begin
        if (reset) begin
            divider  <= 0;
            timer    <= 0;
            timermatch <= 0;
            data_out <= 32'h0;
            complete <= 0;    
        end else begin
            divider <= divider + 1;
            if (divider == 100-1) begin     // Assuming input clock is 100Mhz
                divider <= 0;
                timer <= timer + 1;         // Increment every 1 usec.
            end

            if (read & enable) begin
                case (address)
                16'h4000: data_out <= timermatch[31:0];
                16'h4004: data_out <= timermatch[63:32];
                16'hbff8: data_out <= timer[31:0];
                16'hbffc: data_out <= timer[63:32];
                default: data_out <= 0;
                endcase
                complete   <= 1;
            end else if (write & enable) begin
                case (address)
                16'h4000: timermatch[31:0]  <= data_io[31:0];
                16'h4004: timermatch[63:32] <= data_io[31:0];
                16'hbff8: timer[31:0]  <= data_io[31:0];
                16'hbffc: timer[63:32] <= data_io[31:0];
                endcase
                complete <= 1;
            end else begin;
                complete <= 0;
            end
        end
    end
endmodule
