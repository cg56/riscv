`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/24/2023 11:50:29 PM
// Design Name: 
// Module Name: clint
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
// https://github.com/riscv/riscv-aclint/blob/main/riscv-aclint.adoc
//
// timerl = 1100bff8
// timerh = 1100bffc
//
//////////////////////////////////////////////////////////////////////////////////


module clint(
    input clock,
    input reset,            // Active high reset 
    input[15:0] address,
    inout[31:0] data_io,    // Only supports 32-bit reads and writes
    input enable,           // Enable the memory for reading or writing
    input read,             // Request to read from the I/O device
    input write,            // Request to write to the I/O device
    output reg complete,    // The read or write request is complete
    output interrupt_req,   // Timer interrupt
    input[63:0] instr_count //??
    );
    
    reg[31:0] data_out;
    assign data_io = (read & enable) ? data_out : 32'bZ;
   
    reg[7:0]  divider;
    reg[63:0] timer;
    reg[63:0] timermatch;
    
    //assign interrupt_req = ((timermatch != 0) && (timer >= timermatch)) ? 1 : 0;
    assign interrupt_req = ((timermatch != 0) && (instr_count >= timermatch)) ? 1 : 0;
    //assign timerl = timer[31:0];
    
    always @(posedge clock) begin
        if (reset) begin
            divider  <= 0;
            timer    <= 0;
            timermatch <= 0;
            data_out <= 32'h0;
            complete <= 0;    
        end else begin
            divider <= divider + 1;
            if (divider == 100-1) begin    // Assuming input clock is 100Mhz
                divider <= 0;
                timer <= timer + 1;
            end

            if (read & enable) begin
                case (address)
                16'h4000: data_out <= timermatch[31:0];
                16'h4004: data_out <= timermatch[63:32];
                //16'hbff8: data_out <= timer[31:0];
                //16'hbffc: data_out <= timer[63:32];
                16'hbff8: data_out <= instr_count[31:0];
                16'hbffc: data_out <= instr_count[63:32];
                default: data_out <= 0;
                endcase
                complete   <= 1;
            end else if (write & enable) begin
                case (address)
                16'h4000: timermatch[31:0]  <= data_io[31:0];
                16'h4004: timermatch[63:32] <= data_io[31:0];
                //16'hbff8: timer[31:0]  <= data_io[31:0];
                //16'hbffc: timer[63:32] <= data_io[31:0];
                endcase
                complete <= 1;
            end else begin;
                complete <= 0;
            end
        end
    end
endmodule
