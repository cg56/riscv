`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/23/2023 09:59:32 PM
// Design Name: 
// Module Name: memory_test
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
// Tests the ram by writing a simple incremental pattern
//
//////////////////////////////////////////////////////////////////////////////////


module memory_test(
    input clk_cpu,
    input reset,
    output reg[31:0] address,
    inout[31:0] data_io,
    output reg[1:0] width,  // 0 = 8 bits, 1 = 16 bits, 2 = 32 bits
    output[7:0] led,
    output[31:0] sseg_data,
    output reg write,
    output reg read,
    input ready,
    input complete
    );
    
    reg[31:0] data_in;
    assign data_io = write ? data_in : 32'bZ;
    
    reg passed;
    reg failed;
    reg[2:0] state;
    
    assign led[0] = passed;
    assign led[1] = failed;
    assign led[2] = ready;
    assign led[3] = complete;
    assign led[6:4] = state;
    assign sseg_data = data_in;
    
    always @(posedge clk_cpu) begin
        if (reset) begin
            state   <= 0;
            read    <= 1'b0;
            write   <= 1'b0;
            address <= 32'h80000000;
            data_in <= 32'h0;
            //address <= 27'h0;
            //data_in <= 32'h01010100;
            width   <= 2'h2;
            passed  <= 1'b0;
            failed  <= 1'b0;
         
        end else begin
            case (state)
            0: begin    // Writing
                if (ready) begin
                    write <= 1'b1;
                    state <= 1;
                end
            end
            1: begin    // Wait for write to complete
                if (complete) begin
                    write <= 1'b0;
                    state <= 2;
                end
            end
            2: begin    // Reading
                if (ready) begin
                    read  <= 1'b1;
                    state <= 3;
                end
            end
            3: begin    // Wait for read to complete
                if (complete) begin
                    read <= 1'b0;
                
                    if (data_io[31:0] == data_in[31:0]) begin
                        passed <= 1;
                        state <= 4;
                    end else begin
                        failed <= 1;
                        state <= 5;
                    end
                end
            end
            4: begin
                data_in <= data_in + 1;
                address <= address + 1;
                state <= 0;
            end
            5: begin    // Halt, loop forever
            end
            
            endcase
        end
    end
    
endmodule
