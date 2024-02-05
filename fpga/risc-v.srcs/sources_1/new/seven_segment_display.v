`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Displays a 32-bit value on the 7-segment display as 8 hex characters.
//
// Copyright (c) Colm Gavin, 2024
// 
//////////////////////////////////////////////////////////////////////////////////

module seven_segment_display(
    input clk,                  // 100MHz system clock
    input reset,                // Active high reset
    input [31:0] value,         // What to display
    output reg [6:0] segments,  // Drive the 7 segments
    output reg [7:0] anodes     // Driver the 8 digit anodes
    );
    
    reg[31:0] delay_counter;
    reg[3:0] digit;
    
    always @(posedge clk) begin
        if (reset) begin
            anodes <= 8'b1111_1110;
            delay_counter <= 0;
        end else begin
            if (delay_counter == 100000) begin
                delay_counter <= 0;
                anodes <= {anodes[6:0], anodes[7]}; // Select the next digit
            end else begin
                delay_counter <= delay_counter+1;   // Wait a while to show it
            end
        end
    
        // Which 4 bits of the input value should we display?
        
        case (anodes)
            default:        digit = value[3:0];
            8'b1111_1101:   digit = value[7:4];
            8'b1111_1011:   digit = value[11:8];
            8'b1111_0111:   digit = value[15:12];
            8'b1110_1111:   digit = value[19:16];
            8'b1101_1111:   digit = value[23:20];
            8'b1011_1111:   digit = value[27:24];
            8'b0111_1111:   digit = value[31:28];
        endcase
    end
    
    // What segments do we need to light up for each number?
    
    always @(*)
    case (digit)
        'h0: segments = 7'b1000000;
        'h1: segments = 7'b1111001;
        'h2: segments = 7'b0100100;
        'h3: segments = 7'b0110000;
        'h4: segments = 7'b0011001;
        'h5: segments = 7'b0010010;
        'h6: segments = 7'b0000010;
        'h7: segments = 7'b1111000;
        'h8: segments = 7'b0000000;
        'h9: segments = 7'b0010000;
        'hA: segments = 7'b0001000; 
        'hB: segments = 7'b0000011; 
        'hC: segments = 7'b1000110;
        'hD: segments = 7'b0100001;
        'hE: segments = 7'b0000110;
        'hF: segments = 7'b0001110;
    endcase
endmodule