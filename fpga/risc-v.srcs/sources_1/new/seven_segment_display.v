`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/14/2023 10:11:28 PM
// Design Name: 
// Module Name: seven_segment_display
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


module seven_segment_display(
    input clk,
    input rst_n,
    input [31:0] value,
    output reg [6:0] segments,
    output reg [7:0] anodes
    );
    
    reg[31:0] segment_counter;
    reg[3:0] digit;
    
    always @(posedge clk) begin
        if (!rst_n) begin
            anodes <= 8'b1111_1110;
            segment_counter <= 0;
        end else begin
            if (segment_counter == 100000) begin
                segment_counter <= 0;
                anodes <= {anodes[6:0], anodes[7]};
            end else begin
                segment_counter <= segment_counter +1;
            end
        end
    
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
    
    always @(*)
    case (digit)
        0: segments = 7'b1000000;
        1: segments = 7'b1111001;
        2: segments = 7'b0100100;
        3: segments = 7'b0110000;
        4: segments = 7'b0011001;
        5: segments = 7'b0010010;
        6: segments = 7'b0000010;
        7: segments = 7'b1111000;
        8: segments = 7'b0000000;
        9: segments = 7'b0010000;
        'hA: segments = 7'b0001000; 
        'hB: segments = 7'b0000011; 
        'hC: segments = 7'b1000110;
        'hD: segments = 7'b0100001;
        'hE: segments = 7'b0000110;
        'hF: segments = 7'b0001110;
    endcase
endmodule