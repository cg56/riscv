`ifdef DOIT
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/05/2023 09:35:15 PM
// Design Name: 
// Module Name: alu
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

module alu(
    input[31:0] areg,
    input[31:0] breg,
    input[2:0]  op,
    input sub,
    output reg[31:0] rreg
    );
    
always @(posedge clock) begin
    case (op)
    3'b000: rreg <= areg + breg;
        end
        3'b001: begin   // SLL Logical Shift Left
            xreg[rd]    <= xreg[rs1];
            shift_count <= ireg[24:20];
            state       <= SHIFT;
        end
        3'b010: begin   // SLTI, Set less than immediate (signed)
            xreg[rd] <= ($signed(xreg[rs1]) < $signed(ireg[31:20])) ? 1 : 0;
            state    <= NEXT;
        end
        3'b011: begin   // SLTIU, Set less than immediate unsigned
            xreg[rd] <= (xreg[rs1] < ireg[31:20]) ? 1 : 0;
            state    <= NEXT;
        end
        3'b100: begin   // Xor
            xreg[rd] <= xreg[rs1] ^ {{20{ireg[31]}}, ireg[31:20]};
            state    <= NEXT;
        end
        3'b101: begin   // SRL/SRA Logical/Arithmetic Shift Right
            xreg[rd]    <= xreg[rs1];
            shift_count <= ireg[24:20];
            state       <= SHIFT;
        end
        3'b110: begin   // Or
            xreg[rd] <= xreg[rs1] | {{20{ireg[31]}}, ireg[31:20]};
            state    <= NEXT;
        end
        3'b111: begin   // And
            xreg[rd] <= xreg[rs1] & {{20{ireg[31]}}, ireg[31:20]};
            state    <= NEXT;
        endcase 
end
endmodule
`endif