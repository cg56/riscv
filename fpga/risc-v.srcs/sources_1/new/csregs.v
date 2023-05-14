`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/13/2023 09:44:24 PM
// Design Name: 
// Module Name: csregs
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

// CSREG locations
    
localparam MSTATUS = 12'h300;   // Machine status register
localparam MIE     = 12'h304;   // Machine interrupt enable register
localparam MTVEC   = 12'h305;   // Machine trap-handler base address

// Machine trap handling

localparam MSCRATCH = 12'h340;  // Scratch register for machine trap handlers
localparam MEPC     = 12'h341;  // Machine exception program counter
localparam MCAUSE   = 12'h342;  // Machine trap cause
localparam MTVAL    = 12'h343;  // Machine bad address or instruction
localparam MIP      = 12'h344;  // Machine interrupt pending

// Extra for debug

localparam TSELECT = 12'h7a0;   // Debug/trace trigger register
localparam TDATA1  = 12'h7a1;

module csregs(
    input clock,
    input reset,
    input [11:0] addr,
    output reg[31:0] rdata,
    input [31:0] wdata,
    input read,             // Request to read from the CS registers
    input write,            // Request to write to the CS registers
    output reg complete,    // The read or write request is complete
    input [31:0] trap,
    input mret,
    input [31:0] pc,
    input [1:0]  privilege,
    input interrupt_req,
    output reg[31:0] mtvec,
    output reg[31:0] mepc,
    output[1:0] mpp,
    output interrupt,
    output reg[31:0] trace_count,   //??
    output reg[31:0] trace_mask,    //??
    output[1:0] led                 //??
    );

    reg[31:0] mstatus;
    reg[31:0] mie;
    reg[31:0] mscratch;
    reg[31:0] mcause;
    reg[31:0] mtval;
    reg[31:0] mip;
    
    assign mpp = mstatus[12:11];    // Machine privilege?
    assign interrupt = interrupt_req & mie[7] & mstatus[3];
    assign led[0] = mie[7];
    assign led[1] = mstatus[3];

    always @(posedge clock) begin
        if (reset) begin
            rdata    <= 32'h0;
            complete <= 0;
                      
            mstatus  <= 0;
            mie      <= 0;
            mtvec    <= 0;
            mscratch <= 0;
            mepc     <= 0;
            mcause   <= 0;
            mtval    <= 0;
            mip      <= 0;
        end else if (trap != 0) begin
            mcause  <= trap;
            mstatus <= {19'b0,privilege[1:0],3'b000,mstatus[3],7'b0};
            mepc    <= pc;      // PC is the address of the instruction which caused the trap
            if ((trap == 8) || (trap == 11))
                mtval <= pc;
            else
                mtval <= 0;
        end else if (mret) begin
            mstatus <= {19'b0,privilege[1:0],7'b0001000,mstatus[7],3'b0};
        end else if (read) begin
            case (addr)
            MSTATUS:  rdata <= mstatus;
            MIE:      rdata <= mie;
            MTVEC:    rdata <= mtvec;
            MSCRATCH: rdata <= mscratch;
            MEPC:     rdata <= mepc;
            MCAUSE:   rdata <= mcause;
            MTVAL:    rdata <= mtval;
            MIP:      rdata <= mip | (interrupt_req << 7);   // Don't use bit shift! ??
            default:  rdata <= 32'h0;
            endcase
            complete <= 1;
        end else if (write) begin
            case (addr)
            MSTATUS:  mstatus  <= wdata;
            MIE:      mie      <= wdata;
            MTVEC:    mtvec    <= wdata;
            MSCRATCH: mscratch <= wdata;
            MEPC:     mepc     <= wdata;
            MCAUSE:   mcause   <= wdata;
            MTVAL:    mtval    <= wdata;
            MIP:      mip      <= wdata;
            TSELECT:  trace_count <= wdata;
            TDATA1:   trace_mask  <= wdata;
            endcase
            complete <= 1;
        end else begin
            complete <= 0;
        end
     
            /*
            csreg[mcause] = trap;
            csreg[mstatus] = ((csreg[mstatus] & 0x8) << 4) | ((privilege & 0x3) << 11);

            if (trap & 0x80000000)  // It's an interrupt
                csreg[mepc] = pc;   // PC is where we will return to
            else
            {
                csreg[mepc] = pc - 4;   // PC is the address of the instruction which caused the trap
                privilege = 3;  // Switch to machine mode. Only on traps, not interrupts??
            }

            pc = csreg[mtvec];
            */
    end
endmodule
