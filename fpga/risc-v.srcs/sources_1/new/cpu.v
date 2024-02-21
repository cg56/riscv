`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Implements a single core RISC-V cpu.
// Supports integer multiplication and division.
// 
// Copyright Colm Gavin, 2024. All rights reserved.
// 
///////////////////////////////////////////////////////////////////////////////////

module cpu(
    input clock,                // 100Mhz clock
    input reset,                // Active high reset
    output reg[31:0] address,
    inout[31:0] data_io,
    output reg[1:0] width,      // 0 = 32 bits, 1 = 16 bits, 2 = 8 bits
    output reg write,           // Write cucle
    output reg read,            // Read cycle
    input complete,             // Read/write cycle is complete
    input interrupt_req,        // Timer interrupt request
    //output[7:0] led,            // For debug
    //output reg[31:0] sseg_data, // For debug
    input[1:0] debug            // For debug
    );
    
    reg[31:0] data_out;
    reg drive = 0;
    assign data_io = drive ? data_out : 32'bZ;
    
    reg[31:0] pc;               // Program counter
    reg[31:0] ireg;             // Instruction register
    reg[31:0] xreg[31:0];       // 32x registers
    reg[31:0] areg;             // ALU input register
    reg[31:0] breg;             // ALU input register
    reg[31:0] rreg;             // ALU result register
    reg[31:0] dreg;             // Data (load) input register
    reg[31:0] jaddr;            // Jump address register
    reg[1:0]  privilege;        // 3 = machine mode, 0 = user mode
    reg halt;

    reg[8:0] digit; //??
    reg[7:0] char; //??
    
    // Control & Status Registers
    
    reg[11:0]  csreg_addr;
    wire[31:0] csreg_rdata;
    reg[31:0]  csreg_wdata;
    reg        csreg_read;
    reg        csreg_write;
    reg        instr_fetch;
    reg[31:0] trap;
    reg       mret;
    wire[31:0] mtvec;     // Trap handler address
    wire[31:0] mepc;      // Trap return address
    wire[1:0]  mpp;
    wire interrupt;
    wire[31:0] trace_count;
    wire[31:0] trace_mask;

    csregs csregs1(
        .clock(clock),
        .reset(reset),
        .addr(csreg_addr),
        .rdata(csreg_rdata),
        .wdata(csreg_wdata),
        .read(csreg_read),
        .write(csreg_write),
        .complete(csreg_complete),
        .fetch(instr_fetch),
        .trap(trap),
        .mret(mret),
        .pc(pc),
        .privilege(privilege),
        .interrupt_req(interrupt_req),
        .mtvec(mtvec),
        .mepc(mepc),
        .mpp(mpp),
        .interrupt(interrupt),
        .trace_count(trace_count),
        .trace_mask(trace_mask)//,
        //.led(led[4:3])
        );
        
    // Instruction bit-patterns
    
    wire[4:0] rs1 = ireg[19:15];    // 1st source register
    wire[4:0] rs2 = ireg[24:20];    // 2nd source register
    wire[4:0] rd  = ireg[11:7];     // Destination register

    // The state machine
    
    reg[7:0]  state;
    reg[31:0] load;
    reg[5:0]  shift_count;
    reg[31:0] temp;
    reg[31:0] dividend;
    reg[31:0] divisor;
    reg       ndividend;
    reg       ndivisor;
    reg[31:0] quotient;
    reg[31:0] remainder;
    reg[31:0] memval;
    
    //assign led[7:0] = state;   //??
    
    localparam FETCH    = 0;
    localparam FETCH_W  = 1;
    localparam DECODE   = 2;
    localparam LOAD     = 3;
    localparam LOAD_W   = 4;
    localparam LOAD_END = 36;   //??
    localparam MISCMEM  = 5;
    localparam OPIMM    = 6;
    localparam AUIPC    = 7;
    localparam STORE    = 8;
    localparam STORE_W  = 9;
    localparam AMO      = 10;
    localparam AMO_W    = 11;
    localparam AMO_WR   = 12;
    localparam AMO_WR_W = 13;
    localparam OP       = 14;
    localparam MULT     = 15;
    localparam MULT1    = 16;
    localparam DIV      = 17;
    localparam DIV1     = 18;
    localparam SHIFT    = 19;
    localparam LUI      = 20;
    localparam BRANCH   = 21;
    localparam BRANCH2  = 35;   //??
    localparam JALR     = 22;
    localparam JAL      = 23;
    localparam SYSTEM   = 24;
    localparam CSRREAD  = 25;
    localparam CSRWRITE = 26;
    localparam CSRDONE  = 27;
    localparam DUMP     = 28;
    localparam DEBUG    = 29;
    localparam NEXT     = 30;
    localparam INTR     = 31;
    localparam TRAP     = 32;
    localparam MRET     = 33;
    localparam HALT     = 34;
    
    always @(posedge clock) begin
        if (reset) begin
            state     <= FETCH;
            read      <= 0;
            write     <= 0;
            drive     <= 0;
            address   <= 32'h00000000;
            data_out  <= 32'h0;
            width     <= 2'h2;
            pc        <= 32'h00000000;
            privilege <= 3;
            instr_fetch <= 0;
            halt      <= 0;
            trap      <= 0;
            mret      <= 0;
            //sseg_data <= 32'h00;
        end else begin
            case (state)

            FETCH: begin        // Begin instruction fetch         
                xreg[0] <= 0;   // Xreg[0] must always contain zero. Set it to zero just in case it was written to. Ideally we wouldn't allow it to be written, but it's easier to just reset it.
                
                if (complete == 0) begin    // Ensure previous memory access has completed before continuing
                    address <= pc;
                    width   <= 2'h2;
                    read        <= 1;   // Start the read cycle
                    instr_fetch <= 1;
                    state   <= FETCH_W;
                end
            end

            FETCH_W: begin      // Wait for the instruction to be available
                if (complete == 0) begin
                    //read        <= 1;   // Start the read cycle
                    //instr_fetch <= 1;
                end else begin          // Data is available
                    ireg  <= data_io;
                    read  <= 0;         // We're done
                    instr_fetch <= 0;
                    //state <= (pc[31] & debug[1]) ? DUMP : DECODE;
                    //state <= ((pc[31:24] == 8'h80) & (instr_count > {32'h0,trace_count}) /*& ((instr_count & {32'h0,trace_mask}) == 64'h0)*/ & debug[1]) ? DUMP : DECODE;
                    state <= DECODE;
                    //digit <= 0; //??
                end
            end
            
            /*DUMP: begin    // Wait for the previous read/write to be complete
                case (digit)
                `include "char_debug.vh"
                default: char <= 4'h0;
                endcase
                    
                if (complete == 0) begin
                   // state <= (digit > 393) ? DECODE : DEBUG;
                    state <= (digit > 402) ? DECODE : DEBUG;
                end
            end
            
            DEBUG: begin
                if (complete == 0) begin
                    if (drive == 0) begin
                        data_out <= (char < 10) ? (char+8'h30) : (char < 16) ? (char+8'h57) : (char&8'h7f);
                        address <= 32'h10000000;
                        width   <= 2'h0;
                        drive   <= 1;
                    end else
                        write   <= 1;
                end else begin
                    write <= 0;
                    drive <= 0;
                    digit <= digit+1;
                    state <= DUMP;
                end
            end*/
            
            // inst[4:2] 000  001      010      011      100    101      110            111
            // inst[6:5] (> 32b)
            // 00      LOAD   LOAD-FP  custom-0 MISC-MEM OP-IMM AUIPC    OP-IMM-32      48b
            // 01      STORE  STORE-FP custom-1 AMO      OP     LUI      OP-32          64b
            // 10      MADD   MSUB     NMSUB    NMADD    OP-FP  OP-V     custom-2/rv128 48b
            // 11      BRANCH JALR     reserved JAL      SYSTEM reserved custom-3/rv128 ? 80b

            DECODE: begin
                //sseg_data <= debug[0] ? pc : ireg;

                areg <= xreg[rs1];
                breg <= xreg[rs2];
                jaddr <= {{19{ireg[31]}},ireg[31],ireg[7],ireg[30:25],ireg[11:8],1'b0};
                
                case (ireg[6:0])
                7'b0000011: state <= LOAD;
                7'b0001111: state <= MISCMEM;
                7'b0010011: state <= OPIMM;
                7'b0010111: state <= AUIPC;
                7'b0100011: state <= STORE;
                7'b0101111: state <= AMO;
                7'b0110011: state <= OP;
                7'b0110111: state <= LUI;
                7'b1100011: state <= BRANCH;
                7'b1100111: state <= JALR;
                7'b1101111: state <= JAL;
                7'b1110011: state <= SYSTEM;
                default:    state <= HALT;
                endcase
            end

            LOAD: begin         // Load, I-type. Begin load from memory               
                if (complete == 0) begin    // Ensure previous memory access has completed before continuing
                    address <= areg + {{20{ireg[31]}}, ireg[31:20]};
                    width   <= ireg[13:12];    // 0 = 8 bits, 1 = 16 bits, 2 = 32 bits
                    read    <= 1;
                    state   <= LOAD_W;
                end
            end

            LOAD_W: begin       // Wait for the data to be available
                if (complete == 0) begin
                    //read <= 1;
                end else begin
                    dreg <= data_io;
                    read  <= 0;
                    state <= LOAD_END;
                end
            end
            
            LOAD_END: begin       // Wait for the data to be available
                state <= NEXT;
                
                case (ireg[14:12])
                3'b000: xreg[rd] <= {{24{dreg[7]}},dreg[7:0]};    // LB, Load byte (signed)
                3'b001: xreg[rd] <= {{16{dreg[15]}},dreg[15:0]};  // LH, Load half (signed)
                3'b010: xreg[rd] <= dreg[31:0];                   // LW, Load word
                3'b100: xreg[rd] <= {24'h0,dreg[7:0]};   // LBU, Load byte unsigned
                3'b101: xreg[rd] <= {16'h0,dreg[15:0]};  // LHU, Load half unsigned
                default: state <= HALT;
                endcase
            end

            MISCMEM: begin  // MISC-MEM, I-type
                case (ireg[14:12])
                3'b000: state <= NEXT;  // Fence, nothing to do because no cache
                3'b001: state <= NEXT;  // Fence.i, nothing to do because no cache
                default: state <= HALT;
                endcase
            end
            
            OPIMM: begin    // Immediate operation, I-type
                case (ireg[14:12])
                3'b000: begin   // Add
                    xreg[rd] <= areg + {{20{ireg[31]}}, ireg[31:20]};
                    state    <= NEXT;
                end
                3'b001: begin   // SLL Logical Shift Left
                    xreg[rd]    <= areg;
                    shift_count <= ireg[24:20];
                    state       <= SHIFT;
                end
                3'b010: begin   // SLTI, Set less than immediate (signed)
                    xreg[rd] <= ($signed(areg) < $signed(ireg[31:20])) ? 1 : 0;
                    state    <= NEXT;
                end
                3'b011: begin   // SLTIU, Set less than immediate unsigned
                    xreg[rd] <= (areg < ireg[31:20]) ? 1 : 0;
                    state    <= NEXT;
                end
                3'b100: begin   // Xor
                    xreg[rd] <= areg ^ {{20{ireg[31]}}, ireg[31:20]};
                    state    <= NEXT;
                end
                3'b101: begin   // SRL/SRA Logical/Arithmetic Shift Right
                    xreg[rd]    <= areg;
                    shift_count <= ireg[24:20];
                    state       <= SHIFT;
                end
                3'b110: begin   // Or
                    xreg[rd] <= areg | {{20{ireg[31]}}, ireg[31:20]};
                    state    <= NEXT;
                end
                3'b111: begin   // And
                    xreg[rd] <= areg & {{20{ireg[31]}}, ireg[31:20]};
                    state    <= NEXT;
                end
                endcase
            end

            AUIPC: begin
                xreg[rd] <= pc + {ireg[31:12],12'h000};
                state    <= NEXT;
            end
            
            STORE: begin    // Store, S-type. Begin write to memory
                if (complete == 0) begin
                    data_out <= breg;
                    address  <= areg + {{20{ireg[31]}},ireg[31:25],ireg[11:7]};
                    width    <= ireg[13:12];    // 0 = 8 bits, 1 = 16 bits, 2 = 32 bits, 3 = undefined
                    write <= 1;
                    drive    <= 1;
                    state    <= STORE_W;
                end
            end
            
            STORE_W: begin    // Store, S-type
                if (complete == 0) begin
                    //write <= 1;
                end else begin
                    write <= 0;
                    drive <= 0;
                    state <= NEXT;
                end
            end

            AMO: begin  // Atomic memory operation, R-type
                // First do the read
                if (complete == 0) begin    // Wait for previous memory access to complete
                    address <= areg;
                    temp    <= breg;        // Do we need temp ??
                    width   <= 2'h2;
                    state   <= AMO_W;
                end
            end
            
            AMO_W: begin  // Atomic memory operation, R-type
                // First do the read
                if (complete == 0) begin
                    read <= 1;
                end else begin
                    xreg[rd] <= data_io;
                    read     <= 0;
                    
                    if (ireg[31:27] == 5'b00010)
                        state <= NEXT;  // No need to write anything
                    else
                        state <= AMO_WR;
                end
            end
            
            AMO_WR: begin  // Next do the op and the store
                if (complete == 0) begin
                    case (ireg[31:27])        
                    5'b00000: data_out <= xreg[rd] + temp;  // Add
                    5'b00001: data_out <= temp;             // Swap
                    5'b00011: begin                         // SC
                        data_out <= temp;
                        xreg[rd] <= 0;  // Always succeeds!
                        end
                    5'b00100: data_out <= xreg[rd] ^ temp;  // XOR
                    5'b01000: data_out <= xreg[rd] | temp;  // OR
                    5'b01100: data_out <= xreg[rd] & temp;  // AND
                    5'b10000: if (xreg[rd] < temp)          // Min
                        data_out <= xreg[rd];
                    else
                        data_out <= temp;
                    5'b10100: if (xreg[rd] < temp)          // Max
                        data_out <= temp;
                    else
                        data_out <= xreg[rd];
                    5'b11000: if ($signed(xreg[rd]) < $signed(temp))    // Minu
                        data_out <= xreg[rd];
                    else
                        data_out <= temp;
                    5'b11100: if ($signed(xreg[rd]) < $signed(temp))    // Maxu
                        data_out <= temp;
                    else
                        data_out <= xreg[rd];
                    default: state <= HALT;
                    endcase
                    drive <= 1;
                    state <= AMO_WR_W;
                end
            end
             
            AMO_WR_W: begin
                if (complete == 0) begin
                    write <= 1;
                end else begin
                    write <= 0;
                    drive <= 0;
                    state <= NEXT;
                end
            end
           
            OP: begin   // Operation, R-type
                if (ireg[25] == 0) begin
                    case (ireg[14:12])
                    3'b000: begin   // Add/Sub
                        if (ireg[30] == 0)
                            xreg[rd] <= areg + breg;
                        else
                            xreg[rd] <= areg - breg;
                        state <= NEXT;
                    end
                    3'b001: begin   // SLL, Shift Left Logical
                        xreg[rd]    <= areg;
                        shift_count <= breg[4:0];
                        state       <= SHIFT;
                    end
                    3'b010: begin   // SLT, Set less than (signed)
                        xreg[rd] <= ($signed(areg) < $signed(breg)) ? 1 : 0;
                        state    <= NEXT;
                    end
                    3'b011: begin   // SLTU, Set less than unsigned
                        xreg[rd] <= (areg < breg) ? 1 : 0;
                        state    <= NEXT;
                    end
                    3'b100: begin   // Xor
                        xreg[rd] <= areg ^ breg;
                        state    <= NEXT;
                    end
                    3'b101: begin   // SRL/SRA Logical/Arithmetic Shift Right
                        xreg[rd]    <= areg;
                        shift_count <= breg[4:0];
                        state       <= SHIFT;
                    end                            
                    3'b110: begin   // Or
                        xreg[rd] <= areg | breg;
                        state    <= NEXT;
                    end
                    3'b111: begin   // And
                        xreg[rd] <= areg & breg;
                        state    <= NEXT;
                    end
                    endcase
                end else begin
                    case (ireg[14:12])
                    3'b000: begin           //  MUL multiply
                        dividend  <= areg;  // Not very good register names!
                        divisor   <= breg;
                        ndividend <= 0;
                        ndivisor  <= 0;
                        quotient  <= 0;     // High word
                        remainder <= 0;     // Low word
                        shift_count <= 32;
                        state       <= MULT;
                    end
                    3'b001: begin   // MULH, multiply high word (signed x signed)
                        dividend  <= areg[31] ? 0-$signed(areg) : areg;
                        divisor   <= breg[31] ? 0-$signed(breg) : breg;
                        ndividend <= areg[31];
                        ndivisor  <= breg[31];
                        quotient  <= 0;
                        remainder <= 0;
                        shift_count <= 32;
                        state       <= MULT;
                    end
                    3'b010: begin   // MULHSU, multiply high word (signed x unsigned)
                        dividend  <= areg[31] ? 0-$signed(areg) : areg;
                        divisor   <= breg;
                        ndividend <= areg[31];
                        ndivisor  <= 0;
                        quotient  <= 0;
                        remainder <= 0;
                        shift_count <= 32;
                        state       <= MULT;
                    end
                    3'b011: begin   // MULHU, multiply high word (unsigned x unsigned)
                        dividend  <= areg;
                        divisor   <= breg;
                        ndividend <= 0;
                        ndivisor  <= 0;
                        quotient  <= 0;     // High word
                        remainder <= 0;     // Low word
                        shift_count <= 32;
                        state       <= MULT;
                    end
                    3'b100: begin   // DIV, signed division
                        if (breg == 0) begin
                            xreg[rd] <= ~0;
                            state    <= NEXT;
                        end else if ((areg == 32'h80_00_00_00) && (breg == ~0)) begin
                            xreg[rd] <= areg;
                            state    <= NEXT;
                        end else begin
                            dividend  <= areg[31] ? 0-$signed(areg) : areg;
                            divisor   <= breg[31] ? 0-$signed(areg) : breg;
                            ndividend <= areg[31];
                            ndivisor  <= breg[31];
                            quotient  <= 0;
                            remainder <= 0;
                            shift_count <= 32;
                            state       <= DIV;
                        end
                    end
                    3'b101: begin       // DIVU, unsigned division
                        if (breg == 0) begin
                            xreg[rd] <= ~0;
                            state    <= NEXT;
                        end else begin
                            dividend  <= areg;  // Do we need these divident and divisor registers ??
                            divisor   <= breg;
                            ndividend <= 0;
                            ndivisor  <= 0;
                            quotient  <= 0;
                            remainder <= 0;
                            shift_count <= 32;
                            state       <= DIV;
                        end
                    end
                    3'b110: begin   // REM, signed remainder (modulus)
                        if (breg == 0) begin
                            xreg[rd] <= areg;
                            state    <= NEXT;
                        end else if ((areg == 32'h80_00_00_00) && (breg == ~0)) begin
                            xreg[rd] <= 0;
                            state    <= NEXT;
                        end else begin
                            dividend  <= areg[31] ? 0-$signed(areg) : areg;
                            divisor   <= breg[31] ? 0-$signed(breg) : breg;
                            ndividend <= areg[31];
                            ndivisor  <= breg[31];
                            quotient  <= 0;
                            remainder <= 0;
                            shift_count <= 32;
                            state       <= DIV;
                        end
                    end
                    3'b111: begin   // REMU, unsigned remainder (modulus)
                        if (breg == 0) begin
                            xreg[rd] <= areg;
                            state    <= NEXT;
                        end else begin
                            dividend  <= areg;
                            divisor   <= breg;
                            ndividend <= 0;
                            ndivisor  <= 0;
                            quotient  <= 0;
                            remainder <= 0;
                            shift_count <= 32;
                            state       <= DIV;
                        end
                    end
                    endcase
                end
            end
            
            SHIFT: begin
                case (ireg[14:12])
                3'b001: begin       // SLL, Shift Left Logical
                    if (shift_count == 0)
                        state <= NEXT;
                    else begin
                        xreg[rd] <= {xreg[rd][30:0],1'b0};
                        shift_count <= shift_count - 1;
                    end
                end
                3'b101: begin       // SRL/SRA Logical/Arithmetic Shift Right
                    if (shift_count == 0)
                        state <= NEXT;
                    else begin
                        if (ireg[30] == 0)
                            xreg[rd] <= {1'b0,xreg[rd][31:1]};  // Logical shift right
                        else
                            xreg[rd] <= {xreg[rd][31],xreg[rd][31:1]};  // Arithmetic shift right
                            
                        shift_count <= shift_count - 1;
                    end
                end
                default: state <= NEXT;
                endcase
            end
            
            MULT: begin
                if (shift_count == 0) begin     // Should do if (shift_count != 0) ?? 
                    case (ireg[14:12])
                    3'b000:         // MUL multiply (low 32-bit result)
                        xreg[rd] <= remainder;
                    3'b001,         // MULH, multiply high word (signed x signed)
                    3'b010:         // MULHSU, multiply high word (signed x unsigned)
                        xreg[rd] = (ndividend ^ ndivisor) ? 0-quotient : quotient;
                    3'b011:         // MULHU, multiply high word (unsigned x unsigned)
                        xreg[rd] <= quotient;
                    endcase
                    state <= NEXT;
                end else begin
                    {quotient[31:0],remainder[31:0]} <= {1'b0,quotient[31:0],remainder[31:1]} + (divisor[0] ? {1'b0,dividend[31:0],31'b0} : 0);
                    divisor[31:0] <= {1'b0,divisor[31:1]};
                    shift_count <= shift_count - 1;
                end
            end

            DIV: begin
                if (shift_count == 0) begin
                    case (ireg[14:12])
                    3'b100:         // DIV, signed division
                        xreg[rd] = (ndividend ^ ndivisor) ? 0-quotient : quotient;
                    3'b101:         // DIVU, unsigned division
                        xreg[rd] = quotient;
                    3'b110:         // REM, signed remainder (modulus)
                        xreg[rd] = ndividend ? 0-remainder : remainder;
                    3'b111:         // REMU, unsigned remainder (modulus)
                        xreg[rd] = remainder;
                    endcase
                    state <= NEXT;
                end else begin
                    remainder[31:0] <= {remainder[30:0],dividend[31]};
                    dividend[31:0]  <= {dividend[30:0], 1'b0};
                    quotient[31:0]  <= {quotient[30:0], 1'b0};
                    state <= DIV1;
                end
            end
            
            DIV1: begin
                if (remainder >= divisor) begin
                    quotient[0] <= 1'b1;
                    remainder   <= remainder - divisor;
                end
                shift_count <= shift_count - 1;
                state <= DIV;
            end
            
            LUI: begin      // Load Upper Immediate, U-type
                xreg[rd] <= {ireg[31:12],12'h000};
                state    <= NEXT;
            end
            
            BRANCH: begin       // Branch, B-type
                case (ireg[14:12])
                3'b000: begin       // BEQ, Branch if equal
                    if (areg == breg)
                        state <= BRANCH2;
                    else
                        state <= NEXT;
                end
                3'b001: begin       // BNE, Branch if not equal
                    if (areg != breg)
                        state <= BRANCH2;
                    else
                        state <= NEXT;
                end
                3'b100: begin       // BLT, Branch if Less Than (signed)
                    if ($signed(areg) < $signed(breg))
                        state <= BRANCH2;
                    else
                        state <= NEXT;
                end
                3'b101: begin       // BGE, Branch if Greater or Equal (signed)
                    if ($signed(areg) >= $signed(breg))
                        state <= BRANCH2;
                    else
                        state <= NEXT;
                end
                3'b110: begin       // BLTU, Branch if Less Than (unsigned)
                    if (areg < breg)
                        state <= BRANCH2;
                    else
                        state <= NEXT;
                end
                3'b111: begin       // BGEU, Branch if Greater or Equal (unsigned)
                    if (areg >= breg)
                        state <= BRANCH2;
                    else
                        state <= NEXT;
                end
                default: state <= HALT;
                endcase
            end
            
            BRANCH2: begin       // Branch, B-type
                pc <= pc + jaddr;
                state <= INTR;
            end
            
            JALR: begin  // Jump and link, I-type
                xreg[rd] <= pc + 4;
                pc       <= (areg + {{20{ireg[31]}},ireg[31:20]}) & 32'hfffffffe;    // Set LSB to zero
                state    <= INTR;
            end
            
            JAL: begin  // Jump and link, J-type
                xreg[rd] <= pc + 4;
                pc       <= pc + {{11{ireg[31]}},ireg[31],ireg[19:12],ireg[20],ireg[30:21],1'b0};   // Why did they encode the immediate value so messily!
                state    <= INTR;
            end
            
            SYSTEM: begin  // System
                case (ireg[14:12])
                3'b000: begin
                    case (ireg[31:20])
                    12'b000000000000: begin     // ECALL
                        if (privilege == 0)
                            trap  <= 8;
                        else
                            trap  <= 11;
                        state <= TRAP;
                    end
                    12'b000000000001: begin     // EBREAK
                        trap  <= 3;
                        state <= TRAP;
                    end
                    12'b000100000101: begin     // WFI              
                        if (interrupt_req)
                            state <= NEXT;
                        else
                            state <= FETCH;     // Don't increment PC so we keep busy looping on the WFI instruction
                    end
                    12'b001100000010: begin     // MRET
                        mret  <= 1;
                        state <= MRET;
                    end
                    default: state <= HALT;
                    endcase
                end
                default: begin
                    csreg_addr <= ireg[31:20];
                    csreg_read <= 1;
                    state <= CSRREAD;
                end
                endcase
            end
            
            CSRREAD: begin
                if (csreg_complete) begin
                    temp       <= csreg_rdata;
                    csreg_read <= 0;
                    state <= CSRWRITE;
                end
            end

            CSRWRITE: begin
                if (csreg_complete == 0) begin
                    csreg_write <= 1;

                    case (ireg[14:12])
                    3'b001: begin   // CSRRW, Atomic read/write CSR
                        csreg_wdata <= areg;
                        state <= CSRDONE;
                    end
                    3'b010: begin   // CSRRS, Atomic read and set bits in CSR
                        csreg_wdata <= temp | areg;
                        state <= CSRDONE;
                    end
                    3'b011: begin   // CSRRC, Atomic read and clear bits in CSR
                        csreg_wdata <= temp & ~areg;
                        state <= CSRDONE;
                    end
                    3'b101: begin   // CSRRWI, Atomic read/write CSR immediate
                        csreg_wdata <= rs1;
                        state <= CSRDONE;
                    end
                    3'b110: begin   // CSRRSI, Atomic read and set bits in CSR immediate
                        csreg_wdata <= temp | rs1;
                        state <= CSRDONE;
                    end
                    3'b111: begin   // CSRRCI, Atomic read and clear bits in CSR immediate
                        csreg_wdata <= temp & ~rs1;
                        state <= CSRDONE;
                    end
                    default: state <= HALT;
                    endcase
                end
            end
        
            CSRDONE: begin
                xreg[rd] <= temp;
                csreg_write <= 0;
                state <= NEXT;
            end
            
            NEXT: begin
                pc    <= pc + 4;
                state <= INTR;
            end
        
            INTR: begin
                if (interrupt) begin
                    trap  <= 32'h80000007;
                    state <= TRAP;      // PC is the instruction we will return to
                end else
                    state <= FETCH;
            end

            TRAP: begin
                trap      <= 0;
                privilege <= 3;
                pc        <= mtvec;
                state     <= FETCH;
            end
            
            MRET: begin
                mret      <= 0;
                privilege <= mpp;
                pc        <= mepc;
                state <= FETCH;
            end

            //if (trap & 0x80000000)  // It's an interrupt
            //    csreg[mepc] = pc;   // PC is where we will return to
            //else
            //{
            //    csreg[mepc] = pc - 4;   // PC is the address of the instruction which caused the trap
            //    privilege = 3;  // Switch to machine mode. Only on traps, not interrupts??
            //}

            //pc    <= csreg[MTVEC];
            //state <= FETCH;
            
            HALT: begin
                halt <= 1;
            end
            
            default: state <= HALT;
            endcase
        end
    end
endmodule
