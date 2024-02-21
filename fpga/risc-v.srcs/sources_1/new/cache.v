`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Cache between the CPU and the RAM.
//
// Copyright (c) Colm Gavin, 2024
// 
//////////////////////////////////////////////////////////////////////////////////

module cache(
    input clock,            // 100MHz clock
    input reset,            // Active high reset
    input[26:0] address,    // 128Mbyte address space
    inout[31:0] data_io,    // Data in for write, data out for read
    input[1:0] width,       // 0 = 8 bits, 1 = 16 bits, 2 = 32 bits 
    input enable,           // Enable the memory for reading or writing
    input read,             // Request to read from the memory
    input write,            // Request to write to the memory
    output reg complete,    // The read or write request is complete
    
    // Interface to the RAM module
    output reg[22:0] ram_address,   // 8M x 128-bit words 
    output reg[127:0] ram_data_in,  // Data in for write
    input[127:0] ram_data_out,      // Data out for read
    output reg ram_read,            // Request to read from the memory
    output reg ram_write,           // Request to write to the memory
    input ram_complete              // The read or write request is complete
    );
    
    // Tri-state bus
    
    reg[31:0] data_out;
    assign data_io = (read & enable) ? data_out : 32'bZ;
    
    // The cache, 8 rows of 128 bits
    
    reg valid[7:0];
    reg dirty[7:0];
    reg[19:0] tag[7:0];
    reg[127:0] cache[7:0];
    integer index;

    // The state machine
    
    reg[2:0] state;

    localparam IDLE  = 3'h0;
    localparam FETCH = 3'h1;
    localparam FLUSH = 3'h2;
    localparam READ  = 3'h3;
    localparam WRITE = 3'h4;
    localparam END   = 3'h5;
    
    // The logic
    
    always @(posedge clock) begin
        if (reset) begin
            data_out   <= 32'h0;
            complete   <= 0;
            ram_read   <= 0;
            ram_write  <= 0;
            
            for (index = 0; index < 8; index = index+1) begin
                valid[index] <= 0;
                dirty[index] <= 0;
            end

            state <= IDLE;

        end else begin
            case (state)
            
            IDLE: begin
                // Wait for everything to be ready
                if ((read | write) & enable & !ram_complete) begin
                    if ((valid[address[6:4]]) & (tag[address[6:4]] == address[26:7])) begin
                        if (read) 
                            state <= READ;
                        else
                            state <= WRITE;
                    end else if (valid[address[6:4]] & dirty[address[6:4]]) begin
                        // Begin flush operation
                        ram_address <= {tag[address[6:4]],address[6:4]};
                        ram_data_in <= cache[address[6:4]];
                        ram_write   <= 1;
                        state       <= FLUSH;
                    end else begin
                        // Begin fetch operation
                        ram_address <= address[26:4];
                        ram_read    <= 1;
                        state       <= FETCH;
                    end
                end
            end
                        
            FLUSH: begin
                if (ram_complete) begin
                    dirty[address[6:4]] <= 0;
                    ram_write  <= 0;
                    state      <= IDLE;
                end
            end
            
            FETCH: begin
                if (ram_complete) begin
                    valid[address[6:4]] <= 1;
                    dirty[address[6:4]] <= 0;
                    tag[address[6:4]]   <= address[26:7];
                    cache[address[6:4]] <= ram_data_out;

                    ram_read <= 0;
                    state    <= IDLE;
                end
            end
                        
            READ: begin
                case (address[3:0])
                0: data_out = cache[address[6:4]][31:0];
                1: data_out = cache[address[6:4]][39:8];
                2: data_out = cache[address[6:4]][47:16];
                3: data_out = cache[address[6:4]][55:24];
                4: data_out = cache[address[6:4]][63:32];
                5: data_out = cache[address[6:4]][71:40];
                6: data_out = cache[address[6:4]][79:48];
                7: data_out = cache[address[6:4]][87:56];
                8: data_out = cache[address[6:4]][95:64];
                9: data_out = cache[address[6:4]][103:72];
                10: data_out = cache[address[6:4]][111:80];
                11: data_out = cache[address[6:4]][119:88];
                12: data_out = cache[address[6:4]][127:96];
                13: data_out = { 8'b0, cache[address[6:4]][127:104]};
                14: data_out = {16'b0, cache[address[6:4]][127:112]};
                15: data_out = {24'b0, cache[address[6:4]][127:120]};
                endcase

                complete <= 1;
                state    <= END;
            end

            WRITE: begin
                case (width)
                3,
                2: begin
                    case (address[3:2])
                    0: cache[address[6:4]][31:0]   <= data_io;
                    1: cache[address[6:4]][63:32]  <= data_io;
                    2: cache[address[6:4]][95:64]  <= data_io;
                    3: cache[address[6:4]][127:96] <= data_io;
                    endcase
                end
                1: begin
                    case (address[3:1])
                    0: cache[address[6:4]][15:0]    <= data_io[15:0];
                    1: cache[address[6:4]][31:16]   <= data_io[15:0];
                    2: cache[address[6:4]][47:32]   <= data_io[15:0];
                    3: cache[address[6:4]][63:48]   <= data_io[15:0];
                    4: cache[address[6:4]][79:64]   <= data_io[15:0];
                    5: cache[address[6:4]][95:80]   <= data_io[15:0];
                    6: cache[address[6:4]][111:96]  <= data_io[15:0];
                    7: cache[address[6:4]][127:112] <= data_io[15:0];
                    endcase
                end
                0: begin
                    case (address[3:0])
                    0: cache[address[6:4]][7:0]      <= data_io[7:0];
                    1: cache[address[6:4]][15:8]     <= data_io[7:0];
                    2: cache[address[6:4]][23:16]    <= data_io[7:0];
                    3: cache[address[6:4]][31:24]    <= data_io[7:0];
                    4: cache[address[6:4]][39:32]    <= data_io[7:0];
                    5: cache[address[6:4]][47:40]    <= data_io[7:0];
                    6: cache[address[6:4]][55:48]    <= data_io[7:0];
                    7: cache[address[6:4]][63:56]    <= data_io[7:0];
                    8: cache[address[6:4]][71:64]    <= data_io[7:0];
                    9: cache[address[6:4]][79:72]    <= data_io[7:0];
                    10: cache[address[6:4]][87:80]   <= data_io[7:0];
                    11: cache[address[6:4]][95:88]   <= data_io[7:0];
                    12: cache[address[6:4]][103:96]  <= data_io[7:0];
                    13: cache[address[6:4]][111:104] <= data_io[7:0];
                    14: cache[address[6:4]][119:112] <= data_io[7:0];
                    15: cache[address[6:4]][127:120] <= data_io[7:0];
                    endcase
                end
                endcase
                
                dirty[address[6:4]] <= 1;
                complete <= 1;
                state    <= END;
            end

            END: begin
                if (!read & !write) begin
                    complete <= 0;
                    
                    if (!ram_complete)
                        state <= IDLE;
                end
            end
            
            endcase
        end
    end
endmodule
