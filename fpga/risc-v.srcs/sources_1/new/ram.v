`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Interface between the CPU bus and the DDR memory signals. Most of the logic
// is taken care of by the mig module, but we need a state machine to drive it.
//
// The memory reads and writes two 64-bit packets (128-bits total), so the
// appropriate bits of a packet are accessed for 8, 16, or 32 bytes.
//
// CPU reads and writes must be alligned. E.g: 32-bit words can only be accessed
// on 4 byte boundaries.
//
// Copyright (c) Colm Gavin, 2024
// 
//////////////////////////////////////////////////////////////////////////////////

module ram(
    input clock,            // 200MHz clock for the mig
    input reset,            // Active high reset
    input[22:0] address,    // 128Kbyte address space. 128-bit (16-byte) rows
    input[127:0] data_in,       // Data in for write
    output reg[127:0] data_out, // Data out for read
    input read,             // Request to read from the memory
    input write,            // Request to write to the memory
    output reg complete,    // The read or write request is complete
    
    // DDR memory interface. I've no idea what most of these are, but we don't
    // really care. The mig module takes care of them.
    
    inout [15:0]    ddr2_dq,
    inout [1:0]     ddr2_dqs_n,
    inout [1:0]     ddr2_dqs_p,
    output [12:0]   ddr2_addr,
    output [2:0]    ddr2_ba,
    output          ddr2_ras_n,
    output          ddr2_cas_n,
    output          ddr2_we_n,
    output [0:0]    ddr2_ck_p,
    output [0:0]    ddr2_ck_n,
    output [0:0]    ddr2_cke,
    output [0:0]    ddr2_cs_n,
    output [1:0]    ddr2_dm,
    output [0:0]    ddr2_odt
    );

    // Signals to talk to the mig module
    
    reg[2:0]    command;        // Read or write command to send to the mig
    reg         mem_enable;     // Strobe for the inputs to the mig
    reg         write_enable;
    reg[63:0]   write_data;
    reg         write_end;      // Signals last write clock cycle
    wire[63:0]  read_data;
    wire        read_end;
    wire        read_valid;
    
    // Signals from the mig module
    
    wire        memory_ready;   // Memory is ready to accept a command
    wire        write_ready;    // Write queue has space
    wire        ui_clk;         // Clock from the mig to drive the state machine
    wire        ui_reset;       // When low we can use the mig
  
    // Instantiate the mig to interface with the DDR memory
    // See: https://docs.xilinx.com/v/u/1.4-English/ug586_7Series_MIS
    
    mig mig1(
        .ddr2_dq(ddr2_dq),
        .ddr2_dqs_n(ddr2_dqs_n),
        .ddr2_dqs_p(ddr2_dqs_p),
        .ddr2_addr(ddr2_addr),
        .ddr2_ba(ddr2_ba),
        .ddr2_ras_n(ddr2_ras_n),
        .ddr2_cas_n(ddr2_cas_n),
        .ddr2_we_n(ddr2_we_n),
        .ddr2_ck_p(ddr2_ck_p),
        .ddr2_ck_n(ddr2_ck_n),
        .ddr2_cke(ddr2_cke),
        .ddr2_cs_n(ddr2_cs_n),
        .ddr2_dm(ddr2_dm),
        .ddr2_odt(ddr2_odt),

        .sys_clk_i(clock),
        .app_addr({address[22:0],4'b0}),   // The lest-significant address bit selects the byte within a short
        .app_cmd(command),
        .app_en(mem_enable),
        .app_wdf_data(write_data),
        .app_wdf_end(write_end),
        .app_wdf_mask(8'h00),               // Always write all 128 bits
        .app_wdf_wren(write_enable),
        .app_rd_data(read_data),
        .app_rd_data_end(read_end),
        .app_rd_data_valid(read_valid),
        .app_rdy(memory_ready),
        .app_wdf_rdy(write_ready),
        .app_sr_req(1'b0),  // Reserved input, should be tied to zero
        .app_ref_req(1'b0), // Request a DRAM refresh
        .app_zq_req(1'b0),  // Request calibration
        .app_sr_active(),   // Reserved output
        .app_ref_ack(),     // Refresh ack
        .app_zq_ack(),      // Calibration ack
        .ui_clk(ui_clk),
        .ui_clk_sync_rst(ui_reset), // is ui_reset a good name for this signal??
        .init_calib_complete(),     // Internal initialization and calibration is complete
        .sys_rst(~reset)    // System reset
        );

    // The state machine
    
    reg[2:0] state;

    localparam IDLE        = 3'h0;
    localparam START_READ  = 3'h1;
    localparam WAIT_READ   = 3'h2;
    localparam START_WRITE = 3'h3;
    localparam WRITE_HIGH  = 3'h4;
    localparam WRITE_LOW   = 3'h5;
    localparam FINISHED    = 3'h6;
    
    always @(posedge ui_clk) begin
        if (ui_reset) begin
            data_out <= 32'h0;
            state    <= IDLE;
            complete <= 0;
            
            command      <= 3'h0;
            write_data   <= 64'h0;
            write_enable <= 0;
            write_end    <= 0;
            mem_enable   <= 0;      
        end else begin
            case (state)
            
            IDLE: begin
                if (read) begin
                    mem_enable <= 1;
                    command    <= 3'h1;
                    state      <= START_READ;
                end else if (write) begin
                    mem_enable <= 1;
                    command    <= 3'h0;
                    write_end  <= 0;
                    state      <= START_WRITE;
                end
            end

            START_READ: begin
                // Wait until the command has been accepted
                if (memory_ready) begin
                    mem_enable <= 0;
                    state      <= WAIT_READ;
                end
            end
           
            WAIT_READ: begin
                // Wait until data is ready
                if (read_valid) begin
                    if (read_end) begin
                        data_out[127:64] <= read_data;  // Read from the 2nd packet
                        complete <= 1;
                        state    <= FINISHED;
                    end else
                        data_out[63:0] <= read_data;    // Read from the 1st packet
                end
            end
                       
            START_WRITE: begin
                // Wait until the command has been accepted
                if (memory_ready) begin
                    mem_enable <= 0;
                    state      <= WRITE_LOW;
                end
            end

            WRITE_LOW: begin   // Send the first packet
                // Wait for write queue to have space
                if (write_ready) begin
                    write_data   <= data_in[63:0];
                    write_enable <= 1;
                    state        <= WRITE_HIGH;
                end
            end

            WRITE_HIGH: begin    // Send the 2nd packet
                // Wait for Write Data queue to have space
                if (write_ready) begin 
                    write_data <= data_in[127:64];
                    write_end  <= 1;
                    complete   <= 1;
                    state      <= FINISHED;
                end
            end
            
            FINISHED: begin
                write_enable <= 0;
                if (~read & ~write) begin
                    complete <= 0;
                    state    <= IDLE;
                end
            end
            
            endcase
        end
    end
endmodule
