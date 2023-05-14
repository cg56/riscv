`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/13/2023 07:51:47 PM
// Design Name: 
// Module Name: memory
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
// The client enables the memory's data bus and initates a read or a write cycle.
// When the read or write has been completed by the memory, the complete signal
// will go high to notify the client. In the case of a read cycle, the data_out
// is valid at this point. When the client has obtained the data, it will then
// lower the read/write signal to notify the memory that the read/write request
// has completed. The complete signal will then return low and at this point
// another cycle can begin.
//                  ____   ____
// enable     _____/    ...    \_____
//                  ____   ____
// read/write _____/    ...    \_____
//                           _____
// complete   __________..._/     \__
// 
//////////////////////////////////////////////////////////////////////////////////

module memory(
    input clock,            // 200MHz clock for the mig
    input reset,            // Active high reset
    input[26:0] address,
    inout[31:0] data_io,
    input[1:0] width,       // 0 = 8 bits, 1 = 16 bits, 2 = 32 bits 
    input enable,           // Enable the memory for reading or writing
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
    reg[7:0]    write_mask;
    reg         write_end;      // Signals last write clock cycle
    wire[63:0]  read_data;
    wire        read_end;
    wire        read_valid;
    
    // Signals from the mig module
    
    wire        memory_ready;   // Memory is ready to accept a command
    wire        write_ready;    // Write queue has space
    wire        ui_clk;         // Clock from the mig to drive the caller
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
        .app_addr(address[26:1]),   // The lest-significant address bit selects the byte within a short
        .app_cmd(command),
        .app_en(mem_enable),
        .app_wdf_data(write_data),
        .app_wdf_end(write_end),
        .app_wdf_mask(write_mask),
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

    // Signals
    
    reg[31:0] data_out;
    assign data_io = (read & enable) ? data_out : 32'bZ;

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
            write_mask   <= 8'h00;
            write_data   <= 64'h0;
            write_enable <= 0;
            write_end    <= 0;
            mem_enable   <= 0;      
        end else begin
            case (state)
            
            IDLE: begin
                if (read & enable) begin
                    mem_enable <= 1;
                    command    <= 3'h1;
                    state      <= START_READ;
                end else if (write & enable) begin
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
                        state    <= FINISHED;
                        complete <= 1;
                    end else // Only use the data from the first packet
                        case (width)
                        3,
                        2: data_out <= read_data[31:0];
                        1: data_out <= {16'b0, read_data[15:0]};
                        0: begin
                            if (address[0])
                                data_out <= {24'b0, read_data[15:8]};
                            else
                                data_out <= {24'b0, read_data[7:0]};
                        end
                        endcase
                end
            end
                       
            START_WRITE: begin
                // Wait until the command has been accepted
                if (memory_ready) begin
                    mem_enable <= 0;
                    state      <= WRITE_HIGH;
                end
            end

            WRITE_HIGH: begin   // Send the first packet
                // Wait for write queue to have space
                if (write_ready) begin
                    case (width)
                    3,
                    2: begin
                        write_mask       <= 8'hF0;  // Only write the lower 32 bits
                        write_data[63:0] <= {32'b0, data_io[31:0]};
                    end
                    1: begin
                        write_mask       <= 8'hFC;  // Only write the lower 16 bits
                        write_data[63:0] <= {48'b0, data_io[15:0]};
                    end
                    0: begin
                        if (address[0]) begin
                            write_mask       <= 8'hFD;  // Only write the mid 8 bits
                            write_data[63:0] <= {48'b0, data_io[7:0], 8'b0};
                        end else begin
                            write_mask       <= 8'hFE;  // Only write the lower 8 bits
                            write_data[63:0] <= {56'b0, data_io[7:0]};
                        end
                    end
                    endcase

                    write_enable <= 1;
                    state        <= WRITE_LOW;
                end
            end

            WRITE_LOW: begin    // Send the expected 2nd packet
                // Wait for Write Data queue to have space
                if (write_ready) begin 
                    write_mask <= 8'hFF;  // Don't actually write anything
                    write_data <= 64'h0;

                    write_end <= 1;
                    complete  <= 1;
                    state     <= FINISHED;
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
