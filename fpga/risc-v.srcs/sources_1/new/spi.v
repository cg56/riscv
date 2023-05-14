`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/01/2023 07:52:15 PM
// Design Name: 
// Module Name: spi
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
// Address  Register
//      00  Control/Status register, 1 byte, read/write
//          bit 0 - card detected (r)
//          bit 1 - chip select (r/w)
//      01  Data register, 1 byte, read/write
//          read/write automatically triggers sending byte to/from chip
//      02  Response register, read only, waits for leading zero bit
//      03  Read until zero bit (used before reading a block/sector of data)
//   04-05  16-bit clock divisor, write only
//   06-07  16-bit block CRC, read only
//   08-0b  32-bit data register, read only 
//
//////////////////////////////////////////////////////////////////////////////////

module spi(
    // CPU busses
    input clock,            // 100Mhz
    input reset,
    input[3:0] address,
    inout[31:0] data_io,
    input enable,           // Enable the SPI for reading or writing
    input read,             // Request to read from the SPI
    input write,            // Request to write to the SPI
    //output[7:0] led,    //??
    output reg complete,    // The read or write request is complete
    
    // SPI bus
    output sd_power,        // CF power (negative active)
    output reg sd_clk,      // CF SCLK pin
    output sd_cs,           // CF DAT3 pin (chip select, negative active)
    output mosi,            // CF CMD pin
    input sd_cd,            // Card detect
    input miso              // CF DAT0 pin
);

    localparam RESET    = 0;
    localparam POWER    = 1;
    localparam INIT     = 2;
    localparam READY    = 3;
    localparam READ     = 4;
    localparam RESPONSE = 5;
    localparam ZERO     = 6;
    localparam WRITE    = 7;
    localparam DONE     = 8;
    localparam HALT     = 9;

    reg[3:0]  state = RESET;
    reg       chip_select = 0;
    reg       power       = 0;
    reg[15:0] divisor = 1000;
    reg[15:0] limit   = 1000;
    reg[7:0]  out_bits = 8'hff;
    reg[15:0] clk_count = 1000;
    reg[15:0] crc = 0;
    wire      crc_in = miso^crc[15];

    assign mosi     = out_bits[7];
    assign sd_cs    = ~chip_select;
    assign sd_power = ~power;
   // assign led[7:0] = in_bits[7:0];

    reg[31:0] data_out;
    assign data_io = (read & enable) ? data_out : 32'bZ;

    always @(posedge clock) begin
        if (reset) begin
            divisor     <= 1000;
            limit       <= 1000;    // Default to 100KHz
            clk_count   <= 1000;
            power       <= 1;//??0;       // Turn off the power to the card
            sd_clk      <= 0;
            out_bits[7:0] <= 8'hff;
            crc         <= 0;
            data_out    <= 0;
            chip_select <= 0;
            complete    <= 0;
            state       <= RESET;
        end else begin
            divisor <= divisor - 1;
        
            if (divisor == 0) begin
                divisor <= limit;
                
                case (state)
                RESET: begin    // 10ms delay before turning on the power, let the card stabalize
                    if (clk_count == 0) begin 
                        clk_count <= 1000;
                        state     <= POWER;
                    end
                    clk_count <= clk_count - 1;
                end
                
                POWER: begin    // Power on and wait for 10 more ms
                    power <= 1;       // Turn on the power
                    
                    if (clk_count == 0) begin
                        clk_count <= 100;
                        state     <= INIT;
                    end
                    clk_count <= clk_count - 1;
                end
                
                INIT: begin     // Send >74 initialization clocks
                    if (sd_clk == 1) begin
                        if (clk_count == 0)
                            state <= READY;
                        else
                            clk_count <= clk_count - 1;
                    end
                    sd_clk <= ~sd_clk;
                end
                
                READY: begin
                    // Handle I/O to/from the CPU
            
                    if (read & enable) begin
                        if (!complete) begin
                            //?? out_bits[7:0] <= 8'hff;
                            case (address[3:0]) // 8-bit reads and writes
                            4'b0000: begin      // Status register
                                data_out <= {30'h0,chip_select,~sd_cd};
                                complete <= 1;
                            end
                            4'b0001: begin      // Data register
                                //if (clk_count == 0) begin
                                    out_bits[7:0] <= 8'hff; // Why ??
                                    //in_bits       <= 8'hff; // Why ??
                                    data_out <= 0;
                                    clk_count     <= 8; // 7 ??
                                    state <= READ;
                                //end
                            end
                            4'b0010: begin      // Response register
                                out_bits[7:0] <= 8'hff; // WHy ??
                                //in_bits       <= 8'hff; // Why ??
                                data_out <= 0;
                                clk_count     <= 8; // Why??
                                state <= RESPONSE;
                            end
                            4'b0011: begin      // Read until zero bit
                                out_bits[7:0] <= 8'hff; // WHy ??
                                data_out <= 0;
                                crc      <= 0;
                                state <= ZERO;
                            end
                            4'b0110: begin
                                out_bits[7:0] <= 8'hff; // Why ??
                                data_out  <= {16'h0, crc[15:0]};
                                complete <= 1;
                            end
                            4'b1000: begin
                                out_bits[7:0] <= 8'hff; // Why ??
                                data_out  <= 0;
                                clk_count <= 32;
                                state <= READ;
                            end
                            default:
                                complete <= 1;
                            endcase
                        end
                    end else if (write & enable) begin
                        if (!complete) begin
                            case (address[3:0])
                            4'b0000: begin      // Control register
                                chip_select <= data_io[1];
                                complete <= 1;
                            end
                            4'b0001: begin      // Data register
                                //if (clk_count == 0) begin
                                    out_bits[7:0] <= data_io[7:0];  // Data into card
                                    clk_count <= 8;
                                    state <= WRITE;
                                //end
                            end
                            4'b0100: begin
                                limit <= data_io[15:0];
                                complete <= 1;
                            end
                            default:
                                complete <= 1;
                            endcase
                        end
                    end else
                        complete <= 0;
                end
                
                RESPONSE: begin
                    if (sd_clk == 1) begin
                        if (miso == 0) begin
                            //in_bits[7:0] <= {in_bits[6:0], miso};
                            clk_count <= 7; // 6 ??
                            state <= READ;
                        end     
                    end
                    sd_clk <= ~sd_clk;
                end
                
                READ: begin
                    if (sd_clk == 1) begin
                        //in_bits[7:0] <= {in_bits[6:0], miso};
                        data_out <= {data_out[30:0], miso};
                        crc <= {crc[14:12], crc[11]^crc_in, crc[10:5], crc[4]^crc_in, crc[3:0], crc_in};
                        if (clk_count == 1) begin
                            //data_out <= {24'h0,in_bits[7:0]};
                            complete <= 1;
                            state <= DONE;//READY;// DONE;
                        end else begin
                            //in_bits[7:0] <= {in_bits[6:0], miso};
                            clk_count <= clk_count - 1;
                        end
                    end
                    sd_clk <= ~sd_clk;
                end
                /*READ: begin
                    if (clk_count == 0) begin
                        data_out <= {24'h0,in_bits[7:0]};
                        complete <= 1;
                        state <= READY;// DONE;
                    end else begin
                        if (sd_clk == 1) begin
                            in_bits[7:0] <= {in_bits[6:0], miso};
                            clk_count <= clk_count - 1;
                        end
                        sd_clk <= ~sd_clk;
                    end
                end*/
 
                ZERO: begin
                    if (sd_clk == 1) begin
                        if (miso == 0) begin
                            //in_bits[7:0] <= 0;  // Why??
                            complete <= 1;
                            state <= DONE;//READY;
                        end
                    end
                    sd_clk <= ~sd_clk;
                end
                
                WRITE: begin
                    if (sd_clk == 1) begin
                        if (clk_count == 1) begin
                            out_bits[7:0] <= 8'hff;
                            complete <= 1;
                            state <= DONE;//READY;//DONE;
                        end else begin
                            out_bits[7:0] <= {out_bits[6:0], 1'b1};
                            clk_count <= clk_count - 1;
                        end
                    end
                    sd_clk <= ~sd_clk;
                end  
                /*            
                WRITE: begin
                    if (clk_count == 0) begin
                        out_bits[7:0] <= 8'hff;
                        complete <= 1;
                        state <= READY;//DONE;
                    end else begin
                        if (sd_clk == 1) begin
                            out_bits[7:0] <= {out_bits[6:0], 1'b1};
                            clk_count <= clk_count - 1;
                        end
                        sd_clk <= ~sd_clk;
                    end
                end*/
                
                DONE: begin
                    if (!(read && enable) && !(write && enable)) begin
                        complete <= 0;
                        state    <= READY;
                    end
                end
                
                endcase
            end
        end
    end
endmodule
