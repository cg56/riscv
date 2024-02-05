`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 
// Emulates an 8250 UART. This takes some liberties. If a write is attempted
// before the previous write has finished being transmitted, then the write
// will stall and won't complete until the transmit buffer is empty.
//
// Only two registers are supported:
// Address 00 = UART read/write buffer
// Address 05 = UART status reg
//      bit 0 - data is ready to read
//      bit 5 -
//      bit 6 -
//
// Copyright Colm Gavin, 2024. All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////////

module uart(
    input clk,              // 100Mhz clock
    input reset,            // Active high reset
    input[7:0] address,
    inout[31:0] data_io,    // Only supports 8-bit reads and writes
    input enable,           // Enable the uart for reading or writing
    input read,             // Request to read from the uart
    input write,            // Request to write to the uart
    output reg complete,    // The read or write request is complete
    input txd_in,           // Serial input to the uart
    output reg rxd_out      // Serial output from the uart
    );

    reg[31:0] data_out;
    assign data_io = (read & enable) ? data_out : 32'bZ;
    
    reg[7:0] thr;           // Transmitter Holding Register is at address 0x0
    reg thr_full;           // THR contains data
    reg[7:0] rbr;           // Receiver Buffer Register
    reg data_ready;         // RBR contains data
        
    reg[13:0] tx_divisor;   // 14 bits counter to count the baud rate, divisor = clock / baud rate
    reg[13:0] rx_divisor;
    reg[9:0] tsr;           // Transmitter shift register (not visible externally), 8 bits + start and stop bits
    reg[6:0] rx_count;

    always @(posedge clk) begin
        if (reset) begin
            complete   <= 0;
            rxd_out    <= 1;
            tx_divisor <= 0;
            rx_divisor <= 0;
            rx_count   <= 0;
            thr_full   <= 0;
            data_ready <= 0;
        end else begin
            if (read & enable) begin
                if (!complete) begin
                    case (address)
                    // Read receiving buffer register
                    8'h00: begin
                        data_out <= {24'h000000,rbr};
                        data_ready <= 0;
                    end
                    // Read line Status Register
                    8'h05:
                        //data_out <= {24'h000000,1'b0,(!thr_full&(tsr==0)),!thr_full,4'b0000,data_ready};
                        data_out <= {24'h000000,1'b0,2'b11,4'b0000,data_ready}; // Always say we have space, even though it will block ??
                    endcase
                    complete <= 1;
                end
            end else if (write & enable) begin
                if (!complete) begin
                    //if (thr_full == 0) begin   // Block until transmit holding register has space ??
                    if ((thr_full == 0) & (tsr == 0)) begin
                        case (address)
                        // Write transmit holding register
                        8'h00: begin
                            thr[7:0] <= data_io[7:0];
                            thr_full <= 1;
                        end
                        endcase
                        complete <= 1;
                    end
                end
            end else begin
                complete <= 0;
            end
            
            tx_divisor <= tx_divisor + 1;   // Count from 0 to 10415 (10416 counts) 
            if (tx_divisor == 10415) begin  // 10415 = 9,600 baud
                tx_divisor <= 0;            // Restart the counter
                
                if (tsr == 0) begin         // Tsr will be zero when the stop bit has shifted out
                    if (thr_full) begin
                        tsr <= {1'b1,thr,1'b0};
                        thr_full <= 0;
                    end
                end else begin
                    rxd_out <= tsr[0];
                    tsr <= tsr >> 1;
                end
            end
            
            rx_divisor <= rx_divisor + 1;
            if (rx_divisor == 2603) begin   // 2604 clock cycles = 4 samples/bit at 9600 baud
                rx_divisor <= 0;
                
                if (rx_count == 0) begin
                    if (txd_in == 0)        // Start bit
                        rx_count <= 1;
                end else begin
                    // Shift the bits in on the 2nd count of every 4 counts
                    if (rx_count % 4 == 2)
                        rbr <= {txd_in,rbr[7:1]};
                        
                    rx_count <= rx_count + 1;                   
                    if (rx_count == 37) begin
                        rx_count   <= 0;
                        data_ready <= 1;
                    end
                end
            end
        end
    end
endmodule
