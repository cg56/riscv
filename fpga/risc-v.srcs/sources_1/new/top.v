`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/14/2023 09:45:48 PM
// Design Name: 
// Module Name: top
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

module top(
    input CLK100MHZ,
    input CPU_RESETN,
    input  UART_TXD_IN,
    output UART_RXD_OUT,
    output SD_RESET,
    input SD_CD,
    output SD_SCK,
    output SD_CMD,
    inout[3:0] SD_DAT,

    output[15:0] LED,
    output CA, CB, CC, CD, CE, CF, CG,
    output [7:0] AN,
    input[2:0] SW,
    
    // DDR memory tnterface
    
    inout[15:0] ddr2_dq,
    inout[1:0] ddr2_dqs_n,
    inout[1:0] ddr2_dqs_p,
    output[12:0] ddr2_addr,
    output[2:0] ddr2_ba,
    output ddr2_ras_n,
    output ddr2_cas_n,
    output ddr2_we_n,
    output ddr2_ck_p,
    output ddr2_ck_n,
    output ddr2_cke,
    output ddr2_cs_n,
    output[1:0] ddr2_dm,
    output ddr2_odt
    );

    // Display for debug output
    
    wire[31:0] sseg_data;
    
    seven_segment_display sseg(
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .value(sseg_data),
        .segments({CG,CF,CE,CD,CC,CB,CA}),
        .anodes(AN)
    );
    
    // Create the clocks
    
    wire clk_200mhz;
    wire clk_100mhz;
    wire pll_locked;
    
    clocks clocks1(
        .clk_in(CLK100MHZ),
        .clk_200mhz(clk_200mhz),  // 200MHz memory clock
        .clk_100mhz(clk_100mhz),  // 50Mhz cpu clock (may get faster!)    // Rename ??
        .locked(pll_locked) // High if the clock is stable
        );
    
    wire reset = ~pll_locked | ~CPU_RESETN;
    
    // The memory controller
    
    wire[31:0] address;
    wire[31:0] data_io;
    wire[1:0]  width;   // Byte, short or word
    wire       read;
    wire       write;
    wire       ram_enable;
    wire       ram_complete;
  
    memory mem1(
        .clock(clk_200mhz),
        .reset(reset),

        .address(address[26:0]),
        .data_io(data_io),
        .width(width),
        .enable(ram_enable),
        .read(read),
        .write(write),
        .complete(ram_complete),

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
        .ddr2_odt(ddr2_odt)
    );

    // Rom with the hard-coded boot loader
    
    wire rom_enable;
    wire rom_complete;
    
    rom bootroom(
        .clock(clk_100mhz),
        .reset(reset),
        .address(address[11:0]),
        .data_io(data_io),
        .enable(rom_enable),
        .read(read),
        .complete(rom_complete)
        );
    
    // Core local interrupter
    
    wire clint_enable;
    wire clint_complete;
    wire interrupt_req;
    wire[63:0] instr_count;    //??
    
    clint clint1(
        .clock(clk_100mhz),
        .reset(reset),
        .address(address[15:0]),
        .data_io(data_io),
        .enable(clint_enable),
        .read(read),
        .write(write),
        .complete(clint_complete),
        .interrupt_req(interrupt_req),
        .instr_count(instr_count)
        );
        
    // 8250 UART
    
    wire uart_enable;
    wire uart_complete;
    
    uart uart1(
        .clk(CLK100MHZ/*clk_100mhz*/),
        .reset(reset),
        .address(address[7:0]),
        .data_io(data_io),
        .enable(uart_enable),
        .read(read),
        .write(write),
        .complete(uart_complete),
        .txd_in(UART_TXD_IN),
        .rxd_out(UART_RXD_OUT)
        );
    
    // CF SPI interface
    
    wire spi_enable;
    wire spi_complete;
    assign SD_DAT[2:1] = 2'b11;
    
    spi spi1(
        .clock(clk_100mhz),
        .reset(reset),
        .address(address[3:0]),
        .data_io(data_io),
        .enable(spi_enable),
        .read(read),
        .write(write),
        .complete(spi_complete),
        .sd_power(SD_RESET),
        .sd_clk(SD_SCK),
        .sd_cs(SD_DAT[3]),
        .mosi(SD_CMD),
        .sd_cd(SD_CD),
        .miso(SD_DAT[0])
    );
    
    // Address decoder
    
    address_decoder adec(
        .address(address[31:24]),     // High byte of the address bus
        .ram_enable(ram_enable),
        .rom_enable(rom_enable),
        .uart_enable(uart_enable),
        .clint_enable(clint_enable),
        .spi_enable(spi_enable)
    );
    
    wire complete = ram_complete | rom_complete | uart_complete | clint_complete | spi_complete;
    
    reg[32:0] slow_clk;  // Move into riscv module ??
    always @(posedge clk_100mhz) begin
        slow_clk <= slow_clk + 1;
    end
    
    // The CPU itself
    
    cpu cpu1(
        //.clock(clk_100mhz),
        .clock(SW[2] ? slow_clk[27] : slow_clk[6]),
        .reset(reset), 
        .address(address),
        .data_io(data_io),
        .width(width),
        .write(write),
        .read(read),
        .complete(complete),
        .interrupt_req(interrupt_req),
        .led(LED[15:0]),
        .sseg_data(sseg_data),
        .debug(SW[1:0]),
        .instr_count(instr_count)
    );
    
`ifdef DOIT
    // Testing state machine
        
    memory_test test(
        .clocku(clk_100mhz),
        .reset(reset), 
        .address(address),
        .data_io(data_io),
        .width(width),
        .led(LED[7:0]),
        .sseg_data(sseg_data),
        .write(write),
        .read(read),
        .ready(ready),
        .complete(complete)
    );
`endif
endmodule
