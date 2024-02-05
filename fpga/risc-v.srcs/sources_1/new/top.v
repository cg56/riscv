`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// RISC-V based computer with 128Kbytes of memory an serial comms.
//
// Copyright (c) Colm Gavin, 2024
// 
// Description of the memory bus
//
// The CPU sets the address on the bus and initates a read or a write cycle.
// When the read or write has been completed by the memory (or memory-mapped I/O
// device), it sets the complete signal high to notify the CPU. In the case of a
// read cycle, the data out is valid at this point. When the CPU has latched the
// data, it will then lower the read/write signal to notify the memory that the
// read/write request has completed. The complete signal will then return low and
// at this point another cycle can begin.
//                  ____   ____
// address    _____/    ...    \_____
//                  ____   ____
// read/write _____/    ...    \_____
//                           _____
// complete   __________..._/     \__
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
    input[15:0] SW,
    
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

    // Create the clocks
    
    wire clk_200mhz;    // 200MHz memory clock
    wire clk_100mhz;    // 100Mhz system clock
    wire clk_cpu;       // Cpu clock, also 100MHz but can be changed independently
    wire pll_locked;    // High if the clock is stable
    
    clocks clocks1(
        .clk_in(CLK100MHZ),
        .clk_200mhz(clk_200mhz),
        .clk_100mhz(clk_100mhz),
        .clk_cpu(clk_cpu),
        .locked(pll_locked)
        );

    // Debounce the reset switch
    
    wire reset_switch;
    debounce db(clk_100mhz, ~CPU_RESETN, reset_switch);
    wire reset = ~pll_locked | reset_switch;    // Master reset

    // Display for debug output
    
    wire[31:0] sseg_data;   // The value to be displayed
    
    seven_segment_display sseg(
        .clk(clk_100mhz),
        .reset(reset),
        .value(sseg_data),
        .segments({CG,CF,CE,CD,CC,CB,CA}),
        .anodes(AN)
    );

    // Define the I/O bus from the CPU to memory and memory-mapped I/O
    
    wire[31:0] address;
    wire[31:0] data_io;
    wire[1:0]  width;   // 00 = 8-bit byte, 01 = 16-bit short, 10 = 32-bit word
    wire       read;
    wire       write;
    
    // The memory controller
    
    wire ram_enable;    // Data requested or data is ready to write
    wire ram_complete;  // Data is ready or data has been written
  
    ram memory(
        .clock(clk_200mhz),
        .reset(reset),
        .address(address[26:0]), // 27-bit address lines for 128Kbyte of memory
        .data_io(data_io),
        .width(width),
        .enable(ram_enable),
        .read(read),
        .write(write),
        .complete(ram_complete),
        // DDR memory tnterface
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
    
    wire rom_enable;    // Data requested
    wire rom_complete;  // Data is ready
    
    rom bootroom(
        .clock(clk_200mhz),
        .reset(reset),
        .address(address[12:0]),
        .data_io(data_io),
        .enable(rom_enable),
        .read(read),
        .complete(rom_complete)
        );
    
    // Core local interrupter
    
    wire clint_enable;
    wire clint_complete;
    wire interrupt_req;
    
    clint clint1(
        .clock(clk_100mhz),
        .reset(reset),
        .address(address[15:0]),
        .data_io(data_io),
        .enable(clint_enable),
        .read(read),
        .write(write),
        .complete(clint_complete),
        .interrupt_req(interrupt_req)
        );
        
    // 8250 UART
    
    wire uart_enable;
    wire uart_complete;
    
    uart uart1(
        .clk(clk_100mhz),
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
    assign SD_DAT[2:1] = 2'b11; // We don't use these two pins
    
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
    
    // Memory-map some of the switches
    
    wire sw_enable;
    wire sw_complete;
    
    switches swit(
        .switches(SW[7:0]),
        .clock(clk_100mhz),
        .data_io(data_io),
        .enable(sw_enable),
        .read(read),
        .complete(sw_complete)
    );
    
    // Address decoder
    
    address_decoder adec(
        .address(address[31:24]),     // High byte of the address bus
        .ram_enable(ram_enable),
        .rom_enable(rom_enable),
        .uart_enable(uart_enable),
        .clint_enable(clint_enable),
        .spi_enable(spi_enable),
        .sw_enable(sw_enable)
    );
    
    wire complete = ram_complete | rom_complete | uart_complete | clint_complete | spi_complete | sw_complete;

    // The CPU itself
    
    cpu cpu1(
        .clock(clk_cpu),
        .reset(reset), 
        .address(address),
        .data_io(data_io),
        .width(width),
        .write(write),
        .read(read),
        .complete(complete),
        .interrupt_req(interrupt_req),
        .led(LED[15:8]),
        .sseg_data(sseg_data),
        .debug(SW[15:14])
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
