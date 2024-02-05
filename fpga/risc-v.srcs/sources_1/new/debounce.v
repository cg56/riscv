`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Debounces the input switch level.
//
// Copyright (c) Colm Gavin, 2024
// 
//////////////////////////////////////////////////////////////////////////////////

module debounce(
    input clock,
    input switch,
    output state
    );
    
    reg[21:0] counter = 0;
    assign slow_clock = counter[21];    // Approximately 25Hz
    
    always @(posedge clock)
    begin
        counter <= counter + 1;
    end
    
    wire Q0, Q1, Q2;
    dff d0(slow_clock, switch, Q0);
    dff d1(slow_clock, Q0, Q1);
    dff d2(slow_clock, Q1, Q2);

    assign state = Q1 & ~Q2;
endmodule

// D-type flip-flop

module dff(
    input clock,
    input D,
    output reg Q);

    always @(posedge clock)
        Q <= D;
endmodule
