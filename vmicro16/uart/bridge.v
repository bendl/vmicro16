
// Minispartan 6+ uart passthrough

module top (
    // unused
    input        CLK50,
    input  [3:0] SW,
    
    // input from de1soc
    input        PORTF10,
    output       TXD,

    // output to de1soc
    input        RXD,
    output       PORTF8,
    
    // DE1SoC ground
    input        PORTF2,

    // debug
    output [7:0] LEDS
);
    // TXD <- PORTF10
    // RXD -> PORTF8 -> DE1SOC 
    assign LEDS   = {5'b11111, PORTF2, RXD, TXD};
    assign TXD    = PORTF10;
    assign PORTF8 = RXD;
endmodule