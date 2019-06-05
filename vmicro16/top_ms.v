
// minispartan6+ XC6SLX9

module top_ms # (
    parameter GPIO_PINS = 7
) (
    input           CLK50,
    input  [3:0]    SW,
    // UART
    //input           RXD,
    output          TXD,
    // Peripherals
    output [7:0]    LEDS
);
    vmicro16_soc # (
        .GPIO_PINS (GPIO_PINS)
    ) soc (
        .clk     (CLK50),
        .reset   (~SW[0]),
        
        .uart_tx (TXD),
        .gpio0   (LEDS)
    );

endmodule