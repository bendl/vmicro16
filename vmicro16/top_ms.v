
// minispartan6+ XC6SLX9

module top_ms # (
    parameter GPIO_PINS = 8
) (
    input           CLK50,
    input  [3:0]    SW,
    // UART
    //input           RXD,
    output          TXD,
    // Peripherals
    output [7:0]    LEDS
);
    localparam POR_CLKS = 8;
    reg [3:0] por_timer = 0;
    reg       por_done  = 0;
    reg       por_reset = 1;
    always @(posedge CLK50)
        if (!por_done) begin
            por_reset <= 1;
            if (por_timer < POR_CLKS)
                por_timer <= por_timer + 1;
            else
                por_done <= 1;
        end
        else
            por_reset <= 0;

    //wire [15:0]         M_PADDR;
    //wire                M_PWRITE;
    //wire [5-1:0]        M_PSELx;  // not shared
    //wire                M_PENABLE;
    //wire [15:0]         M_PWDATA; 
    //wire [15:0]         M_PRDATA; // input to intercon
    //wire                M_PREADY; // input to intercon

    (* keep_hierarchy = "yes" *)
    vmicro16_soc # (
        .GPIO_PINS (GPIO_PINS)
    ) soc (
        .clk     (CLK50),
        .reset   (por_reset | (~SW[0])),

        //.M_PADDR    (M_PADDR),
        //.M_PWRITE   (M_PWRITE),
        //.M_PSELx    (M_PSELx),
        //.M_PENABLE  (M_PENABLE),
        //.M_PWDATA   (M_PWDATA),
        //.M_PRDATA   (M_PRDATA),
        //.M_PREADY   (M_PREADY),
        
        .uart_tx (TXD),
        .gpio0   (LEDS[3:0]),

        //.dbug0   (LEDS[3:0]),
        .dbug1   (LEDS[7:4])
    );

endmodule