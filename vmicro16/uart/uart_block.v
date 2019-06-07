//https://github.com/jamieiles/uart/blob/master/uart.v

(* keep_hierarchy = "yes" *)
module uart (
        input wire clk_50m,

        // transmitter
        input wire  [7:0] din,
        input wire  wr_en,
        output wire tx,
        output wire tx_busy,

        // reciever
        input wire  rx,
        output wire rdy,
        input wire  rdy_clr,
        output wire [7:0] dout
);
    wire rxclk_en, txclk_en;

    (* keep_hierarchy = "yes" *)
    baud_rate_gen uart_baud(
        .clk_50m(clk_50m),
        .rxclk_en(rxclk_en),
        .txclk_en(txclk_en)
    );
    
    (* keep_hierarchy = "yes" *)
    transmitter uart_tx(
        .din(din),
        .wr_en(wr_en),
        .clk_50m(clk_50m),
        .clken(txclk_en),
        .tx(tx),
        .tx_busy(tx_busy)
    );
        
    //receiver uart_rx(
    //    .rx(rx),
    //    .rdy(rdy),
    //    .rdy_clr(rdy_clr),
    //    .clk_50m(clk_50m),
    //    .clken(rxclk_en),
    //    .data(dout)
    //);

endmodule