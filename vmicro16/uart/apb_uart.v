// AUTHOR: BDL

module apb_uart_tx # (
    parameter BUS_WIDTH  = 16,
    parameter CELL_DEPTH = 8,

    parameter DATA_WIDTH = 8,
    parameter ADDR_EXP   = 5
) (
    input clk,
    input reset,

    // APB Slave to master interface
    input  [1:0]                    S_PADDR,
    // S_PADDR[0] = write port, 
    // S_PADDR[1] = read  port
    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input  [BUS_WIDTH-1:0]          S_PWDATA,
    
    output [BUS_WIDTH-1:0]      S_PRDATA,
    output reg                  S_PREADY,

    output tx_wire,
    input  rx_wire
);
    localparam APB_ADDR_WRITE = 0;
    localparam APB_ADDR_READ  = 1;

    wire        apb_we  = S_PSELx & S_PENABLE & S_PWRITE;
    wire        apb_sel = S_PSELx & S_PENABLE;

    wire        uart_tx_is_transmitting;
    wire        uart_tx_busy;
    wire        uart_rx_rdy = 1;
    wire [7:0]  uart_rx_dout;
    wire        uart_tx_fifo_full;
    reg         uart_rx_rdy_clear;
    
    assign S_PRDATA = (apb_sel) ? 16'hAAAA : 16'h0000;

    always @(*)
        if (S_PADDR == APB_ADDR_WRITE)
            S_PREADY = (apb_sel) ? (!uart_tx_fifo_full) : 1'b0;
        else
            S_PREADY = (apb_sel) ? uart_rx_rdy : 1'b0;

    always @(posedge clk)
        if (S_PADDR == APB_ADDR_READ)
            if (apb_sel)
                uart_rx_rdy_clear <= 1;
        else
            uart_rx_rdy_clear <= 0;

    always @(posedge clk)
        if (apb_we)
            $display($time, "\t\tUART0 <= %h", S_PWDATA[7:0]);

    wire uart_tx_transmit_en = apb_we && (!uart_tx_fifo_full);
    uart_fifo # (
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_EXP   (ADDR_EXP)
    ) uart_fifo(
        .clk             (clk),
        .rst             (reset),

        // reciever
        //.rx              (uart_rx),
        //.rx_fifo_pop     (uart_rx_fifo_pop)
        //.rx_fifo_empty   (uart_rx_fifo_empty),
        //.rx_byte         (uart_rx_byte[7:0]),
        //.irq             (uart_irq),
        
        // transmitter
        .tx              (tx_wire),
        .busy            (uart_tx_busy),
        .tx_fifo_full    (uart_tx_fifo_full),
        .tx_byte         (S_PWDATA[7:0]),
        .transmit        (uart_tx_transmit_en)
    );

endmodule