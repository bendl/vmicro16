// AUTHOR: BDL

(*dont_touch="true"*)
(* keep_hierarchy = "yes" *)
module apb_uart_tx # (
    parameter BUS_WIDTH  = 16,
    parameter CELL_DEPTH = 8
) (
    input clk,

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

    wire        uart_tx_busy;
    wire        uart_rx_rdy;
    wire [7:0]  uart_rx_dout;
    reg         uart_rx_rdy_clear;
    
    assign S_PRDATA = (apb_sel) ? 16'hAAAA : 16'hZZZZ;

    always @(*)
        if (S_PADDR == APB_ADDR_WRITE)
            S_PREADY = (apb_sel) ? !uart_tx_busy : 1'bZ;
        else
            S_PREADY = (apb_sel) ? uart_rx_rdy : 1'bZ;

    always @(posedge clk)
        if (S_PADDR == APB_ADDR_READ)
            if (apb_sel)
                uart_rx_rdy_clear <= 1;
        else
            uart_rx_rdy_clear <= 0;

    reg disp_once = 0;
    always @(posedge clk)
        if (apb_we) begin
            disp_once <= 1;
            if (disp_once == 0)
                $display($time, "\t\tUART <= %h", S_PWDATA[7:0]);
        end
        else
            disp_once <= 0;
    

    uart uart_blocking (
        .clk_50m    (clk),

        .din        (S_PWDATA[7:0]),
        .wr_en      (apb_we),
        .tx         (tx_wire),
        .tx_busy    (uart_tx_busy),

        .rx         (rx_wire),
        .rdy        (uart_rx_rdy),
        .rdy_clr    (uart_rx_rdy_clear),
        .dout       (uart_rx_dout)
    );

endmodule