//
//

`include "vmicro16_soc_config.v"

(*dont_touch="true"*)
(* keep_hierarchy = "yes" *)
module vmicro16_soc #(
    parameter CORES                 = 1,
    parameter SLAVES                = 4,
    parameter APB_WIDTH             = 16,

    parameter GPIO_PINS             = 8,
    parameter SHARED_MEM_DEPTH      = 64,

    parameter SLAVE_ADDR_REGS0_APB  = 0,
    parameter SLAVE_ADDR_REGS1_APB  = 1,
    parameter SLAVE_ADDR_REGS2_APB  = 2,
    parameter SLAVE_ADDR_UART0_APB  = 3,
    parameter SLAVE_ADDR_GPIO0_APB  = 4
) (
    input clk,
    input reset,

    //input  uart_rx,
    output                 uart_tx,
    output [GPIO_PINS-1:0] gpio0,

    output reg [7:0]       dbug0,
    output     [7:0]       dbug1
);
    initial dbug0 = 0;
    always @(posedge clk)
        dbug0 <= dbug0 + 1;

    // Peripherals (master to slave)
    (*dont_touch="true"*) wire [15:0]         M_PADDR;
    (*dont_touch="true"*) wire                M_PWRITE;
    (*dont_touch="true"*) wire [SLAVES-1:0]   M_PSELx;  // not shared
    (*dont_touch="true"*) wire                M_PENABLE;
    (*dont_touch="true"*) wire [15:0]         M_PWDATA; 
    (*dont_touch="true"*) wire [15:0]         M_PRDATA; // input to intercon
    (*dont_touch="true"*) wire                M_PREADY; // input

    // Master apb interfaces
    (*dont_touch="true"*) wire [CORES*APB_WIDTH-1:0] w_PADDR;
    (*dont_touch="true"*) wire [CORES-1:0]           w_PWRITE;
    (*dont_touch="true"*) wire [CORES-1:0]           w_PSELx;
    (*dont_touch="true"*) wire [CORES-1:0]           w_PENABLE;
    (*dont_touch="true"*) wire [CORES*APB_WIDTH-1:0] w_PWDATA;
    (*dont_touch="true"*) wire [CORES*APB_WIDTH-1:0] w_PRDATA;
    (*dont_touch="true"*) wire [CORES-1:0]           w_PREADY;

    (*dont_touch="true"*)
    (* keep_hierarchy = "yes" *)
    apb_intercon_s # (
        .MASTER_PORTS(CORES),
        .SLAVE_PORTS (SLAVES)
    ) apb (
        //.clk        (clk),
        //.reset      (reset),
        // APB master to slave
        .S_PADDR    (w_PADDR),
        .S_PWRITE   (w_PWRITE),
        .S_PSELx    (w_PSELx),
        .S_PENABLE  (w_PENABLE),
        .S_PWDATA   (w_PWDATA),
        .S_PRDATA   (w_PRDATA),
        .S_PREADY   (w_PREADY),
        // shared bus
        .M_PADDR    (M_PADDR),
        .M_PWRITE   (M_PWRITE),
        .M_PSELx    (M_PSELx),
        .M_PENABLE  (M_PENABLE),
        .M_PWDATA   (M_PWDATA),
        .M_PRDATA   (M_PRDATA),
        .M_PREADY   (M_PREADY)
    );

    (*dont_touch="true"*)
    (* keep_hierarchy = "yes" *)
    vmicro16_gpio_apb # (
        .BUS_WIDTH  (APB_WIDTH),
        .PORTS      (GPIO_PINS)
    ) gpio0_apb (
        .clk        (clk),
        .reset      (reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_GPIO0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA),
        .S_PREADY   (M_PREADY),
        .gpio       (gpio0)
    );
    
    (*dont_touch="true"*)
    (* keep_hierarchy = "yes" *)
    apb_uart_tx apb_uart_inst (
        .clk        (clk),
        .reset      (reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_UART0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA),
        .S_PREADY   (M_PREADY),
        // uart wires
        .tx_wire    (uart_tx),
        .rx_wire    (uart_rx)
    );

    (*dont_touch="true"*)
    (* keep_hierarchy = "yes" *)
    vmicro16_regs_apb # (
        .BUS_WIDTH  (APB_WIDTH),
        .CELL_DEPTH (8)
    ) regs1_apb (
        .clk        (clk),
        .reset      (reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_REGS0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA),
        .S_PREADY   (M_PREADY)
    );

    (*dont_touch="true"*)
    (* keep_hierarchy = "yes" *)
    vmicro16_bram_apb # (
        .MEM_WIDTH  (APB_WIDTH),
        .MEM_DEPTH  (SHARED_MEM_DEPTH)
    ) bram_apb (
        .clk        (clk),
        .reset      (reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_BRAM0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA),
        .S_PREADY   (M_PREADY)
    );

    genvar i;
    generate for(i = 0; i < CORES; i = i + 1) begin : cores
        (* keep_hierarchy = "yes" *)
        vmicro16_core # (
            .CORE_ID    (i)
        ) c1 (
            .clk        (clk),
            .reset      (reset),
            .dbug_pc    (dbug1),

            .w_PADDR    (w_PADDR   [APB_WIDTH*i +: `APB_WIDTH] ),
            .w_PWRITE   (w_PWRITE  [i]                         ),
            .w_PSELx    (w_PSELx   [i]                         ),
            .w_PENABLE  (w_PENABLE [i]                         ),
            .w_PWDATA   (w_PWDATA  [APB_WIDTH*i +: `APB_WIDTH] ),
            .w_PRDATA   (w_PRDATA  [APB_WIDTH*i +: `APB_WIDTH] ),
            .w_PREADY   (w_PREADY  [i]                         )
        );
    end
    endgenerate


endmodule