//
//
//

`include "vmicro16_soc_config.v"
`include "clog2.v"

(*dont_touch="true"*)
(* keep_hierarchy = "yes" *)
module apb_intercon_s # (
    parameter BUS_WIDTH    = 16,
    parameter MASTER_PORTS = 1,
    parameter SLAVE_PORTS  = 4
) (
    input clk,
    input reset,

    // APB master interface (from cores)
    input  [MASTER_PORTS*BUS_WIDTH-1:0]     S_PADDR,
    input  [MASTER_PORTS-1:0]               S_PWRITE,
    input  [MASTER_PORTS-1:0]               S_PSELx,
    input  [MASTER_PORTS-1:0]               S_PENABLE,
    input  [MASTER_PORTS*BUS_WIDTH-1:0]     S_PWDATA,
    output reg [MASTER_PORTS*BUS_WIDTH-1:0] S_PRDATA,
    output reg [MASTER_PORTS-1:0]           S_PREADY,

    // MASTER interface to a slave
    output  [BUS_WIDTH-1:0]   M_PADDR,
    //shared
    output                    M_PWRITE,
    output  [SLAVE_PORTS-1:0] M_PSELx,
    //shared
    output                    M_PENABLE,
    output  [BUS_WIDTH-1:0]   M_PWDATA,
    //shared inout
    input   [BUS_WIDTH-1:0]   M_PRDATA,
    //shared inout
    input                     M_PREADY
);
    reg [`clog2(MASTER_PORTS)-1:0]  last_active = 0;
    reg [`clog2(MASTER_PORTS)-1:0]  active      = 0;

    always @(active)
        $display("active core: %h", active);
    
    // wires for current active master
    wire  [BUS_WIDTH-1:0]   a_S_PADDR   = S_PADDR  [active*BUS_WIDTH +: BUS_WIDTH];
    wire                    a_S_PWRITE  = S_PWRITE [active];
    wire                    a_S_PSELx   = S_PSELx  [active];
    wire                    a_S_PENABLE = S_PENABLE[active];
    wire  [BUS_WIDTH-1:0]   a_S_PWDATA  = S_PWDATA [active*BUS_WIDTH +: BUS_WIDTH];
    wire  [BUS_WIDTH-1:0]   a_S_PRDATA  = S_PRDATA [active*BUS_WIDTH +: BUS_WIDTH];
    wire                    a_S_PREADY  = S_PREADY [active];

    wire active_ended = !a_S_PSELx;
    integer bit_find = 0;
    always @(posedge clk) begin
        // if other requests are waiting
        if ((|S_PSELx) && active_ended)
            if (active >= MASTER_PORTS-1)
                active <= 0;
            else
                // move to next active master
                for (bit_find = (MASTER_PORTS-1); bit_find >= 0; bit_find = bit_find - 1)
                    if (S_PSELx[bit_find]) begin
                        active <= bit_find;
                    end
    end

    
    //assign M_PSELx[0] = (|S_PSELx) & (S_PADDR >= 16'h80 && S_PADDR <= 16'h8F);
    //assign M_PSELx[1] = (|S_PSELx) & (S_PADDR >= 16'h90 && S_PADDR <= 16'h9F);
    //assign M_PSELx[2] = (|S_PSELx) & (S_PADDR >= 16'hA0 && S_PADDR <= 16'hAF);
    //assign M_PSELx[3] = (|S_PSELx) & (S_PADDR >= 16'hB0 && S_PADDR <= 16'hB1);

    // GPIO0
    assign M_PSELx[`APB_PSELX_GPIO0] = |S_PSELx & ((a_S_PADDR == `DEF_MMU_GPIO0_S) 
                                                && (a_S_PADDR <= `DEF_MMU_GPIO0_E));
    // GPIO1
    assign M_PSELx[`APB_PSELX_GPIO1] = |S_PSELx & ((a_S_PADDR == `DEF_MMU_GPIO1_S) 
                                                && (a_S_PADDR <= `DEF_MMU_GPIO1_E));
    // GPIO2
    assign M_PSELx[`APB_PSELX_GPIO2] = |S_PSELx & ((a_S_PADDR == `DEF_MMU_GPIO2_S) 
                                                && (a_S_PADDR <= `DEF_MMU_GPIO2_E));
    // UART0
    assign M_PSELx[`APB_PSELX_UART0] = |S_PSELx & ((a_S_PADDR >= `DEF_MMU_UART0_S) 
                                                && (a_S_PADDR <= `DEF_MMU_UART0_E));
    // REGS0
    assign M_PSELx[`APB_PSELX_REGS0] = |S_PSELx & ((a_S_PADDR >= `DEF_MMU_REGS0_S)
                                                && (a_S_PADDR <= `DEF_MMU_REGS0_E));
    // BRAM0
    assign M_PSELx[`APB_PSELX_BRAM0] = |S_PSELx & ((a_S_PADDR >= `DEF_MMU_BRAM0_S) 
                                                && (a_S_PADDR <= `DEF_MMU_BRAM0_E));


    // Pass through
    assign M_PADDR   = a_S_PADDR;
    assign M_PWRITE  = a_S_PWRITE;
    assign M_PENABLE = a_S_PENABLE;
    assign M_PWDATA  = a_S_PWDATA;
    assign M_PWDATA  = a_S_PWDATA;
    // interconnect is ready while it's slaves are ready
    always @(*) begin
        S_PREADY = 0;
        S_PRDATA = 0;
        S_PREADY[active] = M_PREADY;
        S_PRDATA[active*BUS_WIDTH +: BUS_WIDTH] = M_PRDATA;
    end


endmodule