//
//
//

(*dont_touch="true"*)
(* keep_hierarchy = "yes" *)
module apb_intercon_s # (
    parameter BUS_WIDTH    = 16,
    parameter MASTER_PORTS = 1,
    parameter SLAVE_PORTS  = 5
) (
    //input clk,
    //input reset,

    // APB master interface (from cores)
    input  [MASTER_PORTS*BUS_WIDTH-1:0] S_PADDR,
    input  [MASTER_PORTS-1:0]           S_PWRITE,
    input  [MASTER_PORTS-1:0]           S_PSELx,
    input  [MASTER_PORTS-1:0]           S_PENABLE,
    input  [MASTER_PORTS*BUS_WIDTH-1:0] S_PWDATA,
    output [MASTER_PORTS*BUS_WIDTH-1:0] S_PRDATA,
    output [MASTER_PORTS-1:0]           S_PREADY,

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
    // TODO: Round robin scheduling

    // Pass through
    assign M_PADDR    = S_PADDR[BUS_WIDTH-1:0];
    
    (*dont_touch="true"*) assign M_PWRITE   = |S_PWRITE;
    assign M_PSELx[0] = (|S_PSELx) & (S_PADDR >= 16'h80 && S_PADDR <= 16'h8F);
    assign M_PSELx[1] = (|S_PSELx) & (S_PADDR >= 16'h90 && S_PADDR <= 16'h9F);
    assign M_PSELx[2] = (|S_PSELx) & (S_PADDR >= 16'hA0 && S_PADDR <= 16'hAF);
    assign M_PSELx[3] = (|S_PSELx) & (S_PADDR >= 16'hB0 && S_PADDR <= 16'hB1);
    assign M_PSELx[4] = (|S_PSELx) & (S_PADDR == 16'hC0);
    assign M_PENABLE  = |S_PENABLE;
    assign M_PWDATA   = S_PWDATA[BUS_WIDTH-1:0];
    assign M_PWDATA   = S_PWDATA[BUS_WIDTH-1:0];
    // interconnect is ready while it's slaves are ready
    assign S_PREADY   = M_PREADY;
    assign S_PRDATA   = M_PRDATA;


endmodule