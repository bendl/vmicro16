//
//
//

// accepts an unfiltered MMU address
// and outputs an address for the peripheral

module apb_intercon_s # (
    BUS_WIDTH    = 16,
    MASTER_PORTS = 1,
    SLAVE_PORTS  = 3
) (
    input clk,
    input reset,

    //
    //input  [MASTER_PORTS*BUS_WIDTH-1:0] S_MEM_ADDR,
    //input  [MASTER_PORTS*BUS_WIDTH-1:0] S_MEM_IN,
    //input  [MASTER_PORTS*BUS_WIDTH-1:0] S_MEM_OUT,
    //input  [MASTER_PORTS-1:0]           S_MEM_WE,
    //input  [MASTER_PORTS-1:0]           S_REQ,
    //output [MASTER_PORTS-1:0]           S_ACK,
    //output [MASTER_PORTS-1:0]           S_MEM_BUSY,
    //

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
    assign M_PADDR  = S_PADDR;
    assign M_PWRITE = S_PWRITE;
    assign M_PWDATA = S_PWDATA;
    assign S_PWDATA = M_PWDATA;
    // interconnect is ready while it's slaves are ready
    assign S_PREADY = M_PREADY;

    assign M_PSELx[0] = (S_PADDR >= 16'h80 && S_PADDR <= 16'h8F);
    assign M_PSELx[1] = (S_PADDR >= 16'h90 && S_PADDR <= 16'h9F);
    assign M_PSELx[2] = (S_PADDR >= 16'hA0 && S_PADDR <= 16'hAF);

endmodule