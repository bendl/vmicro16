//
//
//

// accepts an unfiltered MMU address
// and outputs an address for the peripheral

module apb_intercon_s # (
    BUS_WIDTH   = 16,
    SLAVE_PORTS = 3,
) (
    input                     S_PCLK,
    // SLAVE Interface to a master
    input  [BUS_WIDTH-1:0]    S_PADDR,
    input                     S_PWRITE,
    input                     S_PSELx,
    input                     S_PENABLE,
    input  [BUS_WIDTH-1:0]    S_PWDATA,
    output                    S_PRDATA,
    output                    S_PREADY,

    // MASTER interface to a slave
    output  [BUS_WIDTH-1:0]   M_PADDR,
    //shared
    output                    M_PWRITE,
    output  [SLAVE_PORTS-1:0] M_PSELx,
    //shared
    output                    M_PENABLE,
    output  [BUS_WIDTH-1:0]   M_PWDATA,
    //shared inout
    input                     M_PRDATA,
    //shared inout
    input                     M_PREADY
);
    wire s_apb_write = S_PSELx & S_PENABLE & S_PWRITE;
    wire s_apb_read  = S_PSELx & ~S_PWRITE;
    
    assign M_PADDR  = S_PADDR;
    assign M_PWRITE = S_PWRITE;
    assign M_PWDATA = S_PWDATA;
    assign S_PWDATA = M_PWDATA;
    // interconnect is ready while it's slaves are ready
    assign S_PREADY = M_PREADY;

    assign MPSELx[0] = (S_PADDR >= 16'h80 && S_PADDR <= 16'h8F);
    assign MPSELx[1] = (S_PADDR >= 16'h90 && S_PADDR <= 16'h9F);
    assign MPSELx[2] = (S_PADDR >= 16'hA0 && S_PADDR <= 16'hAF);
    assign MPSELx[3] = (S_PADDR >= 16'hB0 && S_PADDR <= 16'hBF);

endmodule