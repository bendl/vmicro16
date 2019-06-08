//
//
//

`include "vmicro16_soc_config.v"

(*dont_touch="true"*)
(* keep_hierarchy = "yes" *)
module apb_intercon_s # (
    parameter BUS_WIDTH    = 16,
    parameter MASTER_PORTS = 1,
    parameter SLAVE_PORTS  = 4
) (
    //input clk,
    //input reset,

    // APB master interface (from cores)
    input  [MASTER_PORTS*BUS_WIDTH-1:0] S_PADDR,
    input  [MASTER_PORTS-1:0]           S_PWRITE,
    input  [MASTER_PORTS-1:0]           S_PSELx,
    input  [MASTER_PORTS-1:0]           S_PENABLE,
    input  [MASTER_PORTS*BUS_WIDTH-1:0] S_PWDATA,
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
    wire  [BUS_WIDTH-1:0]   a_S_PADDR   = S_PADDR  [active*BUS_WIDTH +: BUS_WIDTH];
    wire                    a_S_PWRITE  = S_PWRITE [active];
    wire                    a_S_PSELx   = S_PSELx  [active];
    wire                    a_S_PENABLE = S_PENABLE[active];
    wire  [BUS_WIDTH-1:0]   a_S_PWDATA  = S_PWDATA [active*BUS_WIDTH +: BUS_WIDTH];
    wire  [BUS_WIDTH-1:0]   a_S_PRDATA  = S_PRDATA [active*BUS_WIDTH +: BUS_WIDTH];
    wire                    a_S_PREADY  = S_PREADY [active];

    always @(active)
        $display("active core: %h", active);

    // Arbiter
    reg [SLAVE_PORTS-1:0] active = 0;
    always @(*)
        casez (S_PENABLE)
            //4'b???1 : active <= 4'b0001;
            //4'b??10 : active <= 4'b0010;
            //4'b?100 : active <= 4'b0100;
            //4'b1000 : active <= 4'b1000;
            //4'b0000 : active <= 4'b0000;
            4'b???1 : active = 0;
            4'b??10 : active = 1;
            4'b?100 : active = 2;
            4'b1000 : active = 3;
        endcase
    
    //assign M_PSELx[0] = (|S_PSELx) & (S_PADDR >= 16'h80 && S_PADDR <= 16'h8F);
    //assign M_PSELx[1] = (|S_PSELx) & (S_PADDR >= 16'h90 && S_PADDR <= 16'h9F);
    //assign M_PSELx[2] = (|S_PSELx) & (S_PADDR >= 16'hA0 && S_PADDR <= 16'hAF);
    //assign M_PSELx[3] = (|S_PSELx) & (S_PADDR >= 16'hB0 && S_PADDR <= 16'hB1);

    // GPIO0
    assign M_PSELx[`APB_PSELX_GPIO0] = |S_PSELx & (a_S_PADDR == 16'h00A0);
    // UART0
    assign M_PSELx[`APB_PSELX_UART0] = |S_PSELx & (a_S_PADDR >= 16'h00B0 
                                                && a_S_PADDR <= 16'h00B1);
    // REGS0
    assign M_PSELx[`APB_PSELX_REGS0] = |S_PSELx & (a_S_PADDR >= 16'h80 
                                                && a_S_PADDR <= 16'h8F);
    // BRAM0
    assign M_PSELx[`APB_PSELX_BRAM0] = |S_PSELx & (a_S_PADDR >= 16'hC0 
                                                && a_S_PADDR <= 16'hFF);


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