
module vmicro16_soc #(
    parameter CORES     = 1,
    parameter APB_WIDTH = 16
) (
    input clk,
    input reset
);

    wire [CORES*APB_WIDTH-1:0] w_PADDR;
    wire [CORES-1:0]           w_PWRITE;
    wire [CORES-1:0]           w_PSELx;
    wire [CORES-1:0]           w_PENABLE;
    wire [CORES*APB_WIDTH-1:0] w_PWDATA;
    wire [CORES*APB_WIDTH-1:0] w_PRDATA;
    wire [CORES-1:0]           w_PREADY;
    apb_intercon_s # (
        .MASTER_PORTS(1),
        .SLAVE_PORTS (3)
    ) apb (
        .clk        (clk),
        // APB master to slave
        .S_PADDR    (w_PADDR),
        .S_PWRITE   (w_PWRITE),
        .S_PSELx    (w_PSELx),
        .S_PENABLE  (w_PENABLE),
        .S_PWDATA   (w_PWDATA),
        .S_PRDATA   (w_PRDATA),
        .S_PREADY   (w_PREADY)
    );

    genvar i;
    generate for(i = 0; i < CORES; i = i + 1) begin
        vmicro16_core c1 (
            .clk        (clk),
            .reset      (reset),

            .w_PADDR    (w_PADDR[APB_WIDTH*i +:APB_WIDTH]),
            .w_PWRITE   (w_PWRITE[i]),
            .w_PSELx    (w_PSELx[i]),
            .w_PENABLE  (w_PENABLE[i]),
            .w_PWDATA   (w_PWDATA[APB_WIDTH*i +:APB_WIDTH]),
            .w_PRDATA   (w_PRDATA[APB_WIDTH*i +:APB_WIDTH]),
            .w_PREADY   (w_PREADY[i])
        );
    end
    endgenerate


endmodule