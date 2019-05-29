
module vmicro16_soc (
    input clk,
    input reset
);
    // Internal wishbone master interface
    wire        wb_mosi_stb_o_regs;
    wire        wb_mosi_cyc_o;
    wire [15:0] wb_mosi_addr_o;
    wire        wb_mosi_we_o;
    wire [15:0] wb_mosi_data_o; // seperate data_o and data_i buses
    wire [15:0] wb_miso_data_i; // seperate data_o and data_i buses
    wire        wb_miso_ack_i;

    vmicro16_core # (
    ) c1 (
        .clk    (clk),
        .reset  (reset)
    );

endmodule