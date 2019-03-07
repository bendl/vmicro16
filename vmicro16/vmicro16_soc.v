
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

    vmicro16_regs_wb # (
            .CELL_WIDTH     (16),
            .CELL_DEPTH     (8),
            .CELL_SEL_BITS  (3),
            // Default soc conf registers
            .CELL_DEFAULTS  ("../../soc_sregs.txt"),
            .DEBUG_NAME     ("soc_regs")
    ) soc_regs (
            .clk           (clk),
            .reset         (reset),

            .wb_stb_i      (wb_mosi_stb_o_regs), 
            .wb_cyc_i      (wb_mosi_cyc_o), 
            .wb_we_i       (wb_mosi_we_o), 
            .wb_addr_i     (wb_mosi_addr_o), 
            .wb_data_i     (wb_mosi_data_o), 
            .wb_data_o     (wb_miso_data_i),
            .wb_ack_o      (wb_miso_ack_i)
            //.wb_stall_o    (wb_stall_o), 
            //.wb_err_o      (wb_err_o), 
    );
    

    vmicro16_cpu core (
        .clk    (clk),
        .reset  (reset),

        // Wishbone master interface
        .wb_mosi_stb_o_regs (wb_mosi_stb_o_regs),
        .wb_mosi_cyc_o      (wb_mosi_cyc_o),
        .wb_mosi_addr_o     (wb_mosi_addr_o),
        .wb_mosi_we_o       (wb_mosi_we_o),
        .wb_mosi_data_o     (wb_mosi_data_o),
        .wb_miso_data_i     (wb_miso_data_i),
        .wb_miso_ack_i      (wb_miso_ack_i)
    );

endmodule