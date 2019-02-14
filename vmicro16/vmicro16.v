
// This file contains multiple modules. 
//   Verilator likes 1 file for each module
/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNUSED */
/* verilator lint_off BLKSEQ */
/* verilator lint_off WIDTH */

// This module aims to be a SYNCHRONOUS, WRITE_FIRST BLOCK RAM
//   https://www.xilinx.com/support/documentation/user_guides/ug473_7Series_Memory_Resources.pdf
//   https://www.xilinx.com/support/documentation/user_guides/ug383.pdf
//   https://www.xilinx.com/support/documentation/sw_manuals/xilinx2016_4/ug901-vivado-synthesis.pdf
module vmicro16_bram # (
        parameter MEM_WIDTH    = 16,
        parameter MEM_DEPTH    = 256
) (
        input clk,
        input reset,
        
        input      [MEM_WIDTH-1:0] mem_addr,
        input      [MEM_WIDTH-1:0] mem_in,
        input                      mem_we,
        output reg [MEM_WIDTH-1:0] mem_out
);
        // memory vector
        reg [MEM_WIDTH-1:0] mem [0:MEM_DEPTH-1];

        // not synthesizable
        integer i;
        initial for (i = 0; i < MEM_DEPTH; i = i + 1) mem[i] <= 0;

        always @(posedge clk) begin
                // synchronous WRITE_FIRST (page 13)
                if (mem_we) begin
                        mem[mem_addr] <= mem_in;
                        $display("bram: W mem[%h] <= %h", mem_addr, mem_in);
                end else begin
                        mem_out <= mem[mem_addr];
                        //$display("bram: R mem[%h]", mem[mem_addr]);
                end
        end
endmodule