
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


module vmicro16_regs # (
        parameter CELL_WIDTH    = 16,
        parameter CELL_DEPTH    = 8,
        parameter CELL_SEL_BITS = 3,
        parameter CELL_DEFAULTS = 0,
        parameter DEBUG_NAME    = ""
) (
        input clk, input reset,
        // ID/EX stage reg reads
        // Dual port register reads
        input  [CELL_SEL_BITS-1:0] rs1, // port 1
        output [CELL_WIDTH-1:0]    rd1,
        input  [CELL_SEL_BITS-1:0] rs2, // port 2
        output [CELL_WIDTH-1:0]    rd2,
        // EX/WB final stage write back
        input                      we,
        input [CELL_SEL_BITS-1:0]  ws1,
        input [CELL_WIDTH-1:0]     wd
);
        reg [CELL_WIDTH-1:0] regs [0:CELL_DEPTH-1] /*verilator public_flat*/;
        
        // Initialise registers with default values
        //   Really only used for special registers used by the soc
        // TODO: How to do this on reset?
        initial if (CELL_DEFAULTS) $readmemh(CELL_DEFAULTS, regs);

        integer i;
        always @(posedge clk) 
                if (reset) 
                        if (CELL_DEFAULTS) $readmemh(CELL_DEFAULTS, regs); // TODO:
                        else for(i = 0; i < CELL_DEPTH; i = i + 1) regs[i] <= 16'h00; 
                else if (we) begin
                        $display("%s: Writing %h to reg[%d]", DEBUG_NAME, wd, ws1);
                        regs[ws1] <= wd;
                end

        // TODO: Should reading be continuous? Or 1 clock delayed?
        assign rd1 = regs[rs1];
        assign rd2 = regs[rs2];
endmodule