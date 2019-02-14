
// This file contains multiple modules. 
//   Verilator likes 1 file for each module
/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNUSED */
/* verilator lint_off BLKSEQ */
/* verilator lint_off WIDTH */

`include "vmicro16_isa.v"

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


/* verilator lint_off UNUSED */
module vmicro16_dec (
        input         clk,   // not used yet (all combinational)
        input         reset, // not used yet (all combinational)

        input  [15:0] instr,

        output [4:0]  opcode,
        output [2:0]  rd,
        output [2:0]  ra,
        output [7:0]  imm8,
        output [11:0] imm12,
        output [4:0]  simm5,

        output reg [4:0] alu_op,

        output reg   has_imm8,
        output reg   has_imm12,
        output reg   is_we,
        output reg   is_jmp,
        output reg   is_mem,
        output reg   is_mem_we,
        
        // TODO: Use to identify bad instruction and
        //       raise exceptions
        output        is_bad 
);
        assign opcode    = instr[15:11];
        assign rd        = instr[10:8];
        assign ra        = instr[7:5];
        assign imm8      = instr[7:0];
        assign imm12     = instr[11:0];
        assign simm5     = instr[4:0];

        // alu_op
        always @(*) case (opcode)
            `VMICRO16_OP_NOP:               alu_op = `VMICRO16_ALU_NOP;
        
            `VMICRO16_OP_LW:                alu_op = `VMICRO16_ALU_LW;
            `VMICRO16_OP_SW:                alu_op = `VMICRO16_ALU_SW;

            `VMICRO16_OP_MOV:               alu_op = `VMICRO16_ALU_MOV;
            `VMICRO16_OP_MOVI:              alu_op = `VMICRO16_ALU_MOVI;
            `VMICRO16_OP_MOVI_L:            alu_op = `VMICRO16_ALU_MOVI_L; 
             
            `VMICRO16_OP_BIT: casez (simm5)
                `VMICRO16_OP_BIT_OR:        alu_op = `VMICRO16_ALU_BIT_OR;
                `VMICRO16_OP_BIT_XOR:       alu_op = `VMICRO16_ALU_BIT_XOR;
                `VMICRO16_OP_BIT_AND:       alu_op = `VMICRO16_ALU_BIT_AND;
                `VMICRO16_OP_BIT_NOT:       alu_op = `VMICRO16_ALU_BIT_NOT;
                `VMICRO16_OP_BIT_LSHFT:     alu_op = `VMICRO16_ALU_BIT_LSHFT;
                `VMICRO16_OP_BIT_RSHFT:     alu_op = `VMICRO16_ALU_BIT_RSHFT;
                default:                    alu_op = `VMICRO16_ALU_BAD; endcase

            `VMICRO16_OP_ARITH_U: casez (simm5)
                `VMICRO16_OP_ARITH_UADD:    alu_op = `VMICRO16_ALU_ARITH_UADD;
                `VMICRO16_OP_ARITH_USUB:    alu_op = `VMICRO16_ALU_ARITH_USUB;
                `VMICRO16_OP_ARITH_UADDI:   alu_op = `VMICRO16_ALU_ARITH_UADDI;
                default:                    alu_op = `VMICRO16_ALU_BAD; endcase
            
            `VMICRO16_OP_ARITH_S: casez (simm5)
                `VMICRO16_OP_ARITH_SADD:    alu_op = `VMICRO16_ALU_ARITH_SADD;
                `VMICRO16_OP_ARITH_SSUB:    alu_op = `VMICRO16_ALU_ARITH_SSUB;
                `VMICRO16_OP_ARITH_SSUBI:   alu_op = `VMICRO16_ALU_ARITH_SSUBI; 
                default:                    alu_op = `VMICRO16_ALU_BAD; endcase
        endcase

        always @(*) case (opcode)
            `VMICRO16_OP_LW,
            `VMICRO16_OP_MOV,
            `VMICRO16_OP_MOVI,
            `VMICRO16_OP_MOVI_L,
            `VMICRO16_OP_ARITH_U,
            `VMICRO16_OP_ARITH_S,
            `VMICRO16_OP_CMP,
            `VMICRO16_OP_SETC:
                     is_we = 1'b1;
            default: is_we = 1'b0;
        endcase

        always @(*) case (opcode)
            `VMICRO16_OP_MOVI,
            `VMICRO16_OP_CMP:
                     has_imm8 = 1'b1;
            default: has_imm8 = 1'b0;
        endcase

        always @(*) case (opcode)
            `VMICRO16_OP_MOVI_L:
                     has_imm12 = 1'b1;
            default: has_imm12 = 1'b0;
        endcase
        
        always @(*) case (opcode)
            `VMICRO16_OP_BR:
                     is_jmp = 1'b1;
            default: is_jmp = 1'b0;
        endcase
        
        always @(*) case (opcode)
            `VMICRO16_OP_LW,
            `VMICRO16_OP_SW:
                     is_mem = 1'b1;
            default: is_mem = 1'b0;
        endcase
endmodule

