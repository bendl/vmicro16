
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
        input clk, input reset,
        
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

        // TODO: Reset impl = every clock while reset is asserted, clear each cell
        //       one at a time, mem[i++] <= 0
endmodule

// Wishbone wrapper around the register file to use as a peripheral
module vmicro16_regs_wb # (
        parameter CELL_WIDTH    = 16,
        parameter CELL_DEPTH    = 8,
        parameter CELL_SEL_BITS = 3,
        parameter CELL_DEFAULTS = 0,
        parameter DEBUG_NAME    = "",
        parameter PIPELINE_READ = 0
) (
        input clk, input reset,

        // wishbone slave interface
        input                   wb_stb_i,
        input                   wb_cyc_i,
        input                   wb_we_i,
        input  [CELL_WIDTH-1:0] wb_addr_i,
        input  [CELL_WIDTH-1:0] wb_data_i,
        output [CELL_WIDTH-1:0] wb_data_o,
        output                  wb_ack_o,
        output                  wb_stall_o,
        output                  wb_err_o
);
        // embedded register data
        wire [CELL_SEL_BITS-1:0] reg_rs1;
        wire [CELL_WIDTH-1:0]    reg_rd1;
        wire                     reg_we;
        wire [CELL_WIDTH-1:0]    reg_wd;

        reg selected;
        always @(*) begin
                if (wb_stb_i) 
                        selected = 1'b1;
                else if (selected && wb_cyc_i)
                        selected = 1'b1;
                else if (selected && !wb_cyc_i) 
                        selected = 1'b0;
                else    selected = 1'b0;
        end

        assign reg_we  = wb_we_i && wb_stb_i;
        assign reg_rs1 = wb_addr_i[CELL_SEL_BITS-1:0];
        assign reg_wd  = wb_data_i;
        
        // Only stall on write requests
        //assign wb_stall_o = wb_cyc_i && wb_we_i;
        // TODO: Stall for 1 clock if pipelined
        assign wb_stall_o = PIPELINE_READ ? wb_stb_i : 1'b0;
        // Only use the output bus when we're selected and it's a read req
        assign wb_data_o  = (selected && !wb_we_i)    ? reg_rd1 : {(CELL_WIDTH-1){1'bZ}};
        // TODO: ack on same stb clock allowed?
        assign wb_ack_o   = (wb_cyc_i && !wb_stall_o) ? 1'b1 : 1'bZ;

        vmicro16_regs # (
                .CELL_WIDTH     (CELL_WIDTH),
                .CELL_DEPTH     (CELL_DEPTH),
                .CELL_SEL_BITS  (CELL_SEL_BITS),
                .CELL_DEFAULTS  (CELL_DEFAULTS),
                .DEBUG_NAME     (DEBUG_NAME),
                .PIPELINE_READ  (PIPELINE_READ)
        ) vmicro16_regs_wb_i (
                 .clk           (clk)
                ,.reset         (reset)
                // Read port 1 (IDEX)
                ,.rs1           (reg_rs1)
                ,.rd1           (reg_rd1)
                // Read port 2 (IDEX) unused
                //,.rs2           (dec_rs2)
                //,.rd2           (reg_rd2)
                // Write port (MEWB)
                ,.we            (reg_we)
                ,.ws1           (reg_rs1)
                ,.wd            (reg_wd)
        );
endmodule

module vmicro16_regs # (
        parameter CELL_WIDTH    = 16,
        parameter CELL_DEPTH    = 8,
        parameter CELL_SEL_BITS = 3,
        parameter CELL_DEFAULTS = 0,
        parameter DEBUG_NAME    = "",
        parameter PIPELINE_READ = 0
) (
        input clk, input reset,
        // ID/EX stage reg reads
        // Dual port register reads
        input      [CELL_SEL_BITS-1:0] rs1, // port 1
        output reg [CELL_WIDTH-1:0]    rd1,
        input      [CELL_SEL_BITS-1:0] rs2, // port 2
        output reg [CELL_WIDTH-1:0]    rd2,
        // EX/WB final stage write back
        input                          we,
        input [CELL_SEL_BITS-1:0]      ws1,
        input [CELL_WIDTH-1:0]         wd
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
                        else for(i = 0; i < CELL_DEPTH; i = i + 1) regs[i] <= {(CELL_WIDTH-1){1'b0}}; 
                else if (we) begin
                        $display("%s: Writing %h to reg[%d]", DEBUG_NAME, wd, ws1);
                        regs[ws1] <= wd;
                end

        generate if (PIPELINE_READ) 
                always @(posedge clk) 
                        if (reset) begin
                                rd1 <= {(CELL_WIDTH-1){1'b0}};
                                rd2 <= {(CELL_WIDTH-1){1'b0}};
                        end else begin
                                rd1 <= regs[rs1];
                                rd2 <= regs[rs2];
                        end
        else
                always @(*) begin
                        assign rd1 = regs[rs1];
                        assign rd2 = regs[rs2];
                end
        endgenerate
endmodule


// Decoder is hard to parameterise as it's very closely linked to the ISA.
module vmicro16_dec # (
        parameter INSTR_WIDTH    = 16,
        parameter INSTR_OP_WIDTH = 5,
        parameter INSTR_RS_WIDTH = 3,
        parameter ALU_OP_WIDTH   = 5
) (
        //input clk,   // not used yet (all combinational)
        //input reset, // not used yet (all combinational)

        input  [INSTR_WIDTH-1:0]    instr,

        output [INSTR_OP_WIDTH-1:0] opcode,
        output [INSTR_RS_WIDTH-1:0] rd,
        output [INSTR_RS_WIDTH-1:0] ra,
        output [7:0]                imm8,
        output [11:0]               imm12,
        output [4:0]                simm5,

        // This can be freely increased without affecting the isa
        output reg [ALU_OP_WIDTH-1:0] exme_op,

        output reg has_imm8,
        output reg has_imm12,
        output reg has_we,
        output reg has_br,
        output reg has_mem,
        output reg has_mem_we
        
        // TODO: Use to identify bad instruction and
        //       raise exceptions
        //,output     is_bad 
);
        assign opcode = instr[15:11];
        assign rd     = instr[10:8];
        assign ra     = instr[7:5];
        assign imm8   = instr[7:0];
        assign imm12  = instr[11:0];
        assign simm5  = instr[4:0];

        // exme_op
        always @(*) case (opcode)
            default:                      exme_op = `VMICRO16_ALU_BAD;
            `VMICRO16_OP_NOP:             exme_op = `VMICRO16_ALU_NOP;
        
            `VMICRO16_OP_LW:              exme_op = `VMICRO16_ALU_LW;
            `VMICRO16_OP_SW:              exme_op = `VMICRO16_ALU_SW;

            `VMICRO16_OP_MOV:             exme_op = `VMICRO16_ALU_MOV;
            `VMICRO16_OP_MOVI:            exme_op = `VMICRO16_ALU_MOVI;
            `VMICRO16_OP_MOVI_L:          exme_op = `VMICRO16_ALU_MOVI_L; 
             
            `VMICRO16_OP_BIT:         casez (simm5)
                `VMICRO16_OP_BIT_OR:      exme_op = `VMICRO16_ALU_BIT_OR;
                `VMICRO16_OP_BIT_XOR:     exme_op = `VMICRO16_ALU_BIT_XOR;
                `VMICRO16_OP_BIT_AND:     exme_op = `VMICRO16_ALU_BIT_AND;
                `VMICRO16_OP_BIT_NOT:     exme_op = `VMICRO16_ALU_BIT_NOT;
                `VMICRO16_OP_BIT_LSHFT:   exme_op = `VMICRO16_ALU_BIT_LSHFT;
                `VMICRO16_OP_BIT_RSHFT:   exme_op = `VMICRO16_ALU_BIT_RSHFT;
                default:                  exme_op = `VMICRO16_ALU_BAD; endcase

            `VMICRO16_OP_ARITH_U:     casez (simm5)
                `VMICRO16_OP_ARITH_UADD:  exme_op = `VMICRO16_ALU_ARITH_UADD;
                `VMICRO16_OP_ARITH_USUB:  exme_op = `VMICRO16_ALU_ARITH_USUB;
                `VMICRO16_OP_ARITH_UADDI: exme_op = `VMICRO16_ALU_ARITH_UADDI;
                default:                  exme_op = `VMICRO16_ALU_BAD; endcase
            
            `VMICRO16_OP_ARITH_S:     casez (simm5)
                `VMICRO16_OP_ARITH_SADD:  exme_op = `VMICRO16_ALU_ARITH_SADD;
                `VMICRO16_OP_ARITH_SSUB:  exme_op = `VMICRO16_ALU_ARITH_SSUB;
                `VMICRO16_OP_ARITH_SSUBI: exme_op = `VMICRO16_ALU_ARITH_SSUBI; 
                default:                  exme_op = `VMICRO16_ALU_BAD; endcase
        endcase

        // Register writes
        always @(*) case (opcode)
            `VMICRO16_OP_LW,
            `VMICRO16_OP_MOV,
            `VMICRO16_OP_MOVI,
            `VMICRO16_OP_MOVI_L,
            `VMICRO16_OP_ARITH_U,
            `VMICRO16_OP_ARITH_S,
            `VMICRO16_OP_CMP,
            `VMICRO16_OP_SETC:          has_we = 1'b1;
            default:                    has_we = 1'b0;
        endcase

        // Contains 8-bit immediate
        always @(*) case (opcode)
            `VMICRO16_OP_MOVI,
            `VMICRO16_OP_CMP:           has_imm8 = 1'b1;
            default:                    has_imm8 = 1'b0;
        endcase

        // Contains 12-bit immediate
        always @(*) case (opcode)
            `VMICRO16_OP_MOVI_L:        has_imm12 = 1'b1;
            default:                    has_imm12 = 1'b0;
        endcase
        
        // Will branch the pc
        always @(*) case (opcode)
            `VMICRO16_OP_BR:            has_br = 1'b1;
            default:                    has_br = 1'b0;
        endcase
        
        // Requires external memory
        always @(*) case (opcode)
            `VMICRO16_OP_LW,
            `VMICRO16_OP_SW:            has_mem = 1'b1;
            default:                    has_mem = 1'b0;
        endcase
endmodule


module vmicro16_alu # (
    parameter OP_WIDTH   = 5,
    parameter DATA_WIDTH = 16
) (
        // input clk, // TODO: make clocked

        input      [OP_WIDTH-1:0]   op,
        input      [DATA_WIDTH-1:0] d1, // rs1/dst
        input      [DATA_WIDTH-1:0] d2, // rs2
        output reg [DATA_WIDTH-1:0] q
);
        always @(*) case (op)
            `VMICRO16_ALU_LW,
            `VMICRO16_ALU_SW:           q = d2;

            `VMICRO16_ALU_BIT_OR:       q = d1 | d2;
            `VMICRO16_ALU_BIT_XOR:      q = d1 ^ d2;
            `VMICRO16_ALU_BIT_AND:      q = d1 & d2;
            `VMICRO16_ALU_BIT_NOT:      q = ~(d2);
            `VMICRO16_ALU_BIT_LSHFT:    q = d1 << d2;
            `VMICRO16_ALU_BIT_RSHFT:    q = d1 >> d2;

            `VMICRO16_ALU_MOV:          q = d2;
            `VMICRO16_ALU_MOVI:         q = d2;
            `VMICRO16_ALU_MOVI_L:       q = d2;

            `VMICRO16_ALU_ARITH_UADD:   q = d1 + d2;
            `VMICRO16_ALU_ARITH_USUB:   q = d1 - d2;
            // TODO: ALU should have simm5 as input
            `VMICRO16_ALU_ARITH_UADDI:  q = d1 + d2;
            
            `VMICRO16_ALU_ARITH_SADD:   q = $signed(d1) + $signed(d2);
            `VMICRO16_ALU_ARITH_SSUB:   q = $signed(d1) - $signed(d2);
            // TODO: ALU should have simm5 as input
            `VMICRO16_ALU_ARITH_SSUBI:  q = $signed(d1) + $signed(d2);

            // TODO: Parameterise
            default:                    q = {(DATA_WIDTH-1){1'bX}};
        endcase
endmodule

