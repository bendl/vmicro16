
// This file contains multiple modules. 
//   Verilator likes 1 file for each module
/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNUSED */
/* verilator lint_off BLKSEQ */
/* verilator lint_off WIDTH */

// Include Vmicro16 ISA containing definitions for the bits
`include "vmicro16_isa.v"

`include "clog2.v"

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
            $display($time, "\tMEM: W mem[%h] <= %h", mem_addr, mem_in);
        end else begin
            mem_out <= mem[mem_addr];
        end
    end

    // TODO: Reset impl = every clock while reset is asserted, clear each cell
    //       one at a time, mem[i++] <= 0
endmodule

module vmicro16_regs # (
    parameter CELL_WIDTH     = 16,
    parameter CELL_DEPTH     = 8,
    parameter CELL_SEL_BITS  = `clog2(CELL_DEPTH),
    parameter CELL_DEFAULTS  = 0,
    parameter DEBUG_NAME     = "",
    parameter SYNCHRONOUS    = 0,
    parameter DUAL_PORT_READ = 0
) (
    input clk, 
    input reset,
    // Dual port register reads
    input      [CELL_SEL_BITS-1:0]  rs1, // port 1
    output reg [CELL_WIDTH-1   :0]  rd1,
    input      [CELL_SEL_BITS-1:0]  rs2, // port 2
    output reg [CELL_WIDTH-1   :0]  rd2,
    // EX/WB final stage write back
    input                           we,
    input [CELL_SEL_BITS-1:0]       ws1,
    input [CELL_WIDTH-1:0]          wd
);
    reg [CELL_WIDTH-1:0] regs [0:CELL_DEPTH-1] /*verilator public_flat*/;
    
    // Initialise registers with default values
    //   Really only used for special registers used by the soc
    // TODO: How to do this on reset?
    initial if (CELL_DEFAULTS) $readmemh(CELL_DEFAULTS, regs);

    integer i;
    always @(posedge clk) 
        if (reset)
            for(i = 0; i < CELL_DEPTH; i = i + 1) 
                regs[i] <= {(CELL_WIDTH-1){1'b0}};
        
        else if (we) begin
            $display($time, "\tREGS #%s: Writing %h to reg[%d]", 
                DEBUG_NAME, wd, ws1);
            $display($time, "\t\t\t| %h %h %h %h | %h %h %h %h |", 
                regs[0], regs[1], regs[2], regs[3], 
                regs[4], regs[5], regs[6], regs[7]);
            
            // Perform the write
            regs[ws1] <= wd;
        end

    assign rd1 = regs[rs1];
    assign rd2 = regs[rs2];

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
    output [7:0]        imm8,
    output [11:0]           imm12,
    output [4:0]        simm5,

    // This can be freely increased without affecting the isa
    output reg [ALU_OP_WIDTH-1:0] alu_op,

    output reg has_imm8,
    output reg has_imm12,
    output reg has_we,
    output reg has_br,
    output reg has_mem,
    output reg has_mem_we,

    output halt
    
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
    // Special opcodes
    assign halt   = (opcode == `VMICRO16_OP_HALT);

    // exme_op
    always @(*) case (opcode)
    `VMICRO16_OP_HALT, // TODO: stop ifid
    `VMICRO16_OP_NOP:     alu_op = `VMICRO16_ALU_NOP;
    
    `VMICRO16_OP_LW:      alu_op = `VMICRO16_ALU_LW;
    `VMICRO16_OP_SW:      alu_op = `VMICRO16_ALU_SW;

    `VMICRO16_OP_MOV:     alu_op = `VMICRO16_ALU_MOV;
    `VMICRO16_OP_MOVI:    alu_op = `VMICRO16_ALU_MOVI;
    `VMICRO16_OP_MOVI_L:      alu_op = `VMICRO16_ALU_MOVI_L; 

    `VMICRO16_OP_BR:      alu_op = `VMICRO16_ALU_BR;
    
    `VMICRO16_OP_BIT:     casez (simm5)
        `VMICRO16_OP_BIT_OR:      alu_op = `VMICRO16_ALU_BIT_OR;
        `VMICRO16_OP_BIT_XOR:     alu_op = `VMICRO16_ALU_BIT_XOR;
        `VMICRO16_OP_BIT_AND:     alu_op = `VMICRO16_ALU_BIT_AND;
        `VMICRO16_OP_BIT_NOT:     alu_op = `VMICRO16_ALU_BIT_NOT;
        `VMICRO16_OP_BIT_LSHFT:   alu_op = `VMICRO16_ALU_BIT_LSHFT;
        `VMICRO16_OP_BIT_RSHFT:   alu_op = `VMICRO16_ALU_BIT_RSHFT;
        default:      alu_op = `VMICRO16_ALU_BAD; endcase

    `VMICRO16_OP_ARITH_U:     casez (simm5)
        `VMICRO16_OP_ARITH_UADD:  alu_op = `VMICRO16_ALU_ARITH_UADD;
        `VMICRO16_OP_ARITH_USUB:  alu_op = `VMICRO16_ALU_ARITH_USUB;
        `VMICRO16_OP_ARITH_UADDI: alu_op = `VMICRO16_ALU_ARITH_UADDI;
        default:      alu_op = `VMICRO16_ALU_BAD; endcase
    
    `VMICRO16_OP_ARITH_S:     casez (simm5)
        `VMICRO16_OP_ARITH_SADD:  alu_op = `VMICRO16_ALU_ARITH_SADD;
        `VMICRO16_OP_ARITH_SSUB:  alu_op = `VMICRO16_ALU_ARITH_SSUB;
        `VMICRO16_OP_ARITH_SSUBI: alu_op = `VMICRO16_ALU_ARITH_SSUBI; 
        default:      alu_op = `VMICRO16_ALU_BAD; endcase
    
    default: begin
        alu_op = `VMICRO16_ALU_BAD;
        $display($time, "\tDEC: unknown opcode: %h", opcode);
    end
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
    `VMICRO16_OP_SETC:      has_we = 1'b1;
    default:        has_we = 1'b0;
    endcase

    // Contains 8-bit immediate
    always @(*) case (opcode)
    `VMICRO16_OP_MOVI,
    `VMICRO16_OP_CMP:       has_imm8 = 1'b1;
    default:        has_imm8 = 1'b0;
    endcase

    // Contains 12-bit immediate
    always @(*) case (opcode)
    `VMICRO16_OP_MOVI_L:    has_imm12 = 1'b1;
    default:        has_imm12 = 1'b0;
    endcase
    
    // Will branch the pc
    always @(*) case (opcode)
    `VMICRO16_OP_BR:    has_br = 1'b1;
    default:        has_br = 1'b0;
    endcase
    
    // Requires external memory
    always @(*) case (opcode)
    `VMICRO16_OP_LW,
    `VMICRO16_OP_SW:    has_mem = 1'b1;
    default:        has_mem = 1'b0;
    endcase
    
    // Requires external memory write
    always @(*) case (opcode)
    `VMICRO16_OP_SW:    has_mem_we = 1'b1;
    default:        has_mem_we = 1'b0;
    endcase
endmodule

module vmicro16_alu # (
    parameter OP_WIDTH   = 5,
    parameter DATA_WIDTH = 16
) (
    // input clk, // TODO: make clocked

    input      [OP_WIDTH-1:0]   op,
    input      [DATA_WIDTH-1:0] a, // rs1/dst
    input      [DATA_WIDTH-1:0] b, // rs2
    output reg [DATA_WIDTH-1:0] c
);
    always @(*) case (op)
        // branch/nop, output nothing
        `VMICRO16_ALU_BR,
        `VMICRO16_ALU_NOP:          c = 0;
        // load/store addresses (use value in rd2)
        `VMICRO16_ALU_LW,
        `VMICRO16_ALU_SW:           c = b;
        // bitwise operations
        `VMICRO16_ALU_BIT_OR:       c = a | b;
        `VMICRO16_ALU_BIT_XOR:      c = a ^ b;
        `VMICRO16_ALU_BIT_AND:      c = a & b;
        `VMICRO16_ALU_BIT_NOT:      c = ~(b);
        `VMICRO16_ALU_BIT_LSHFT:    c = a << b;
        `VMICRO16_ALU_BIT_RSHFT:    c = a >> b;

        `VMICRO16_ALU_MOV:          c = b;
        `VMICRO16_ALU_MOVI:         c = b;
        `VMICRO16_ALU_MOVI_L:       c = b;

        `VMICRO16_ALU_ARITH_UADD:   c = a + b;
        `VMICRO16_ALU_ARITH_USUB:   c = a - b;
        // TODO: ALU should have simm5 as input
        `VMICRO16_ALU_ARITH_UADDI:  c = a + b;
        
        `VMICRO16_ALU_ARITH_SADD:   c = $signed(a) + $signed(b);
        `VMICRO16_ALU_ARITH_SSUB:   c = $signed(a) - $signed(b);
        // TODO: ALU should have simm5 as input
        `VMICRO16_ALU_ARITH_SSUBI:  c = $signed(a) + $signed(b);

        // TODO: Parameterise
        default: begin
            $display($time, "\tALU: unknown op: %h", op);
            c = {(DATA_WIDTH-1){1'bZZZZ}};
        end
    endcase
endmodule


module vmicro16_core # (
    parameter MEM_INSTR_DEPTH   = 64,
    parameter MEM_SCRATCH_DEPTH = 64,
) (
    input       clk,
    input       reset
);
    reg  [2:0] r_state;
    localparam STATE_IF = 0;
    localparam STATE_ME = 1;
    localparam STATE_WB = 2;

    reg  [15:0] r_pc;
    reg  [15:0] r_instr;
    wire [15:0] w_mem_instr_out;

    reg  [4:0]  r_instr_alu_op;
    reg  [2:0]  r_instr_rsd;
    reg  [2:0]  r_instr_rsa;
    reg  [15:0] r_instr_rsd;
    reg  [15:0] r_instr_rsa;

    reg  [15:0] r_mem_scratch_addr;
    reg  [15:0] r_mem_scratch_in;
    wire [15:0] r_mem_scratch_out;
    reg         r_mem_scratch_we;

    always @(posedge clk)
        if (reset) begin
            r_pc    <= 0;
            r_state <= STATE_IF;
        end else begin
            if (r_state == STATE_IF) begin
                r_pc    <= r_pc + 1;
                r_instr <= w_mem_instr_out;
            end
        end

    vmicro16_bram # (
        .MEM_WIDTH(15),
        .MEM_DEPTH(MEM_INSTR_DEPTH)
    ) mem_instr (
        .clk        (clk), 
        .reset      (reset), 
        // port 1
        .mem_addr   (r_pc), 
        .mem_in     (16'hXX), 
        .mem_we     (0), 
        .mem_out    (w_mem_instr_out)
    );

    vmicro16_bram # (
        .MEM_WIDTH(15),
        .MEM_DEPTH(MEM_SCRATCH_DEPTH)
    ) mem_scratch (
        .clk        (clk), 
        .reset      (reset), 
        // port 1
        .mem_addr   (r_mem_scratch_addr), 
        .mem_in     (r_mem_scratch_in), 
        .mem_we     (r_mem_scratch_we), 
        .mem_out    (r_mem_scratch_out)
    );


    vmicro16_dec (
        .instr (r_instr),
        .r_opcode (r_opcode),
        .r_rd     (r_instr_rd1)
    );


endmodule

