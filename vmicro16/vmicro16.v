
// This file contains multiple modules. 
//   Verilator likes 1 file for each module
/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNUSED */
/* verilator lint_off BLKSEQ */
/* verilator lint_off WIDTH */

// Include Vmicro16 ISA containing definitions for the bits
`include "vmicro16_isa.v"

`include "clog2.v"
`include "formal.v"



// This module aims to be a SYNCHRONOUS, WRITE_FIRST BLOCK RAM
//   https://www.xilinx.com/support/documentation/user_guides/ug473_7Series_Memory_Resources.pdf
//   https://www.xilinx.com/support/documentation/user_guides/ug383.pdf
//   https://www.xilinx.com/support/documentation/sw_manuals/xilinx2016_4/ug901-vivado-synthesis.pdf
module vmicro16_bram # (
    parameter MEM_WIDTH     = 16,
    parameter MEM_DEPTH     = 64,
    parameter CORE_ID       = 0,
    parameter USE_INITS     = 0,
    parameter PARAM_DEFAULTS_R0 = 0,
    parameter PARAM_DEFAULTS_R1 = 0,
    parameter PARAM_DEFAULTS_R2 = 0,
    parameter PARAM_DEFAULTS_R3 = 0,
    parameter NAME          = "BRAM"
) (
    input clk, 
    input reset,
    
    input      [`clog2(MEM_DEPTH)-1:0] mem_addr,
    input      [MEM_WIDTH-1:0]         mem_in,
    input                              mem_we,
    output reg [MEM_WIDTH-1:0]         mem_out
);
    // memory vector
    (* ram_style = "block" *)
    reg [MEM_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    // not synthesizable
    integer i;
    initial begin
        for (i = 0; i < MEM_DEPTH; i = i + 1) mem[i] = 0;
        mem[0] = PARAM_DEFAULTS_R0;
        mem[1] = PARAM_DEFAULTS_R1;
        mem[2] = PARAM_DEFAULTS_R2;
        mem[3] = PARAM_DEFAULTS_R3;

        if (USE_INITS) begin
            //`define TEST_SW
            `ifdef TEST_SW
            $readmemh("E:\\Projects\\uni\\vmicro16\\sw\\verilog_memh.txt", mem);
            `endif

            `define TEST_ASM
            `ifdef TEST_ASM
            $readmemh("E:\\Projects\\uni\\vmicro16\\sw\\asm.s.hex", mem);
            `endif
            
            //`define TEST_COND
            `ifdef TEST_COND
            mem[0] = {`VMICRO16_OP_MOVI,    3'h7, 8'hC0}; // lock
            mem[0] = {`VMICRO16_OP_MOVI,    3'h7, 8'hC0}; // lock
            `endif

            //`define TEST_CMP
            `ifdef TEST_CMP
            mem[0] = {`VMICRO16_OP_MOVI,    3'h0, 8'h0A};
            mem[1] = {`VMICRO16_OP_MOVI,    3'h1, 8'h0B};
            mem[2] = {`VMICRO16_OP_CMP,     3'h1, 3'h0, 5'h1};
            `endif

            //`define TEST_LWEX
            `ifdef TEST_LWEX
            mem[0] = {`VMICRO16_OP_MOVI,    3'h0, 8'hC5};
            mem[1] = {`VMICRO16_OP_SW,      3'h0, 3'h0, 5'h1};
            mem[2] = {`VMICRO16_OP_LW,      3'h2, 3'h0, 5'h1};
            mem[3] = {`VMICRO16_OP_LWEX,    3'h2, 3'h0, 5'h1};
            mem[4] = {`VMICRO16_OP_SWEX,    3'h3, 3'h0, 5'h1};
            `endif

            //`define TEST_MULTICORE
            `ifdef TEST_MULTICORE
            mem[0] = {`VMICRO16_OP_MOVI,    3'h0, 8'h90};
            mem[1] = {`VMICRO16_OP_MOVI,    3'h1, 8'h33};
            mem[2] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
            mem[3] = {`VMICRO16_OP_MOVI,    3'h0, 8'h80};
            mem[4] = {`VMICRO16_OP_LW,      3'h2, 3'h0, 5'h0};
            mem[5] = {`VMICRO16_OP_MOVI,    3'h1, 8'h33};
            mem[6] = {`VMICRO16_OP_MOVI,    3'h1, 8'h33};
            mem[7] = {`VMICRO16_OP_MOVI,    3'h1, 8'h33};
            mem[8] = {`VMICRO16_OP_MOVI,    3'h0, 8'h91};
            mem[9] = {`VMICRO16_OP_SW,      3'h2, 3'h0, 5'h0};
            `endif

            //`define TEST_BR
            `ifdef TEST_BR
            mem[0] = {`VMICRO16_OP_MOVI,    3'h0, 8'h0};
            mem[1] = {`VMICRO16_OP_MOVI,    3'h3, 8'h3};
            mem[2] = {`VMICRO16_OP_MOVI,    3'h1, 8'h2};
            mem[3] = {`VMICRO16_OP_ARITH_U, 3'h0, 3'h1, 5'b11111};
            mem[4] = {`VMICRO16_OP_BR,      3'h3, `VMICRO16_OP_BR_U};
            mem[5] = {`VMICRO16_OP_MOVI,    3'h0, 8'hFF};
            `endif
            
            //`define ALL_TEST
            `ifdef ALL_TEST
            // Standard all test
            // REGS0
            mem[0] = {`VMICRO16_OP_MOVI,    3'h0, 8'h81};
            mem[1] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0}; // MMU[0x81] = 6
            mem[2] = {`VMICRO16_OP_SW,      3'h2, 3'h0, 5'h1}; // MMU[0x82] = 6
            // GPIO0
            mem[3] = {`VMICRO16_OP_MOVI,    3'h0, 8'h90};
            mem[4] = {`VMICRO16_OP_MOVI,    3'h1, 8'hD};
            mem[5] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
            mem[6] = {`VMICRO16_OP_LW,      3'h2, 3'h0, 5'h0};
            // TIM0
            mem[7] = {`VMICRO16_OP_MOVI,    3'h0, 8'h07};
            mem[8] = {`VMICRO16_OP_LW,      3'h3, 3'h0, 5'h03};
            // UART0
            mem[9]  = {`VMICRO16_OP_MOVI,    3'h0, 8'hA0};      // UART0
            mem[10] = {`VMICRO16_OP_MOVI,    3'h1, 8'h41};      // ascii A
            mem[11] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0}; 
            mem[12] = {`VMICRO16_OP_MOVI,    3'h1, 8'h42}; // ascii B
            mem[13] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
            mem[14] = {`VMICRO16_OP_MOVI,    3'h1, 8'h43}; // ascii C
            mem[15] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
            mem[16] = {`VMICRO16_OP_MOVI,    3'h1, 8'h44}; // ascii D
            mem[17] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
            mem[18] = {`VMICRO16_OP_MOVI,    3'h1, 8'h45}; // ascii D
            mem[19] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
            mem[20] = {`VMICRO16_OP_MOVI,    3'h1, 8'h46}; // ascii E
            mem[21] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
            // BRAM0
            mem[22] = {`VMICRO16_OP_MOVI,    3'h0, 8'hC0};
            mem[23] = {`VMICRO16_OP_MOVI,    3'h1, 8'hA};
            mem[24] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h5};
            mem[25] = {`VMICRO16_OP_LW,      3'h2, 3'h0, 5'h5};
            // GPIO1 (SSD 24-bit port)
            mem[26] = {`VMICRO16_OP_MOVI,    3'h0, 8'h91};
            mem[27] = {`VMICRO16_OP_MOVI,    3'h1, 8'h12};
            mem[28] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
            mem[29] = {`VMICRO16_OP_LW,      3'h2, 3'h0, 5'h0};
            // GPIO2
            mem[30] = {`VMICRO16_OP_MOVI,    3'h0, 8'h92};
            mem[31] = {`VMICRO16_OP_MOVI,    3'h1, 8'h56};
            mem[32] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
            `endif

            //`define TEST_BRAM
            `ifdef TEST_BRAM
            // 2 core BRAM0 test
            mem[0] = {`VMICRO16_OP_MOVI,    3'h0, 8'hC0};
            mem[1] = {`VMICRO16_OP_MOVI,    3'h1, 8'hA};
            mem[2] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h5};
            mem[3] = {`VMICRO16_OP_LW,      3'h2, 3'h0, 5'h5};
            `endif
        end
    end

    always @(posedge clk) begin
        // synchronous WRITE_FIRST (page 13)
        if (mem_we) begin
            mem[mem_addr] <= mem_in;
            $display($time, "\t\t%s[%h] <= %h", 
                    NAME, mem_addr, mem_in);
        end else
            mem_out <= mem[mem_addr];
    end

    // TODO: Reset impl = every clock while reset is asserted, clear each cell
    //       one at a time, mem[i++] <= 0
endmodule


module vmicro16_core_mmu # (
    parameter MEM_WIDTH     = 16,
    parameter MEM_DEPTH     = 64,

    parameter CORE_ID       = 3'h0,
    parameter CORE_ID_BITS  = `clog2(`CORES)
) (
    input clk,
    input reset,
    
    input  req,
    output busy,
    
    // From core
    input      [MEM_WIDTH-1:0]  mmu_addr,
    input      [MEM_WIDTH-1:0]  mmu_in,
    input                       mmu_we,
    input                       mmu_lwex,
    input                       mmu_swex,
    output reg [MEM_WIDTH-1:0]  mmu_out,

    // interrupts
    output reg [`DATA_WIDTH*`DEF_NUM_INT-1:0] ints_vector,
    output reg [`DEF_NUM_INT-1:0]             ints_mask,

    // TO APB interconnect
    output reg [`APB_WIDTH-1:0]  M_PADDR,
    output reg                   M_PWRITE,
    output reg                   M_PSELx,
    output reg                   M_PENABLE,
    output reg [MEM_WIDTH-1:0]   M_PWDATA,
    // from interconnect
    input      [MEM_WIDTH-1:0]   M_PRDATA,
    input                        M_PREADY
);
    localparam MMU_STATE_T1  = 0;
    localparam MMU_STATE_T2  = 1;
    localparam MMU_STATE_T3  = 2;
    reg [1:0]  mmu_state      = MMU_STATE_T1;
    
    reg  [MEM_WIDTH-1:0] per_out = 0;
    wire [MEM_WIDTH-1:0] tim0_out;

    assign busy = req || (mmu_state == MMU_STATE_T2);

    // more luts than below but easier
    wire tim0_en = (mmu_addr >= `DEF_MMU_TIM0_S) 
                && (mmu_addr <= `DEF_MMU_TIM0_E);
    wire sreg_en = (mmu_addr >= `DEF_MMU_SREG_S) 
                && (mmu_addr <= `DEF_MMU_SREG_E);
    wire intv_en = (mmu_addr >= `DEF_MMU_INTSV_S) 
                && (mmu_addr <= `DEF_MMU_INTSV_E);
    wire intm_en = (mmu_addr >= `DEF_MMU_INTSM_S) 
                && (mmu_addr <= `DEF_MMU_INTSM_E);

    //wire tim0_en = ~mmu_addr[12] && ~mmu_addr[9] && ~mmu_addr[7];
    //wire sreg_en = mmu_addr[7] && ~mmu_addr[4] && ~mmu_addr[5];
    //wire intv_en = mmu_addr[8] && ~mmu_addr[3];
    //wire intm_en = mmu_addr[8] && mmu_addr[3];
    
    wire apb_en    = !(|{tim0_en, sreg_en, intv_en, intm_en});
    wire tim0_we   = (tim0_en && mmu_we);
    wire intv_we   = (intv_en && mmu_we);
    wire intm_we   = (intm_en && mmu_we);

    // Special register selects
    localparam SPECIAL_REGS = 8;
    wire [MEM_WIDTH-1:0] sr_val;

    // Interrupt vector and mask
    initial ints_vector = 0;
    initial ints_mask   = 0;
    wire [2:0] intv_addr = mmu_addr[`clog2(`DEF_NUM_INT)-1:0];
    always @(posedge clk)
        if (intv_we)
            ints_vector[intv_addr*`DATA_WIDTH +: `DATA_WIDTH] <= mmu_in;

    always @(posedge clk)
        if (intm_we)
            ints_mask <= mmu_in;
            

    always @(ints_vector)
        $display($time, 
                "\tC%d\t\tints_vector W: | %h %h %h %h | %h %h %h %h |",
                 CORE_ID, 
            ints_vector[0*`DATA_WIDTH +: `DATA_WIDTH],
            ints_vector[1*`DATA_WIDTH +: `DATA_WIDTH],
            ints_vector[2*`DATA_WIDTH +: `DATA_WIDTH],
            ints_vector[3*`DATA_WIDTH +: `DATA_WIDTH],
            ints_vector[4*`DATA_WIDTH +: `DATA_WIDTH],
            ints_vector[5*`DATA_WIDTH +: `DATA_WIDTH],
            ints_vector[6*`DATA_WIDTH +: `DATA_WIDTH],
            ints_vector[7*`DATA_WIDTH +: `DATA_WIDTH]
            );

    always @(intm_we)
        $display($time, "\tC%d\t\tintm_we W: %b", CORE_ID, ints_mask);

    // Output port
    always @(*)
        if      (tim0_en) mmu_out = tim0_out;
        else if (sreg_en) mmu_out = sr_val;
        else if (intv_en) mmu_out = ints_vector[mmu_addr[2:0]*`DATA_WIDTH 
                                                    +: `DATA_WIDTH];
        else if (intm_en) mmu_out = ints_mask;
        else              mmu_out = per_out;

    // APB master to slave interface
    always @(posedge clk)
        if (reset) begin
            mmu_state <= MMU_STATE_T1;
            M_PENABLE <= 0;
            M_PADDR   <= 0;
            M_PWDATA  <= 0;
            M_PSELx   <= 0;
            M_PWRITE  <= 0;
        end 
        else
            casex (mmu_state)
                MMU_STATE_T1: begin
                    if (req && apb_en) begin
                        M_PADDR   <= {mmu_lwex, 
                                      mmu_swex, 
                                      CORE_ID[CORE_ID_BITS-1:0], 
                                      mmu_addr[MEM_WIDTH-1:0]};
                        
                        M_PWDATA  <= mmu_in;
                        M_PSELx   <= 1;
                        M_PWRITE  <= mmu_we;

                        mmu_state <= MMU_STATE_T2;
                    end
                end

                `ifdef FIX_T3
                    MMU_STATE_T2: begin
                        M_PENABLE <= 1;
                        
                        if (M_PREADY == 1'b1) begin
                            mmu_state <= MMU_STATE_T3;
                        end
                    end

                    MMU_STATE_T3: begin
                        // Slave has output a ready signal (finished)
                        M_PENABLE <= 0;
                        M_PADDR   <= 0;
                        M_PWDATA  <= 0;
                        M_PSELx   <= 0;
                        M_PWRITE  <= 0;
                        // Clock the peripheral output into a reg,
                        //   to output on the next clock cycle
                        per_out   <= M_PRDATA;

                        mmu_state <= MMU_STATE_T1;
                    end
                `else
                    // No FIX_T3
                    MMU_STATE_T2: begin
                        if (M_PREADY == 1'b1) begin
                            M_PENABLE <= 0;
                            M_PADDR   <= 0;
                            M_PWDATA  <= 0;
                            M_PSELx   <= 0;
                            M_PWRITE  <= 0;
                            // Clock the peripheral output into a reg,
                            //   to output on the next clock cycle
                            per_out   <= M_PRDATA;

                            mmu_state <= MMU_STATE_T1;
                        end else begin
                            M_PENABLE <= 1;
                        end
                    end
                `endif
            endcase

    (* ram_style = "block" *)
    vmicro16_bram # (
        .MEM_WIDTH  (MEM_WIDTH),
        .MEM_DEPTH  (SPECIAL_REGS),
        .USE_INITS  (0),
        .PARAM_DEFAULTS_R0  (CORE_ID),
        .PARAM_DEFAULTS_R1  (`CORES),
        .PARAM_DEFAULTS_R2  (`APB_BRAM0_CELLS),
        .PARAM_DEFAULTS_R3  (`SLAVES),
        .NAME       ("ram_sr")
    ) ram_sr (
        .clk        (clk),
        .reset      (reset),
        .mem_addr   (mmu_addr[`clog2(SPECIAL_REGS)-1:0]),
        .mem_in     (),
        .mem_we     (),
        .mem_out    (sr_val)
    );

    // Each M core has a TIM0 scratch memory
    (* ram_style = "block" *)
    vmicro16_bram # (
        .MEM_WIDTH  (MEM_WIDTH),
        .MEM_DEPTH  (MEM_DEPTH),
        .USE_INITS  (0),
        .NAME       ("TIM0")
    ) TIM0 (
        .clk        (clk),
        .reset      (reset),
        .mem_addr   (mmu_addr[7:0]),
        .mem_in     (mmu_in),
        .mem_we     (tim0_we),
        .mem_out    (tim0_out)
    );
endmodule



module vmicro16_regs # (
    parameter CELL_WIDTH        = 16,
    parameter CELL_DEPTH        = 8,
    parameter CELL_SEL_BITS     = `clog2(CELL_DEPTH),
    parameter CELL_DEFAULTS     = 0,
    parameter DEBUG_NAME        = "",
    parameter CORE_ID           = 0,
    parameter PARAM_DEFAULTS_R0 = 16'h0000,
    parameter PARAM_DEFAULTS_R1 = 16'h0000
) (
    input clk, 
    input reset,
    // Dual port register reads
    input      [CELL_SEL_BITS-1:0]  rs1, // port 1
    output     [CELL_WIDTH-1   :0]  rd1,
    //input      [CELL_SEL_BITS-1:0]  rs2, // port 2
    //output     [CELL_WIDTH-1   :0]  rd2,
    // EX/WB final stage write back
    input                           we,
    input [CELL_SEL_BITS-1:0]       ws1,
    input [CELL_WIDTH-1:0]          wd
); 
    (* ram_style = "distributed" *)
    reg [CELL_WIDTH-1:0] regs [0:CELL_DEPTH-1] /*verilator public_flat*/;
    
    // Initialise registers with default values
    //   Really only used for special registers used by the soc
    // TODO: How to do this on reset?
    integer i;
    initial 
        if (CELL_DEFAULTS) 
            $readmemh(CELL_DEFAULTS, regs);
        else begin
            for(i = 0; i < CELL_DEPTH; i = i + 1) 
                regs[i] = 0;
            regs[0] = PARAM_DEFAULTS_R0;
            regs[1] = PARAM_DEFAULTS_R1;
            end

    `ifdef ICARUS
        always @(regs)
            $display($time, "\tC%02h\t\t| %h %h %h %h | %h %h %h %h |", 
                CORE_ID, 
                regs[0], regs[1], regs[2], regs[3], 
                regs[4], regs[5], regs[6], regs[7]);
    `endif

    always @(posedge clk) 
        if (reset) begin
            for(i = 0; i < CELL_DEPTH; i = i + 1) 
                regs[i] <= 0;
            regs[0] <= PARAM_DEFAULTS_R0;
            regs[1] <= PARAM_DEFAULTS_R1;
        end
        else if (we) begin
            $display($time, "\tC%02h: REGS #%s: Writing %h to reg[%d]", 
                CORE_ID, DEBUG_NAME, wd, ws1);
            
            // Perform the write
            regs[ws1] <= wd;
        end

    // sync writes, async reads
    assign rd1 = regs[rs1];
    //assign rd2 = regs[rs2];
endmodule

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
    output [3:0]                imm4,
    output [7:0]                imm8,
    output [11:0]               imm12,
    output [4:0]                simm5,

    // This can be freely increased without affecting the isa
    output reg [ALU_OP_WIDTH-1:0] alu_op,

    output reg has_imm4,
    output reg has_imm8,
    output reg has_imm12,
    output reg has_we,
    output reg has_br,
    output reg has_mem,
    output reg has_mem_we,
    output reg has_cmp,

    output halt,
    output intr,

    output reg has_lwex,
    output reg has_swex
    
    // TODO: Use to identify bad instruction and
    //       raise exceptions
    //,output     is_bad 
);
    assign opcode = instr[15:11];
    assign rd     = instr[10:8];
    assign ra     = instr[7:5];
    assign imm4   = instr[3:0];
    assign imm8   = instr[7:0];
    assign imm12  = instr[11:0];
    assign simm5  = instr[4:0];

    // exme_op
    always @(*) case (opcode)
        `VMICRO16_OP_SPCL: casez(instr[11:0])
            `VMICRO16_OP_SPCL_NOP,
            `VMICRO16_OP_SPCL_HALT,
            `VMICRO16_OP_SPCL_INTR:   alu_op = `VMICRO16_ALU_NOP;
            default:                  alu_op = `VMICRO16_ALU_NOP; endcase
        
        `VMICRO16_OP_LW:              alu_op = `VMICRO16_ALU_LW;
        `VMICRO16_OP_SW:              alu_op = `VMICRO16_ALU_SW;
        `VMICRO16_OP_LWEX:            alu_op = `VMICRO16_ALU_LW;
        `VMICRO16_OP_SWEX:            alu_op = `VMICRO16_ALU_SW;

        `VMICRO16_OP_MOV:             alu_op = `VMICRO16_ALU_MOV;
        `VMICRO16_OP_MOVI:            alu_op = `VMICRO16_ALU_MOVI;

        `VMICRO16_OP_BR:              alu_op = `VMICRO16_ALU_BR;
        `VMICRO16_OP_MULT:            alu_op = `VMICRO16_ALU_MULT;

        `VMICRO16_OP_CMP:             alu_op = `VMICRO16_ALU_CMP;
        `VMICRO16_OP_SETC:            alu_op = `VMICRO16_ALU_SETC;
        
        `VMICRO16_OP_BIT:     casez (simm5)
            `VMICRO16_OP_BIT_OR:      alu_op = `VMICRO16_ALU_BIT_OR;
            `VMICRO16_OP_BIT_XOR:     alu_op = `VMICRO16_ALU_BIT_XOR;
            `VMICRO16_OP_BIT_AND:     alu_op = `VMICRO16_ALU_BIT_AND;
            `VMICRO16_OP_BIT_NOT:     alu_op = `VMICRO16_ALU_BIT_NOT;
            `VMICRO16_OP_BIT_LSHFT:   alu_op = `VMICRO16_ALU_BIT_LSHFT;
            `VMICRO16_OP_BIT_RSHFT:   alu_op = `VMICRO16_ALU_BIT_RSHFT;
            default:                  alu_op = `VMICRO16_ALU_BAD; endcase

        `VMICRO16_OP_ARITH_U:     casez (simm5)
            `VMICRO16_OP_ARITH_UADD:  alu_op = `VMICRO16_ALU_ARITH_UADD;
            `VMICRO16_OP_ARITH_USUB:  alu_op = `VMICRO16_ALU_ARITH_USUB;
            `VMICRO16_OP_ARITH_UADDI: alu_op = `VMICRO16_ALU_ARITH_UADDI;
            default:                  alu_op = `VMICRO16_ALU_BAD; endcase
        
        `VMICRO16_OP_ARITH_S:     casez (simm5)
            `VMICRO16_OP_ARITH_SADD:  alu_op = `VMICRO16_ALU_ARITH_SADD;
            `VMICRO16_OP_ARITH_SSUB:  alu_op = `VMICRO16_ALU_ARITH_SSUB;
            `VMICRO16_OP_ARITH_SSUBI: alu_op = `VMICRO16_ALU_ARITH_SSUBI; 
            default:                  alu_op = `VMICRO16_ALU_BAD; endcase
        
        default: begin  
                                      alu_op = `VMICRO16_ALU_NOP;
            $display($time, "\tDEC: unknown opcode: %h ... NOPPING", opcode);
        end
    endcase

    // Special opcodes
    //assign nop  == ((opcode == `VMICRO16_OP_SPCL) & (~instr[0]));
    assign halt = ((opcode == `VMICRO16_OP_SPCL) &   instr[0]);
    assign intr = ((opcode == `VMICRO16_OP_SPCL) &   instr[1]);

    // Register writes
    always @(*) case (opcode)
        `VMICRO16_OP_LWEX,
        `VMICRO16_OP_SWEX,
        `VMICRO16_OP_LW,
        `VMICRO16_OP_MOV,
        `VMICRO16_OP_MOVI,
        //`VMICRO16_OP_MOVI_L,
        `VMICRO16_OP_ARITH_U,
        `VMICRO16_OP_ARITH_S,
        `VMICRO16_OP_SETC,
        `VMICRO16_OP_BIT,
        `VMICRO16_OP_MULT:      has_we = 1'b1;
        default:                has_we = 1'b0;
    endcase

    // Contains 4-bit immediate
    always @(*) 
        if( ((opcode == `VMICRO16_OP_ARITH_U) && (simm5[4] == 0)) ||
            ((opcode == `VMICRO16_OP_ARITH_S) && (simm5[4] == 0)) )
            has_imm4 = 1'b1;
        else
            has_imm4 = 1'b0;

    // Contains 8-bit immediate
    always @(*) case (opcode)
        `VMICRO16_OP_MOVI,
        `VMICRO16_OP_BR:        has_imm8 = 1'b1;
        default:                has_imm8 = 1'b0;
    endcase

    //// Contains 12-bit immediate
    //always @(*) case (opcode)
    //    `VMICRO16_OP_MOVI_L:    has_imm12 = 1'b1;
    //    default:                has_imm12 = 1'b0;
    //endcase
    
    // Will branch the pc
    always @(*) case (opcode)
        `VMICRO16_OP_BR:    has_br = 1'b1;
        default:            has_br = 1'b0;
    endcase
    
    // Requires external memory
    always @(*) case (opcode)
        `VMICRO16_OP_LW,
        `VMICRO16_OP_SW,
        `VMICRO16_OP_LWEX,
        `VMICRO16_OP_SWEX:  has_mem = 1'b1;
        default:            has_mem = 1'b0;
    endcase
    
    // Requires external memory write
    always @(*) case (opcode)
        `VMICRO16_OP_SW,
        `VMICRO16_OP_SWEX:  has_mem_we = 1'b1;
        default:            has_mem_we = 1'b0;
    endcase

    // Affects status registers (cmp instructions)
    always @(*) case (opcode)
        `VMICRO16_OP_CMP:   has_cmp = 1'b1;
        default:            has_cmp = 1'b0;
    endcase

    // Performs exclusive checks
    always @(*) case (opcode)
        `VMICRO16_OP_LWEX:   has_lwex = 1'b1;
        default:             has_lwex = 1'b0;
    endcase

    always @(*) case (opcode)
        `VMICRO16_OP_SWEX:   has_swex = 1'b1;
        default:             has_swex = 1'b0;
    endcase
endmodule


module vmicro16_alu # (
    parameter OP_WIDTH   = 5,
    parameter DATA_WIDTH = 16,
    parameter CORE_ID    = 0
) (
    // input clk, // TODO: make clocked

    input      [OP_WIDTH-1:0]   op,
    input      [DATA_WIDTH-1:0] a, // rs1/dst
    input      [DATA_WIDTH-1:0] b, // rs2
    input      [3:0]            flags,
    output reg [DATA_WIDTH-1:0] c
);
    localparam TOP_BIT = (DATA_WIDTH-1);
    // 17-bit register
    reg [DATA_WIDTH:0] cmp_tmp = 0; // = {carry, [15:0]}
    wire r_setc;

    always @(*) begin
			cmp_tmp = 0;
			case (op)
        // branch/nop, output nothing
        `VMICRO16_ALU_BR,
        `VMICRO16_ALU_NOP:          c = {DATA_WIDTH{1'b0}};
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

        `ifdef DEF_ALU_HW_MULT
            `VMICRO16_ALU_MULT:  c = a * b;
        `endif
        
        `VMICRO16_ALU_ARITH_SADD:   c = $signed(a) + $signed(b);
        `VMICRO16_ALU_ARITH_SSUB:   c = $signed(a) - $signed(b);
        // TODO: ALU should have simm5 as input
        `VMICRO16_ALU_ARITH_SSUBI:  c = $signed(a) - $signed(b);

        `VMICRO16_ALU_CMP: begin
            // TODO: Do a-b in 17-bit register
            //       Set zero, overflow, carry, signed bits in result
            cmp_tmp = a - b;
            c = 0;

            // N   Negative condition code flag
            // Z   Zero condition code flag
            // C   Carry condition code flag
            // V   Overflow condition code flag
            c[`VMICRO16_SFLAG_N] = cmp_tmp[TOP_BIT];
            c[`VMICRO16_SFLAG_Z] = (cmp_tmp == 0);
            c[`VMICRO16_SFLAG_C] = 0; //cmp_tmp[TOP_BIT+1]; // not used

            // Overflow flag
            // https://stackoverflow.com/questions/30957188/
            // https://github.com/bendl/prco304/blob/master/prco_core/rtl/prco_alu.v#L50
            case(cmp_tmp[TOP_BIT+1:TOP_BIT])
                2'b01:   c[`VMICRO16_SFLAG_V] = 1;
                2'b10:   c[`VMICRO16_SFLAG_V] = 1;
                default: c[`VMICRO16_SFLAG_V] = 0;
            endcase

            $display($time, "\tC%02h: ALU CMP: %h %h = %h = %b", CORE_ID, a, b, cmp_tmp, c[3:0]);
        end

        `VMICRO16_ALU_SETC: c = { {15{1'b0}}, r_setc };

        // TODO: Parameterise
        default: begin
            $display($time, "\tALU: unknown op: %h", op);
            c       = 0;
            cmp_tmp = 0;
        end
		endcase
		end

    branch setc_check (
        .flags      (flags),
        .cond       (b[7:0]),
        .en         (r_setc)
    );
endmodule

// flags = 4 bit r_cmp_flags register
// cond  = 8 bit VMICRO16_OP_BR_? value. See vmicro16_isa.v
module branch (
    input [3:0] flags,
    input [7:0] cond,
    output reg  en
);
    always @(*)
        case (cond)
            `VMICRO16_OP_BR_U:  en = 1; `VMICRO16_OP_BR_U:  en = 1;
            `VMICRO16_OP_BR_E:  en = (flags[`VMICRO16_SFLAG_Z] == 1);
            `VMICRO16_OP_BR_NE: en = (flags[`VMICRO16_SFLAG_Z] == 0);
            `VMICRO16_OP_BR_G:  en = (flags[`VMICRO16_SFLAG_Z] == 0) && 
                                     (flags[`VMICRO16_SFLAG_N] == flags[`VMICRO16_SFLAG_V]);
            `VMICRO16_OP_BR_L:  en = (flags[`VMICRO16_SFLAG_Z] != flags[`VMICRO16_SFLAG_N]);
            `VMICRO16_OP_BR_GE: en = (flags[`VMICRO16_SFLAG_Z] == flags[`VMICRO16_SFLAG_N]);
            `VMICRO16_OP_BR_LE: en = (flags[`VMICRO16_SFLAG_Z] == 1) || 
                                     (flags[`VMICRO16_SFLAG_N] != flags[`VMICRO16_SFLAG_V]);
            default:            en = 0;
        endcase
endmodule



module vmicro16_core # (
    parameter DATA_WIDTH        = 16,
    parameter MEM_INSTR_DEPTH   = 64,
    parameter MEM_SCRATCH_DEPTH = 64,
    parameter MEM_WIDTH         = 16,

    parameter CORE_ID           = 3'h0
) (
    input        clk,
    input        reset,

    output [7:0] dbug,

    output       halt,

    // interrupt sources
    input  [`DEF_NUM_INT-1:0]             ints,
    input  [`DEF_NUM_INT*`DATA_WIDTH-1:0] ints_data,
    output [`DEF_NUM_INT-1:0]             ints_ack,
    
    // APB master to slave interface (apb_intercon)
    output  [`APB_WIDTH-1:0]    w_PADDR,
    output                      w_PWRITE,
    output                      w_PSELx,
    output                      w_PENABLE,
    output  [DATA_WIDTH-1:0]    w_PWDATA,
    input   [DATA_WIDTH-1:0]    w_PRDATA,
    input                       w_PREADY

`ifndef DEF_CORE_HAS_INSTR_MEM
    , // APB master interface to slave instruction memory
    output reg [`APB_WIDTH-1:0]    w2_PADDR,
    output reg                     w2_PWRITE,
    output reg                     w2_PSELx,
    output reg                     w2_PENABLE,
    output reg [DATA_WIDTH-1:0]    w2_PWDATA,
    input      [DATA_WIDTH-1:0]    w2_PRDATA,
    input                          w2_PREADY
`endif
);
    localparam STATE_IF = 0;
    localparam STATE_R1 = 1;
    localparam STATE_R2 = 2;
    localparam STATE_ME = 3;
    localparam STATE_WB = 4;
    localparam STATE_FE = 5;
    localparam STATE_IDLE = 6;
    localparam STATE_HALT = 7;
    reg  [2:0] r_state = STATE_IF;

    reg  [DATA_WIDTH-1:0] r_pc          = 16'h0000;
    reg  [DATA_WIDTH-1:0] r_pc_saved    = 16'h0000;
    reg  [DATA_WIDTH-1:0] r_instr       = 16'h0000;
    wire [DATA_WIDTH-1:0] w_mem_instr_out;
    wire                  w_halt;

    assign dbug = {7'h00, w_halt};
    assign halt = w_halt;

    wire [4:0]            r_instr_opcode;
    wire [4:0]            r_instr_alu_op;
    wire [2:0]            r_instr_rsd;
    wire [2:0]            r_instr_rsa;
    reg  [DATA_WIDTH-1:0] r_instr_rdd = 0;
    reg  [DATA_WIDTH-1:0] r_instr_rda = 0;
    wire [3:0]            r_instr_imm4;
    wire [7:0]            r_instr_imm8;
    wire [4:0]            r_instr_simm5;
    wire                  r_instr_has_imm4;
    wire                  r_instr_has_imm8;
    wire                  r_instr_has_we;
    wire                  r_instr_has_br;
    wire                  r_instr_has_cmp;
    wire                  r_instr_has_mem;
    wire                  r_instr_has_mem_we;
    wire                  r_instr_halt;
    wire                  r_instr_has_lwex;
    wire                  r_instr_has_swex;

    wire [DATA_WIDTH-1:0] r_alu_out;

    wire [DATA_WIDTH-1:0] r_mem_scratch_addr = $signed(r_alu_out) + $signed(r_instr_simm5);
    wire [DATA_WIDTH-1:0] r_mem_scratch_in   = r_instr_rdd;
    wire [DATA_WIDTH-1:0] r_mem_scratch_out;
    wire                  r_mem_scratch_we   = r_instr_has_mem_we && (r_state == STATE_ME);
    reg                   r_mem_scratch_req  = 0;
    wire                  r_mem_scratch_busy;

    reg  [2:0]            r_reg_rs1 = 0;
    wire [DATA_WIDTH-1:0] r_reg_rd1_s;
    wire [DATA_WIDTH-1:0] r_reg_rd1_i;
    wire [DATA_WIDTH-1:0] r_reg_rd1 = regs_use_int ? r_reg_rd1_i : r_reg_rd1_s;
    //wire [15:0] r_reg_rd2;
    wire [DATA_WIDTH-1:0] r_reg_wd = (r_instr_has_mem) ? r_mem_scratch_out : r_alu_out;
    wire                  r_reg_we = r_instr_has_we && (r_state == STATE_WB);

    // branching
    wire        w_intr;
    wire        w_branch_en;
    wire        w_branching   = r_instr_has_br && w_branch_en;
    reg  [3:0]  r_cmp_flags   = 4'h00; // N, Z, C, V
    
    always @(r_cmp_flags)
        $display($time, "\tC%02h:\tALU CMP: %b", CORE_ID, r_cmp_flags);

    // 2 cycle register fetch
    always @(*) begin
        r_reg_rs1 = 0;
        if (r_state == STATE_R1)
            r_reg_rs1 = r_instr_rsd;
        else if (r_state == STATE_R2)
            r_reg_rs1 = r_instr_rsa;
        else
            r_reg_rs1 = 3'h0;
    end

    reg regs_use_int = 0;
    `ifdef DEF_ENABLE_INT
    wire [`DEF_NUM_INT*`DATA_WIDTH-1:0] ints_vector;
    wire [`DEF_NUM_INT-1:0]             ints_mask;
    wire                                has_int = ints & ints_mask;
    reg int_pending = 0;
    reg int_pending_ack = 0;
    always @(posedge clk)
        if (int_pending_ack)
            // We've now branched to the isr
            int_pending <= 0;
        else if (has_int)
            // Notify fsm to switch to the ints_vector at the last stage
            int_pending <= 1;
        else if (w_intr)
            // Return to Interrupt instruction called, 
            //   so we've finished with the interrupt
            int_pending <= 0;
    `endif

    // Next program counter logic
    reg [`DATA_WIDTH-1:0] next_pc = 0;
    always @(posedge clk)
        if (reset)
            r_pc <= 0;
        else if (r_state == STATE_WB) begin
            `ifdef DEF_ENABLE_INT
            if (int_pending) begin
                $display($time, "\tC%02h: Jumping to ISR: %h", 
                    CORE_ID, 
                    ints_vector[0 +: `DATA_WIDTH]);
                // TODO: check bounds
                // Save state
                r_pc_saved      <= r_pc + 1;
                regs_use_int    <= 1;
                int_pending_ack <= 1;
                // Jump to ISR
                r_pc            <= ints_vector[0 +: `DATA_WIDTH];
            end else if (w_intr) begin
                $display($time, "\tC%02h: Returning from ISR: %h", 
                    CORE_ID, r_pc_saved);

                // Restore state
                r_pc            <= r_pc_saved;
                regs_use_int    <= 0;
                int_pending_ack <= 0;
            end else
            `endif
            if (w_branching) begin
                $display($time, "\tC%02h: branching to %h", CORE_ID, r_instr_rdd);
                r_pc            <= r_instr_rdd;

                `ifdef DEF_ENABLE_INT
                    int_pending_ack <= 0;
                `endif
            end else if (r_pc < (MEM_INSTR_DEPTH-1)) begin
                // normal increment
                // pc <= pc + 1
                r_pc            <= r_pc + 1;

                `ifdef DEF_ENABLE_INT
                    int_pending_ack <= 0;
                `endif
            end
        end // end r_state == STATE_WB
        else if (r_state == STATE_HALT) begin
            `ifdef DEF_ENABLE_INT
            // Only an interrupt can return from halt
            // duplicate code form STATE_ME!
            if (int_pending) begin
                $display($time, "\tC%02h: Jumping to ISR: %h", CORE_ID, ints_vector[0 +: `DATA_WIDTH]);
                // TODO: check bounds
                // Save state
                r_pc_saved      <= r_pc;// + 1; HALT = stay with same PC
                regs_use_int    <= 1;
                int_pending_ack <= 1;
                // Jump to ISR
                r_pc            <= ints_vector[0 +: `DATA_WIDTH];
            end else if (w_intr) begin
                $display($time, "\tC%02h: Returning from ISR: %h", CORE_ID, r_pc_saved);
                r_pc            <= r_pc_saved;
                regs_use_int    <= 0;
                int_pending_ack <= 0;
            end
            `endif
        end
    
`ifndef DEF_CORE_HAS_INSTR_MEM
    initial w2_PSELx   = 0;
    initial w2_PENABLE = 0;
    initial w2_PADDR   = 0;
`endif

    // cpu state machine
    always @(posedge clk)
        if (reset) begin
            r_state           <= STATE_IF;
            r_instr           <= 0;
            r_mem_scratch_req <= 0;
            r_instr_rdd       <= 0;
            r_instr_rda       <= 0;
        end
        else begin

`ifdef DEF_CORE_HAS_INSTR_MEM
            if (r_state == STATE_IF) begin
                    r_instr <= w_mem_instr_out;

                    $display("");
                    $display($time, "\tC%02h: PC: %h",    CORE_ID, r_pc);
                    $display($time, "\tC%02h: INSTR: %h", CORE_ID, w_mem_instr_out);
                    
                    r_state <= STATE_R1;
            end
`else
            // wait for global instruction rom to give us our instruction
            if (r_state == STATE_IF) begin
                // wait for ready signal
                if (!w2_PREADY) begin
                    w2_PSELx   <= 1;
                    w2_PWRITE  <= 0;
                    w2_PENABLE <= 1;
                    w2_PWDATA  <= 0;
                    w2_PADDR   <= r_pc;
                end else begin
                    w2_PSELx   <= 0;
                    w2_PWRITE  <= 0;
                    w2_PENABLE <= 0;
                    w2_PWDATA  <= 0;

                    r_instr <= w2_PRDATA;

                    $display("");
                    $display($time, "\tC%02h: PC: %h",    CORE_ID, r_pc);
                    $display($time, "\tC%02h: INSTR: %h", CORE_ID, w2_PRDATA);
                    
                    r_state <= STATE_R1;
                end
            end
`endif

            else if (r_state == STATE_R1) begin
                if (w_halt) begin
                    $display("");
                    $display("");
                    $display($time, "\tC%02h: PC: %h HALT", CORE_ID, r_pc);
                    r_state <= STATE_HALT;
                end else begin
                    // primary operand
                    r_instr_rdd <= r_reg_rd1;
                    r_state     <= STATE_R2;
                end
            end
            else if (r_state == STATE_R2) begin
                // Choose secondary operand (register or immediate)
                if      (r_instr_has_imm8)  r_instr_rda <= r_instr_imm8;
                else if (r_instr_has_imm4)  r_instr_rda <= r_reg_rd1 + r_instr_imm4;
                else                        r_instr_rda <= r_reg_rd1;

                if (r_instr_has_mem) begin
                    r_state           <= STATE_ME;
                    // Pulse req
                    r_mem_scratch_req <= 1;
                end else
                    r_state <= STATE_WB;
            end
            else if (r_state == STATE_ME) begin
                // Pulse req
                r_mem_scratch_req <= 0;
                // Wait for MMU to finish
                if (!r_mem_scratch_busy) 
                    r_state <= STATE_WB;
            end
            else if (r_state == STATE_WB) begin
                if (r_instr_has_cmp) begin
                    $display($time, "\tC%02h: CMP: %h", CORE_ID, r_alu_out[3:0]);
                    r_cmp_flags <= r_alu_out[3:0];
                end

                r_state <= STATE_FE;
            end
            else if (r_state == STATE_FE) 
                r_state <= STATE_IF;
            else if (r_state == STATE_HALT) begin
                `ifdef DEF_ENABLE_INT
                    if (int_pending) begin
                        r_state <= STATE_FE;
                    end
                `endif
            end
        end

`ifdef DEF_CORE_HAS_INSTR_MEM
    // Instruction ROM
    (* rom_style = "distributed" *)
    vmicro16_bram # (
        .MEM_WIDTH      (DATA_WIDTH),
        .MEM_DEPTH      (MEM_INSTR_DEPTH),
        .CORE_ID        (CORE_ID),
        .USE_INITS      (1),
        .NAME           ("INSTR_MEM")
    ) mem_instr (
        .clk            (clk), 
        .reset          (reset), 
        // port 1       
        .mem_addr       (r_pc), 
        .mem_in         (0), 
        .mem_we         (1'b0),  // ROM
        .mem_out        (w_mem_instr_out)
    );
`endif

    // MMU
    vmicro16_core_mmu # (
        .MEM_WIDTH      (DATA_WIDTH),
        .MEM_DEPTH      (MEM_SCRATCH_DEPTH),
        .CORE_ID        (CORE_ID)
    ) mmu (
        .clk            (clk), 
        .reset          (reset), 
        .req            (r_mem_scratch_req),
        .busy           (r_mem_scratch_busy),
        // interrupts
        .ints_vector    (ints_vector),
        .ints_mask      (ints_mask),
        // port 1
        .mmu_addr       (r_mem_scratch_addr), 
        .mmu_in         (r_mem_scratch_in), 
        .mmu_we         (r_mem_scratch_we), 
        .mmu_lwex       (r_instr_has_lwex),
        .mmu_swex       (r_instr_has_swex),
        .mmu_out        (r_mem_scratch_out),
        // APB maste    r to slave
        .M_PADDR        (w_PADDR),
        .M_PWRITE       (w_PWRITE),
        .M_PSELx        (w_PSELx),
        .M_PENABLE      (w_PENABLE),
        .M_PWDATA       (w_PWDATA),
        .M_PRDATA       (w_PRDATA),
        .M_PREADY       (w_PREADY)
    );

    // Instruction decoder
    vmicro16_dec dec (
        // input
        .instr          (r_instr),
        // output async
        .opcode         (),
        .rd             (r_instr_rsd),
        .ra             (r_instr_rsa),
        .imm4           (r_instr_imm4),
        .imm8           (r_instr_imm8),
        .imm12          (),
        .simm5          (r_instr_simm5),
        .alu_op         (r_instr_alu_op),
        .has_imm4       (r_instr_has_imm4),
        .has_imm8       (r_instr_has_imm8),
        .has_we         (r_instr_has_we),
        .has_br         (r_instr_has_br),
        .has_cmp        (r_instr_has_cmp),
        .has_mem        (r_instr_has_mem),
        .has_mem_we     (r_instr_has_mem_we),
        .halt           (w_halt),
        .intr           (w_intr),
        .has_lwex       (r_instr_has_lwex),
        .has_swex       (r_instr_has_swex)
    );
    
    // Software registers
    vmicro16_regs # (
        .CORE_ID    (CORE_ID),
        .CELL_WIDTH (`DATA_WIDTH)
    ) regs (
        .clk        (clk),
        .reset      (reset),
        // async port 0
        .rs1        (r_reg_rs1),
        .rd1        (r_reg_rd1_s),
        // async port 1
        //.rs2        (),
        //.rd2        (),
        // write port
        .we         (r_reg_we && ~regs_use_int),
        .ws1        (r_instr_rsd),
        .wd         (r_reg_wd)
    );

    // Interrupt replacement registers
    `ifdef DEF_ENABLE_INT
    vmicro16_regs # (
        .CORE_ID    (CORE_ID),
        .CELL_WIDTH (`DATA_WIDTH),
        .DEBUG_NAME ("REGSINT")
    ) regs_intr (
        .clk        (clk),
        .reset      (reset),
        // async port 0
        .rs1        (r_reg_rs1),
        .rd1        (r_reg_rd1_i),
        // async port 1
        //.rs2        (),
        //.rd2        (),
        // write port
        .we         (r_reg_we && regs_use_int),
        .ws1        (r_instr_rsd),
        .wd         (r_reg_wd)
    );
    `endif

    // ALU
    vmicro16_alu # (
        .CORE_ID(CORE_ID)
    ) alu (
        .op         (r_instr_alu_op),
        .a          (r_instr_rdd),
        .b          (r_instr_rda),
        .flags      (r_cmp_flags),
        // async output
        .c          (r_alu_out)
    );

    branch branch_check (
        .flags      (r_cmp_flags),
        .cond       (r_instr_imm8),
        .en         (w_branch_en)
    );

endmodule

