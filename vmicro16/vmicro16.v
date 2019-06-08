
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

(* keep_hierarchy = "yes" *)
(* dont_touch = "yes" *)
module vmicro16_bram_apb # (
    parameter BUS_WIDTH    = 16,
    parameter MEM_WIDTH    = 16,
    parameter MEM_DEPTH    = 64,
    parameter APB_PADDR    = 0
) (
    input clk,
    input reset,
    // APB Slave to master interface
    input  [`clog2(MEM_DEPTH)-1:0]  S_PADDR,
    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input  [BUS_WIDTH-1:0]          S_PWDATA,
    
    output [BUS_WIDTH-1:0]          S_PRDATA,
    output                          S_PREADY
);
    wire [MEM_WIDTH-1:0] mem_out;

    assign S_PRDATA = (S_PSELx & S_PENABLE) ? mem_out : 16'hZZZZ;
    assign S_PREADY = (S_PSELx & S_PENABLE) ? 1'b1    : 1'bZ;
    assign we       = (S_PSELx & S_PENABLE & S_PWRITE);

    always @(posedge clk)
        if (we)
            $display($time, "\t\tBRAM_APB[%h] <= %h", 
                S_PADDR, S_PWDATA);

    vmicro16_bram # (
        .MEM_WIDTH  (MEM_WIDTH),
        .MEM_DEPTH  (MEM_DEPTH)
    ) bram_apb (
        .clk        (clk),
        .reset      (reset),

        .mem_addr   (S_PADDR),
        .mem_in     (S_PWDATA),
        .mem_we     (we),
        .mem_out    (mem_out)
    );
endmodule


// This module aims to be a SYNCHRONOUS, WRITE_FIRST BLOCK RAM
//   https://www.xilinx.com/support/documentation/user_guides/ug473_7Series_Memory_Resources.pdf
//   https://www.xilinx.com/support/documentation/user_guides/ug383.pdf
//   https://www.xilinx.com/support/documentation/sw_manuals/xilinx2016_4/ug901-vivado-synthesis.pdf
(* keep_hierarchy = "yes" *)
module vmicro16_bram # (
    parameter MEM_WIDTH    = 16,
    parameter MEM_DEPTH    = 64
) (
    input clk, 
    input reset,
    
    input      [`clog2(MEM_DEPTH)-1:0] mem_addr,
    input      [MEM_WIDTH-1:0] mem_in,
    input                      mem_we,
    output reg [MEM_WIDTH-1:0] mem_out
);
    // memory vector
    reg [MEM_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    // not synthesizable
    integer i;
    initial begin
        for (i = 0; i < MEM_DEPTH; i = i + 1) 
            mem[i] = 0;
        //$readmemh("../../test.hex", mem);
        
        //// Standard all test
        //// REGS0
        //mem[0] = {`VMICRO16_OP_MOVI,    3'h0, 8'h81};
        //mem[1] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0}; // MMU[0x81] = 6
        //mem[2] = {`VMICRO16_OP_SW,      3'h2, 3'h0, 5'h1}; // MMU[0x82] = 6
        //// GPIO0
        //mem[3] = {`VMICRO16_OP_MOVI,    3'h0, 8'hA0};
        //mem[4] = {`VMICRO16_OP_MOVI,    3'h1, 8'hD};
        //mem[5] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
        //mem[6] = {`VMICRO16_OP_LW,      3'h2, 3'h0, 5'h0};
        //// TIM0
        //mem[7] = {`VMICRO16_OP_MOVI,    3'h0, 8'h07};
        //mem[8] = {`VMICRO16_OP_LW,      3'h3, 3'h0, 5'h03};
        //// UART0
        //mem[9]  = {`VMICRO16_OP_MOVI,    3'h0, 8'hB0};      // UART0
        //mem[10] = {`VMICRO16_OP_MOVI,    3'h1, 8'h41};      // ascii A
        //mem[11] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0}; 
        //mem[12] = {`VMICRO16_OP_MOVI,    3'h1, 8'h42}; // ascii B
        //mem[13] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
        //mem[14] = {`VMICRO16_OP_MOVI,    3'h1, 8'h43}; // ascii C
        //mem[15] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
        //mem[16] = {`VMICRO16_OP_MOVI,    3'h1, 8'h44}; // ascii D
        //mem[17] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
        //mem[18] = {`VMICRO16_OP_MOVI,    3'h1, 8'h45}; // ascii D
        //mem[19] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
        //mem[20] = {`VMICRO16_OP_MOVI,    3'h1, 8'h46}; // ascii E
        //mem[21] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h0};
        //// BRAM0
        //mem[22] = {`VMICRO16_OP_MOVI,    3'h0, 8'hC0};
        //mem[23] = {`VMICRO16_OP_MOVI,    3'h1, 8'hA};
        //mem[24] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h5};
        //mem[25] = {`VMICRO16_OP_LW,      3'h2, 3'h0, 5'h5};

        // 2 core BRAM0 test
        mem[0] = {`VMICRO16_OP_MOVI,    3'h0, 8'hC0};
        mem[1] = {`VMICRO16_OP_MOVI,    3'h1, 8'hA};
        mem[2] = {`VMICRO16_OP_SW,      3'h1, 3'h0, 5'h5};
        //mem[3] = {`VMICRO16_OP_LW,      3'h2, 3'h0, 5'h5};
    end

    always @(posedge clk) begin
        // synchronous WRITE_FIRST (page 13)
        if (mem_we) begin
            mem[mem_addr] <= mem_in;
            $display($time, "\t\tTIM0: W TIM0[%h] <= %h", mem_addr, mem_in);
        end else
            mem_out <= mem[mem_addr];
    end

    // TODO: Reset impl = every clock while reset is asserted, clear each cell
    //       one at a time, mem[i++] <= 0
endmodule

(* keep_hierarchy = "yes" *)
module vmicro16_core_mmu # (
    parameter MEM_WIDTH    = 16,
    parameter MEM_DEPTH    = 64,
    // TIM0 addr
    parameter ADDR_TIM0_S  = 16'h00,
    parameter ADDR_TIM0_E  = 16'h3F
) (
    input clk,
    input reset,
    
    input  req,
    output busy,
    
    // From core
    input      [MEM_WIDTH-1:0]  mmu_addr,
    input      [MEM_WIDTH-1:0]  mmu_in,
    input                       mmu_we,
    output reg [MEM_WIDTH-1:0]  mmu_out,

    // TO APB interconnect
    output reg [MEM_WIDTH-1:0]   M_PADDR,
    output reg                   M_PWRITE,
    output reg                   M_PSELx,
    output reg                   M_PENABLE,
    output reg [MEM_WIDTH-1:0]   M_PWDATA,
    // from interconnect
    input      [MEM_WIDTH-1:0]   M_PRDATA,
    input                        M_PREADY
);
    localparam TIM_BITS_ADDR = `clog2(MEM_DEPTH);
    localparam MMU_STATE_T1  = 0;
    localparam MMU_STATE_T2  = 1;
    localparam MMU_STATE_T3  = 2;
    reg [1:0] mmu_state      = MMU_STATE_T1;
    
    reg  [MEM_WIDTH-1:0] per_out = 0;
    wire [MEM_WIDTH-1:0] tim0_out;

    assign busy = req || (mmu_state == MMU_STATE_T2);

    // Output port
    always @(*)
        if (tim0_en)
            mmu_out = tim0_out;
        else
            mmu_out = per_out;

    // tightly integrated memory usage
    wire tim0_en = (mmu_addr >= ADDR_TIM0_S) && (mmu_addr <=  ADDR_TIM0_E);
    wire [TIM_BITS_ADDR-1:0] tim0_addr = (mmu_addr - ADDR_TIM0_S);
    wire tim0_we = (tim0_en && mmu_we);

    // APB master to slave interface
    always @(posedge clk) begin
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
                    if (req && (!tim0_en)) begin
                        M_PADDR   <= mmu_addr;
                        M_PWDATA  <= mmu_in;
                        M_PSELx   <= 1;
                        M_PWRITE  <= mmu_we;

                        mmu_state <= MMU_STATE_T2;
                    end
                end

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
            endcase
    end

    // Each M core has a TIM0 scratch memory
    (* keep_hierarchy = "yes" *)
    vmicro16_bram # (
        .MEM_WIDTH  (MEM_WIDTH),
        .MEM_DEPTH  (MEM_DEPTH)
    ) TIM0 (
        .clk        (clk),
        .reset      (reset),
        .mem_addr   (tim0_addr),
        .mem_in     (mmu_in),
        .mem_we     (tim0_we),
        .mem_out    (tim0_out)
    );
endmodule


(* keep_hierarchy = "yes" *)
module vmicro16_regs # (
    parameter CELL_WIDTH     = 16,
    parameter CELL_DEPTH     = 8,
    parameter CELL_SEL_BITS  = `clog2(CELL_DEPTH),
    parameter CELL_DEFAULTS  = 0,
    parameter DEBUG_NAME     = ""
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
    reg [CELL_WIDTH-1:0] regs [0:CELL_DEPTH-1] /*verilator public_flat*/;
    
    // Initialise registers with default values
    //   Really only used for special registers used by the soc
    // TODO: How to do this on reset?
    integer i;
    initial 
        if (CELL_DEFAULTS) 
            $readmemh(CELL_DEFAULTS, regs);
        else 
            for(i = 0; i < CELL_DEPTH; i = i + 1) 
                regs[i] <= i;

    always @(posedge clk) 
        if (reset)
            for(i = 0; i < CELL_DEPTH; i = i + 1) 
                regs[i] <= i;
        
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
    //assign rd2 = regs[rs2];
endmodule

(* keep_hierarchy = "yes" *)
(* dont_touch = "yes" *)
module vmicro16_regs_apb # (
    parameter BUS_WIDTH  = 16,
    parameter CELL_DEPTH = 8
) (
    input clk,
    input reset,
    // APB Slave to master interface
    input  [`clog2(CELL_DEPTH)-1:0] S_PADDR,
    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input  [BUS_WIDTH-1:0]          S_PWDATA,
    
    output [BUS_WIDTH-1:0]          S_PRDATA,
    output                          S_PREADY
);
    wire [BUS_WIDTH-1:0] rd1;

    assign S_PRDATA = (S_PSELx & S_PENABLE) ? rd1  : 16'hZZZZ;
    assign S_PREADY = (S_PSELx & S_PENABLE) ? 1'b1 : 1'bZ;
    assign reg_we   = (S_PSELx & S_PENABLE & S_PWRITE);

    always @(*)
        if (reg_we)
            $display($time, "\t\tREGS_APB[%h] <= %h", S_PADDR, S_PWDATA);

    always @(*) 
        `rassert(reg_we == (S_PSELx & S_PENABLE & S_PWRITE))

    vmicro16_regs # (
        .CELL_DEPTH(CELL_DEPTH)
    ) regs_apb (
        .clk    (clk),
        .reset  (reset),

        .rs1    (S_PADDR),
        .rd1    (rd1),

        //.rs2    (),
        //.rd2    (),
        
        .we     (reg_we),
        .ws1    (S_PADDR),
        .wd     (S_PWDATA) // either alu_c or mem_out
    );
endmodule


(*dont_touch="true"*)
(* keep_hierarchy = "yes" *)
module vmicro16_gpio_apb # (
    parameter BUS_WIDTH  = 16,
    parameter PORTS      = 8
) (
    input clk,
    input reset,
    // APB Slave to master interface
    input  [0:0]                    S_PADDR, // not used
    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input  [BUS_WIDTH-1:0]          S_PWDATA,
    
    output [BUS_WIDTH-1:0]          S_PRDATA,
    output                          S_PREADY,
    output reg [PORTS-1:0]          gpio
);
    assign S_PRDATA = (S_PSELx & S_PENABLE) ? gpio : 16'hZZZZ;
    assign S_PREADY = (S_PSELx & S_PENABLE) ? 1'b1 : 1'bZ;
    assign ports_we = (S_PSELx & S_PENABLE & S_PWRITE);

    always @(posedge clk)
        if (reset)
            gpio <= 0;
        else if (ports_we) begin
            $display($time, "\t\tGPIO <= %h", S_PWDATA[PORTS-1:0]);
            gpio <= S_PWDATA[PORTS-1:0];
        end
endmodule

// Decoder is hard to parameterise as it's very closely linked to the ISA.
(* keep_hierarchy = "yes" *)
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
        `VMICRO16_OP_HALT,    // TODO: stop ifid
        `VMICRO16_OP_NOP:     alu_op = `VMICRO16_ALU_NOP;
        
        `VMICRO16_OP_LW:      alu_op = `VMICRO16_ALU_LW;
        `VMICRO16_OP_SW:      alu_op = `VMICRO16_ALU_SW;

        `VMICRO16_OP_MOV:     alu_op = `VMICRO16_ALU_MOV;
        `VMICRO16_OP_MOVI:    alu_op = `VMICRO16_ALU_MOVI;
        `VMICRO16_OP_MOVI_L:  alu_op = `VMICRO16_ALU_MOVI_L; 

        `VMICRO16_OP_BR:      alu_op = `VMICRO16_ALU_BR;
        
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
        default:                has_we = 1'b0;
    endcase

    // Contains 8-bit immediate
    always @(*) case (opcode)
        `VMICRO16_OP_MOVI,
        `VMICRO16_OP_CMP:       has_imm8 = 1'b1;
        default:                has_imm8 = 1'b0;
    endcase

    // Contains 12-bit immediate
    always @(*) case (opcode)
        `VMICRO16_OP_MOVI_L:    has_imm12 = 1'b1;
        default:                has_imm12 = 1'b0;
    endcase
    
    // Will branch the pc
    always @(*) case (opcode)
        `VMICRO16_OP_BR:    has_br = 1'b1;
        default:            has_br = 1'b0;
    endcase
    
    // Requires external memory
    always @(*) case (opcode)
        `VMICRO16_OP_LW,
        `VMICRO16_OP_SW:    has_mem = 1'b1;
        default:            has_mem = 1'b0;
    endcase
    
    // Requires external memory write
    always @(*) case (opcode)
        `VMICRO16_OP_SW:    has_mem_we = 1'b1;
        default:            has_mem_we = 1'b0;
    endcase
endmodule

(* keep_hierarchy = "yes" *)
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
            c = 16'h0000;
        end
    endcase
endmodule

(*dont_touch="true"*)
(* keep_hierarchy = "yes" *)
module vmicro16_core # (
    parameter MEM_INSTR_DEPTH   = 64,
    parameter MEM_SCRATCH_DEPTH = 64,
    parameter MEM_WIDTH         = 16,

    parameter CORE_ID           = 0
) (
    input        clk,
    input        reset,

    output [7:0] dbug_pc,
    
    // APB master to slave interface (apb_intercon)
    output  [MEM_WIDTH-1:0]     w_PADDR,
    output                      w_PWRITE,
    output                      w_PSELx,
    output                      w_PENABLE,
    output  [MEM_WIDTH-1:0]     w_PWDATA,
    input   [MEM_WIDTH-1:0]     w_PRDATA,
    input                       w_PREADY
);
    reg  [2:0] r_state = STATE_IF;
    localparam STATE_IF = 0;
    localparam STATE_R1 = 1;
    localparam STATE_R2 = 2;
    localparam STATE_ME = 3;
    localparam STATE_WB = 4;

    reg  [15:0] r_pc    = 16'h0000;
    reg  [15:0] r_instr = 16'h0000;
    wire [15:0] w_mem_instr_out;

    assign dbug_pc = r_pc[7:0];

    wire [4:0]  r_instr_opcode;
    wire [4:0]  r_instr_alu_op;
    wire [2:0]  r_instr_rsd;
    wire [2:0]  r_instr_rsa;
    reg  [15:0] r_instr_rdd = 0;
    reg  [15:0] r_instr_rda = 0;
    wire [7:0]  r_instr_imm8;
    wire [4:0]  r_instr_simm5;
    wire        r_instr_has_imm8;
    wire        r_instr_has_we;
    wire        r_instr_has_br;
    wire        r_instr_has_mem;
    wire        r_instr_has_mem_we;
    wire        r_instr_halt;

    wire [15:0] r_alu_out;
    reg  [2:0]  r_reg_rs1 = 0;
    wire [15:0] r_reg_rd1;
    //wire [15:0] r_reg_rd2;
    wire [15:0] r_reg_wd = (r_instr_has_mem) ? r_mem_scratch_out : r_alu_out;
    wire r_reg_we        = r_instr_has_we && (r_state == STATE_WB);

    wire [15:0] r_mem_scratch_addr = r_alu_out + r_instr_simm5;
    wire [15:0] r_mem_scratch_in   = r_instr_rdd;
    wire [15:0] r_mem_scratch_out;
    wire        r_mem_scratch_we   = r_instr_has_mem_we && (r_state == STATE_ME);
    reg         r_mem_scratch_req  = 0;
    wire        r_mem_scratch_busy;

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

    // cpu state machine
    always @(posedge clk)
        if (reset) begin
            r_pc              <= 0;
            r_state           <= STATE_IF;
            r_instr           <= 0;
            r_mem_scratch_req <= 0;
            r_instr_rdd       <= 0;
            r_instr_rda       <= 0;
        end 
        else begin
            if (r_state == STATE_IF) begin
                r_instr <= w_mem_instr_out;

                if (r_pc < (MEM_INSTR_DEPTH-1))
                    r_pc <= r_pc + 1;

                $display($time, "\tPC: %h", r_pc);
                $display($time, "\tINSTR: %h", w_mem_instr_out);
                
                r_state <= STATE_R1;
            end
            else if (r_state == STATE_R1) begin
                r_instr_rdd <= r_reg_rd1;
                r_state     <= STATE_R2;
            end
            else if (r_state == STATE_R2) begin
                if (r_instr_has_imm8)
                    r_instr_rda <= r_instr_imm8;
                else
                    r_instr_rda <= r_reg_rd1;

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
                r_state <= STATE_IF;
            end
        end

    // Instruction ROM
    (* keep_hierarchy = "yes" *)
    vmicro16_bram # (
        .MEM_WIDTH      (16),
        .MEM_DEPTH      (MEM_INSTR_DEPTH)
    ) mem_instr (
        .clk            (clk), 
        .reset          (reset), 
        // port 1       
        .mem_addr       (r_pc), 
        .mem_in         (16'h0000), 
        .mem_we         (1'b0),  // ROM
        .mem_out        (w_mem_instr_out)
    );

    // MMU
    (* keep_hierarchy = "yes" *)
    vmicro16_core_mmu # (
        .MEM_WIDTH      (16),
        .MEM_DEPTH      (MEM_SCRATCH_DEPTH),
        .ADDR_TIM0_S    (16'h00),
        .ADDR_TIM0_E    (16'h3F)
    ) mmu (
        .clk            (clk), 
        .reset          (reset), 
        .req            (r_mem_scratch_req),
        .busy           (r_mem_scratch_busy),
        // port 1
        .mmu_addr       (r_mem_scratch_addr), 
        .mmu_in         (r_mem_scratch_in), 
        .mmu_we         (r_mem_scratch_we), 
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
    (* keep_hierarchy = "yes" *)
    vmicro16_dec dec (
        // input
        .instr          (r_instr),
        // output async
        .opcode         (),
        .rd             (r_instr_rsd),
        .ra             (r_instr_rsa),
        .imm8           (r_instr_imm8),
        .imm12          (),
        .simm5          (r_instr_simm5),
        .alu_op         (r_instr_alu_op),
        .has_imm8       (r_instr_has_imm8),
        .has_we         (r_instr_has_we),
        .has_br         (r_instr_has_br),
        .has_mem        (r_instr_has_mem),
        .has_mem_we     (r_instr_has_mem_we),
        .halt           ()
    );
    
    // Software registers
    (* keep_hierarchy = "yes" *)
    vmicro16_regs regs (
        .clk        (clk),
        .reset      (reset),
        // async port 0
        .rs1        (r_reg_rs1),
        .rd1        (r_reg_rd1),
        // async port 1
        //.rs2        (),
        //.rd2        (),
        // write port
        .we         (r_reg_we),
        .ws1        (r_instr_rsd),
        .wd         (r_reg_wd)
    );

    // ALU
    (* keep_hierarchy = "yes" *)
    vmicro16_alu alu (
        .op         (r_instr_alu_op),
        .a          (r_instr_rdd),
        .b          (r_instr_rda),
        // async output
        .c          (r_alu_out)
    );

endmodule

