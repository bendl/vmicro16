`timescale 1ns / 1ps


module tb_vmicro16_idex;

    // Inputs
    reg clk;
    reg reset;
    reg [15:0] ifid_pc;
    reg [15:0] ifid_instr;
    reg reg_rd1;
    reg reg_rd2;
    reg stall;
    reg jmping;
    reg ifid_valid;

    // Outputs
    wire [15:0] idex_pc;
    wire [15:0] idex_instr;
    wire [15:0] idex_rd1;
    wire [15:0] idex_rd2;
    wire reg_rs1;
    wire reg_rs2;
    wire [15:0] idex_rd3;
    wire [2:0] idex_rs1;
    wire [2:0] idex_rs2;
    wire idex_has_br;
    wire idex_has_we;
    wire idex_has_mem;
    wire idex_has_mem_we;
    wire idex_valid;
    wire [4:0] exme_op;

    always #10 clk = ~clk;

    // Instantiate the Unit Under Test (UUT)
    vmicro16_idex uut (
        .clk(clk), 
        .reset(reset), 
        .ifid_pc(ifid_pc), 
        .idex_pc(idex_pc), 
        .ifid_instr(ifid_instr), 
        .idex_instr(idex_instr), 
        .idex_rd1(idex_rd1), 
        .idex_rd2(idex_rd2), 
        .reg_rs1(reg_rs1), 
        .reg_rs2(reg_rs2), 
        .reg_rd1(reg_rd1), 
        .reg_rd2(reg_rd2), 
        .idex_rd3(idex_rd3), 
        .idex_rs1(idex_rs1), 
        .idex_rs2(idex_rs2), 
        .idex_has_br(idex_has_br), 
        .idex_has_we(idex_has_we), 
        .idex_has_mem(idex_has_mem), 
        .idex_has_mem_we(idex_has_mem_we), 
        .stall(stall), 
        .jmping(jmping), 
        .ifid_valid(ifid_valid), 
        .idex_valid(idex_valid), 
        .exme_op(exme_op)
    );

    initial begin
        // Initialize Inputs
        clk = 0;
        reset = 0;
        ifid_pc = 0;
        ifid_instr = 0;
        reg_rd1 = 0;
        reg_rd2 = 0;
        stall = 0;
        jmping = 0;
        ifid_valid = 0;

        // Wait 100 ns for global reset to finish
        #100;
        
        // Add stimulus here

    end
      
endmodule

