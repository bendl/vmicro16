`timescale 1ns / 1ps

`include "../vmicro16_isa.v"
`include "../formal.v"

module tb_alu_setc();
    
    // Inputs
    reg clk;
    reg reset;
    
    reg  [15:0] a;
    reg  [15:0] b;
    reg  [3:0] flags;
    wire [15:0] c;

    // Instantiate the Unit Under Test (UUT)
    vmicro16_alu uut (
        .op (`VMICRO16_ALU_SETC),
        .a  (a),
        .b  (b),
        .flags (flags),
        .c  (c)
    );

    always #10 clk = ~clk;

    // Nanosecond time format
    initial $timeformat(-9, 0, " ns", 10);

    initial begin
        $monitor($time, "\tb=%h N=%b Z=%b C=%b V=%b\tc=%h", 
            b, flags[3], flags[2], flags[1], flags[0], c[15:0]);
        
        // Initialize Inputs
        clk   = 0;
        flags = 4'b0000;
        a = 16'h00;
        b = 16'h00;
        // NZCV
        @(posedge clk);
        
        b = { 8'h00, `VMICRO16_OP_BR_U };
        flags = 4'b0000;
        @(posedge clk);
        `rassert(c == 16'h01);
        
        b = { 8'h00, `VMICRO16_OP_BR_E };
        flags = 4'b0000;
        @(posedge clk);
        `rassert(c == 16'h00);
        
        b = { 8'h00, `VMICRO16_OP_BR_E };
        flags = 4'b0100;
        @(posedge clk);
        `rassert(c == 16'h01);
        
        b = { 8'h00, `VMICRO16_OP_BR_L };
        flags = 4'b0100; // NZCV
        @(posedge clk);
        `rassert(c == 16'h01);
        
        b = { 8'h00, `VMICRO16_OP_BR_G };
        flags = 4'b0100; // NZCV
        @(posedge clk);
        `rassert(c == 16'h00);
        
        b = { 8'h00, `VMICRO16_OP_BR_G };
        flags = 4'b0000; // NZCV
        @(posedge clk);
        `rassert(c == 16'h01);
        
        $display("ALL tests passed!");
    end
endmodule
