`timescale 1ns / 1ps

`include "../vmicro16_isa.v"
`include "../formal.v"

module tb_alu();
    
    // Inputs
    reg clk;
    reg reset;
    
    reg [15:0] a;
    reg [15:0] b;
    wire [15:0] c;
    reg [7:0] cond;
    wire en;

    // Instantiate the Unit Under Test (UUT)
    vmicro16_alu uut (
        .op (5'h19),
        .a  (a),
        .b  (b),
        .c  (c)
    );

    // Instantiate the Unit Under Test (UUT)
    branch branch_check (
        .flags (c[3:0]),
        .cond  (cond),
        .en    (en)
    );

    always #10 clk = ~clk;

    // Nanosecond time format
    initial $timeformat(-9, 0, " ns", 10);

    initial begin
        $monitor($time, "\ta=%h b=%h c=%b N=%b Z=%b C=%b V=%b COND=%h EN=%b", 
            a, b, c[3:0], c[3], c[2], c[1], c[0], cond, en);
        
        // Initialize Inputs
        clk  = 0;
        cond = `VMICRO16_OP_BR_U; 
        
        a = 5'h00;
        b = 5'h00;
        `rassert(c == 4'b0100);
        `rassert(en == 1'b1);
        @(posedge clk);
        
        @(posedge clk);
        a = 5'h0A;
        b = 5'h0B;
        @(posedge clk);
        cond = `VMICRO16_OP_BR_U;
        @(posedge clk);
        `rassert(c == 4'b1000);
        `rassert(en == 1'b1);
        
        cond = `VMICRO16_OP_BR_E;
        @(posedge clk);
        `rassert(c == 4'b1000);
        `rassert(en == 1'b0);
        
        cond = `VMICRO16_OP_BR_NE;
        @(posedge clk);
        `rassert(c == 4'b1000);
        `rassert(en == 1'b1);
        
        cond = `VMICRO16_OP_BR_L;
        @(posedge clk);
        `rassert(c == 4'b1000);
        `rassert(en == 1'b1);
        
        cond = `VMICRO16_OP_BR_G;
        @(posedge clk);
        `rassert(c == 4'b1000);
        `rassert(en == 1'b0);
        
        a = 5'h0B;
        b = 5'h0A;
        cond = `VMICRO16_OP_BR_G;
        @(posedge clk);
        `rassert(c == 4'b0000);
        `rassert(en == 1'b1);
        
        cond = `VMICRO16_OP_BR_L;
        @(posedge clk);
        `rassert(c == 4'b0000);
        `rassert(en == 1'b0);
        
        cond = `VMICRO16_OP_BR_E;
        @(posedge clk);
        `rassert(c == 4'b0000);
        `rassert(en == 1'b0);
        
        a = 5'h0A;
        b = 5'h0A;
        @(posedge clk);
        `rassert(c == 4'b0100);
        `rassert(en == 1'b1);
        
        $display("ALL tests passed!");
    end
endmodule
