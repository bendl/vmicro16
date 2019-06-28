`timescale 1ns / 1ps

`include "../vmicro16_isa.v"

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
            a, b, c, c[3], c[2], c[1], c[0], cond, en);
        
        // Initialize Inputs
        clk  = 0;
        cond = 8'h00; 
        
        a = 5'h00;
        b = 5'h00;
        @(posedge clk);
        
        @(posedge clk);
        a = 5'h0A;
        b = 5'h0B;
        @(posedge clk);
        cond = `VMICRO16_OP_BR_U;
        @(posedge clk);
        cond = `VMICRO16_OP_BR_E;
        @(posedge clk);
        cond = `VMICRO16_OP_BR_NE;
        @(posedge clk);
        cond = `VMICRO16_OP_BR_L;
        @(posedge clk);
        cond = `VMICRO16_OP_BR_G;
        @(posedge clk);
        
        a = 5'h0B;
        b = 5'h0A;
        cond = `VMICRO16_OP_BR_G;
        @(posedge clk);
        cond = `VMICRO16_OP_BR_L;
        @(posedge clk);
        cond = `VMICRO16_OP_BR_E;
        @(posedge clk);
        
        @(posedge clk);
        a = 5'h0A;
        b = 5'h0A;

        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
    end
endmodule
