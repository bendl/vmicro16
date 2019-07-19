`timescale 1ns / 1ps

`define FORMAL

module tb_vmicro16_core;

    // Inputs
    reg clk;
    reg reset;

    // Instantiate the Unit Under Test (UUT)
    vmicro16_core uut (
        .clk(clk), 
        .reset(reset)
    );

    always #10 clk = ~clk;

    // Nanosecond time format
    initial $timeformat(-9, 0, " ns", 10);

    initial begin
        // Initialize Inputs
        clk   = 0;
        reset = 1;

        # 30;
        reset = 1;
        @(posedge clk);
        reset = 0;

    end
      
endmodule

