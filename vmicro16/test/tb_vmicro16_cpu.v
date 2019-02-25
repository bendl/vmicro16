`timescale 1ns / 1ps

module tb_vmicro16_cpu;

    // Inputs
    reg clk;
    reg reset;

    // Instantiate the Unit Under Test (UUT)
    vmicro16_cpu uut (
        .clk(clk), 
        .reset(reset)
    );

    always #10 clk = ~clk;

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

