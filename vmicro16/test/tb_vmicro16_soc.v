`timescale 1ns / 1ps

module tb_vmicro16_soc;

    // Inputs
    reg clk;
    reg reset;

    // Create clock signal
    always #10 clk = ~clk;

    // Instantiate the Unit Under Test (UUT)
    vmicro16_soc uut (
        .clk    (clk),
        .reset  (reset)
    );

    initial begin
        // Initialize Inputs
        clk = 0;
        reset = 1;
        
        // Assert reset for n clocks minumum
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        reset = 0;
    end
endmodule

