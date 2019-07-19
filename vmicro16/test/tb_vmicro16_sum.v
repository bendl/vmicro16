`timescale 1ns / 1ps

`define FORMAL

`include "../formal.v"
`include "../vmicro16_soc_config.v"

module tb_vmicro16_soc;

    // Inputs
    reg clk;
    reg reset;

    wire halt;
    wire [`APB_GPIO1_PINS-1:0] gpio1;

    // Create clock signal
    always #10 clk = ~clk;

    // Instantiate the Unit Under Test (UUT)
    vmicro16_soc uut (
        .clk    (clk),
        .halt   (halt),
        .gpio1  (gpio1),
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

        // Verify correct summation result
        @(halt);
        `rassert(gpio1 == 16'h7008);

        $display("");
        $display("SUCCESS!");
    end
endmodule

