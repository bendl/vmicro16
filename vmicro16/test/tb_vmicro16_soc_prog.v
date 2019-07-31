`timescale 1ns / 1ps

`define FORMAL

module tb_vmicro16_soc_prog;

    // Inputs
    reg clk;
    reg reset;

    reg uart_rx;

    // Create clock signal
    always #10 clk = ~clk;

    ///////////////////////////////////////////////
    // task from https://www.edaplayground.com/x/4Lyz
    //////////////////////////////////////////////
    // Testbench uses a 25 MHz clock (same as Go Board)
    // Want to interface to 115200 baud UART
    // 25000000 / 115200 = 217 Clocks Per Bit.
    parameter c_BIT_PERIOD      = 8600;
    parameter c_WAIT            = 85_000;
    
    reg r_RX_Serial = 1;
    wire [7:0] w_RX_Byte;

    // Takes in input byte and serializes it 
    task UART_WRITE_BYTE;
        input [7:0] i_Data;
        integer     ii;
    begin
        // Send Start Bit
        r_RX_Serial <= 1'b0;
        #(c_BIT_PERIOD);
        #1000;
        // Send Data Byte
        for (ii=0; ii<8; ii=ii+1) begin
            r_RX_Serial <= i_Data[ii];
            #(c_BIT_PERIOD);
        end
        // Send Stop Bit
        r_RX_Serial <= 1'b1;
        #(c_BIT_PERIOD);
        end
    endtask // UART_WRITE_BYTE

    // Instantiate the Unit Under Test (UUT)
    vmicro16_soc uut (
        .clk     (clk),
        .reset   (reset),
        .uart_rx (r_RX_Serial)
    );

    initial begin
        // Initialize Inputs
        clk     = 0;
        reset   = 1;
        uart_rx = 0;
        
        // Assert reset for n clocks minumum
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        reset = 0;

        @(posedge clk);
        #(c_WAIT);

        @(posedge clk);
        UART_WRITE_BYTE(8'h80);
        @(posedge clk);
        UART_WRITE_BYTE(8'h28);

        @(posedge clk);
        UART_WRITE_BYTE(8'h00);
        @(posedge clk);
        UART_WRITE_BYTE(8'h08);

        @(posedge clk);
        UART_WRITE_BYTE(8'hff);
        @(posedge clk);
        UART_WRITE_BYTE(8'hff);

        #(c_WAIT);
        @(posedge clk);
        UART_WRITE_BYTE(8'h80);
        @(posedge clk);
        UART_WRITE_BYTE(8'h28);

        @(posedge clk);
        UART_WRITE_BYTE(8'h00);
        @(posedge clk);
        UART_WRITE_BYTE(8'h08);

        @(posedge clk);
        UART_WRITE_BYTE(8'hff);
        @(posedge clk);
        UART_WRITE_BYTE(8'hff);
    end
endmodule

