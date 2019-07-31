
`include "../clog2.v"
`include "../vmicro16_soc_config.v"

module uart_prog # (
    parameter DATA_WIDTH = 8,
    parameter ADDR_EXP   = 5
) (
    input clk,
    input reset,

    // input UART RX
    input uart_rx,
    
    // outputs an interface to program the
    //   instruction memory
    output     [`clog2(`DEF_MEM_INSTR_DEPTH)-1:0] addr,
    output     [`DATA_WIDTH-1:0]                  data,
    output                                        we,
    output  reg                                      prog
);
    wire uart_rx_fifo_pop;
    wire uart_rx_fifo_empty;
    wire [7:0] uart_rx_byte;
    wire uart_irq;
    uart_fifo # (
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_EXP   (ADDR_EXP)
    ) uart_fifo(
        .clk             (clk),
        .rst             (reset),

        // reciever
        .rx              (uart_rx),
        .rx_fifo_pop     (read),
        .rx_fifo_empty   (uart_rx_fifo_empty),
        .rx_byte         (uart_rx_byte),
        .irq             (uart_irq)
    );

    reg [7:0] word_l = 0;
    reg [7:0] word_h = 0;
    reg [`clog2(`DEF_MEM_INSTR_DEPTH)-1:0] word_i  = 0;
    reg word_ib;

    reg read = 0;
    always @(posedge clk)
        read <= uart_irq;
    
    reg new_data = 0;
    always @(posedge clk)
        new_data <= read;


    // every bytes, increment word_i pointer
    always @(posedge clk)
        if (reset)
            word_ib <= 0;
        else
            if (read)
                word_ib <= word_ib + 1;
    
    always @(posedge clk)
        if (reset) begin
            word_i  <= 0;
        end
        else if (we)
            word_i <= word_i + 1;

    // every byte, store it in a half word
    always @(posedge clk)
        if (reset) begin
           word_h <= 0;
           word_l <= 0; 
        end else if (read)
            if (word_ib)
                word_h <= uart_rx_byte;
            else
                word_l <= uart_rx_byte;

    assign data = {word_h, word_l};
    assign addr = word_i;
    assign we   = new_data && (~word_ib);

    initial prog = 0;
    always @(posedge clk)
        if (we)
            if (data != 16'hffff)
                prog <= 1;
            else
                prog <= 0;
    


endmodule