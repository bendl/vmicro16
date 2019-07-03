module sync_dual_port_ram
    
    #( parameter ADDRESS_WIDTH = 6,  // number of words in ram
                 DATA_WIDTH    = 8  // number of bits in word
     )
    
    // IO ports
    (
        input wire  clk,                                             // clk for synchronous read/write 
        input wire  write_en,                                        // signal to enable synchronous write
        input wire  [ADDRESS_WIDTH-1:0] read_address, write_address, // inputs for dual port addresses
        input wire  [DATA_WIDTH-1:0] write_data_in,                  // input for data to write to ram
        output wire [DATA_WIDTH-1:0] read_data_out, write_data_out  // outputs for dual data ports
    );
    
    // internal signal declarations
    reg [DATA_WIDTH-1:0]    ram [2**ADDRESS_WIDTH-1:0];             // ADDRESS_WIDTH x DATA_WIDTH RAM declaration
    reg [ADDRESS_WIDTH-1:0] read_address_reg, write_address_reg; // dual port address declarations
    
    // synchronous write and address update
    always @(posedge clk)
        begin
        if (write_en)                               // if write enabled
           ram[write_address] <= write_data_in; // write data to ram and write_address 
            
                read_address_reg  <= read_address;      // store read_address to reg
            write_address_reg <= write_address;     // store write_address to reg
            end
    
    // assignments for two data out ports
    assign read_data_out  = ram[read_address_reg];
    assign write_data_out = ram[write_address_reg];
endmodule

// FIFO buffer implemented with synchronous dual-port block ram
module fifo2

    #( parameter ADDRESS_WIDTH = 6, // number of words in ram
                 DATA_WIDTH    = 8  // number of bits in word
     )
     
    // IO ports
    (
        input wire clk, reset,
        input wire read, write,
        input wire [DATA_WIDTH-1:0] write_data,
        output wire empty, full,
        output wire [DATA_WIDTH-1:0] read_data
    );
    
    // internal signal declarations
    reg [ADDRESS_WIDTH-1:0] write_address_reg, write_address_next, write_address_after;
    reg [ADDRESS_WIDTH-1:0] read_address_reg, read_address_next, read_address_after;
    reg full_reg, empty_reg, full_next, empty_next;
    wire write_en;
    
    // write enable is asserted when write input is asserted and FIFO isn't full
    assign write_en = write & ~full_reg;
    
    // instantiate synchronous block ram
    sync_dual_port_ram #(
            .ADDRESS_WIDTH  (ADDRESS_WIDTH),
            .DATA_WIDTH     (DATA_WIDTH)) 
    ram  (
        .clk                (clk), 
        .write_en           (write_en),
        .write_address      (write_address_reg),
        .read_address       (read_address_reg),
        .write_data_in      (write_data),
        .write_data_out     (),
        .read_data_out      (read_data)
    );
    
    // register for address pointers, full/empty status
    always @(posedge clk, posedge reset)
        if (reset)
            begin
                    write_address_reg <= 0;
                    read_address_reg  <= 0;
                    full_reg          <= 1'b0;
                    empty_reg         <= 1'b1;
            end
        else
            begin
                    write_address_reg <= write_address_next;
                    read_address_reg  <= read_address_next;
                    full_reg          <= full_next;
                    empty_reg         <= empty_next;
            end
            
    // next-state logic for address index values after read/write operations
    always @*
        begin
        write_address_after = write_address_reg + 1;
        read_address_after  = read_address_reg + 1;
        end
        
    // next-state logic for address pointers
    always @*
        begin
        // defaults
        write_address_next = write_address_reg;
        read_address_next  = read_address_reg;
        full_next          = full_reg;
        empty_next         = empty_reg;
        
        // if read input asserted and FIFO isn't empty
        if(read && ~empty_reg && ~write)
            begin
            read_address_next = read_address_after;       // read address moves forward
            full_next = 1'b0;                             // FIFO isn't full if a read occured
            
            if (read_address_after == write_address_reg)  // if read address caught up with write address,
                empty_next = 1'b1;                        // FIFO is empty
            end
        
        // if write input asserted and FIFO isn't full
        else if(write && ~full_reg && ~read)
            begin
            write_address_next = write_address_after;     // write address moves forward
            empty_next = 1'b0;                            // FIFO isn't empty if write occured
            
            if (write_address_after == read_address_reg)    // if write address caught up with read address
                full_next = 1'b1;                         // FIFO is full
                        end
        
        // if write and read are asserted
                else if(write && read)
            begin
            write_address_next = write_address_after;     // write address moves forward
            read_address_next  = read_address_after;      // read address moves forward
                        end
        end 
    
   // assign full/empty status to output ports
   assign full  = full_reg;
   assign empty = empty_reg;
endmodule