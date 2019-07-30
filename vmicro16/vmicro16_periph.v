// Vmicro16 peripheral modules

`include "vmicro16_soc_config.v"
`include "formal.v"

// Simple watchdog peripheral
module vmicro16_watchdog_apb # (
    parameter BUS_WIDTH  = 16,
    parameter NAME       = "WD",
    parameter CLK_HZ     = 50_000_000
) (
    input clk,
    input reset,

    // APB Slave to master interface
    input  [0:0]                    S_PADDR, // not used (optimised out)
    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input  [0:0]                    S_PWDATA,
    
    // prdata not used
    output [0:0]                    S_PRDATA,
    output                          S_PREADY,

    // watchdog reset, active high
    output reg                      wdreset
);
    //assign S_PRDATA = (S_PSELx & S_PENABLE) ? gpio : 16'h0000;
    assign S_PREADY = (S_PSELx & S_PENABLE) ? 1'b1 : 1'b0;
    wire   we       = (S_PSELx & S_PENABLE & S_PWRITE);

    // countdown timer
    reg [`clog2(CLK_HZ)-1:0] timer = CLK_HZ;

    wire w_wdreset = (timer == 0);

    // infer a register to aid timing
    initial wdreset = 0;
    always @(posedge clk)
        wdreset <= w_wdreset;

    always @(posedge clk)
        if (we) begin
            $display($time, "\t\%s <= RESET", NAME);
            timer <= CLK_HZ;
        end else begin
            timer <= timer - 1;
        end
endmodule

module timer_apb # (
    parameter CLK_HZ = 50_000_000
) (
    input clk,
    input reset,

    input clk_en,
    
    // 0 16-bit value   R/W
    // 1 16-bit control R    b0 = start, b1 = reset
    // 2 16-bit prescaler
    input      [1:0]                S_PADDR,

    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input      [`DATA_WIDTH-1:0]    S_PWDATA,
    
    output reg [`DATA_WIDTH-1:0]    S_PRDATA,
    output                          S_PREADY,

    output out,
    output [`DATA_WIDTH-1:0] int_data
);
    //assign S_PRDATA = (S_PSELx & S_PENABLE) ? swex_success ? 16'hF0F0 : 16'h0000;
    assign S_PREADY = (S_PSELx & S_PENABLE) ? 1'b1 : 1'b0;
    wire   en       = (S_PSELx & S_PENABLE);
    wire   we       = (en & S_PWRITE);

    reg [`DATA_WIDTH-1:0] r_counter = 0;
    reg [`DATA_WIDTH-1:0] r_load = 0;
    reg [`DATA_WIDTH-1:0] r_pres = 0;
    reg [`DATA_WIDTH-1:0] r_ctrl = 0;

    localparam CTRL_START = 0;
    localparam CTRL_RESET = 1;
    localparam CTRL_INT   = 2;

    localparam ADDR_LOAD = 2'b00;
    localparam ADDR_CTRL = 2'b01;
    localparam ADDR_PRES = 2'b10;
    
    always @(*) begin
        S_PRDATA = 0;
        if (en)
            case(S_PADDR)
                ADDR_LOAD: S_PRDATA = r_counter;
                ADDR_CTRL: S_PRDATA = r_ctrl;
                //ADDR_CTRL: S_PRDATA = r_pres;
                default:   S_PRDATA = 0;
            endcase
    end

    // prescaler counts from r_pres to 0, emitting a stb signal
    //   to enable the r_counter step
    reg [`DATA_WIDTH-1:0] r_pres_counter = 0;
    wire counter_en = (r_pres_counter == 0);
    always @(posedge clk)
        if (r_pres_counter == 0)
            r_pres_counter <= r_pres;
        else
            r_pres_counter <= r_pres_counter - 1;

    always @(posedge clk)
        if (we)
            case(S_PADDR)
                // Write to the load register:
                //   Set load register
                //   Set counter register
                ADDR_LOAD: begin
                    r_load          <= S_PWDATA;
                    r_counter       <= S_PWDATA;
                    $display($time, "\t\ttimr0: WRITE LOAD: %h", S_PWDATA);
                end
                ADDR_CTRL: begin
                    r_ctrl   <= S_PWDATA;
                    $display($time, "\t\ttimr0: WRITE CTRL: %h", S_PWDATA);
                end
                ADDR_PRES: begin
                    r_pres   <= S_PWDATA;
                    $display($time, "\t\ttimr0: WRITE PRES: %h", S_PWDATA);
                end
            endcase
        else
            if (r_ctrl[CTRL_START]) begin
                if (r_counter == 0)
                    r_counter <= r_load;
                else if(counter_en)
                    r_counter <= r_counter -1;
            end else if (r_ctrl[CTRL_RESET])
                r_counter <= r_load;
            
    // generate the output pulse when r_counter == 0
    //   out = (counter reached zero && counter started)
    assign out      = (r_counter == 0) && r_ctrl[CTRL_START]; // && r_ctrl[CTRL_INT];
    assign int_data = {`DATA_WIDTH{1'b1}};
endmodule

// APB wrapped vmicro16_bram
module vmicro16_bram_apb # (
    parameter BUS_WIDTH    = 16,
    parameter MEM_WIDTH    = 16,
    parameter MEM_DEPTH    = 64,
    parameter APB_PADDR    = 0,
    parameter USE_INITS    = 0,
    parameter NAME         = "BRAM",
    parameter CORE_ID      = 0
) (
    input clk,
    input reset,
    // APB Slave to master interface
    input  [`clog2(MEM_DEPTH)-1:0]  S_PADDR,
    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input  [BUS_WIDTH-1:0]          S_PWDATA,
    
    output [BUS_WIDTH-1:0]          S_PRDATA,
    output                          S_PREADY
);
    wire [MEM_WIDTH-1:0] mem_out;

    assign S_PRDATA = (S_PSELx & S_PENABLE) ? mem_out : 16'h0000;
    assign S_PREADY = (S_PSELx & S_PENABLE) ? 1'b1    : 1'b0;
    assign we       = (S_PSELx & S_PENABLE & S_PWRITE);

    always @(*)
        if (S_PSELx && S_PENABLE)
            $display($time, "\t\t%s => %h", NAME, mem_out);

    always @(posedge clk)
        if (we)
            $display($time, "\t\t%s[%h] <= %h", NAME, 
                S_PADDR, S_PWDATA);

    vmicro16_bram # (
        .MEM_WIDTH  (MEM_WIDTH),
        .MEM_DEPTH  (MEM_DEPTH),
        .NAME       (NAME),
        .USE_INITS  (1),
        .CORE_ID    (-1)
    ) bram_apb (
        .clk        (clk),
        .reset      (reset),

        .mem_addr   (S_PADDR),
        .mem_in     (S_PWDATA),
        .mem_we     (we),
        .mem_out    (mem_out)
    );
endmodule

// Shared memory with hardware monitor (LWEX/SWEX)
module vmicro16_bram_ex_apb # (
    parameter BUS_WIDTH    = 16,
    parameter MEM_WIDTH    = 16,
    parameter MEM_DEPTH    = 64,
    parameter CORE_ID_BITS = 3,
    parameter SWEX_SUCCESS = 16'h0000,
    parameter SWEX_FAIL    = 16'h0001
) (
    input clk,
    input reset,

    // |19    |18    |16             |15          0|
    // | LWEX | SWEX | 3 bit CORE_ID |     S_PADDR |
    input  [`APB_WIDTH-1:0]         S_PADDR,

    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input  [MEM_WIDTH-1:0]          S_PWDATA,
    
    output reg [MEM_WIDTH-1:0]      S_PRDATA,
    output                          S_PREADY
);
    // exclusive flag checks
    wire [MEM_WIDTH-1:0] mem_out;
    reg                  swex_success = 0;

    localparam ADDR_BITS = `clog2(MEM_DEPTH);

    // hack to create a 1 clock delay to S_PREADY 
    // for bram to be ready
    reg cdelay = 1;
    always @(posedge clk)
        if (S_PSELx)
            cdelay <= 0;
        else
            cdelay <= 1;

    //assign S_PRDATA = (S_PSELx & S_PENABLE) ? swex_success ? 16'hF0F0 : 16'h0000;
    assign S_PREADY = (S_PSELx & S_PENABLE & (!cdelay)) ? 1'b1       : 1'b0;
    assign we       = (S_PSELx & S_PENABLE & S_PWRITE);
    wire   en       = (S_PSELx & S_PENABLE);

    // Similar to:
    //   http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0204f/Cihbghef.html

    // mem_wd is the CORE_ID sent in bits [18:16] 
    localparam TOP_BIT_INDEX         = `APB_WIDTH -1;
    localparam PADDR_CORE_ID_MSB     = TOP_BIT_INDEX - 2;
    localparam PADDR_CORE_ID_LSB     = PADDR_CORE_ID_MSB - (CORE_ID_BITS-1);

    // [LWEX, CORE_ID, mem_addr] from S_PADDR
    wire                    lwex        = S_PADDR[TOP_BIT_INDEX];
    wire                    swex        = S_PADDR[TOP_BIT_INDEX-1];
    wire [CORE_ID_BITS-1:0] core_id     = S_PADDR[PADDR_CORE_ID_MSB:PADDR_CORE_ID_LSB];
    // CORE_ID to write to ex_flags register
    wire [ADDR_BITS-1:0]    mem_addr    = S_PADDR[ADDR_BITS-1:0];

    wire [CORE_ID_BITS:0]   ex_flags_read;
    wire                    is_locked      = |ex_flags_read;
    wire                    is_locked_self = is_locked && (core_id == (ex_flags_read-1));

    // Check exclusive access flags
    always @(*) begin
        swex_success = 0;
        if (en)
            // bug!
            if (!swex && !lwex)
                swex_success = 1;
            else if (swex)
                if (is_locked && !is_locked_self)
                    // someone else has locked it
                    swex_success = 0;
                else if (is_locked && is_locked_self)
                    swex_success = 1;
    end

    always @(*)
        if (swex)
            if (swex_success)
                S_PRDATA = SWEX_SUCCESS;
            else
                S_PRDATA = SWEX_FAIL;
        else
            S_PRDATA = mem_out;

    wire reg_we = en && ((lwex && !is_locked) 
                      || (swex && swex_success));

    reg  [CORE_ID_BITS:0] reg_wd;
    always @(*) begin
        reg_wd = {{CORE_ID_BITS}{1'b0}};

        if (en)
            // if wanting to lock the addr
            if (lwex)
                // and not already locked
                if (!is_locked) begin
                    reg_wd = (core_id + 1);
                end
            else if (swex)
                if (is_locked && is_locked_self)
                    reg_wd = {{CORE_ID_BITS}{1'b0}};
    end

    // Exclusive flag for each memory cell
    vmicro16_bram # (
        .MEM_WIDTH  (CORE_ID_BITS + 1),
        .MEM_DEPTH  (MEM_DEPTH),
        .USE_INITS  (0),
        .NAME       ("rexram")
    ) ram_exflags (
        .clk        (clk),
        .reset      (reset),

        .mem_addr   (mem_addr),
        .mem_in     (reg_wd),
        .mem_we     (reg_we),
        .mem_out    (ex_flags_read)
    );

    always @(*)
        if (S_PSELx && S_PENABLE)
            $display($time, "\t\tBRAMex[%h] READ %h\tCORE: %h", mem_addr, mem_out, S_PADDR[16 +: CORE_ID_BITS]);

    always @(posedge clk)
        if (we)
            $display($time, "\t\tBRAMex[%h] WRITE %h\tCORE: %h", mem_addr, S_PWDATA, S_PADDR[16 +: CORE_ID_BITS]);

    vmicro16_bram # (
        .MEM_WIDTH  (MEM_WIDTH),
        .MEM_DEPTH  (MEM_DEPTH),
        .USE_INITS  (0),
        .NAME       ("BRAMexinst")
    ) bram_apb (
        .clk        (clk),
        .reset      (reset),

        .mem_addr   (mem_addr),
        .mem_in     (S_PWDATA),
        .mem_we     (we && swex_success),
        .mem_out    (mem_out)
    );
endmodule

// Simple APB memory-mapped register set
module vmicro16_regs_apb # (
    parameter BUS_WIDTH         = 16,
    parameter DATA_WIDTH        = 16,
    parameter CELL_DEPTH        = 8,
    parameter PARAM_DEFAULTS_R0 = 0,
    parameter PARAM_DEFAULTS_R1 = 0
) (
    input clk,
    input reset,
    // APB Slave to master interface
    input  [`clog2(CELL_DEPTH)-1:0] S_PADDR,
    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input  [DATA_WIDTH-1:0]         S_PWDATA,
    
    output [DATA_WIDTH-1:0]         S_PRDATA,
    output                          S_PREADY
);
    wire [DATA_WIDTH-1:0] rd1;

    assign S_PRDATA = (S_PSELx & S_PENABLE) ? rd1  : 16'h0000;
    assign S_PREADY = (S_PSELx & S_PENABLE) ? 1'b1 : 1'b0;
    assign reg_we   = (S_PSELx & S_PENABLE & S_PWRITE);

    always @(*)
        if (reg_we)
            $display($time, "\t\tREGS_APB[%h] <= %h", 
                S_PADDR, S_PWDATA);

    always @(*) 
        `rassert(reg_we == (S_PSELx & S_PENABLE & S_PWRITE))

    vmicro16_regs # (
        .CELL_DEPTH         (CELL_DEPTH),
        .CELL_WIDTH         (DATA_WIDTH),
        .PARAM_DEFAULTS_R0  (PARAM_DEFAULTS_R0),
        .PARAM_DEFAULTS_R1  (PARAM_DEFAULTS_R1)
    ) regs_apb (
        .clk    (clk),
        .reset  (reset),
        // port 1
        .rs1    (S_PADDR),
        .rd1    (rd1),
        .we     (reg_we),
        .ws1    (S_PADDR),
        .wd     (S_PWDATA)
        // port 2 unconnected
        //.rs2    (),
        //.rd2    ()
    );
endmodule

// Simple GPIO write only peripheral
module vmicro16_gpio_apb # (
    parameter BUS_WIDTH  = 16,
    parameter DATA_WIDTH = 16,
    parameter PORTS      = 8,
    parameter NAME       = "GPIO"
) (
    input clk,
    input reset,
    // APB Slave to master interface
    input  [0:0]                    S_PADDR, // not used (optimised out)
    input                           S_PWRITE,
    input                           S_PSELx,
    input                           S_PENABLE,
    input  [DATA_WIDTH-1:0]         S_PWDATA,
    
    output [DATA_WIDTH-1:0]         S_PRDATA,
    output                          S_PREADY,
    output reg [PORTS-1:0]          gpio
);
    assign S_PRDATA = (S_PSELx & S_PENABLE) ? gpio : 16'h0000;
    assign S_PREADY = (S_PSELx & S_PENABLE) ? 1'b1 : 1'b0;
    assign ports_we = (S_PSELx & S_PENABLE & S_PWRITE);

    always @(posedge clk)
        if (reset)
            gpio <= 0;
        else if (ports_we) begin
            $display($time, "\t\%s <= %h", NAME, S_PWDATA[PORTS-1:0]);
            gpio <= S_PWDATA[PORTS-1:0];
        end
endmodule
