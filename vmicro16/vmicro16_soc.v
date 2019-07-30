//
//

`include "vmicro16_soc_config.v"
`include "clog2.v"
`include "formal.v"

module pow_reset # (
    parameter INIT  = 1,
    parameter N     = 8
) (
    input       clk,
    input       reset,
    output reg  resethold
);
    initial resethold = INIT ? (N-1) : 0;

    always @(*)
        resethold = |hold;

    reg [`clog2(N)-1:0] hold = (N-1);
    always @(posedge clk)
        if (reset)
            hold <= N-1;
        else
            if (hold)
                hold <= hold - 1;
endmodule

// Vmicro16 multi-core SoC with various peripherals
// and interrupts
module vmicro16_soc (
    input clk,
    input reset,

    //input  uart_rx,
    output                          uart_tx,
    output [`APB_GPIO0_PINS-1:0]    gpio0,
    output [`APB_GPIO1_PINS-1:0]    gpio1,
    output [`APB_GPIO2_PINS-1:0]    gpio2,

    output                          halt,

    output     [`CORES-1:0]         dbug0,
    output     [`CORES*8-1:0]       dbug1
);
    wire [`CORES-1:0] w_halt;
    assign halt = &w_halt;
    
    assign dbug0 = w_halt;

    // Watchdog reset pulse signal.
    //   Passed to pow_reset to generate a longer reset pulse
    wire wdreset;

    // soft register reset hold for brams and registers
    wire soft_reset;
    `ifdef DEF_GLOBAL_RESET
        pow_reset # (
            .INIT       (1),
            .N          (8)
        ) por_inst (
            .clk        (clk),
            `ifdef DEF_USE_WATCHDOG
            .reset      (reset | wdreset),
            `else
            .reset      (reset),
            `endif
            .resethold  (soft_reset)
        );
    `else
        assign soft_reset = 0;
    `endif

    // Peripherals (master to slave)
     wire [`APB_WIDTH-1:0]          M_PADDR;
     wire                           M_PWRITE;
     wire [`SLAVES-1:0]             M_PSELx;  // not shared
     wire                           M_PENABLE;
     wire [`DATA_WIDTH-1:0]         M_PWDATA; 
     wire [`SLAVES*`DATA_WIDTH-1:0] M_PRDATA; // input to intercon
     wire [`SLAVES-1:0]             M_PREADY; // input

    // Master apb interfaces
     wire [`CORES*`APB_WIDTH-1:0]   w_PADDR;
     wire [`CORES-1:0]              w_PWRITE;
     wire [`CORES-1:0]              w_PSELx;
     wire [`CORES-1:0]              w_PENABLE;
     wire [`CORES*`DATA_WIDTH-1:0]  w_PWDATA;
     wire [`CORES*`DATA_WIDTH-1:0]  w_PRDATA;
     wire [`CORES-1:0]              w_PREADY;

    // Interrupts
`ifdef DEF_ENABLE_INT
    wire [`DEF_NUM_INT-1:0]              ints;
    wire [`DEF_NUM_INT*`DATA_WIDTH-1:0]  ints_data;
    assign ints[7:1] = 0;
    assign ints_data[`DEF_NUM_INT*`DATA_WIDTH-1:`DATA_WIDTH] = 
                {`DEF_NUM_INT*(`DATA_WIDTH-1){1'b0}};
`endif
    
    apb_intercon_s # (
        .MASTER_PORTS   (`CORES),
        .SLAVE_PORTS    (`SLAVES),
        .BUS_WIDTH      (`APB_WIDTH),
        .DATA_WIDTH     (`DATA_WIDTH),
        .HAS_PSELX_ADDR (1)
    ) apb (
        .clk        (clk),
        .reset      (soft_reset),
        // APB master to slave
        .S_PADDR    (w_PADDR),
        .S_PWRITE   (w_PWRITE),
        .S_PSELx    (w_PSELx),
        .S_PENABLE  (w_PENABLE),
        .S_PWDATA   (w_PWDATA),
        .S_PRDATA   (w_PRDATA),
        .S_PREADY   (w_PREADY),
        // shared bus
        .M_PADDR    (M_PADDR),
        .M_PWRITE   (M_PWRITE),
        .M_PSELx    (M_PSELx),
        .M_PENABLE  (M_PENABLE),
        .M_PWDATA   (M_PWDATA),
        .M_PRDATA   (M_PRDATA),
        .M_PREADY   (M_PREADY)
    );

`ifdef DEF_USE_WATCHDOG
    vmicro16_watchdog_apb # (
        .BUS_WIDTH  (`APB_WIDTH),
        .NAME       ("WDOG0")
    ) wdog0_apb (
        .clk        (clk),
        .reset      (),
        // apb slave to master interface
        .S_PADDR    (),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_WDOG0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (),
        .S_PRDATA   (),
        .S_PREADY   (M_PREADY[`APB_PSELX_WDOG0]),

        .wdreset    (wdreset)
    );
`endif

    vmicro16_gpio_apb # (
        .BUS_WIDTH  (`APB_WIDTH),
        .DATA_WIDTH (`DATA_WIDTH),
        .PORTS      (`APB_GPIO0_PINS),
        .NAME       ("GPIO0")
    ) gpio0_apb (
        .clk        (clk),
        .reset      (soft_reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_GPIO0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA[`APB_PSELX_GPIO0*`DATA_WIDTH +: `DATA_WIDTH]),
        .S_PREADY   (M_PREADY[`APB_PSELX_GPIO0]),
        .gpio       (gpio0)
    );

    // GPIO1 for Seven segment displays (16 pin)
    vmicro16_gpio_apb # (
        .BUS_WIDTH  (`APB_WIDTH),
        .DATA_WIDTH (`DATA_WIDTH),
        .PORTS      (`APB_GPIO1_PINS),
        .NAME       ("GPIO1")
    ) gpio1_apb (
        .clk        (clk),
        .reset      (soft_reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_GPIO1]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA[`APB_PSELX_GPIO1*`DATA_WIDTH +: `DATA_WIDTH]),
        .S_PREADY   (M_PREADY[`APB_PSELX_GPIO1]),
        .gpio       (gpio1)
    );

    // GPI02 for Seven segment displays (8 pin)
    vmicro16_gpio_apb # (
        .BUS_WIDTH  (`APB_WIDTH),
        .DATA_WIDTH (`DATA_WIDTH),
        .PORTS      (`APB_GPIO2_PINS),
        .NAME       ("GPI02")
    ) gpio2_apb (
        .clk        (clk),
        .reset      (soft_reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_GPIO2]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA[`APB_PSELX_GPIO2*`DATA_WIDTH +: `DATA_WIDTH]),
        .S_PREADY   (M_PREADY[`APB_PSELX_GPIO2]),
        .gpio       (gpio2)
    );
    
    apb_uart_tx # (
        .DATA_WIDTH (8),
        .ADDR_EXP   (4) //2^^4 = 16 FIFO words
    ) uart0_apb (
        .clk        (clk),
        .reset      (soft_reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_UART0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA[`APB_PSELX_UART0*`DATA_WIDTH +: `DATA_WIDTH]),
        .S_PREADY   (M_PREADY[`APB_PSELX_UART0]),
        // uart wires
        .tx_wire    (uart_tx),
        .rx_wire    (uart_rx)
    );

    timer_apb timr0 (
        .clk        (clk),
        .reset      (soft_reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_TIMR0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA[`APB_PSELX_TIMR0*`DATA_WIDTH +: `DATA_WIDTH]),
        .S_PREADY   (M_PREADY[`APB_PSELX_TIMR0])
        //
        `ifdef DEF_ENABLE_INT
        ,.out       (ints     [`DEF_INT_TIMR0]),
         .int_data  (ints_data[`DEF_INT_TIMR0*`DATA_WIDTH +: `DATA_WIDTH])
        `endif
    );

    // Shared register set for system-on-chip info
    // R0 = number of cores
    vmicro16_regs_apb # (
        .BUS_WIDTH          (`APB_WIDTH),
        .DATA_WIDTH         (`DATA_WIDTH),
        .CELL_DEPTH         (8),
        .PARAM_DEFAULTS_R0  (`CORES),
        .PARAM_DEFAULTS_R1  (`SLAVES)
    ) regs0_apb (
        .clk        (clk),
        .reset      (soft_reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_REGS0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA[`APB_PSELX_REGS0*`DATA_WIDTH +: `DATA_WIDTH]),
        .S_PREADY   (M_PREADY[`APB_PSELX_REGS0])
    );
    
    vmicro16_bram_ex_apb # (
        .BUS_WIDTH    (`APB_WIDTH),
        .MEM_WIDTH    (`DATA_WIDTH),
        .MEM_DEPTH    (`APB_BRAM0_CELLS),
        .CORE_ID_BITS (`clog2(`CORES))
    ) bram_apb (
        .clk        (clk),
        .reset      (soft_reset),
        // apb slave to master interface
        .S_PADDR    (M_PADDR),
        .S_PWRITE   (M_PWRITE),
        .S_PSELx    (M_PSELx[`APB_PSELX_BRAM0]),
        .S_PENABLE  (M_PENABLE),
        .S_PWDATA   (M_PWDATA),
        .S_PRDATA   (M_PRDATA[`APB_PSELX_BRAM0*`DATA_WIDTH +: `DATA_WIDTH]),
        .S_PREADY   (M_PREADY[`APB_PSELX_BRAM0])
    );

    // There must be atleast 1 core
    `static_assert(`CORES > 0)
    `static_assert(`DEF_MEM_INSTR_DEPTH > 0)
    `static_assert(`DEF_MMU_TIM0_CELLS > 0)


    // Single instruction memory
`ifndef DEF_CORE_HAS_INSTR_MEM
    // slave input/outputs from interconnect
    wire [`APB_WIDTH-1:0]          instr_M_PADDR;
    wire                           instr_M_PWRITE;
    wire [1-1:0]                   instr_M_PSELx;  // not shared
    wire                           instr_M_PENABLE;
    wire [`DATA_WIDTH-1:0]         instr_M_PWDATA; 
    wire [1*`DATA_WIDTH-1:0]       instr_M_PRDATA; // slave response
    wire [1-1:0]                   instr_M_PREADY; // slave response

    // Master apb interfaces
    wire [`CORES*`APB_WIDTH-1:0]   instr_w_PADDR;
    wire [`CORES-1:0]              instr_w_PWRITE;
    wire [`CORES-1:0]              instr_w_PSELx;
    wire [`CORES-1:0]              instr_w_PENABLE;
    wire [`CORES*`DATA_WIDTH-1:0]  instr_w_PWDATA;
    wire [`CORES*`DATA_WIDTH-1:0]  instr_w_PRDATA;
    wire [`CORES-1:0]              instr_w_PREADY;

    vmicro16_bram_apb # (
        .BUS_WIDTH      (`APB_WIDTH),
        .MEM_WIDTH      (`DATA_WIDTH),
        .MEM_DEPTH      (`DEF_MEM_INSTR_DEPTH),
        .USE_INITS      (1),
        .NAME           ("INSTR_ROM_G")
    ) instr_rom_apb (
        .clk            (clk),
        .reset          (soft_reset),
        .S_PADDR        (instr_M_PADDR),
        .S_PWRITE       (),
        .S_PSELx        (instr_M_PSELx),
        .S_PENABLE      (instr_M_PENABLE),
        .S_PWDATA       (),
        .S_PRDATA       (instr_M_PRDATA),
        .S_PREADY       (instr_M_PREADY)
    );
     
    apb_intercon_s # (
        .MASTER_PORTS   (`CORES),
        .SLAVE_PORTS    (1),
        .BUS_WIDTH      (`APB_WIDTH),
        .DATA_WIDTH     (`DATA_WIDTH),
        .HAS_PSELX_ADDR (0)
    ) apb_instr_intercon (
        .clk        (clk),
        .reset      (soft_reset),
        // APB master from cores
        // master
        .S_PADDR    (instr_w_PADDR),
        .S_PWRITE   (instr_w_PWRITE),
        .S_PSELx    (instr_w_PSELx),
        .S_PENABLE  (instr_w_PENABLE),
        .S_PWDATA   (instr_w_PWDATA),
        .S_PRDATA   (instr_w_PRDATA),
        .S_PREADY   (instr_w_PREADY),
        // shared bus slaves
        // slave outputs
        .M_PADDR    (instr_M_PADDR),
        .M_PWRITE   (instr_M_PWRITE),
        .M_PSELx    (instr_M_PSELx),
        .M_PENABLE  (instr_M_PENABLE),
        .M_PWDATA   (instr_M_PWDATA),
        .M_PRDATA   (instr_M_PRDATA),
        .M_PREADY   (instr_M_PREADY)
    );
`endif

    genvar i;
    generate for(i = 0; i < `CORES; i = i + 1) begin : cores
        
        vmicro16_core # (
            .CORE_ID            (i),
            .DATA_WIDTH         (`DATA_WIDTH),
            
            .MEM_INSTR_DEPTH    (`DEF_MEM_INSTR_DEPTH),
            .MEM_SCRATCH_DEPTH  (`DEF_MMU_TIM0_CELLS)
        ) c1 (
            .clk        (clk),
            .reset      (soft_reset),

            // debug
            .halt       (w_halt[i]),

            // interrupts
            .ints       (ints),
            .ints_data  (ints_data),

            // Output master port 1
            .w_PADDR    (w_PADDR   [`APB_WIDTH*i +: `APB_WIDTH]  ),
            .w_PWRITE   (w_PWRITE  [i]                           ),
            .w_PSELx    (w_PSELx   [i]                           ),
            .w_PENABLE  (w_PENABLE [i]                           ),
            .w_PWDATA   (w_PWDATA  [`DATA_WIDTH*i +: `DATA_WIDTH]),
            .w_PRDATA   (w_PRDATA  [`DATA_WIDTH*i +: `DATA_WIDTH]),
            .w_PREADY   (w_PREADY  [i]                           )

`ifndef DEF_CORE_HAS_INSTR_MEM
            // APB instruction rom
            , // Output master port 2
            .w2_PADDR   (instr_w_PADDR   [`APB_WIDTH*i +: `APB_WIDTH]  ),
            //.w2_PWRITE  (instr_w_PWRITE  [i]                           ),
            .w2_PSELx   (instr_w_PSELx   [i]                           ),
            .w2_PENABLE (instr_w_PENABLE [i]                           ),
            //.w2_PWDATA  (instr_w_PWDATA  [`DATA_WIDTH*i +: `DATA_WIDTH]),
            .w2_PRDATA  (instr_w_PRDATA  [`DATA_WIDTH*i +: `DATA_WIDTH]),
            .w2_PREADY  (instr_w_PREADY  [i]                           )
`endif
        );
    end
    endgenerate

    
    /////////////////////////////////////////////////////
    // Formal Verification
    /////////////////////////////////////////////////////
    `ifdef FORMAL
    wire all_halted = &w_halt;
    /////////////////////////////////////////////////////
    // Count number of clocks each core is spending on
    //   bus transactions
    /////////////////////////////////////////////////////
    reg [15:0] bus_core_times       [0:`CORES-1];
    reg [15:0] core_work_times      [0:`CORES-1];
    reg [15:0] instr_fetch_times    [0:`CORES-1];
    integer i2;
    initial 
        for(i2 = 0; i2 < `CORES; i2 = i2 + 1) begin
            bus_core_times[i2] = 0;
            core_work_times[i2] = 0;
        end

    // total bus time
    generate
        genvar g2;
        for (g2 = 0; g2 < `CORES; g2 = g2 + 1) begin : formal_for_times
			  always @(posedge clk) begin
					if (w_PSELx[g2])
						 bus_core_times[g2] <= bus_core_times[g2] + 1;

					// Core working time
					`ifndef DEF_CORE_HAS_INSTR_MEM
						 if (!w_PSELx[g2] && !instr_w_PSELx[g2])
					`else
						 if (!w_PSELx[g2])
					`endif
							  if (!w_halt[g2])
									core_work_times[g2] <= core_work_times[g2] + 1;

			  end
		  end
    endgenerate

    reg [15:0] bus_time_average = 0;
    reg [15:0] bus_reqs_average = 0;
    reg [15:0] fetch_time_average = 0;
    reg [15:0] work_time_average = 0;
    //
    always @(all_halted) begin
        for (i2 = 0; i2 < `CORES; i2 = i2 + 1) begin
            bus_time_average   = bus_time_average   + bus_core_times[i2];
            bus_reqs_average   = bus_reqs_average   + bus_core_reqs_count[i2];
            work_time_average  = work_time_average  + core_work_times[i2];
            fetch_time_average = fetch_time_average + instr_fetch_times[i2];
        end

        bus_time_average   = bus_time_average   / `CORES;
        bus_reqs_average   = bus_reqs_average   / `CORES;
        work_time_average  = work_time_average  / `CORES;
        fetch_time_average = fetch_time_average / `CORES;
    end

    ////////////////////////////////////////////////////
    // Count number of bus requests per core
    ////////////////////////////////////////////////////
    // 1 clock delay of w_PSELx
    reg [`CORES-1:0] bus_core_reqs_last;
    // rising edges of each 
    wire [`CORES-1:0] bus_core_reqs_real;
    // storage for counters for each core
    reg [15:0] bus_core_reqs_count [0:`CORES-1];
    initial 
        for(i2 = 0; i2 < `CORES; i2 = i2 + 1)
            bus_core_reqs_count[i2] = 0;

    // 1 clk delay to detect rising edge
    always @(posedge clk)
        bus_core_reqs_last <= w_PSELx;
    
    generate
        genvar g3;
			  for (g3 = 0; g3 < `CORES; g3 = g3 + 1) begin : formal_for_reqs
			  // Detect new reqs for each core
			  assign bus_core_reqs_real[g3] = w_PSELx[g3] > 
														bus_core_reqs_last[g3];
			  
			  always @(posedge clk)
					if (bus_core_reqs_real[g3])
						 bus_core_reqs_count[g3] <= bus_core_reqs_count[g3] + 1;

		 end
    endgenerate
    

    `ifndef DEF_CORE_HAS_INSTR_MEM
        ////////////////////////////////////////////////////
        // Time waiting for instruction fetches 
        //   from global  memory
        ////////////////////////////////////////////////////
        integer i3;
        initial 
            for(i3 = 0; i3 < `CORES; i3 = i3 + 1)
                instr_fetch_times[i3] = 0;

        // total bus time
        // Instruction fetches occur on the w2 master port
        generate
            genvar g4;
            for (g4 = 0; g4 < `CORES; g4 = g4 + 1) begin : formal_for_fetch_times
                always @(posedge clk)
                    if (instr_w_PSELx[g4])
                        instr_fetch_times[g4] <= instr_fetch_times[g4] + 1;
            end
        endgenerate
    `endif


    `endif // end FORMAL

endmodule