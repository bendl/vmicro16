//
//
//

`include "vmicro16_soc_config.v"
`include "clog2.v"
`include "formal.v"


module addr_dec # (
    parameter BUS_WIDTH   = 16,
    parameter SLAVE_PORTS = 10
) (
    input      [BUS_WIDTH-1:0]           addr,
    output     [SLAVE_PORTS-1:0]         sel,
    output reg [`clog2(SLAVE_PORTS)-1:0] seli
);
    // binary bit output
    // GPIO0
    assign sel[`APB_PSELX_GPIO0] = ((addr >= `DEF_MMU_GPIO0_S) 
                                 && (addr <= `DEF_MMU_GPIO0_E));
    // GPIO1
    assign sel[`APB_PSELX_GPIO1] = ((addr >= `DEF_MMU_GPIO1_S) 
                                 && (addr <= `DEF_MMU_GPIO1_E));
    // GPIO2
    assign sel[`APB_PSELX_GPIO2] = ((addr >= `DEF_MMU_GPIO2_S) 
                                 && (addr <= `DEF_MMU_GPIO2_E));
    // UART0
    assign sel[`APB_PSELX_UART0] = ((addr >= `DEF_MMU_UART0_S) 
                                 && (addr <= `DEF_MMU_UART0_E));
    // REGS0
    assign sel[`APB_PSELX_REGS0] = ((addr >= `DEF_MMU_REGS0_S)
                                 && (addr <= `DEF_MMU_REGS0_E));
    // BRAM0
    assign sel[`APB_PSELX_BRAM0] = ((addr >= `DEF_MMU_BRAM0_S) 
                                 && (addr <= `DEF_MMU_BRAM0_E));
    // TIMR0
    assign sel[`APB_PSELX_TIMR0] = ((addr >= `DEF_MMU_TIMR0_S) 
                                 && (addr <= `DEF_MMU_TIMR0_E));
    // WDOG0
    assign sel[`APB_PSELX_WDOG0] = ((addr >= `DEF_MMU_WDOG0_S) 
                                 && (addr <= `DEF_MMU_WDOG0_E));

    // binary number output
    always @(*)
        if      ((addr >= `DEF_MMU_GPIO0_S) && (addr <= `DEF_MMU_GPIO0_E))
            seli = `APB_PSELX_GPIO0;
        else if ((addr >= `DEF_MMU_GPIO1_S) && (addr <= `DEF_MMU_GPIO1_E))
            seli = `APB_PSELX_GPIO1;
        else if ((addr >= `DEF_MMU_GPIO2_S) && (addr <= `DEF_MMU_GPIO2_E))
            seli = `APB_PSELX_GPIO2;
        else if ((addr >= `DEF_MMU_UART0_S) && (addr <= `DEF_MMU_UART0_E))
            seli = `APB_PSELX_UART0;
        else if ((addr >= `DEF_MMU_REGS0_S) && (addr <= `DEF_MMU_REGS0_E))
            seli = `APB_PSELX_REGS0;
        else if ((addr >= `DEF_MMU_BRAM0_S) && (addr <= `DEF_MMU_BRAM0_E))
            seli = `APB_PSELX_BRAM0;
        else if ((addr >= `DEF_MMU_TIMR0_S) && (addr <= `DEF_MMU_TIMR0_E))
            seli = `APB_PSELX_TIMR0;
        else if ((addr >= `DEF_MMU_WDOG0_S) && (addr <= `DEF_MMU_WDOG0_E))
            seli = `APB_PSELX_WDOG0;
        else
            seli = 0;
endmodule

/**
 * Module: arbiter
 *
 * Description:
 *  A look ahead, round-robing parametrized arbiter.
 *
 * <> request
 *  each bit is controlled by an actor and each actor can 'request' ownership
 *  of the shared resource by bring high its request bit.
 *
 * <> grant
 *  when an actor has been given ownership of shared resource its 'grant' bit
 *  is driven high
 *
 * <> select
 *  binary representation of the grant signal (optional use)
 *
 * <> active
 *  is brought high by the arbiter when (any) actor has been given ownership
 *  of shared resource.
 *
 *
 * Created: Sat Jun  1 20:26:44 EDT 2013
 *
 * Author:  Berin Martini // berin.martini@gmail.com
 */
module arbiter
  #(parameter
    NUM_PORTS = 6)
   (input                               clk,
    input                               rst,
    input      [NUM_PORTS-1:0]          request,
    output reg [NUM_PORTS-1:0]          grant,
    output reg [`clog2(NUM_PORTS)-1:0]  select,
    output reg                          active
);
    /**
     * Local parameters
     */

    localparam WRAP_LENGTH = 2*NUM_PORTS;

    // Find First 1 - Start from MSB and count downwards, returns 0 when no
    // bit set
    function [`clog2(NUM_PORTS)-1:0] ff1;
        input [NUM_PORTS-1:0] in;
        integer i;
        begin
            ff1 = 0;
            for (i = NUM_PORTS-1; i >= 0; i=i-1) begin
                if (in[i])
                    ff1 = i;
            end
        end
    endfunction

`ifdef VERBOSE
    initial $display("Bus arbiter with %d units", NUM_PORTS);
`endif

    /**
     * Internal signals
     */
    integer                 yy;
    wire                    next;
    wire [NUM_PORTS-1:0]    order;
    reg  [NUM_PORTS-1:0]    token;
    wire [NUM_PORTS-1:0]    token_lookahead [NUM_PORTS-1:0];
    wire [WRAP_LENGTH-1:0]  token_wrap;

    /**
     * Implementation
     */
    assign token_wrap   = {token, token};
    assign next         = ~|(token & request);

    always @(posedge clk)
        grant <= token & request;

    always @(posedge clk)
        select <= ff1(token & request);

    always @(posedge clk)
        active <= |(token & request);

    always @(posedge clk)
        if (rst) token <= 'b1;
        else if (next) begin
            for (yy = 0; yy < NUM_PORTS; yy = yy + 1) begin : TOKEN_

                if (order[yy]) begin
                    token <= token_lookahead[yy];
                end
            end
        end

    genvar xx;
    generate
        for (xx = 0; xx < NUM_PORTS; xx = xx + 1) begin : ORDER_
            assign token_lookahead[xx]  = token_wrap[xx +: NUM_PORTS];
            assign order[xx]            = |(token_lookahead[xx] & request);
        end
    endgenerate
endmodule


module apb_intercon_s # (
    parameter BUS_WIDTH    = 16,
    parameter DATA_WIDTH   = 16,
    parameter MASTER_PORTS = 1,
    parameter SLAVE_PORTS  = 16,
    parameter ARBITER_ROTATE_INC  = 1,
    parameter ARBITER_ROTATE_NEXT = 0,
    parameter ARBITER_HIGHBIT     = 0,
    parameter HAS_PSELX_ADDR = 1
) (
    input clk,
    input reset,

    // APB master interface (from cores)
    input      [MASTER_PORTS*BUS_WIDTH-1:0]  S_PADDR,
    input      [MASTER_PORTS-1:0]            S_PWRITE,
    input      [MASTER_PORTS-1:0]            S_PSELx,
    input      [MASTER_PORTS-1:0]            S_PENABLE,
    input      [MASTER_PORTS*DATA_WIDTH-1:0] S_PWDATA,
    output reg [MASTER_PORTS*DATA_WIDTH-1:0] S_PRDATA,
    output reg [MASTER_PORTS-1:0]            S_PREADY,

    // MASTER interface to a slave
    output  [BUS_WIDTH-1:0]                 M_PADDR,
    output                                  M_PWRITE,
    output  [SLAVE_PORTS-1:0]               M_PSELx,
    output                                  M_PENABLE,
    output  [DATA_WIDTH-1:0]                M_PWDATA,
    // inputs from each slave
    input   [SLAVE_PORTS*DATA_WIDTH-1:0]    M_PRDATA,
    input   [SLAVE_PORTS-1:0]               M_PREADY
);
    // APB state machine
    localparam STATE_IDLE = 0;
    localparam STATE_T1   = 1;
    localparam STATE_T2   = 2;
    reg [1:0] state = STATE_IDLE;

    reg  [`clog2(MASTER_PORTS)-1:0] active = 0;
    wire [`clog2(MASTER_PORTS)-1:0] active_w;
    wire [MASTER_PORTS-1:0]         granted;

    generate if (ARBITER_HIGHBIT) begin : gen_arbiter_high_bit
        arbiter #(
            .NUM_PORTS(MASTER_PORTS)
        ) apb_arbiter (
            .clk     (clk),
            .rst     (reset),
            .request (S_PSELx),
            .grant   (granted),
            .select  (active_w)
        );

        always @(*)
            active = active_w;
    end
    endgenerate
    
    generate if (ARBITER_ROTATE_NEXT) begin : gen_arbiter_rotate_next
        integer bit_find = 0;
        always @(posedge clk)
            // if other requests are waiting
            if ((|S_PSELx) && active_ended)
                if (active >= MASTER_PORTS-1)
                    active <= 0;
                else
                    // move to next active master
                    for (bit_find = (MASTER_PORTS-1); bit_find >= 0; bit_find = bit_find - 1)
                        if (S_PSELx[bit_find]) begin
                            active <= bit_find;
                        end
    end
    endgenerate

    generate if (ARBITER_ROTATE_INC) begin : gen_arbiter_rotate_inc
        always @(posedge clk)
            if (|S_PSELx)
                if (active_ended)
                    if (active == (MASTER_PORTS-1))
                        active <= 0;
                    else
                        active <= active + 1;
    end
    endgenerate

    always @(active)
        $display($time, "\tactive core: %h", active);

    // wires for current active master
    wire  [BUS_WIDTH-1:0]   a_S_PADDR   = S_PADDR  [active*BUS_WIDTH +: BUS_WIDTH];
    wire                    a_S_PWRITE  = S_PWRITE [active];
    wire                    a_S_PSELx   = S_PSELx  [active];
    wire                    a_S_PENABLE = S_PENABLE[active] & S_PENABLE_gate;
    wire  [DATA_WIDTH-1:0]  a_S_PWDATA  = S_PWDATA [active*DATA_WIDTH +: DATA_WIDTH];
    wire  [DATA_WIDTH-1:0]  a_S_PRDATA  = S_PRDATA [active*DATA_WIDTH +: DATA_WIDTH];
    wire                    a_S_PREADY  = S_PREADY [active];

    reg S_PENABLE_gate = 0;
    always @(posedge clk)
        S_PENABLE_gate <= ((|a_S_PSELx));

    wire active_ended = !a_S_PSELx;

    wire [`clog2(SLAVE_PORTS)-1:0] M_PSELx_int;
    generate if (HAS_PSELX_ADDR) begin : gen_pselx_dec
        // if there are more than 1 slave ports,
        // we need to decode the PADDR to determine which
        // slave the address is for
        addr_dec # (
            .SLAVE_PORTS    (SLAVE_PORTS)
        ) paddr_dec (
            .addr           (a_S_PADDR),
            .sel            (M_PSELx),
            .seli           (M_PSELx_int));
    end else begin
        // if only 1 SLAVE_PORTS, 
        //   its always active.
        `static_assert_ng(SLAVE_PORTS == 1)
        assign M_PSELx_int = 0;
        assign M_PSELx     = a_S_PSELx;
    end
    endgenerate

    // Pass through
    assign M_PADDR   = a_S_PADDR;
    assign M_PWRITE  = a_S_PWRITE;
    assign M_PENABLE = a_S_PENABLE;
    assign M_PWDATA  = a_S_PWDATA;
    assign M_PWDATA  = a_S_PWDATA;
    
    // Demuxed transfer response back from slave to active master
    wire [BUS_WIDTH-1:0]    a_M_PRDATA = M_PRDATA[M_PSELx_int*DATA_WIDTH +: DATA_WIDTH];
    wire [SLAVE_PORTS-1:0]  a_M_PREADY = |(M_PSELx & M_PREADY);

    // transfer back to the active master
    always @(*) begin
        S_PREADY = 0;
        S_PRDATA = 0;
        S_PREADY[active]                          = a_M_PREADY;
        S_PRDATA[active*DATA_WIDTH +: DATA_WIDTH] = a_M_PRDATA;
    end
endmodule