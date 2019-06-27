
`ifndef VMICRO16_SOC_CONFIG_H
`define VMICRO16_SOC_CONFIG_H

`include "clog2.v"

`define CORES           2
`define SLAVES          6

`define DEF_ALU_HW_MULT 1

`define DATA_WIDTH      16
`define APB_WIDTH       (2 + `clog2(`CORES) + `DATA_WIDTH)

`define APB_PSELX_GPIO0 0
`define APB_PSELX_UART0 1
`define APB_PSELX_REGS0 2
`define APB_PSELX_BRAM0 3
`define APB_PSELX_GPIO1 4
`define APB_PSELX_GPIO2 5

`define APB_GPIO0_PINS  8
`define APB_GPIO1_PINS  16
`define APB_GPIO2_PINS  8

`define APB_PADDR_BRAM0 16'h00C0
`define APB_BRAM0_CELLS 64

///////////////////////////////////////////////////////////
// Memory mapping
//////////////////////////////////////////////////////////
// TIM0
`define DEF_MMU_TIM0_CELLS  128
`define DEF_MMU_TIM0_S      16'h0000
`define DEF_MMU_TIM0_E      16'h007F

// SREG
`define DEF_MMU_SREG_S      16'h0080
`define DEF_MMU_SREG_E      16'h008F

// GPIO0
`define DEF_MMU_GPIO0_S     16'h0090
`define DEF_MMU_GPIO0_E     16'h0090
// GPIO1
`define DEF_MMU_GPIO1_S     16'h0091
`define DEF_MMU_GPIO1_E     16'h0091
// GPIO2
`define DEF_MMU_GPIO2_S     16'h0092
`define DEF_MMU_GPIO2_E     16'h0092

// UART0
`define DEF_MMU_UART0_S     16'h00A0
`define DEF_MMU_UART0_E     16'h00A1

// REGS0
`define DEF_MMU_REGS0_S     16'h00B0
`define DEF_MMU_REGS0_E     16'h00B7

// BRAM0
`define DEF_MMU_TIM0_CELLS  128
`define DEF_MMU_BRAM0_S     16'h00C0
`define DEF_MMU_BRAM0_E     16'h00FF

///////////////////////////////////////////////////////////
// Core parameters
//////////////////////////////////////////////////////////
// Set this to use a workaround for the MMU's APB T2 clock
//`define FIX_T3



`endif