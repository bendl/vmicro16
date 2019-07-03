
`ifndef VMICRO16_SOC_CONFIG_H
`define VMICRO16_SOC_CONFIG_H

`include "clog2.v"

`define CORES           2
`define SLAVES          7

///////////////////////////////////////////////////////////
// Core parameters
//////////////////////////////////////////////////////////
// Top level data width for registers, memory cells, bus widths
`define DATA_WIDTH      16

// Set this to use a workaround for the MMU's APB T2 clock
//`define FIX_T3

// Instruction memory (read only) on each core.
//   Must be large enough to support software program.
`define DEF_MEM_INSTR_DEPTH 512

// Scratch memory (read/write) on each core.
//   See `DEF_MMU_TIM0_* defines for info.
`define DEF_MEM_SCRATCH_DEPTH 64

// Enables hardware multiplier and mult rr instruction
`define DEF_ALU_HW_MULT 1


//////////////////////////////////////////////////////////
// Memory mapping
//////////////////////////////////////////////////////////
`define APB_WIDTH       (2 + `clog2(`CORES) + `DATA_WIDTH)

`define APB_PSELX_GPIO0 0
`define APB_PSELX_UART0 1
`define APB_PSELX_REGS0 2
`define APB_PSELX_BRAM0 3
`define APB_PSELX_GPIO1 4
`define APB_PSELX_GPIO2 5
`define APB_PSELX_TIMR0 6

`define APB_GPIO0_PINS  8
`define APB_GPIO1_PINS  16
`define APB_GPIO2_PINS  8

// Shared memory words
`define APB_BRAM0_CELLS 4096

//////////////////////////////////////////////////////////
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
`define DEF_MMU_BRAM0_S     16'h1000
`define DEF_MMU_BRAM0_E     16'h1fff
// TIMR0
`define DEF_MMU_TIMR0_S     16'h0200
`define DEF_MMU_TIMR0_E     16'h0201

//////////////////////////////////////////////////////////
// Interrupts
//////////////////////////////////////////////////////////
`define DEF_NUM_INT     8
`define DEF_INT_MASK    0
`define DEF_INT_TIMR0   0
// Interrupt vector memory location
`define DEF_MMU_INTSV_S     16'h0100
`define DEF_MMU_INTSV_E     16'h0107
// Interrupt vector memory location
`define DEF_MMU_INTSM_S     16'h0108
`define DEF_MMU_INTSM_E     16'h0108


`endif