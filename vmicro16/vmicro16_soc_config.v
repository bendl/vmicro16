
`ifndef VMICRO16_SOC_CONFIG_H
`define VMICRO16_SOC_CONFIG_H

`define CORES           1
`define SLAVES          6

`define DEF_ALU_HW_MULT 1

`define DATA_WIDTH      16
`define APB_WIDTH       `DATA_WIDTH

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

`endif