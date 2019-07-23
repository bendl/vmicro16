
`ifndef FORMAL_H
`define FORMAL_H

// Verilog does not support $assert or compile time assertions
// So if a bad condition is met, produce a syntax error at the current line.
`define static_assert(c) \
    generate if (!(c)) bad_assertion bad_assertion(); endgenerate

// No generate version of static_assert
`define static_assert_ng(c) \
    if (!(c)) bad_assertion bad_assertion();

// Runtime assertion
`define rassert(c)                  \
    if (!(c)) begin                 \
        $display("BAD_ASSERRTION"); \
        $stop(1);                   \
    end

`endif