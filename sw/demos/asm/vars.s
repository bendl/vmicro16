// vars.s
//   Test global stack variables
var1:
    nop     r0, r0
var2:
    nop     r0, r0
var3:
    nop     r0, r0

entry:
    // get core idx 0x80 in r7
    movi    r7, #0x80
    lw      r7, r7

    // Write 1 to var1
    movi    r0, #0x01
    movi    r1, var1
    sw      r0, r1

    // Write 2 to var2
    movi    r0, #0x02
    movi    r1, var2
    sw      r0, r1

    // Write 2 to var3
    movi    r0, #0x03
    movi    r1, var3
    sw      r0, r1

exit:
    // Halt processor
    halt    r0, r0
