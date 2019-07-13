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

    // set timr0 address 0x200 into r0
    movi    r0, #0x01
    // shift left 8 places
    movi    r1, #0x09
    lshft   r0, r1

    // Set load value
    movi    r1, #0x30
    sw      r1, r0
    // test we the expected value back
    lw      r2, r0

    // Start the timer (write 0x0001 to 0x0201)
    movi    r1, #0x01
    sw      r1, r0 + #0x01
    
    // Read 
    lw      r2, r0

    // Stop the timer (write 0x0002 to 0x0201)
    movi    r1, #0x02
    sw      r1, r0 + #0x01

    // Restart the timer
    movi    r1, #0x01
    sw      r1, r0 + #0x01

exit:
    // Halt processor
    halt    r0, r0
