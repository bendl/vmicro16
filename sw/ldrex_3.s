// LWEX/SWEX test
entry:
    // get core idx 0x80 in r7
    movi    r7, #0x80
    lw      r7, r7
    
    // BRAM0 shared memory 0x1000
    movi    r1, #0x01
    movi    r2, #0x0C
    lshft   r1, r2

    // branch if not core 0
    cmp     r7, r6
    movi    r6, try_inc
    br      r6, BR_NE

    // core 0 sets initial shared value
    lwex    r0, r1
    movi    r0, #0x30
    swex    r0, r1

try_inc:
    // increment by 1
    movi    r2, #0x01
    
    lwex    r0, r1
    // increment value
    add     r0, r2
    // backup value
    mov     r5, r0
    // attempt store
    swex    r0, r1
    // check success (== 0)
    cmp     r0, r3
    // branch if failed
    movi    r4, try_inc
    br      r4, BR_NE

print:
    // write the newest value to gpio
    movi    r1, #0x92
    sw      r5, r1

    // and to uart0
    movi    r1, #0xA0
    sw      r5, r1

exit:
    halt    r0, r0
