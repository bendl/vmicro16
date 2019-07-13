// LWEX/SWEX test

entry:
    // get core idx 0x80 in r7
    movi    r7, #0x80
    lw      r7, r7
    
    // BRAM0 shared memory C0
    movi    r1, #0xC1
    movi    r2, #0x01

try_inc:
    lwex    r0, r1
    // increment value
    add     r0, r2
    swex    r0, r1
    cmp     r0, r3
    movi    r4, try_inc
    br      r4, BR_NE
    //try_increment:
    //ldrex r0, [r1]
    //add r0, r0, #1
    //strex r2, r0, r1
    //cmp r2, #0
    //bne try_increment

    // Halt processor
    halt    r0, r0
