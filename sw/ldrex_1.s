// LWEX/SWEX test

entry:
    // get core idx 0x80 in r0
    movi    r0, #0x80
    lw      r0, r0
    
    // write it to GPIO0 0x90 in r1
    movi    r1, #0x90
    sw      r0, r1

    // jump back to entry
    movi    r1, entry
    br      r1, BR_U
    
    halt    r0, r0