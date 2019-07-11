// semaphore barrier example

entry:
    // Core id in r0
    movi    r0, #0x80
    lw      r0, r0
    // store in r6
    mov     r6, r0
    
    // get number of threads
    movi    r0, #0x81
    lw      r0, r0
    // store in r7
    mov     r7, r0

    // BRAM0 shared memory 0x1000
    movi    r5, #0x01
    movi    r2, #0x0C
    lshft   r5, r2

    // core 0 does some long process
    cmp     r6, r3
    movi    r4, try_inc
    br      r4, BR_NE
    
    // long loop
    movi    r0, #0x00
    movi    r1, #0x0a
loop:
    addi    r0, r3 + #0x01
    cmp     r0, r1
    movi    r4, try_inc
    br      r4, BR_E
    movi    r4, loop
    br      r4, BR_NE

try_inc:
    // load latest count
    lwex    r0, r5
    // try increment count
    // increment by 1
    addi    r0, r3 + #0x01
    // attempt store
    swex    r0, r5

    // check success (== 0)
    cmp     r0, r3
    // branch if failed
    movi    r4, try_inc
    br      r4, BR_NE

barrier:
    // load the count
    lw      r0, r5
    // compare with number of threads
    cmp     r0, r7
    // jump back to barrier if not equal
    movi    r4, barrier
    br      r4, BR_NE

critical:
    movi    r2, #0xcc
    movi    r2, #0xdd
    movi    r2, #0xee
    movi    r2, #0xff

exit:
    halt    r0, r0