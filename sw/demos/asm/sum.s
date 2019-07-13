// sum.s

//0 
//1
//2 nstart
//3 0
//4 nsamples per node
//5 bram address
//6 core id
//7 num threads

entry:
    // Core id in r6
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

    // NOT_ROOT
    //   wait at barrier
    cmp     r6, r3
    movi    r4, sem_inc
    br      r4, BR_NE
    
    // ROOT
    //   calculates nsamples_per_thread
    //     ns = 100
    //     nst = ns / (num_threads)
    //     nst = ns >> (num_threads - 1)
    movi    r4, #0x64
    subi    r7, r3 + #0x01
    rshft   r4, r7
    
    // ROOT
    //   write nsamples_per_thread to shared bram (broadcast)
    //   0x1001
    sw      r4, r5 + #0x01

    // ALl wait at barrier
sem_inc:
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
    movi    r4, sem_inc
    br      r4, BR_NE

barrier:
    // load the count
    lw      r0, r5
    // compare with number of threads
    cmp     r0, r7
    // jump back to barrier if not equal
    movi    r4, barrier
    br      r4, BR_NE

synced1:
    // Now load the nsamples_per_thread
    lw      r4, r5 + #0x01
    // Calculate start index
    //   nstart = idx * nsamples_per_thread
    mov     r2, r6
    mult    r2, r4

    // Loop from: nstart -> (nstart + nsamples_per_thread)
    movi    r2, #0xcc
    movi    r2, #0xdd
    movi    r2, #0xee
    movi    r2, #0xff

exit:
    halt    r0, r0