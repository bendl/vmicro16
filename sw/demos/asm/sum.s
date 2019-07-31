// sum.s

//0 
//1
//2 nstart
//3 0
//4 nsamples per node
//5 bram address
//6 core id
//7 num threads

// update:
//   assume 8 cores
//     = rshft by 3

    nop     r0, r0
    nop     r0, r0

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
    //      ns = 100
    //      nst = ns / (num_threads)
    //      nst = ns >> (num_threads - 1)
    //      r0 = (num_threads -1) WRONG!!!
    //mov     r0, r7
    //subi    r0, r3 + #0x01
    //movi    r4, #0x80
    //rshft   r4, r0
    
    // update: hex(240 // 8)
    // 1 cores
    //movi    r4, #0xf0
    // 2 cores
    //movi    r4, #0x78
    // 3 cores
    //movi    r4, #0x50
    // 4 cores
    movi    r4, #0x3c
    // 8 cores
    //movi    r4, #0x1e
    // 12 cores
    //movi    r4, #0x14
    // 15 cores
    //movi    r4, #0x0f
    // 16 cores
    //movi    r4, #0x0f
    // 30 cores
    //movi    r4, #0x08
    
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

    // EACH CORE
synced1:
    // Retrieve load the nsamples_per_thread
    lw      r4, r5 + #0x01
    // Calculate nstart = idx * nsamples_per_thread
    //   in r2
    mov     r2, r6
    mult    r2, r4

    // Loop limit in r4
    // samples_per_thread -> samples_per_thread + nstart
    add     r4, r2

    // Sum numbers from nstart to limit
sum_loop:
    // sum += i
    add     r1, r2
    // increment i
    addi    r2, r3 + #0x01
    // check end
    cmp     r2, r4
    movi    r0, sum_loop
    br      r0, BR_NE

    // partial sum finished in r1
    // use mutex to write to shared value
sum_mutex:
    // load latest count
    lwex    r0, r5 + #0x2
    // try increment count
    // increment by 1
    add     r0, r1
    // make copy as swex has a return value
    mov     r2, r0
    // attempt store
    swex    r0, r5 + #0x02
    // check success (== 0)
    cmp     r0, r3
    // branch if failed
    movi    r4, sum_mutex
    br      r4, BR_NE

write_gpio:
    movi    r3, #0x91
    sw      r2, r3

write_uart_done:
    // write ascii value to uart0
    movi    r3, #0xa0
    movi    r2, #0x30
    add     r2, r6
    sw      r2, r3

exit:
    halt    r0, r0