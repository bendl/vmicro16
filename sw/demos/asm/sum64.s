// sum64.s
// Simple 1-160 core summation program

// Set up common values, such as: Core id (r6), 
//   number of threads (cores) (r7), shared memory addresses (r5)
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

jmp_to_barrier:
    // NOT_ROOT
    //   wait at barrier
    cmp     r6, r3
    movi    r4, barrier_arrive
    br      r4, BR_NE
    
    // ROOT
    //   calculates nsamples_per_thread
    //      ns = 100
    //      nst = ns / (num_threads)
    //      nst = ns >> (num_threads - 1)
    //      r0 = (num_threads -1) WRONG!!!
    
root_broadcast:
    // The root (core idx 0) broadcasts the number of samples
    // 16 cores
    //movi    r4, #0x14
    // 32 cores
    //movi    r4, #0x0a
    // 64 cores
    movi    r4, #0x05
    // 80 cores
    //movi    r4, #0x04
    // 160 cores
    //movi    r4, #0x02
    
    // ROOT
    // Do the broadcast
    //   write nsamples_per_thread to shared bram (broadcast)
    //   0x1001
    sw      r4, r5 + #0x01

// Reach the barrier to tell everone
// that we have arrived
barrier_arrive:
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
    movi    r4, barrier_arrive
    br      r4, BR_NE

// Wait in an infinite loop
// for all cores to 'arrive'
barrier:
    // load the count
    lw      r0, r5
    // compare with number of threads
    cmp     r0, r7
    // jump back to barrier if not equal
    movi    r4, barrier
    br      r4, BR_NE

// EACH CORE
// All cores have arrived and in sync
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

// Perform the summation in a tight for loop
//   Sum numbers from nstart to limit
sum_loop:
    // sum += i
    add     r1, r2
    // increment i
    addi    r2, r3 + #0x01
    // check end
    cmp     r2, r4
    movi    r0, sum_loop
    br      r0, BR_NE

// Summation of the subset finished, result is in r1
//   Now use a mutex to add it to the global sum value in shared mem
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

// Write the latest global sum value to gpio1
write_gpio:
    movi    r3, #0x91
    sw      r2, r3

// Write the latest global sum value to uart0 tx
write_uart_done:
    movi    r3, #0xa0
    movi    r2, #0x30
    add     r2, r6
    sw      r2, r3

// This core has finished
// Enter a low power state
exit:
    halt    r0, r0
