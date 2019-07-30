// interrupts.s
//   Toggle LED in ISR

entry:
    // get core idx 0x80 in r7
    movi    r7, #0x80
    lw      r7, r7

    // core1 sets up the timer
    // Core0 enables interrupts and performs the isr
    cmp     r7, r0
    movi    r0, timer
    br      r0, BR_NE

    // Set interrupt vector (0)
    movi    r0, isr0
    movi    r1, #0x1
    movi    r2, #0x08
    lshft   r1, r2
    sw      r0, r1
    
    // enable all interrupts
    movi    r0, #0x0f
    sw      r0, r1 + #0x8

    // enter idle state
    halt    r0, r0

timer:
    // set timr0 address 0x200 into r0
    // shift left 8 places
    movi    r0, #0x01
    movi    r1, #0x09
    lshft   r0, r1

    // Set load value
    //movi    r1, #0x31
    //sw      r1, r0
    // test we the expected value back
    //lw      r2, r0

    // set load = 0x3000
    movi    r1, #0x3
    movi    r2, #0x0C
    //movi    r2, #0x04
    lshft   r1, r2
    sw      r1, r0

    // Set prescale value to 0x1000
    // 20ns * load * prescaler = nanosecond delay
    // 20ns * 10000 * 5000     = 1.0s
    // 20.0 * 0x3000 * 0x1000  = ~1.0s
    movi    r1, #0x1
    // 1.0 second
    //movi    r2, #0x0C
    // 0.5 second
    movi    r2, #0x0B
    // 0.25 second
    //movi    r2, #0x0a
    // 0.0625 second
    //movi    r2, #0x04
    lshft   r1, r2
    sw      r1, r0 + #0x02
    
    // Start the timer (write 0x0001 to 0x0101)
    movi    r1, #0x01
    sw      r1, r0 + #0x01

exit:
    // enter idle state
    halt    r0, r0

isr0:
    movi    r0, #0x90
    lw      r1, r0
    // xor with 1
    movi    r2, #0x1
    xor     r1, r2
    // write back
    sw      r1, r0

    // write ascii value to uart0
    movi    r0, #0xa0
    movi    r2, #0x30
    add     r1, r2
    sw      r1, r0

    // reset watchdog
    movi    r0, #0xb8
    sw      r1, r0

    // return from interrupt
    intr    r0, r0