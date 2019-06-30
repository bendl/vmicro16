// LWEX/SWEX test

    sw      r0, r5 + #0x2
main:
    mov     r0, r1

    // jump to main
    movi    r5, main
    br      r5, #0x01

    lw      r0, r5 + #0x1
    sw      r0, r5 + #0x2