// LWEX/SWEX test

    movi r0, #0x80
main:
    mov  r0, r1
    br   main, #0x01
    foo1  r0, r1
    foo2  r0, r1