// func.s
// Test stack frames and stack variables

//ARITH_S $+1,    Sp, R5  3fa1            2       Function/sf entry
//0x04      SW    Bp,     +0(Sp)  16e0            0       (null)
//0x05    MOV     Bp,     Sp      26e0            0       main