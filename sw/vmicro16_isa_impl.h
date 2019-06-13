//
// Created by BDL on 03/03/2018.
//

#ifndef VMICRO16_ISA_IMPL_H
#define VMICRO16_ISA_IMPL_H


#include "adt/ast.h"
#include "types.h"
#include <assert.h>


#ifdef __cplusplus
extern "C" {
#endif


#define BINP "%c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c"
#define BIN(byte)  \
  (byte & 0x8000 ? '1' : '0'), \
  (byte & 0x4000 ? '1' : '0'), \
  (byte & 0x2000 ? '1' : '0'), \
  (byte & 0x1000 ? '1' : '0'), \
  (byte & 0x0800 ? '1' : '0'), \
  (byte & 0x0400 ? '1' : '0'), \
  (byte & 0x0200 ? '1' : '0'), \
  (byte & 0x0100 ? '1' : '0'), \
  (byte & 0x0080 ? '1' : '0'), \
  (byte & 0x0040 ? '1' : '0'), \
  (byte & 0x0020 ? '1' : '0'), \
  (byte & 0x0010 ? '1' : '0'), \
  (byte & 0x0008 ? '1' : '0'), \
  (byte & 0x0004 ? '1' : '0'), \
  (byte & 0x0002 ? '1' : '0'), \
  (byte & 0x0001 ? '1' : '0')


#define BINP5 "%c%c%c%c%c"
#define BIN5(byte)  \
    (byte & 0x0010 ? '1' : '0'), \
    (byte & 0x0008 ? '1' : '0'), \
    (byte & 0x0004 ? '1' : '0'), \
    (byte & 0x0002 ? '1' : '0'), \
    (byte & 0x0001 ? '1' : '0')

#define PRCO_OP_BITS_OP   0b11111
#define PRCO_OP_BITS_REG  0b111
#define PRCO_OP_BITS_IMM8 0b11111111

struct prco_op_struct {
        unsigned short      opcode;
        unsigned char       op;
        unsigned char       flags;  //< Depracated

        unsigned char       regD;
        unsigned char       regA;
        unsigned char       regB;

        unsigned char imm8  : 8;
        signed char   simm5 : 5;

        unsigned char   port;

        void            *ast;
        unsigned char   asm_offset;
        unsigned int    asm_flags;
        char            *comment;
        unsigned int    id;
};

#define ASM_FUNC_EXIT   0x001
#define ASM_FUNC_START  0x002
#define ASM_IF_BRANCH   0x004
#define ASM_IF_ELSE     0x008
#define ASM_NOP_NOP     0x010
#define ASM_FUNC_CALL   0x020
#define ASM_CALL_NEXT   0x040
#define ASM_JMP_JMP     0x080
#define ASM_JMP_DEST    0x100

#define ASM_ASCII       0x200

#define ASM_POINTER     0x400

#define MAX_CALL_ARGS (8)

#define FOREACH_REG(REG) \
    REG(R0) \
    REG(R1) \
    REG(R2) \
    REG(R3) \
    REG(R4) \
    REG(R5) \
    REG(Bp) \
    REG(Sp) \
    REG(__prco_reg_MAX) \

#define FOREACH_OP(OP) \
    OP(NOP) \
    OP(LW) \
    OP(SW) \
    OP(BIT) \
    OP(MOV) \
    OP(MOVI) \
    OP(ARITH_U) \
    OP(ARITH_S) \
    OP(BR) \
    OP(CMP) \
    OP(SETC) \
    OP(MULT) \
    OP(HALT) \
    OP(__prco_op_MAX) \


#define UADD  0b11111
#define USUB  0b10000
#define UADDI 0b10001
#define FOREACH_ARITH_U(U) \
        U(U_ADD)            \
        U(U_USUB)            \
        U(U_UADDI)            \

#define SADD  0b11111
#define SSUB  0b10000
#define SADDI 0b10001
#define FOREACH_ARITH_S(S) \
        S(S_ADD)            \
        S(S_SSUB)            \
        S(S_SADDI)            \

#define FOREACH_BR(COND) \
        COND(JMP_UC)            \
        COND(JMP_JE)            \
        COND(JMP_JNE)           \
        COND(JMP_JG)            \
        COND(JMP_JGE)           \
        COND(JMP_JL)            \
        COND(JMP_JLE)           \
        COND(JMP_JS)            \
        COND(JMP_JNS)           \

#define FOREACH_BIT(BIT) \
        BIT(BIT_OR)            \
        BIT(BIT_XOR)           \
        BIT(BIT_AND)           \
        BIT(BIT_NOT)            \
        BIT(BIT_LSHFT)           \
        BIT(BIT_RSHFT)            \

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STR(STR) #STR,

enum prco_reg {
        FOREACH_REG(GENERATE_ENUM)
};

enum prco_op {
        FOREACH_OP(GENERATE_ENUM)
};

enum prco_br {
        FOREACH_BR(GENERATE_ENUM)
};

enum prco_arith_u {
        FOREACH_ARITH_U(GENERATE_ENUM)
};

enum prco_arith_s {
        FOREACH_ARITH_S(GENERATE_ENUM)
};

enum prco_bit {
        FOREACH_BIT(GENERATE_ENUM)
};

STATIC_ASSERT(__prco_op_MAX <= PRCO_OP_BITS_OP+1, vm16_opcode_bit_length_exceeded);
STATIC_ASSERT(__prco_reg_MAX <= PRCO_OP_BITS_REG+1, 3_bit_vm16_opcode_exceeded);


static const char *REG_STR[]    = { FOREACH_REG (GENERATE_STR) };
static const char *OP_STR[]     = { FOREACH_OP  (GENERATE_STR) };
static const char *BR_STR[]    = { FOREACH_BR (GENERATE_STR) };
static const char *BIT_STR[]    = { FOREACH_BIT (GENERATE_STR) };
static const char *ARITH_U_STR[]    = { FOREACH_ARITH_U (GENERATE_STR) };
static const char *ARITH_S_STR[]    = { FOREACH_ARITH_S (GENERATE_STR) };


void vm16_assert_opcode(struct prco_op_struct *op, char print);

struct prco_op_struct vm16_opcode_t1(enum prco_op iop, enum prco_reg rd, enum prco_reg ra,
                                signed char simm5);

struct prco_op_struct vm16_opcode_nop(void);
struct prco_op_struct vm16_opcode_halt(void);

struct prco_op_struct vm16_opcode_mov_rr(enum prco_reg regD, enum prco_reg regA);
struct prco_op_struct vm16_opcode_mov_ri(enum prco_reg regD, unsigned char imm8);

struct prco_op_struct vm16_opcode_add_rr(enum prco_reg regA, enum prco_reg regD);
struct prco_op_struct vm16_opcode_add_ri(enum prco_reg regD, enum prco_reg regA, signed char imm8);
struct prco_op_struct vm16_opcode_sub_rr(enum prco_reg regA, enum prco_reg regD);
struct prco_op_struct vm16_opcode_sub_ri(enum prco_reg regD, enum prco_reg regA, signed char imm8);

struct prco_op_struct vm16_opcode_mul_rr(enum prco_reg regA, enum prco_reg regD);

struct prco_op_struct vm16_opcode_jmp_r(enum prco_reg rd);
struct prco_op_struct vm16_opcode_jmp_rc(enum prco_reg rd, enum prco_br cond);
struct prco_op_struct vm16_opcode_cmp_rr(enum prco_reg rd, enum prco_reg ra);

struct prco_op_struct vm16_opcode_neg_r(enum prco_reg regD);

struct prco_op_struct vm16_opcode_lw(enum prco_reg rd, enum prco_reg ra, signed char imm5);
struct prco_op_struct vm16_opcode_sw(enum prco_reg rd, enum prco_reg ra, signed char imm5);

struct prco_op_struct vm16_opcode_set_ri(enum prco_reg rd, unsigned char imm8);

struct prco_op_struct vm16_opcode_byte(unsigned char low);
struct prco_op_struct vm16_opcode_word(unsigned char high, unsigned char low);

#ifdef __cplusplus
}
#endif


#endif //VMICRO16_ISA_IMPL_H
