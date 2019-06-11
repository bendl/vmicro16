//
// Created by BDL on 09/06/2019
//


#include "vmicro16_isa_impl.h"
#include "vmicro16_cg_impl.h"
#include "dbug.h"
#include <assert.h>

void
assert_opcode(struct prco_op_struct *op, char print)
{
        assert(op);
        //printf("op: %s %d %s %s %s\r\n", OP_STR[prco_op->op], prco_op->flags,
        //  REG_STR[prco_op->regD], REG_STR[prco_op->regA], REG_STR[prco_op->regB]);
        switch (op->op) {
        case HALT:
                assert((op->opcode >> 11) == op->op);
                assert(((op->opcode >> 8) & PRCO_OP_BITS_REG) == op->regD);
                assert(((op->opcode >> 5) & PRCO_OP_BITS_REG) == op->regA);
                dbprintf(D_GEN, "%s\t\t\t%04x\t\t%d\t%s\r\n",
                       OP_STR[op->op],
                       op->opcode,
                       op->asm_flags,
                       op->comment);
                break;

        case NOP:
                // It might not have an OP assigned,
                // link raw bytes
                if(op->asm_flags & ASM_ASCII) {
                        assert((op->op >> 11) == op->op);
                        assert(((op->opcode >> 0) & PRCO_OP_BITS_IMM8) == op->imm8);
                        dbprintf(D_GEN, "%s\t%c\t%d\t%04x\t\t%d\t%s\r\n",
                                "ASCII",
                                op->imm8,
                                op->id,
                                op->opcode,
                                op->asm_flags,
                                op->comment);
                        break;
                } else {
                        assert((op->op >> 11) == op->op);
                        dbprintf(D_GEN, "%s\t%c\t\t%04x\t\t%d\t%s\r\n",
                                OP_STR[op->op],
                                op->imm8,
                                op->opcode,
                                op->asm_flags,
                                op->comment);
                        break;
                }


        case LW:
        case SW:
                assert((op->opcode >> 11) == op->op);
                assert(((op->opcode >> 8) & PRCO_OP_BITS_REG) == op->regD);
                assert(((op->opcode >> 5) & PRCO_OP_BITS_REG) == op->regA);
                dbprintf(D_GEN, "  %s\t%s,\t%+d(%s)\t%04x\t\t%d\t%s\r\n",
                       OP_STR[op->op],
                       REG_STR[op->regD],
                       op->simm5,
                       REG_STR[op->regA],
                       op->opcode,
                       op->asm_flags,
                       op->comment);
                break;


        case BR:
                assert((op->opcode >> 11) == op->op);
                assert(((op->opcode >> 8) & PRCO_OP_BITS_REG) == op->regD);
                dbprintf(D_GEN, "%s\t%s,\t%s\t%04x\t\t%d\t%s\r\n",
                        OP_STR[op->op],
                        REG_STR[op->regD],
                        BR_STR[op->imm8],
                        op->opcode,
                        op->asm_flags,
                        op->comment);
                break;

        case ARITH_U:
        case ARITH_S:
                assert((op->opcode >> 11) == op->op);
                assert(((op->opcode >> 8) & PRCO_OP_BITS_REG) == op->regD);
                assert(((op->opcode >> 0) & PRCO_OP_BITS_IMM8) == op->imm8);
                dbprintf(D_GEN, "%s\t$%+d,\t%s\t%04x\t\t%d\t%s\r\n",
                        OP_STR[op->op], (signed char) op->imm8,
                        REG_STR[op->regD],
                        op->opcode, op->asm_flags,
                        op->comment);
                break;

        case SETC:
                assert((op->opcode >> 11) == op->op);
                assert(((op->opcode >> 8) & PRCO_OP_BITS_REG) == op->regD);
                assert(((op->opcode >> 0) & PRCO_OP_BITS_IMM8) == op->imm8);
                dbprintf(D_GEN, "%s\t%s,\t%s\t%04x\t\t%d\t%s\r\n",
                        OP_STR[op->op],
                        REG_STR[op->regD],
                        BR_STR[op->imm8],
                        op->opcode, op->asm_flags,
                        op->comment);
                break;

        case MOV:
        case CMP:
                assert((op->opcode >> 11) == op->op);
                assert(((op->opcode >> 8) & PRCO_OP_BITS_REG) == op->regD);
                assert(((op->opcode >> 5) & PRCO_OP_BITS_REG) == op->regA);
                dbprintf(D_GEN, "%s\t%s,\t%s\t%04x\t\t%d\t%s\r\n",
                       OP_STR[op->op],
                       REG_STR[op->regD],
                       REG_STR[op->regA],
                       op->opcode,
                       op->asm_flags,
                       op->comment);
                break;

        case MOVI:
                assert((op->opcode >> 11) == op->op);
                assert(((op->opcode >> 8) & PRCO_OP_BITS_REG) == op->regD);
                if(((op->opcode >> 0) & PRCO_OP_BITS_IMM8) != op->imm8) {
                        dbprintf(D_ERR, "MOVI Assert error!\r\n");
                }
                assert(((op->opcode >> 0) & PRCO_OP_BITS_IMM8) == op->imm8);
                dbprintf(D_GEN, "%s\t$%x,\t%s\t%04x\t\t%d\t%s\r\n",
                       OP_STR[op->op],
                       op->imm8,
                       REG_STR[op->regD],
                       op->opcode,
                       op->asm_flags,
                       op->comment);
                break;
        default:
                dbprintf(D_GEN, "UNKNOWN OP!\r\n");
                assert("UNKNOWN OP!" && 0);
                break;
        }

}

struct prco_op_struct
opcode_nop(void)
{
        struct prco_op_struct op = {0};
        op.op = NOP;
        op.opcode |= op.op << 11;
        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_mov_ri(enum prco_reg regD, unsigned char imm8)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = MOVI;
        op.regD = regD;
        op.imm8 = imm8;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.imm8 << 0;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_mov_rr(enum prco_reg regD, enum prco_reg regA)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = MOV;
        op.regD = regD;
        op.regA = regA;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.regA << 5;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_add_rr(enum prco_reg regD, enum prco_reg regA)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = ARITH_U;
        op.regD = regD;
        op.regA = regA;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.regA << 5;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_add_ri(enum prco_reg regD, signed char imm8)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = ARITH_U;
        op.regD = regD;
        op.imm8 = imm8;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.imm8 << 0;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_sub_ri(enum prco_reg regD, signed char imm8)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = ARITH_U;
        op.regD = regD;
        op.imm8 = imm8;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.imm8 << 0;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_sub_rr(enum prco_reg regD, enum prco_reg regA)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = ARITH_U;
        op.regD = regD;
        op.regA = regA;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.regA << 5;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_jmp_r(enum prco_reg rd)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = BR;
        op.regD = rd;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_cmp_rr(enum prco_reg rd,
              enum prco_reg ra)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = CMP;
        op.regD = rd;
        op.regA = ra;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.regA << 5;

        assert_opcode(&op, 0);
        return op;
}


struct prco_op_struct
opcode_lw(enum prco_reg rd, enum prco_reg ra, signed char imm5)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = LW;
        op.regD = rd;
        op.regA = ra;
        op.simm5 = (imm5 & 0b11111);
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.regA << 5;
        op.opcode |= (op.simm5 & 0b11111) << 0;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_sw(enum prco_reg rd, enum prco_reg ra, signed char imm5)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = SW;
        op.regD = rd;
        op.regA = ra;
        op.simm5 = imm5;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.regA << 5;
        op.opcode |= (op.simm5 & 0b11111) << 0;

        assert_opcode(&op, 0);
        return op;
}


struct prco_op_struct
opcode_jmp_rc(enum prco_reg rd, enum prco_br cond)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = BR;
        op.regD = rd;
        op.imm8 = cond;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.imm8 << 0;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_set_ri(enum prco_reg rd, unsigned char imm8)
{
        struct prco_op_struct op = {0};
        op.flags = 0;
        op.op = SETC;
        op.regD = rd;
        op.imm8 = imm8;
        op.opcode |= op.op << 11;
        op.opcode |= op.regD << 8;
        op.opcode |= op.imm8 << 0;

        assert_opcode(&op, 0);
        return op;
}

struct prco_op_struct
opcode_byte(unsigned char low)
{
        struct prco_op_struct op = {0};
        op.asm_flags |= ASM_ASCII;

        op.op = 0;
        op.imm8 = low;

        op.opcode |= op.imm8 << 0;

        assert_opcode(&op, 0);
        return op;
}
