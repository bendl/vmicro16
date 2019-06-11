//
// Created by BDL on 09/06/2019
//

#include <assert.h>

#include <stdlib.h>
#include <string.h>

#include "dbug.h"
#include "vmicro16_cg_impl.h"
#include "vmicro16_isa_impl.h"
#include "gen.h"

#define ASM_OFFSET_BYTES 1

// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=53119
struct prco_op_struct asm_list[0xff] = {{0}};
int asm_list_it = 0;

// Unique instruction IDs
unsigned int g_asm_id = 0;
#define NEW_ASM_ID() ++g_asm_id

// Deprecated instruction tagging scheme
// Replaced with inserting NOPs (easier, but slower)
int asm_tag_stack = -1;
int asm_tag_next = 0;
int asm_tag_id = 0;

void
cg_target_vmicro16_init(struct target_delegate *dt)
{
        dbprintf(D_INFO, "cg: %s\r\n", __FUNCTION__);

        // Initialise function pointers to codegen routines
        dt->cg_postcode = cg_postcode_vmicro16;
        dt->cg_precode = cg_precode_vmicro16;

        // Logic statements
        dt->cg_function = cg_function_vmicro16;
        dt->cg_bin = cg_bin_vmicro16;
        dt->cg_expr = cg_expr_vmicro16;
        dt->cg_number = cg_number_vmicro16;

        // Control statements
        dt->cg_if = cg_if_vmicro16;

        // Variable referencing
        dt->cg_local_decl = cg_local_decl_vmicro16;
        dt->cg_var_ref = cg_var_ref_vmicro16;
        dt->cg_assignment = cg_assignment_vmicro16;
}

void
asm_push(struct prco_op_struct op)
{
        asm_list[asm_list_it] = op;
        asm_list[asm_list_it].asm_offset = asm_list_it * ASM_OFFSET_BYTES;

        // Depraced, using NOPs for now
        if (0 && asm_tag_stack > 0) {
                dbprintf(D_GEN, "GEN: asm_tag_next: %d\r\n", asm_tag_next);
                asm_list[asm_list_it].asm_flags |= asm_tag_next;
                asm_list[asm_list_it].id = asm_tag_id;
                asm_tag_stack--;
                if (asm_tag_stack < 0) asm_tag_stack = 0;
        }

        asm_list_it++;
}

#define asm_comment(s)                                                        \
        asm_list[(asm_list_it - 1) < 0 ? 0 : (asm_list_it-1)].comment = (s);

#define asm_tag_last(GUID)                                                    \
        asm_list[(asm_list_it - 1) < 0 ? 0 : (asm_list_it-1)].id = (GUID);

#define for_each_asm(it, asm_p)                                               \
        for ((it) = 0, (asm_p) = &asm_list[(it)];                             \
                (it) < asm_list_it;                                           \
                (it)++, (asm_p) = &asm_list[(it)])

void
assembler_labels(void)
{
        int it, find;
        int offset_check = 0x00;
        struct prco_op_struct *op, *findop;

        dbprintf(D_GEN, "assembler_labels:\r\n");

        for_each_asm(it, op) {
                assert(op->asm_offset == offset_check);
                offset_check += ASM_OFFSET_BYTES;

                // Its pointer
                // Find where it's pointing
                if(op->asm_flags & ASM_POINTER) {
                        assert(op->op == MOVI);

                        // It's pointing to something with the same id
                        for_each_asm(find, findop) {
                                // Limit pointers to ASM_ASCII as there is a conflict
                                // between assembler's g_asm_id and parser's g_uid
                                // TODO: Easy fix, use a single global GUID
                                if (findop->asm_flags & ASM_ASCII &&
                                        op->id == findop->id)
                                {
                                        op->imm8 = findop->asm_offset;
                                        op->opcode |= op->imm8;
                                }
                        }

                        continue;
                }

                // If we need to work out the return address
                // of a function
                if (op->asm_flags & ASM_CALL_NEXT) {
                        // Return addresses are placed into a register
                        // by a MOVI instsruction
                        assert(op->op == MOVI);
                        // 5 is the number of instructions to Create and push
                        //  the return pointer for the call
                        op->imm8 = op->asm_offset + (5 * ASM_OFFSET_BYTES);
                        // encode the new value in the opcode
                        op->opcode |= (op->imm8 & 0xff);
                        assert((op->opcode & 0xff) == op->imm8);

                        // Remove the flag as we are done with it
                        op->asm_flags &= ~ASM_CALL_NEXT;

                        continue;
                }

                // If we need to work out where to jump
                if (op->asm_flags & ASM_FUNC_CALL) {
                        struct ast_call *caller = (struct ast_call *) op->ast;

                        // Find offset of function entry
                        for_each_asm(find, findop) {
                                if (it == find) continue;

                                if ((findop->asm_flags & ASM_FUNC_START) && findop->ast) {
                                        struct ast_func *callee = (struct ast_func *) findop->ast;
                                        if (strcmp(caller->callee, callee->proto->name) ==
                                            0) {
                                                op->imm8 = findop->asm_offset;
                                                op->opcode |= op->imm8 & 0xff;

                                                // Remove the flag
                                                op->asm_flags &= ~ASM_FUNC_CALL;
                                        }
                                }
                        }

                        continue;
                }

                // Opcode is a JMP instruction jumping somewhere
                // Find where...
                if (op->asm_flags & ASM_JMP_JMP) {
                        // Jump operation start with a MOVI to
                        // put address of destination into a register
                        assert(op->op == MOVI);

                        // Find findop with same <id>
                        for_each_asm(find, findop) {
                                if (it == find) continue;

                                if(op->ast) {
                                        if(findop->asm_flags & ASM_FUNC_START &&
                                                findop->ast == op->ast) {
                                                op->imm8 = findop->asm_offset;
                                                op->opcode |= op->imm8 & 0xff;
                                        }
                                } else {
                                        if ((findop->asm_flags & ASM_JMP_DEST) &&
                                            (findop->id == op->id)) {
                                                op->imm8 = findop->asm_offset;
                                                op->opcode |= op->imm8 & 0xff;
                                                // Remove the flag
                                                //op->asm_flags &= ~ASM_JMP_JMP;
                                        }
                                }
                        }

                }
        }
}

int
is_entry_func(struct ast_proto *p)
{
        return strcmp(p->name, "main") == 0;
}

void
create_verilog_memh_file(void)
{
        FILE *fcoe;
        int it;
        struct prco_op_struct *op;

        dbprintf(D_GEN, "\r\ncreate_verilgo_memh_file...\r\n");

        fcoe = fopen("verilog_memh.txt", "w");
        if (!fcoe) {
                dbprintf(D_ERR, "Unable to open uvm_coe.coe!\r\n");
                return;
        }

        // Write each instruction opcode on each line
        for_each_asm(it, op) {
                fprintf(fcoe, "%04x\n", op->opcode);
        }

        // Write top of stack address
        // /fprintf(fcoe, "@ff\n00ff");

        fclose(fcoe);
}

void
cg_precode_vmicro16(void)
{
        int it;
        struct prco_op_struct init_jmp;

        dbprintf(D_GEN, ".precode\r\n");
        for (it = NOP; it < __prco_op_MAX; it++) {
                dbprintf(D_GEN, "`define PRCO_OP_%s\t5'b"BINP5"\n",
                        OP_STR[it], BIN5(it));
        }

        for (it = UART1; it < __prco_port_MAX; it++) {
                dbprintf(D_GEN, "`define PRCO_PORT_%s\t\t8'b"BINP5"\n",
                        PORT_STR[it], BIN5(it));
        }

        /*
        opcode_mov_ri(Ax, 0x10);
        opcode_mov_ri(Bx, 0x10);
        opcode_cmp_rr(Ax, Bx);
        opcode_mov_ri(Cx, 0x00);
        opcode_jmp_r(Cx);

        opcode_mov_ri(Ax, 0x42);
        opcode_write(Ax, UART1);
        opcode_mov_ri(Ax, 0x45);
        opcode_write(Ax, UART1);
        opcode_mov_ri(Ax, 0x4E);
        opcode_write(Ax, UART1);
        opcode_mov_ri(Ax, 0x32);
        opcode_write(Ax, UART1);
        opcode_mov_ri(Bx, 0x00);
        opcode_jmp_r(Bx);
        */

        // First words in memory must jmp to main() function
        {
                // Make sure it exists
                assert(get_g_module()->entry);
                // Create the jmp
                init_jmp = opcode_mov_ri(Bx, 0);
                init_jmp.asm_flags |= ASM_JMP_JMP;
                init_jmp.ast = get_g_module()->entry;
                init_jmp.comment = "ENTRY JMP MAIN";
                asm_push(init_jmp);
                asm_push(opcode_jmp_r(Bx));
        }

        // Now emit global variables
        {
                struct list_item        *string_it;
                struct ast_cstring      *string;
                char                    *char_it;
                int                     first_it = 0;
                string_it = get_g_module()->strings;

                list_for_each(string_it) {
                        first_it = 0;
                        // We'll use ASCII encoding as its easy
                        // and widely used
                        string = (struct ast_cstring*)string_it->value;
                        char_it = string->string;
                        // Output a 16-bit word for each ASCII value
                        // (PRCO304 processor does not support byte indexing)
                        while(*(char_it)) {
                                asm_push(opcode_byte(*char_it));
                                if(first_it == 0) {
                                        // tag the first character with it's GUID
                                        asm_tag_last(string->string_id);
                                }

                                char_it++;
                                first_it++;
                        }
                        // Output null terminator
                        asm_push(opcode_byte(0));
                }
        }

        dbprintf(D_GEN, "\r\n\r\n");
}

void
cg_postcode_vmicro16(void)
{
        int it;
        struct prco_op_struct *op;

        dbprintf(D_INFO, "\r\n\r\nPostcode:\r\n");

        // Print each instruction in human readable format
        for_each_asm(it, op) {
                dbprintf(D_GEN, "0x%02X\t", op->asm_offset);
                assert_opcode(op, 1);
        }

        dbprintf(D_GEN, "\r\n\r\n");
        assembler_labels();

        // Final pass
        // Check each label address is OK
        for_each_asm(it, op) {
                if(op->asm_flags & ASM_JMP_JMP) {
                        if(!op->imm8) {
                                dbprintf(D_GEN,
                                        "GEN: ERR: [0x%02x] Jump address error\r\n",
                                        it);
                                assert(op->imm8);
                        }
                }
        }

        // Print each instruction in human readable format
        for_each_asm(it, op) {
                dbprintf(D_GEN, "0x%02X\t", op->asm_offset);
                assert_opcode(op, 1);
        }

        // Debug
        // Inline verilog prco_lmem.v memory
        for_each_asm(it, op) {
                dbprintf(D_GEN, "r_lmem[%d] = 16'h%04x;\n", it, op->opcode);
        }


        // Write machine code to file
        create_verilog_memh_file();
}

void
cg_push_prco(enum prco_reg rd)
{
        asm_push(opcode_add_ri(Sp, -1));
        asm_push(opcode_sw(rd, Sp, 0));
        asm_comment("PUSH");
}

void
cg_pop_prco(enum prco_reg rd)
{
        asm_push(opcode_lw(rd, Sp, 0));
        asm_comment("POP");
        asm_push(opcode_add_ri(Sp, 1));
}

void
cg_expr_vmicro16(struct ast_item *e)
{
        list_for_each(e) {
                switch (e->type) {
                case AST_NUM:
                        cg_number_vmicro16(e->expr);
                        break;
                case AST_CSTRING:
                        cg_cstring_ref(e->expr);
                        break;
                case AST_DEREF:
                        cg_deref_vmicro16(e->expr);
                        break;
                case AST_BIN:
                        cg_bin_vmicro16(e->expr);
                        break;
                case AST_CALL:
                        cg_call_vmicro16(e->expr);
                        break;

                case AST_IF:
                        cg_if_vmicro16(e->expr);
                        break;
                case AST_FOR:
                        cg_for_vmicro16(e->expr);
                        break;
                case AST_WHILE:
                        cg_while_vmicro16(e->expr);
                        break;

                case AST_LOCAL_VAR:
                        cg_local_decl_vmicro16(e->expr);
                        break;
                case AST_VAR_REF:
                        cg_var_ref_vmicro16(e->expr);
                        break;
                case AST_ASSIGNMENT:
                        cg_assignment_vmicro16(e->expr);
                        break;

                case AST_UART:
                        cg_port_uart_vmicro16(e->expr);
                        break;
                default:
                        dbprintf(D_ERR, "Unknown cg routine for %d\r\n",
                                e->type);
                        assert("Unknown cg routine for %d\r\n" && 0);
                }
        }
}

inline void
cg_sf_start(struct ast_func *f)
{
        struct prco_op_struct op;

        dbprintf(D_GEN, "SF START for %s\r\n", f->proto->name);

        op = opcode_add_ri(Sp, -1);
        op.ast = f;
        op.asm_flags |= ASM_FUNC_START;
        op.comment = "Function/sf entry";
        asm_push(op);

        asm_push(opcode_sw(Bp, Sp, 0));
        // Mov Sp -> Bp
        asm_push(opcode_mov_rr(Bp, Sp));
        asm_comment(f->proto->name);

        //("push %%bp\r\n");
        //eprintf("mov %%bp, %%sp\r\n");
}

void
cg_sf_exit(void)
{
        // Mov Bp -> Sp
        asm_push(opcode_mov_rr(Sp, Bp));
        asm_comment("Function/sf exit");
        // Pop Bp
        cg_pop_prco(Bp);
}

void
cg_for_vmicro16(struct ast_for *a)
{
        struct prco_op_struct jmp_cond;
        struct prco_op_struct cond_dest;
        struct prco_op_struct jmp_after;
        struct prco_op_struct for_after_dest;

        int cond_dest_id = NEW_ASM_ID();
        int for_after_id = NEW_ASM_ID();

        cg_expr_vmicro16(a->start);

        cond_dest = opcode_nop();
        cond_dest.asm_flags |= ASM_JMP_DEST;
        cond_dest.comment = malloc(32);
        snprintf(cond_dest.comment, 32, "FOR COND DEST %d", cond_dest.id);
        cond_dest.id = cond_dest_id;
        asm_push(cond_dest);

        cg_expr_vmicro16(a->cond);
        asm_comment("FOR CONDITION CG");
        asm_push(opcode_mov_ri(Bx, 0));
        asm_push(opcode_cmp_rr(Ax, Bx));

        jmp_after = opcode_mov_ri(Bx, 0);
        jmp_after.asm_flags |= ASM_JMP_JMP;
        jmp_after.id = for_after_id;
        jmp_after.comment = malloc(32);
        snprintf(jmp_after.comment, 32, "JMP AFTER %d", jmp_after.id);
        asm_push(jmp_after);
        asm_push(opcode_jmp_rc(Bx, JMP_JE));

        cg_expr_vmicro16(a->body);
        asm_comment("BODY END");

        cg_expr_vmicro16(a->step);
        jmp_cond = opcode_mov_ri(Bx, 0);
        jmp_cond.asm_flags |= ASM_JMP_JMP;
        jmp_cond.id = cond_dest_id;
        jmp_cond.comment = malloc(32);
        snprintf(jmp_cond.comment, 32, "FOR COND BACK %d", jmp_cond.id);
        asm_push(jmp_cond);
        asm_push(opcode_jmp_r(Bx));

        for_after_dest = opcode_nop();
        for_after_dest.asm_flags |= ASM_JMP_DEST;
        for_after_dest.id = for_after_id;
        for_after_dest.comment = "FOR LOOP AFTER";
        asm_push(for_after_dest);
}

void cg_while_vmicro16(struct ast_while *v)
{
        struct prco_op_struct cond_start;
        struct prco_op_struct while_jmp_to_cond;
        struct prco_op_struct while_jmp_to_after;
        struct prco_op_struct while_after;

        int guid_cond_start  = NEW_ASM_ID();
        int guid_while_after = NEW_ASM_ID();

        cond_start = opcode_nop();
        cond_start.id = guid_cond_start;
        cond_start.asm_flags |= ASM_JMP_DEST;
        cond_start.comment = "WHILE COND DEST";
        asm_push(cond_start);

        // cg for condition
        cg_expr_vmicro16(v->cond);

        // CMP the condition
        asm_push(opcode_mov_ri(Bx, 0));
        asm_push(opcode_cmp_rr(Ax, Bx));

        // Jmp to after if false
        while_jmp_to_after = opcode_mov_ri(Bx, 0x00);
        while_jmp_to_after.id = guid_while_after;
        while_jmp_to_after.asm_flags |= ASM_JMP_JMP;
        while_jmp_to_after.comment = "WHILE AFTER JMP";
        asm_push(while_jmp_to_after);
        asm_push(opcode_jmp_rc(Bx, JMP_JE));

        // cg the while loop body
        cg_expr_vmicro16(v->body);

        // Jump back to condition check
        while_jmp_to_cond = opcode_mov_ri(Bx, 0x00);
        while_jmp_to_cond.id = guid_cond_start;
        while_jmp_to_cond.comment = "WHILE COND JMP";
        while_jmp_to_cond.asm_flags |= ASM_JMP_JMP;
        asm_push(while_jmp_to_cond);
        asm_push(opcode_jmp_r(Bx));

        // Emit the jump after destination
        while_after = opcode_nop();
        while_after.asm_flags |= ASM_JMP_DEST;
        while_after.id = guid_while_after;
        while_after.comment = "WHILE AFTER DEST";
        asm_push(while_after);
}

void
cg_assignment_vmicro16(struct ast_assign *a)
{
        dbprintf(D_GEN, "cg_assignment_vmicro16 %s\r\n",
                a->var->var->name);

        // codegen the value
        cg_expr_vmicro16(a->val);

        // Value now in Ax register,
        // store it in stack location
        asm_push(opcode_sw(Ax, Bp, a->var->bp_offset));
        asm_comment(a->var->var->name);
}

void
cg_var_ref_vmicro16(struct ast_lvar *v)
{
        asm_push(opcode_lw(Ax, Bp, v->bp_offset));
        asm_comment(v->var->name);
}

void
cg_local_decl_vmicro16(struct ast_lvar *v)
{
        struct list_item *item_it;
        struct ast_lvar *sv;
        struct prco_op_struct op_stack_alloc;
        int offset = -1;

        dbprintf(D_INFO, "cg_local_decl_vmicro16\r\n");

        item_it = cg_cur_function->locals;
        list_for_each(item_it) {
                sv = item_it->value;
                sv->bp_offset = offset;

                if (strcmp(v->var->name, sv->var->name) == 0) {
                        dbprintf(D_INFO, "Found var: %s %+d\r\n",
                                sv->var->name,
                                sv->bp_offset);
                        break;
                }

                offset -= 1;
        }

        op_stack_alloc = opcode_sub_ri(Sp, 1);
        op_stack_alloc.comment = malloc(32);
        snprintf(op_stack_alloc.comment, 32, "VAR ALLOC %s %d",
                 v->var->name,
                 v->bp_offset);
        asm_push(op_stack_alloc);
}

void
cg_call_vmicro16(struct ast_call *c)
{
        struct prco_op_struct   op_next,
                                op_call;

        struct list_item *args;
        struct ast_item *arg_item;
        struct ast_item *args_reversed[MAX_CALL_ARGS] = {0};
        int reversed_i = MAX_CALL_ARGS-1;

        // We don't need to reverse the argument pushing as the
        // parameter variable's bp_offsets have already been reversed
        // So just cg the parameter, and push it in order.
        //
        // TODO: Push in reverse order to support stdcall and cdecl standards
        args = c->args;
        list_for_each(args) {
                cg_expr_vmicro16(args->value);
                cg_push_prco(Ax);
                asm_comment("PUSH ARG");
        }

        // Create return address after called function returns
        op_next = opcode_mov_ri(Cx, 0x00);
        op_next.asm_flags = ASM_CALL_NEXT;
        op_next.comment = "CALL AFTER";
        asm_push(op_next);

        // Push return address
        cg_push_prco(Cx);

        // Now, we jump to the function
        op_call = opcode_mov_ri(Cx, 0x00);
        op_call.asm_flags = ASM_FUNC_CALL;
        op_call.ast = c;
        op_call.comment = "call";

        asm_push(op_call);
        asm_push(opcode_jmp_r(Cx));
        asm_comment("JMP TO FUNC");

        // ASM_CALL_NEXT points here
        // After function, pop arguments from stack
        asm_push(opcode_add_ri(Sp, c->argc));
}

void
cg_if_vmicro16(struct ast_if *v)
{
        struct prco_op_struct op_movi;
        struct prco_op_struct op_else_dest;
        struct prco_op_struct op_true_jmp;
        struct prco_op_struct op_after;

        unsigned int jmp_else  = NEW_ASM_ID();
        unsigned int jmp_after = NEW_ASM_ID();

        // Emit condition
        cg_expr_vmicro16(v->cond);

        // Emit comparison
        asm_push(opcode_mov_ri(Cx, 0));
        asm_push(opcode_cmp_rr(Ax, Cx));

        // Create jmp location
        op_movi = opcode_mov_ri(Bx, 0x00);
        op_movi.asm_flags |= ASM_JMP_JMP;
        op_movi.comment = malloc(32);

        if (v->els) {
                op_movi.id = jmp_else;
                snprintf(op_movi.comment, 32, "JMP ELSE %x", op_movi.id);
        } else {
                op_movi.id = jmp_after;
                snprintf(op_movi.comment, 32, "JMP AFTER %x", op_movi.id);
        }


        asm_push(op_movi);
        asm_push(opcode_jmp_rc(Bx, JMP_JE));

        // If true
        cg_expr_vmicro16(v->then);

        if (v->els) {
                op_true_jmp = opcode_mov_ri(Bx, 0x00);
                op_true_jmp.asm_flags |= ASM_JMP_JMP;
                op_true_jmp.id = jmp_after;
                op_true_jmp.comment = malloc(32);
                snprintf(op_true_jmp.comment, 32, "JMP AFTER %x", op_true_jmp.id);

                asm_push(op_true_jmp);
                asm_push(opcode_jmp_rc(Bx, JMP_UC));

                op_else_dest = opcode_nop();
                op_else_dest.asm_flags |= ASM_JMP_DEST;
                op_else_dest.id = jmp_else;
                op_else_dest.comment = malloc(32);
                snprintf(op_else_dest.comment, 32, "JMP ELSE DEST %x", op_else_dest.id);

                asm_push(op_else_dest);

                // Emit else code
                cg_expr_vmicro16(v->els);
        }

        op_after = opcode_nop();
        op_after.asm_flags |= ASM_JMP_DEST;
        op_after.id = jmp_after;
        op_after.comment = malloc(32);
        snprintf(op_after.comment, 32, "JMP AFTER DEST %x", op_after.id);
        asm_push(op_after);
}

void
cg_function_vmicro16(struct ast_func *f)
{
        dbprintf(D_GEN, "Starting cg for function: %s %d\r\n",
                f->proto->name, f->num_local_vars);

        struct list_item *arg_it = f->proto->args;
        list_for_each(arg_it) {
                struct ast_lvar *var = arg_it->value;
                if(!var) continue;

                dbprintf(D_GEN, "PROTO ARGS: %s %d\r\n",
                        var->var->name, var->bp_offset);
        }

        arg_it = f->locals;
        list_for_each(arg_it) {
                struct ast_lvar *var = arg_it->value;
                if(!var) continue;

                dbprintf(D_GEN, "FUNC LOCALS: %s %d\r\n",
                        var->var->name, var->bp_offset);
        }
        assert(f);
        assert(f->proto);
        assert(f->body);

        // Set current function
        cg_cur_function = f;

        // Create stack frame
        cg_sf_start(f);

        // cg the function body
        cg_expr_vmicro16(f->body);

        // Create the stack exit
        // if required
        cg_sf_exit();

        // TODO: Decide if function is cdecl or stdcall
        // Then clean up stack as per calling convention
        // cg_cc_cleanup(f);

        // clean up
        cg_cur_function = NULL;

        dbprintf(D_GEN, "End function");

        if (is_entry_func(f->proto)) {
                asm_push(opcode_t1(HALT, 0, 0, 0));
                asm_comment("MAIN HALT\r\n------------------------------------");
        } else {
                cg_pop_prco(Cx);
                asm_push(opcode_jmp_r(Cx));
                asm_comment(
                        "FUNC RETURN to CALL\r\n------------------------------");
        }
        dbprintf(D_GEN, "\r\n");
}

void
cg_bin_vmicro16(struct ast_bin *b)
{
        dbprintf(D_GEN, "Starting cg for bin\r\n");

        cg_expr_vmicro16(b->lhs);

        if (b->rhs) {
                cg_push_prco(Ax);
                cg_expr_vmicro16(b->rhs);
        }

        switch (b->op) {
        case TOK_PLUS:
                cg_pop_prco(Cx);
                asm_push(opcode_add_rr(Ax, Cx));
                asm_comment("BIN ADD");
                break;

        case TOK_SUB:
                cg_pop_prco(Cx);
                asm_push(opcode_sub_rr(Ax, Cx));
                asm_comment("BIN SUB");
                break;

        case TOK_STAR:
                eprintf("POP %%cx\r\n");
                break;

        case TOK_BOOL_L:
                cg_pop_prco(Cx);
                asm_push(opcode_cmp_rr(Cx, Ax));
                asm_push(opcode_set_ri(Ax, JMP_JL));
                break;
        case TOK_BOOL_LE:
                cg_pop_prco(Cx);
                asm_push(opcode_cmp_rr(Cx, Ax));
                asm_push(opcode_set_ri(Ax, JMP_JLE));
                break;
        case TOK_BOOL_G:
                cg_pop_prco(Cx);
                asm_push(opcode_cmp_rr(Cx, Ax));
                asm_push(opcode_set_ri(Ax, JMP_JG));
                break;
        case TOK_BOOL_GE:
                cg_pop_prco(Cx);
                asm_push(opcode_cmp_rr(Cx, Ax));
                asm_push(opcode_set_ri(Ax, JMP_JGE));
                break;
        case TOK_BOOL_EQ:
                cg_pop_prco(Cx);
                asm_push(opcode_cmp_rr(Cx, Ax));
                asm_push(opcode_set_ri(Ax, JMP_JE));
                break;

        default:
                dbprintf(D_ERR, "Unimplemented cg_bin_vmicro16 b->op %d\r\n",
                        b->op);
                assert("Unimplemented cg_bin_vmicro16 b->op!" && 0);
                break;
        }
}

void
cg_number_vmicro16(struct ast_num *n)
{
        assert(n);

        if(n->val > 0xff) {
                // Can't fit more than 8-bits into a MOVI,
                // TODO: Build up 16 bit word using:
                //   MOVI, LSHIFT, MOVI, OR
                asm_push(opcode_mov_ri(Ax, n->val));
        } else {
                // Use MOVI
                asm_push(opcode_mov_ri(Ax, n->val));
        }

        asm_comment("NUMBER");
}


void cg_port_uart_vmicro16(struct ast_expr *v)
{
        cg_expr_vmicro16(v->val);
        asm_push(opcode_write(Ax, UART1));
}

void
cg_cstring_ref(struct ast_cstring *v)
{
        struct prco_op_struct mov;

        dbprintf(D_GEN, "CSTRING REF: %d %s\r\n",
                v->string_id, v->string);

        // 1. Move address of cstring into Ax
        // 2. LW address of Ax

        // 1.
        mov = opcode_mov_ri(Ax, 0x00);
        mov.asm_flags = ASM_POINTER;
        mov.id = v->string_id;
        mov.comment = "POINTER";
        asm_push(mov);

        // 2. LW address of Ax
        //asm_push(opcode_lw(Ax, Bx, 0));
}


void
cg_deref_vmicro16(struct ast_deref *v)
{
        // A dereference is just LW of Ax register
        cg_expr_vmicro16(v->item);

        // Ax <- RAM[Ax]
        asm_push(opcode_lw(Ax, Ax, 0));
        asm_comment("DEREF");
}
