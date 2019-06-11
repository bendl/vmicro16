//
// Created by BDL on 03/03/2018.
//

#ifndef VMICRO16_CG_IMPL_H
#define VMICRO16_CG_IMPL_H

#include "arch/target.h"

void cg_target_vmicro16_init(struct target_delegate *dt);

extern struct ast_func *cg_cur_function;

extern void cg_expr_vmicro16            (struct ast_item *e);
extern void cg_function_vmicro16        (struct ast_func *f);
extern void cg_bin_vmicro16             (struct ast_bin *b);
extern void cg_number_vmicro16          (struct ast_num *n);
extern void cg_local_decl_vmicro16      (struct ast_lvar *v);
extern void cg_call_vmicro16            (struct ast_call *c);
extern void cg_precode_vmicro16         (void);
extern void cg_postcode_vmicro16        (void);
extern void cg_if_vmicro16              (struct ast_if *v);

extern void cg_var_vmicro16             (struct ast_var *v);

extern void cg_assignment_vmicro16      (struct ast_assign *a);
extern void cg_var_ref_vmicro16         (struct ast_lvar *v);
extern void cg_local_decl_vmicro16      (struct ast_lvar *v);

extern void cg_if_vmicro16              (struct ast_if *i);
extern void cg_for_vmicro16             (struct ast_for *f);
extern void cg_while_vmicro16             (struct ast_while *v);

extern void cg_port_uart_vmicro16       (struct ast_expr *v);

extern void vm16_cg_cstring_ref              (struct ast_cstring *v);
extern void cg_deref_vmicro16           (struct ast_deref *v);

/*
extern void cg_call_vmicro16(struct ast_call *v);
extern void cg_dir_extern_vmicro16(ast_proto_t *p);
*/
extern int get_dt_size_vmicro16         (target_datatype dt);

#endif //VMICRO16_CG_IMPL_H
