/*
 *  m68k translation
 *
 *  Copyright (c) 2005-2007 CodeSourcery
 *  Written by Paul Brook
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "config.h"
#include "cpu.h"
#include "disas.h"
#include "tcg-op.h"
#include "qemu-log.h"

#include "helpers.h"
#define GEN_HELPER 1
#include "helpers.h"

//#define DEBUG_DISPATCH 1

/* Fake floating point.  */
#define tcg_gen_mov_f64 tcg_gen_mov_i64
#define tcg_gen_qemu_ldf64 tcg_gen_qemu_ld64
#define tcg_gen_qemu_stf64 tcg_gen_qemu_st64

#define DEFO32(name, offset) static TCGv QREG_##name;
#define DEFO64(name, offset) static TCGv_i64 QREG_##name;
#define DEFF64(name, offset) static TCGv_i64 QREG_##name;
#include "qregs.def"
#undef DEFO32
#undef DEFO64
#undef DEFF64

static TCGv_ptr cpu_env;

static char cpu_reg_names[3*8*3 + 5*4];
static TCGv cpu_dregs[8];
static TCGv cpu_aregs[8];
static TCGv_i64 cpu_fregs[8];
static TCGv_i64 cpu_macc[4];

#define DREG(insn, pos) cpu_dregs[((insn) >> (pos)) & 7]
#define AREG(insn, pos) cpu_aregs[((insn) >> (pos)) & 7]
#define FREG(insn, pos) cpu_fregs[((insn) >> (pos)) & 7]
#define MACREG(acc) cpu_macc[acc]
#define QREG_SP cpu_aregs[7]

static TCGv NULL_QREG;
#define IS_NULL_QREG(t) (TCGV_EQUAL(t, NULL_QREG))
/* Used to distinguish stores from bad addressing modes.  */
static TCGv store_dummy;

#include "gen-icount.h"

void m68k_tcg_init(void)
{
    char *p;
    int i;

#define DEFO32(name,  offset) QREG_##name = tcg_global_mem_new_i32(TCG_AREG0, offsetof(CPUState, offset), #name);
#define DEFO64(name,  offset) QREG_##name = tcg_global_mem_new_i64(TCG_AREG0, offsetof(CPUState, offset), #name);
#define DEFF64(name,  offset) DEFO64(name, offset)
#include "qregs.def"
#undef DEFO32
#undef DEFO64
#undef DEFF64

    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");

    p = cpu_reg_names;
    for (i = 0; i < 8; i++) {
        sprintf(p, "D%d", i);
        cpu_dregs[i] = tcg_global_mem_new(TCG_AREG0,
                                          offsetof(CPUM68KState, dregs[i]), p);
        p += 3;
        sprintf(p, "A%d", i);
        cpu_aregs[i] = tcg_global_mem_new(TCG_AREG0,
                                          offsetof(CPUM68KState, aregs[i]), p);
        p += 3;
        sprintf(p, "F%d", i);
        cpu_fregs[i] = tcg_global_mem_new_i64(TCG_AREG0,
                                          offsetof(CPUM68KState, fregs[i]), p);
        p += 3;
    }
    for (i = 0; i < 4; i++) {
        sprintf(p, "ACC%d", i);
        cpu_macc[i] = tcg_global_mem_new_i64(TCG_AREG0,
                                         offsetof(CPUM68KState, macc[i]), p);
        p += 5;
    }

    NULL_QREG = tcg_global_mem_new(TCG_AREG0, -4, "NULL");
    store_dummy = tcg_global_mem_new(TCG_AREG0, -8, "NULL");

#define GEN_HELPER 2
#include "helpers.h"
}

static inline void qemu_assert(int cond, const char *msg)
{
    if (!cond) {
        fprintf (stderr, "badness: %s\n", msg);
        abort();
    }
}

/* internal defines */
typedef struct DisasContext {
    CPUM68KState *env;
    target_ulong insn_pc; /* Start of the current instruction.  */
    target_ulong pc;
    int is_jmp;
    int cc_op;
    int user;
    uint32_t fpcr;
    struct TranslationBlock *tb;
    int singlestep_enabled;
    int is_mem;
    TCGv_i64 mactmp;
    int done_mac;
} DisasContext;

#define DISAS_JUMP_NEXT 4

#if defined(CONFIG_USER_ONLY)
#define IS_USER(s) 1
#else
#define IS_USER(s) s->user
#endif

/* XXX: move that elsewhere */
/* ??? Fix exceptions.  */
static void *gen_throws_exception;
#define gen_last_qop NULL

#define OS_BYTE 0
#define OS_WORD 1
#define OS_LONG 2
#define OS_SINGLE 4
#define OS_DOUBLE 5

typedef void (*disas_proc)(DisasContext *, uint16_t);

#ifdef DEBUG_DISPATCH
#define DISAS_INSN(name) \
  static void real_disas_##name (DisasContext *s, uint16_t insn); \
  static void disas_##name (DisasContext *s, uint16_t insn) { \
    qemu_log("Dispatch " #name "\n"); \
    real_disas_##name(s, insn); } \
  static void real_disas_##name (DisasContext *s, uint16_t insn)
#else
#define DISAS_INSN(name) \
  static void disas_##name (DisasContext *s, uint16_t insn)
#endif

/* Generate a load from the specified address.  Narrow values are
   sign extended to full register width.  */
static inline TCGv gen_load(DisasContext * s, int opsize, TCGv addr, int sign)
{
    TCGv tmp;
    int index = IS_USER(s);
    s->is_mem = 1;
    tmp = tcg_temp_new_i32();
    switch(opsize) {
    case OS_BYTE:
        if (sign)
            tcg_gen_qemu_ld8s(tmp, addr, index);
        else
            tcg_gen_qemu_ld8u(tmp, addr, index);
        break;
    case OS_WORD:
        if (sign)
            tcg_gen_qemu_ld16s(tmp, addr, index);
        else
            tcg_gen_qemu_ld16u(tmp, addr, index);
        break;
    case OS_LONG:
    case OS_SINGLE:
        tcg_gen_qemu_ld32u(tmp, addr, index);
        break;
    default:
        qemu_assert(0, "bad load size");
    }
    gen_throws_exception = gen_last_qop;
    return tmp;
}

static inline TCGv_i64 gen_load64(DisasContext * s, TCGv addr)
{
    TCGv_i64 tmp;
    int index = IS_USER(s);
    s->is_mem = 1;
    tmp = tcg_temp_new_i64();
    tcg_gen_qemu_ldf64(tmp, addr, index);
    gen_throws_exception = gen_last_qop;
    return tmp;
}

/* Generate a store.  */
static inline void gen_store(DisasContext *s, int opsize, TCGv addr, TCGv val)
{
    int index = IS_USER(s);
    s->is_mem = 1;
    switch(opsize) {
    case OS_BYTE:
        tcg_gen_qemu_st8(val, addr, index);
        break;
    case OS_WORD:
        tcg_gen_qemu_st16(val, addr, index);
        break;
    case OS_LONG:
    case OS_SINGLE:
        tcg_gen_qemu_st32(val, addr, index);
        break;
    default:
        qemu_assert(0, "bad store size");
    }
    gen_throws_exception = gen_last_qop;
}

static inline void gen_store64(DisasContext *s, TCGv addr, TCGv_i64 val)
{
    int index = IS_USER(s);
    s->is_mem = 1;
    tcg_gen_qemu_stf64(val, addr, index);
    gen_throws_exception = gen_last_qop;
}

typedef enum {
    EA_STORE,
    EA_LOADU,
    EA_LOADS
} ea_what;

/* Generate an unsigned load if VAL is 0 a signed load if val is -1,
   otherwise generate a store.  */
static TCGv gen_ldst(DisasContext *s, int opsize, TCGv addr, TCGv val,
                     ea_what what)
{
    if (what == EA_STORE) {
        gen_store(s, opsize, addr, val);
        return store_dummy;
    } else {
        return gen_load(s, opsize, addr, what == EA_LOADS);
    }
}

/* Read an 8-bit immediate constant */
static inline uint32_t read_im8(DisasContext *s)
{
    uint32_t im;
    im = ldsb_code(s->pc + 1);
    s->pc += 2;
    return im;
}
/* Read a 16-bit immediate constant */
static inline uint32_t read_im16(DisasContext *s)
{
    uint32_t im;
    im = ldsw_code(s->pc);
    s->pc += 2;
    return im;
}
/* Read a 32-bit immediate constant.  */
static inline uint32_t read_im32(DisasContext *s)
{
    uint32_t im;
    im = ((uint32_t)lduw_code(s->pc)) << 16;
    s->pc += 2;
    im |= lduw_code(s->pc);
    s->pc += 2;
    return im;
}

/* Calculate and address index.  */
static TCGv gen_addr_index(uint16_t ext, TCGv tmp)
{
    TCGv add;
    int scale;

    add = (ext & 0x8000) ? AREG(ext, 12) : DREG(ext, 12);
    if ((ext & 0x800) == 0) {
        tcg_gen_ext16s_i32(tmp, add);
        add = tmp;
    }
    scale = (ext >> 9) & 3;
    if (scale != 0) {
        tcg_gen_shli_i32(tmp, add, scale);
        add = tmp;
    }
    return add;
}

/* Handle a base + index + displacement effective addresss.
   A NULL_QREG base means pc-relative.  */
static TCGv gen_lea_indexed(DisasContext *s, int opsize, TCGv base)
{
    uint32_t offset;
    uint16_t ext;
    TCGv add;
    TCGv tmp;
    uint32_t bd, od;

    offset = s->pc;
    ext = lduw_code(s->pc);
    s->pc += 2;

    if ((ext & 0x800) == 0 && !m68k_feature(s->env, M68K_FEATURE_WORD_INDEX))
        return NULL_QREG;

    if (!m68k_feature(s->env, M68K_FEATURE_SCALED_INDEX))
        ext &= ~(3 << 9);

    if (ext & 0x100) {
        /* full extension word format */
        if (!m68k_feature(s->env, M68K_FEATURE_EXT_FULL))
            return NULL_QREG;

        if ((ext & 0x30) > 0x10) {
            /* base displacement */
            if ((ext & 0x30) == 0x20) {
                bd = ldsw_code(s->pc);
                s->pc += 2;
            } else {
                bd = read_im32(s);
            }
        } else {
            bd = 0;
        }
        tmp = tcg_temp_new();
        if ((ext & 0x44) == 0) {
            /* pre-index */
            add = gen_addr_index(ext, tmp);
        } else {
            add = NULL_QREG;
        }
        if ((ext & 0x80) == 0) {
            /* base not suppressed */
            if (IS_NULL_QREG(base)) {
                base = tcg_const_i32(offset + bd);
                bd = 0;
            }
            if (!IS_NULL_QREG(add)) {
                tcg_gen_add_i32(tmp, add, base);
                add = tmp;
            } else {
                add = base;
            }
        }
        if (!IS_NULL_QREG(add)) {
            if (bd != 0) {
                tcg_gen_addi_i32(tmp, add, bd);
                add = tmp;
            }
        } else {
            add = tcg_const_i32(bd);
        }
        if ((ext & 3) != 0) {
            /* memory indirect */
            base = gen_load(s, OS_LONG, add, 0);
            if ((ext & 0x44) == 4) {
                add = gen_addr_index(ext, tmp);
                tcg_gen_add_i32(tmp, add, base);
                add = tmp;
            } else {
                add = base;
            }
            if ((ext & 3) > 1) {
                /* outer displacement */
                if ((ext & 3) == 2) {
                    od = ldsw_code(s->pc);
                    s->pc += 2;
                } else {
                    od = read_im32(s);
                }
            } else {
                od = 0;
            }
            if (od != 0) {
                tcg_gen_addi_i32(tmp, add, od);
                add = tmp;
            }
        }
    } else {
        /* brief extension word format */
        tmp = tcg_temp_new();
        add = gen_addr_index(ext, tmp);
        if (!IS_NULL_QREG(base)) {
            tcg_gen_add_i32(tmp, add, base);
            if ((int8_t)ext)
                tcg_gen_addi_i32(tmp, tmp, (int8_t)ext);
        } else {
            tcg_gen_addi_i32(tmp, add, offset + (int8_t)ext);
        }
        add = tmp;
    }
    return add;
}

/* Update the CPU env CC_OP state.  */
static inline void gen_flush_cc_op(DisasContext *s)
{
    if (s->cc_op != CC_OP_DYNAMIC)
        tcg_gen_movi_i32(QREG_CC_OP, s->cc_op);
}

/* Evaluate all the CC flags.  */
static inline void gen_flush_flags(DisasContext *s)
{
    if (s->cc_op == CC_OP_FLAGS)
        return;
    gen_flush_cc_op(s);
    gen_helper_flush_flags(cpu_env, QREG_CC_OP);
    s->cc_op = CC_OP_FLAGS;
}

#define SET_CC_OP(opsize, op) do { \
    switch (opsize) { \
    case OS_BYTE: s->cc_op = CC_OP_##op##B; break; \
    case OS_WORD: s->cc_op = CC_OP_##op##W; break; \
    case OS_LONG: s->cc_op = CC_OP_##op; break; \
    default: abort(); \
    } \
} while (0)

#define SET_X_FLAG(opsize, a, b) do { \
    switch (opsize) { \
    case OS_BYTE: gen_helper_xflag_lt_i8(QREG_CC_X, a, b); break; \
    case OS_WORD: gen_helper_xflag_lt_i16(QREG_CC_X, a, b); break; \
    case OS_LONG: gen_helper_xflag_lt_i32(QREG_CC_X, a, b); break; \
    default: abort(); \
    } \
} while (0)

static void gen_logic_cc(DisasContext *s, TCGv val, int opsize)
{
    tcg_gen_mov_i32(QREG_CC_DEST, val);
    SET_CC_OP(opsize, LOGIC);
}

static void gen_update_cc_add(TCGv dest, TCGv src)
{
    tcg_gen_mov_i32(QREG_CC_DEST, dest);
    tcg_gen_mov_i32(QREG_CC_SRC, src);
}

static inline int opsize_bytes(int opsize)
{
    switch (opsize) {
    case OS_BYTE: return 1;
    case OS_WORD: return 2;
    case OS_LONG: return 4;
    case OS_SINGLE: return 4;
    case OS_DOUBLE: return 8;
    default:
        qemu_assert(0, "bad operand size");
        return 0;
    }
}

static inline int insn_opsize(int insn, int pos)
{
    switch ((insn >> pos) & 3) {
    case 0: return OS_BYTE;
    case 1: return OS_WORD;
    case 2: return OS_LONG;
    default: abort();
    }
}

/* Assign value to a register.  If the width is less than the register width
   only the low part of the register is set.  */
static void gen_partset_reg(int opsize, TCGv reg, TCGv val)
{
    TCGv tmp;
    switch (opsize) {
    case OS_BYTE:
        tcg_gen_andi_i32(reg, reg, 0xffffff00);
        tmp = tcg_temp_new();
        tcg_gen_ext8u_i32(tmp, val);
        tcg_gen_or_i32(reg, reg, tmp);
        break;
    case OS_WORD:
        tcg_gen_andi_i32(reg, reg, 0xffff0000);
        tmp = tcg_temp_new();
        tcg_gen_ext16u_i32(tmp, val);
        tcg_gen_or_i32(reg, reg, tmp);
        break;
    case OS_LONG:
    case OS_SINGLE:
        tcg_gen_mov_i32(reg, val);
        break;
    default:
        qemu_assert(0, "Bad operand size");
        break;
    }
}

/* Sign or zero extend a value.  */
static inline TCGv gen_extend(TCGv val, int opsize, int sign)
{
    TCGv tmp;

    switch (opsize) {
    case OS_BYTE:
        tmp = tcg_temp_new();
        if (sign)
            tcg_gen_ext8s_i32(tmp, val);
        else
            tcg_gen_ext8u_i32(tmp, val);
        break;
    case OS_WORD:
        tmp = tcg_temp_new();
        if (sign)
            tcg_gen_ext16s_i32(tmp, val);
        else
            tcg_gen_ext16u_i32(tmp, val);
        break;
    case OS_LONG:
    case OS_SINGLE:
        tmp = val;
        break;
    default:
        qemu_assert(0, "Bad operand size");
    }
    return tmp;
}

/* Generate code for an "effective address".  Does not adjust the base
   register for autoincrement addressing modes.  */
static TCGv gen_lea(DisasContext *s, uint16_t insn, int opsize)
{
    TCGv reg;
    TCGv tmp;
    uint16_t ext;
    uint32_t offset;

    switch ((insn >> 3) & 7) {
    case 0: /* Data register direct.  */
    case 1: /* Address register direct.  */
        return NULL_QREG;
    case 2: /* Indirect register */
    case 3: /* Indirect postincrement.  */
        return AREG(insn, 0);
    case 4: /* Indirect predecrememnt.  */
        reg = AREG(insn, 0);
        tmp = tcg_temp_new();
        tcg_gen_subi_i32(tmp, reg, opsize_bytes(opsize));
        return tmp;
    case 5: /* Indirect displacement.  */
        reg = AREG(insn, 0);
        tmp = tcg_temp_new();
        ext = read_im16(s);
        tcg_gen_addi_i32(tmp, reg, (int16_t)ext);
        return tmp;
    case 6: /* Indirect index + displacement.  */
        reg = AREG(insn, 0);
        return gen_lea_indexed(s, opsize, reg);
    case 7: /* Other */
        switch (insn & 7) {
        case 0: /* Absolute short.  */
            offset = ldsw_code(s->pc);
            s->pc += 2;
            return tcg_const_i32(offset);
        case 1: /* Absolute long.  */
            offset = read_im32(s);
            return tcg_const_i32(offset);
        case 2: /* pc displacement  */
            offset = s->pc;
            offset += ldsw_code(s->pc);
            s->pc += 2;
            return tcg_const_i32(offset);
        case 3: /* pc index+displacement.  */
            return gen_lea_indexed(s, opsize, NULL_QREG);
        case 4: /* Immediate.  */
        default:
            return NULL_QREG;
        }
    }
    /* Should never happen.  */
    return NULL_QREG;
}

/* Helper function for gen_ea. Reuse the computed address between the
   for read/write operands.  */
static inline TCGv gen_ea_once(DisasContext *s, uint16_t insn, int opsize,
                              TCGv val, TCGv *addrp, ea_what what)
{
    TCGv tmp;

    if (addrp && what == EA_STORE) {
        tmp = *addrp;
    } else {
        tmp = gen_lea(s, insn, opsize);
        if (IS_NULL_QREG(tmp))
            return tmp;
        if (addrp)
            *addrp = tmp;
    }
    return gen_ldst(s, opsize, tmp, val, what);
}

/* Generate code to load/store a value ito/from an EA.  If VAL > 0 this is
   a write otherwise it is a read (0 == sign extend, -1 == zero extend).
   ADDRP is non-null for readwrite operands.  */
static TCGv gen_ea(DisasContext *s, uint16_t insn, int opsize, TCGv val,
                   TCGv *addrp, ea_what what)
{
    TCGv reg;
    TCGv result;
    uint32_t offset;

    switch ((insn >> 3) & 7) {
    case 0: /* Data register direct.  */
        reg = DREG(insn, 0);
        if (what == EA_STORE) {
            gen_partset_reg(opsize, reg, val);
            return store_dummy;
        } else {
            return gen_extend(reg, opsize, what == EA_LOADS);
        }
    case 1: /* Address register direct.  */
        reg = AREG(insn, 0);
        if (what == EA_STORE) {
            tcg_gen_mov_i32(reg, val);
            return store_dummy;
        } else {
            return gen_extend(reg, opsize, what == EA_LOADS);
        }
    case 2: /* Indirect register */
        reg = AREG(insn, 0);
        return gen_ldst(s, opsize, reg, val, what);
    case 3: /* Indirect postincrement.  */
        reg = AREG(insn, 0);
        result = gen_ldst(s, opsize, reg, val, what);
        /* ??? This is not exception safe.  The instruction may still
           fault after this point.  */
        if (what == EA_STORE || !addrp)
            tcg_gen_addi_i32(reg, reg, opsize_bytes(opsize));
        return result;
    case 4: /* Indirect predecrememnt.  */
        {
            TCGv tmp;
            if (addrp && what == EA_STORE) {
                tmp = *addrp;
            } else {
                tmp = gen_lea(s, insn, opsize);
                if (IS_NULL_QREG(tmp))
                    return tmp;
                if (addrp)
                    *addrp = tmp;
            }
            result = gen_ldst(s, opsize, tmp, val, what);
            /* ??? This is not exception safe.  The instruction may still
               fault after this point.  */
            if (what == EA_STORE || !addrp) {
                reg = AREG(insn, 0);
                tcg_gen_mov_i32(reg, tmp);
            }
        }
        return result;
    case 5: /* Indirect displacement.  */
    case 6: /* Indirect index + displacement.  */
        return gen_ea_once(s, insn, opsize, val, addrp, what);
    case 7: /* Other */
        switch (insn & 7) {
        case 0: /* Absolute short.  */
        case 1: /* Absolute long.  */
        case 2: /* pc displacement  */
        case 3: /* pc index+displacement.  */
            return gen_ea_once(s, insn, opsize, val, addrp, what);
        case 4: /* Immediate.  */
            /* Sign extend values for consistency.  */
            switch (opsize) {
            case OS_BYTE:
                if (what == EA_LOADS)
                    offset = ldsb_code(s->pc + 1);
                else
                    offset = ldub_code(s->pc + 1);
                s->pc += 2;
                break;
            case OS_WORD:
                if (what == EA_LOADS)
                    offset = ldsw_code(s->pc);
                else
                    offset = lduw_code(s->pc);
                s->pc += 2;
                break;
            case OS_LONG:
            case OS_SINGLE:
                offset = read_im32(s);
                break;
            default:
                qemu_assert(0, "Bad immediate operand");
            }
            return tcg_const_i32(offset);
        default:
            return NULL_QREG;
        }
    }
    /* Should never happen.  */
    return NULL_QREG;
}

/* This generates a conditional branch, clobbering all temporaries.  */
static void gen_jmpcc(DisasContext *s, int cond, int l1)
{
    TCGv tmp;

    /* TODO: Optimize compare/branch pairs rather than always flushing
       flag state to CC_OP_FLAGS.  */
    gen_flush_flags(s);
    switch (cond) {
    case 0: /* T */
        tcg_gen_br(l1);
        break;
    case 1: /* F */
        break;
    case 2: /* HI (!C && !Z) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_C | CCF_Z);
        tcg_gen_brcondi_i32(TCG_COND_EQ, tmp, 0, l1);
        break;
    case 3: /* LS (C || Z) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_C | CCF_Z);
        tcg_gen_brcondi_i32(TCG_COND_NE, tmp, 0, l1);
        break;
    case 4: /* CC (!C) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_C);
        tcg_gen_brcondi_i32(TCG_COND_EQ, tmp, 0, l1);
        break;
    case 5: /* CS (C) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_C);
        tcg_gen_brcondi_i32(TCG_COND_NE, tmp, 0, l1);
        break;
    case 6: /* NE (!Z) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_Z);
        tcg_gen_brcondi_i32(TCG_COND_EQ, tmp, 0, l1);
        break;
    case 7: /* EQ (Z) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_Z);
        tcg_gen_brcondi_i32(TCG_COND_NE, tmp, 0, l1);
        break;
    case 8: /* VC (!V) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_V);
        tcg_gen_brcondi_i32(TCG_COND_EQ, tmp, 0, l1);
        break;
    case 9: /* VS (V) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_V);
        tcg_gen_brcondi_i32(TCG_COND_NE, tmp, 0, l1);
        break;
    case 10: /* PL (!N) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_N);
        tcg_gen_brcondi_i32(TCG_COND_EQ, tmp, 0, l1);
        break;
    case 11: /* MI (N) */
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_N);
        tcg_gen_brcondi_i32(TCG_COND_NE, tmp, 0, l1);
        break;
    case 12: /* GE (!(N ^ V)) */
        tmp = tcg_temp_new();
        assert(CCF_V == (CCF_N >> 2));
        tcg_gen_shri_i32(tmp, QREG_CC_DEST, 2);
        tcg_gen_xor_i32(tmp, tmp, QREG_CC_DEST);
        tcg_gen_andi_i32(tmp, tmp, CCF_V);
        tcg_gen_brcondi_i32(TCG_COND_EQ, tmp, 0, l1);
        break;
    case 13: /* LT (N ^ V) */
        tmp = tcg_temp_new();
        assert(CCF_V == (CCF_N >> 2));
        tcg_gen_shri_i32(tmp, QREG_CC_DEST, 2);
        tcg_gen_xor_i32(tmp, tmp, QREG_CC_DEST);
        tcg_gen_andi_i32(tmp, tmp, CCF_V);
        tcg_gen_brcondi_i32(TCG_COND_NE, tmp, 0, l1);
        break;
    case 14: /* GT (!(Z || (N ^ V))) */
        tmp = tcg_temp_new();
        assert(CCF_V == (CCF_N >> 2));
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_N);
        tcg_gen_shri_i32(tmp, tmp, 2);
        tcg_gen_xor_i32(tmp, tmp, QREG_CC_DEST);
        tcg_gen_andi_i32(tmp, tmp, CCF_V | CCF_Z);
        tcg_gen_brcondi_i32(TCG_COND_EQ, tmp, 0, l1);
        break;
    case 15: /* LE (Z || (N ^ V)) */
        tmp = tcg_temp_new();
        assert(CCF_V == (CCF_N >> 2));
        tcg_gen_andi_i32(tmp, QREG_CC_DEST, CCF_N);
        tcg_gen_shri_i32(tmp, tmp, 2);
        tcg_gen_xor_i32(tmp, tmp, QREG_CC_DEST);
        tcg_gen_andi_i32(tmp, tmp, CCF_V | CCF_Z);
        tcg_gen_brcondi_i32(TCG_COND_NE, tmp, 0, l1);
        break;
    default:
        /* Should ever happen.  */
        abort();
    }
}

DISAS_INSN(scc)
{
    int l1;
    int cond;
    TCGv reg;

    l1 = gen_new_label();
    cond = (insn >> 8) & 0xf;
    reg = DREG(insn, 0);
    tcg_gen_andi_i32(reg, reg, 0xffffff00);
    /* This is safe because we modify the reg directly, with no other values
       live.  */
    gen_jmpcc(s, cond ^ 1, l1);
    tcg_gen_ori_i32(reg, reg, 0xff);
    gen_set_label(l1);
}

/* Force a TB lookup after an instruction that changes the CPU state.  */
static void gen_lookup_tb(DisasContext *s)
{
    gen_flush_cc_op(s);
    tcg_gen_movi_i32(QREG_PC, s->pc);
    s->is_jmp = DISAS_UPDATE;
}

/* Generate a jump to an immediate address.  */
static void gen_jmp_im(DisasContext *s, uint32_t dest)
{
    gen_flush_cc_op(s);
    tcg_gen_movi_i32(QREG_PC, dest);
    s->is_jmp = DISAS_JUMP;
}

/* Generate a jump to the address in qreg DEST.  */
static void gen_jmp(DisasContext *s, TCGv dest)
{
    gen_flush_cc_op(s);
    tcg_gen_mov_i32(QREG_PC, dest);
    s->is_jmp = DISAS_JUMP;
}

static void gen_exception(DisasContext *s, uint32_t where, int nr)
{
    gen_flush_cc_op(s);
    gen_jmp_im(s, where);
    gen_helper_raise_exception(tcg_const_i32(nr));
}

static inline void gen_addr_fault(DisasContext *s)
{
    gen_exception(s, s->insn_pc, EXCP_ADDRESS);
}

#define SRC_EA(result, opsize, op_sign, addrp) do { \
    result = gen_ea(s, insn, opsize, NULL_QREG, addrp, op_sign ? EA_LOADS : EA_LOADU); \
    if (IS_NULL_QREG(result)) { \
        gen_addr_fault(s); \
        return; \
    } \
    } while (0)

#define DEST_EA(insn, opsize, val, addrp) do { \
    TCGv ea_result = gen_ea(s, insn, opsize, val, addrp, EA_STORE); \
    if (IS_NULL_QREG(ea_result)) { \
        gen_addr_fault(s); \
        return; \
    } \
    } while (0)

/* Generate a jump to an immediate address.  */
static void gen_jmp_tb(DisasContext *s, int n, uint32_t dest)
{
    TranslationBlock *tb;

    tb = s->tb;
    if (unlikely(s->singlestep_enabled)) {
        gen_exception(s, dest, EXCP_DEBUG);
    } else if ((tb->pc & TARGET_PAGE_MASK) == (dest & TARGET_PAGE_MASK) ||
               (s->pc & TARGET_PAGE_MASK) == (dest & TARGET_PAGE_MASK)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_i32(QREG_PC, dest);
        tcg_gen_exit_tb((tcg_target_long)tb + n);
    } else {
        gen_jmp_im(s, dest);
        tcg_gen_exit_tb(0);
    }
    s->is_jmp = DISAS_TB_JUMP;
}

DISAS_INSN(scc_mem)
{
    int l1;
    int cond;
    TCGv dest;

    l1 = gen_new_label();
    cond = (insn >> 8) & 0xf;
    dest = tcg_temp_local_new();
    tcg_gen_movi_i32(dest, 0);
    gen_jmpcc(s, cond ^ 1, l1);
    tcg_gen_movi_i32(dest, 0xff);
    gen_set_label(l1);
    DEST_EA(insn, OS_BYTE, dest, NULL);
    tcg_temp_free(dest);
}

DISAS_INSN(dbcc)
{
    int l1;
    TCGv reg;
    TCGv tmp;
    int16_t offset;
    uint32_t base;

    reg = DREG(insn, 0);
    base = s->pc;
    offset = ldsw_code(s->pc);
    s->pc += 2;
    l1 = gen_new_label();
    gen_jmpcc(s, (insn >> 8) & 0xf, l1);

    tmp = tcg_temp_new();
    tcg_gen_ext16s_i32(tmp, reg);
    tcg_gen_addi_i32(tmp, tmp, -1);
    gen_partset_reg(OS_WORD, reg, tmp);
    tcg_gen_brcondi_i32(TCG_COND_EQ, tmp, -1, l1);
    gen_jmp_tb(s, 1, base + offset);
    gen_set_label(l1);
    gen_jmp_tb(s, 0, s->pc);
}

DISAS_INSN(undef_mac)
{
    gen_exception(s, s->pc - 2, EXCP_LINEA);
}

DISAS_INSN(undef_fpu)
{
    gen_exception(s, s->pc - 2, EXCP_LINEF);
}

DISAS_INSN(undef)
{
    gen_exception(s, s->pc - 2, EXCP_UNSUPPORTED);
    cpu_abort(cpu_single_env, "Illegal instruction: %04x @ %08x",
              insn, s->pc - 2);
}

DISAS_INSN(mulw)
{
    TCGv reg;
    TCGv tmp;
    TCGv src;
    int sign;

    sign = (insn & 0x100) != 0;
    reg = DREG(insn, 9);
    tmp = tcg_temp_new();
    if (sign)
        tcg_gen_ext16s_i32(tmp, reg);
    else
        tcg_gen_ext16u_i32(tmp, reg);
    SRC_EA(src, OS_WORD, sign, NULL);
    tcg_gen_mul_i32(tmp, tmp, src);
    tcg_gen_mov_i32(reg, tmp);
    gen_logic_cc(s, tmp, OS_WORD);
}

DISAS_INSN(divw)
{
    TCGv dest;
    TCGv tmp;
    TCGv src;
    int sign;
    int l1;

    sign = (insn & 0x100) != 0;

    /* dest.l / src.w */

    dest = DREG(insn, 9);
    tcg_gen_mov_i32(QREG_DIV1, dest);

    SRC_EA(src, OS_WORD, sign, NULL);
    tcg_gen_mov_i32(QREG_DIV2, src);

    /* div1 / div2 */

    if (sign) {
        gen_helper_divs(cpu_env, tcg_const_i32(1));
    } else {
        gen_helper_divu(cpu_env, tcg_const_i32(1));
    }

    s->cc_op = CC_OP_FLAGS;

    l1 = gen_new_label();
    gen_jmpcc(s, 9 /* V */, l1);
    tmp = tcg_temp_new();
    src = tcg_temp_new();
    tcg_gen_ext16u_i32(tmp, QREG_DIV1);
    tcg_gen_shli_i32(src, QREG_DIV2, 16);
    tcg_gen_or_i32(dest, tmp, src);
    gen_set_label(l1);
}

DISAS_INSN(divl)
{
    TCGv num;
    TCGv den;
    TCGv reg;
    uint16_t ext;

    ext = read_im16(s);
    if (ext & 0x400) {
        if (!m68k_feature(s->env, M68K_FEATURE_QUAD_MULDIV)) {
            gen_exception(s, s->pc - 4, EXCP_UNSUPPORTED);
            return;
        }
        num = DREG(ext, 12);
        reg = DREG(ext, 0);
        tcg_gen_mov_i32(QREG_DIV1, num);
        tcg_gen_mov_i32(QREG_QUADH, reg);
        SRC_EA(den, OS_LONG, 0, NULL);
        tcg_gen_mov_i32(QREG_DIV2, den);
        if (ext & 0x0800) {
            gen_helper_divs64(cpu_env);
        } else {
            gen_helper_divu64(cpu_env);
        }
        tcg_gen_mov_i32(num, QREG_DIV1);
        if (!TCGV_EQUAL(num, reg))
            tcg_gen_mov_i32(reg, QREG_DIV2);
        s->cc_op = CC_OP_FLAGS;
        return;
    }
    num = DREG(ext, 12);
    reg = DREG(ext, 0);
    tcg_gen_mov_i32(QREG_DIV1, num);
    SRC_EA(den, OS_LONG, 0, NULL);
    tcg_gen_mov_i32(QREG_DIV2, den);
    if (ext & 0x0800) {
        gen_helper_divs(cpu_env, tcg_const_i32(0));
    } else {
        gen_helper_divu(cpu_env, tcg_const_i32(0));
    }
    if (TCGV_EQUAL(num, reg) ||
        m68k_feature(s->env, M68K_FEATURE_LONG_MULDIV)) {
        /* div */
        tcg_gen_mov_i32 (num, QREG_DIV1);
    }
    if (!TCGV_EQUAL(num, reg)) {
        /* rem */
        tcg_gen_mov_i32 (reg, QREG_DIV2);
    }
    s->cc_op = CC_OP_FLAGS;
}

DISAS_INSN(abcd_reg)
{
    TCGv src;
    TCGv dest;

    src = DREG(insn, 0);
    dest = DREG(insn, 9);
    gen_helper_abcd_cc(dest, cpu_env, src, dest);
}

DISAS_INSN(abcd_mem)
{
    TCGv src;
    TCGv addr_src;
    TCGv dest;
    TCGv addr_dest;

    addr_src = AREG(insn, 0);
    tcg_gen_subi_i32(addr_src, addr_src, OS_BYTE);
    src = gen_load(s, OS_BYTE, addr_src, 0);

    addr_dest = AREG(insn, 9);
    tcg_gen_subi_i32(addr_dest, addr_dest, OS_BYTE);
    dest = gen_load(s, OS_BYTE, addr_dest, 0);

    gen_helper_abcd_cc(dest, cpu_env, src, dest);

    gen_store(s, OS_BYTE, addr_dest, dest);
}

DISAS_INSN(sbcd_reg)
{
    TCGv src;
    TCGv dest;

    src = DREG(insn, 0);
    dest = DREG(insn, 9);
    gen_helper_sbcd_cc(dest, cpu_env, src, dest);
}

DISAS_INSN(sbcd_mem)
{
    TCGv src;
    TCGv addr_src;
    TCGv dest;
    TCGv addr_dest;

    addr_src = AREG(insn, 0);
    tcg_gen_subi_i32(addr_src, addr_src, OS_BYTE);
    src = gen_load(s, OS_BYTE, addr_src, 0);

    addr_dest = AREG(insn, 9);
    tcg_gen_subi_i32(addr_dest, addr_dest, OS_BYTE);
    dest = gen_load(s, OS_BYTE, addr_dest, 0);

    gen_helper_sbcd_cc(dest, cpu_env, src, dest);

    gen_store(s, OS_BYTE, addr_dest, dest);
}

DISAS_INSN(nbcd)
{
    TCGv dest;
    TCGv addr;

    SRC_EA(dest, OS_BYTE, -1, &addr);

    gen_helper_sbcd_cc(dest, cpu_env, dest, tcg_const_i32(0));

    DEST_EA(insn, OS_BYTE, dest, &addr);
}

DISAS_INSN(addsub)
{
    TCGv reg;
    TCGv dest;
    TCGv src;
    TCGv tmp;
    TCGv addr;
    int add;
    int opsize;

    add = (insn & 0x4000) != 0;
    opsize = insn_opsize(insn, 6);
    reg = DREG(insn, 9);
    dest = tcg_temp_new();
    if (insn & 0x100) {
        SRC_EA(tmp, opsize, -1, &addr);
        src = reg;
    } else {
        tmp = reg;
        SRC_EA(src, opsize, -1, NULL);
    }
    if (add) {
        tcg_gen_add_i32(dest, tmp, src);
        SET_X_FLAG(opsize, dest, src);
        SET_CC_OP(opsize, ADD);
    } else {
        SET_X_FLAG(opsize, tmp, src);
        tcg_gen_sub_i32(dest, tmp, src);
        SET_CC_OP(opsize, SUB);
    }
    gen_update_cc_add(dest, src);
    if (insn & 0x100) {
        DEST_EA(insn, opsize, dest, &addr);
    } else {
        gen_partset_reg(opsize, reg, dest);
    }
}


/* Reverse the order of the bits in REG.  */
DISAS_INSN(bitrev)
{
    TCGv reg;
    reg = DREG(insn, 0);
    gen_helper_bitrev(reg, reg);
}

DISAS_INSN(bitop_reg)
{
    int opsize;
    int op;
    TCGv src1;
    TCGv src2;
    TCGv tmp;
    TCGv addr;
    TCGv dest;

    if ((insn & 0x38) != 0)
        opsize = OS_BYTE;
    else
        opsize = OS_LONG;
    op = (insn >> 6) & 3;
    SRC_EA(src1, opsize, 0, op ? &addr: NULL);
    src2 = DREG(insn, 9);
    dest = tcg_temp_new();

    gen_flush_flags(s);
    tmp = tcg_temp_new();
    if (opsize == OS_BYTE)
        tcg_gen_andi_i32(tmp, src2, 7);
    else
        tcg_gen_andi_i32(tmp, src2, 31);
    src2 = tmp;
    tmp = tcg_temp_new();
    tcg_gen_shr_i32(tmp, src1, src2);
    tcg_gen_andi_i32(tmp, tmp, 1);
    tcg_gen_shli_i32(tmp, tmp, 2);
    /* Clear CCF_Z if bit set.  */
    tcg_gen_ori_i32(QREG_CC_DEST, QREG_CC_DEST, CCF_Z);
    tcg_gen_xor_i32(QREG_CC_DEST, QREG_CC_DEST, tmp);

    tcg_gen_shl_i32(tmp, tcg_const_i32(1), src2);
    switch (op) {
    case 1: /* bchg */
        tcg_gen_xor_i32(dest, src1, tmp);
        break;
    case 2: /* bclr */
        tcg_gen_not_i32(tmp, tmp);
        tcg_gen_and_i32(dest, src1, tmp);
        break;
    case 3: /* bset */
        tcg_gen_or_i32(dest, src1, tmp);
        break;
    default: /* btst */
        break;
    }
    if (op)
        DEST_EA(insn, opsize, dest, &addr);
}

DISAS_INSN(sats)
{
    TCGv reg;
    reg = DREG(insn, 0);
    gen_flush_flags(s);
    gen_helper_sats(reg, reg, QREG_CC_DEST);
    gen_logic_cc(s, reg, OS_LONG);
}

static void gen_push(DisasContext *s, TCGv val)
{
    TCGv tmp;

    tmp = tcg_temp_new();
    tcg_gen_subi_i32(tmp, QREG_SP, 4);
    gen_store(s, OS_LONG, tmp, val);
    tcg_gen_mov_i32(QREG_SP, tmp);
}

DISAS_INSN(movem)
{
    TCGv addr;
    int i;
    uint16_t mask;
    TCGv reg;
    TCGv tmp;
    int is_load;
    int opsize;
    int32_t incr;

    mask = read_im16(s);
    tmp = gen_lea(s, insn, OS_LONG);
    if (IS_NULL_QREG(tmp)) {
        gen_addr_fault(s);
        return;
    }
    addr = tcg_temp_new();
    tcg_gen_mov_i32(addr, tmp);
    is_load = ((insn & 0x0400) != 0);
    opsize = (insn & 0x40) != 0 ? OS_LONG : OS_WORD;
    incr = opsize_bytes(opsize);
    if (!is_load && (insn & 070) == 040) {
       for (i = 15; i >= 0; i--, mask >>= 1) {
           if (mask & 1) {
               if (i < 8)
                   reg = DREG(i, 0);
               else
                   reg = AREG(i, 0);
               gen_store(s, opsize, addr, reg);
               if (mask != 1)
                   tcg_gen_subi_i32(addr, addr, incr);
           }
       }
       tcg_gen_mov_i32(AREG(insn, 0), addr);
    } else {
       for (i = 0; i < 16; i++, mask >>= 1) {
           if (mask & 1) {
               if (i < 8)
                   reg = DREG(i, 0);
               else
                   reg = AREG(i, 0);
               if (is_load) {
                   tmp = gen_load(s, opsize, addr, 1);
                   tcg_gen_mov_i32(reg, tmp);
               } else {
                   gen_store(s, opsize, addr, reg);
               }
               if (mask != 1 || (insn & 070) == 030)
                   tcg_gen_addi_i32(addr, addr, incr);
           }
       }
       if ((insn & 070) == 030)
           tcg_gen_mov_i32(AREG(insn, 0), addr);
    }
}

DISAS_INSN(bitop_im)
{
    int opsize;
    int op;
    TCGv src1;
    uint32_t mask;
    int bitnum;
    TCGv tmp;
    TCGv addr;

    if ((insn & 0x38) != 0)
        opsize = OS_BYTE;
    else
        opsize = OS_LONG;
    op = (insn >> 6) & 3;

    bitnum = read_im16(s);
    if (bitnum & 0xff00) {
        disas_undef(s, insn);
        return;
    }

    SRC_EA(src1, opsize, 0, op ? &addr: NULL);

    gen_flush_flags(s);
    if (opsize == OS_BYTE)
        bitnum &= 7;
    else
        bitnum &= 31;
    mask = 1 << bitnum;

    tmp = tcg_temp_new();
    assert (CCF_Z == (1 << 2));
    if (bitnum > 2)
        tcg_gen_shri_i32(tmp, src1, bitnum - 2);
    else if (bitnum < 2)
        tcg_gen_shli_i32(tmp, src1, 2 - bitnum);
    else
        tcg_gen_mov_i32(tmp, src1);
    tcg_gen_andi_i32(tmp, tmp, CCF_Z);
    /* Clear CCF_Z if bit set.  */
    tcg_gen_ori_i32(QREG_CC_DEST, QREG_CC_DEST, CCF_Z);
    tcg_gen_xor_i32(QREG_CC_DEST, QREG_CC_DEST, tmp);
    if (op) {
        switch (op) {
        case 1: /* bchg */
            tcg_gen_xori_i32(tmp, src1, mask);
            break;
        case 2: /* bclr */
            tcg_gen_andi_i32(tmp, src1, ~mask);
            break;
        case 3: /* bset */
            tcg_gen_ori_i32(tmp, src1, mask);
            break;
        default: /* btst */
            break;
        }
        DEST_EA(insn, opsize, tmp, &addr);
    }
}

DISAS_INSN(arith_im)
{
    int op;
    uint32_t im;
    TCGv src1;
    TCGv dest;
    TCGv addr;
    int opsize;

    op = (insn >> 9) & 7;
    opsize = insn_opsize(insn, 6);
    switch (opsize) {
    case OS_BYTE:
        im = read_im8(s);
        break;
    case OS_WORD:
        im = read_im16(s);
        break;
    case OS_LONG:
        im = read_im32(s);
        break;
    default:
       abort();
    }
    SRC_EA(src1, opsize, -1, (op == 6) ? NULL : &addr);
    dest = tcg_temp_new();
    switch (op) {
    case 0: /* ori */
        tcg_gen_ori_i32(dest, src1, im);
        gen_logic_cc(s, dest, opsize);
        break;
    case 1: /* andi */
        tcg_gen_andi_i32(dest, src1, im);
        gen_logic_cc(s, dest, opsize);
        break;
    case 2: /* subi */
        tcg_gen_mov_i32(dest, src1);
        SET_X_FLAG(opsize, dest, tcg_const_i32(im));
        tcg_gen_subi_i32(dest, dest, im);
        gen_update_cc_add(dest, tcg_const_i32(im));
        SET_CC_OP(opsize, SUB);
        break;
    case 3: /* addi */
        tcg_gen_mov_i32(dest, src1);
        tcg_gen_addi_i32(dest, dest, im);
        gen_update_cc_add(dest, tcg_const_i32(im));
        SET_X_FLAG(opsize, dest, tcg_const_i32(im));
	SET_CC_OP(opsize, ADD);
        break;
    case 5: /* eori */
        tcg_gen_xori_i32(dest, src1, im);
        gen_logic_cc(s, dest, opsize);
        break;
    case 6: /* cmpi */
        tcg_gen_mov_i32(dest, src1);
        tcg_gen_subi_i32(dest, dest, im);
        gen_update_cc_add(dest, tcg_const_i32(im));
        SET_CC_OP(opsize, SUB);
        break;
    default:
        abort();
    }
    if (op != 6) {
        DEST_EA(insn, opsize, dest, &addr);
    }
}

static TCGv gen_get_ccr(DisasContext *s)
{
    TCGv dest;

    gen_flush_flags(s);
    dest = tcg_temp_new();
    tcg_gen_shli_i32(dest, QREG_CC_X, 4);
    tcg_gen_or_i32(dest, dest, QREG_CC_DEST);
    return dest;
}

static TCGv gen_get_sr(DisasContext *s)
{
    TCGv ccr;
    TCGv sr;

    ccr = gen_get_ccr(s);
    sr = tcg_temp_new();
    tcg_gen_andi_i32(sr, QREG_SR, 0xffe0);
    tcg_gen_or_i32(sr, sr, ccr);
    return sr;
}

DISAS_INSN(arith_im_sr)
{
    int op;
    uint32_t im;
    TCGv src;
    TCGv dest;
    int opsize;

    op = (insn >> 9) & 7;
    opsize = insn_opsize(insn, 6);
    switch (opsize) {
    case OS_BYTE:
        im = read_im8(s);
        break;
    case OS_WORD:
        if (IS_USER(s)) {
            gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
            return;
        }
        im = read_im16(s);
        break;
    default:
       abort();
    }
    src = gen_get_sr(s);
    dest = tcg_temp_new();
    switch (op) {
    case 0: /* ori */
        tcg_gen_ori_i32(dest, src, im);
        break;
    case 1: /* andi */
        tcg_gen_andi_i32(dest, src, im);
        break;
    case 5: /* eori */
        tcg_gen_xori_i32(dest, src, im);
        break;
    default:
        abort();
    }
    if (opsize) {
        gen_helper_set_sr(cpu_env, dest);
    }
    tcg_gen_andi_i32(QREG_CC_DEST, dest, 0xf);
    tcg_gen_shri_i32(dest, dest, 4);
    tcg_gen_andi_i32(QREG_CC_X, dest, 1);
}

DISAS_INSN(cas)
{
    int opsize;
    TCGv dest;
    TCGv tmp;
    TCGv cmp;
    TCGv update;
    TCGv taddr;
    TCGv addr;
    TCGv res;
    uint16_t ext;
    int l1, l2;

    if ((insn & 0x3f) == 0x3c) {
        /* CAS2: Not yet implemented */
        gen_exception(s, s->pc - 4, EXCP_UNSUPPORTED);
    }

    switch((insn >> 9) & 3) {
    case 1:
        opsize = OS_BYTE;
        break;
    case 2:
        opsize = OS_WORD;
        break;
    case 3:
        opsize = OS_LONG;
        break;
    default:
        abort();
    }

    ext = read_im16(s);
    taddr = gen_lea(s, insn, opsize);
    if (IS_NULL_QREG(taddr)) {
        gen_addr_fault(s);
        return;
    }

    cmp = DREG(ext, 0);
    update = DREG(ext, 6);
    tmp = gen_load(s, opsize, taddr, 0);
    dest = tcg_temp_local_new();
    tcg_gen_mov_i32(dest, tmp);
    addr = tcg_temp_local_new ();
    tcg_gen_mov_i32(addr, taddr);

    res = tcg_temp_new();
    tcg_gen_sub_i32(res, dest, cmp);
    gen_logic_cc(s, res, opsize);

    l1 = gen_new_label();
    l2 = gen_new_label();

    gen_jmpcc(s, 6 /* !Z */, l1);
    gen_store(s, opsize, addr, update);
    tcg_gen_br(l2);
    gen_set_label(l1);
    tcg_gen_mov_i32(cmp, dest);
    gen_set_label(l2);
    tcg_temp_free(dest);
    tcg_temp_free(addr);
}

DISAS_INSN(byterev)
{
    TCGv reg;

    reg = DREG(insn, 0);
    tcg_gen_bswap32_i32(reg, reg);
}

DISAS_INSN(move)
{
    TCGv src;
    TCGv dest;
    int op;
    int opsize;

    switch (insn >> 12) {
    case 1: /* move.b */
        opsize = OS_BYTE;
        break;
    case 2: /* move.l */
        opsize = OS_LONG;
        break;
    case 3: /* move.w */
        opsize = OS_WORD;
        break;
    default:
        abort();
    }
    SRC_EA(src, opsize, 1, NULL);
    op = (insn >> 6) & 7;
    if (op == 1) {
        /* movea */
        /* The value will already have been sign extended.  */
        dest = AREG(insn, 9);
        tcg_gen_mov_i32(dest, src);
    } else {
        /* normal move */
        uint16_t dest_ea;
        dest_ea = ((insn >> 9) & 7) | (op << 3);
        DEST_EA(dest_ea, opsize, src, NULL);
        /* This will be correct because loads sign extend.  */
        gen_logic_cc(s, src, opsize);
    }
}

DISAS_INSN(negx)
{
    TCGv src;
    TCGv dest;
    TCGv addr;
    int opsize;

    opsize = insn_opsize(insn, 6);
    SRC_EA(src, opsize, -1, &addr);
    dest = tcg_temp_new();
    switch(opsize) {
    case OS_BYTE:
        gen_helper_subx8_cc(dest, cpu_env, tcg_const_i32(0), src);
        break;
    case OS_WORD:
        gen_helper_subx16_cc(dest, cpu_env, tcg_const_i32(0), src);
        break;
    case OS_LONG:
        gen_helper_subx32_cc(dest, cpu_env, tcg_const_i32(0), src);
        break;
    }
    s->cc_op = CC_OP_FLAGS;
    DEST_EA(insn, opsize, dest, &addr);
}

DISAS_INSN(lea)
{
    TCGv reg;
    TCGv tmp;

    reg = AREG(insn, 9);
    tmp = gen_lea(s, insn, OS_LONG);
    if (IS_NULL_QREG(tmp)) {
        gen_addr_fault(s);
        return;
    }
    tcg_gen_mov_i32(reg, tmp);
}

DISAS_INSN(clr)
{
    int opsize;

    opsize = insn_opsize(insn, 6);
    DEST_EA(insn, opsize, tcg_const_i32(0), NULL);
    gen_logic_cc(s, tcg_const_i32(0), opsize);
}

DISAS_INSN(move_from_ccr)
{
    TCGv reg;
    TCGv ccr;

    ccr = gen_get_ccr(s);
    reg = DREG(insn, 0);
    gen_partset_reg(OS_WORD, reg, ccr);
}

DISAS_INSN(neg)
{
    TCGv src1;
    TCGv dest;
    TCGv addr;
    int opsize;

    opsize = insn_opsize(insn, 6);
    SRC_EA(src1, opsize, -1, &addr);
    dest = tcg_temp_new();
    tcg_gen_neg_i32(dest, src1);
    SET_CC_OP(opsize, SUB);
    gen_update_cc_add(dest, src1);
    SET_X_FLAG(opsize, tcg_const_i32(0), dest);
    DEST_EA(insn, opsize, dest, &addr);
}

static void gen_set_sr_im(DisasContext *s, uint16_t val, int ccr_only)
{
    tcg_gen_movi_i32(QREG_CC_DEST, val & 0xf);
    tcg_gen_movi_i32(QREG_CC_X, (val & 0x10) >> 4);
    if (!ccr_only) {
        gen_helper_set_sr(cpu_env, tcg_const_i32(val & 0xff00));
    }
}

static void gen_set_sr(DisasContext *s, uint16_t insn, int ccr_only)
{
    TCGv tmp;
    TCGv reg;

    s->cc_op = CC_OP_FLAGS;
    if ((insn & 0x3f) != 0x3c)
      {
        tmp = tcg_temp_new();
        SRC_EA(reg, OS_WORD, 1, NULL);
        tcg_gen_andi_i32(QREG_CC_DEST, reg, 0xf);
        tcg_gen_shri_i32(tmp, reg, 4);
        tcg_gen_andi_i32(QREG_CC_X, tmp, 1);
        if (!ccr_only) {
            gen_helper_set_sr(cpu_env, reg);
        }
      }
    else
      {
        uint16_t val;
        val = read_im16(s);
        gen_set_sr_im(s, val, ccr_only);
      }
}

DISAS_INSN(move_to_ccr)
{
    gen_set_sr(s, insn, 1);
}

DISAS_INSN(not)
{
    TCGv src1;
    TCGv dest;
    TCGv addr;
    int opsize;

    opsize = insn_opsize(insn, 6);
    SRC_EA(src1, opsize, -1, &addr);
    dest = tcg_temp_new();
    tcg_gen_not_i32(dest, src1);
    DEST_EA(insn, opsize, dest, &addr);
    gen_logic_cc(s, dest, opsize);
}

DISAS_INSN(swap)
{
    TCGv src1;
    TCGv src2;
    TCGv reg;

    src1 = tcg_temp_new();
    src2 = tcg_temp_new();
    reg = DREG(insn, 0);
    tcg_gen_shli_i32(src1, reg, 16);
    tcg_gen_shri_i32(src2, reg, 16);
    tcg_gen_or_i32(reg, src1, src2);
    gen_logic_cc(s, reg, OS_LONG);
}

DISAS_INSN(pea)
{
    TCGv tmp;

    tmp = gen_lea(s, insn, OS_LONG);
    if (IS_NULL_QREG(tmp)) {
        gen_addr_fault(s);
        return;
    }
    gen_push(s, tmp);
}

DISAS_INSN(ext)
{
    int op;
    TCGv reg;
    TCGv tmp;

    reg = DREG(insn, 0);
    op = (insn >> 6) & 7;
    tmp = tcg_temp_new();
    if (op == 3)
        tcg_gen_ext16s_i32(tmp, reg);
    else
        tcg_gen_ext8s_i32(tmp, reg);
    if (op == 2)
        gen_partset_reg(OS_WORD, reg, tmp);
    else
        tcg_gen_mov_i32(reg, tmp);
    gen_logic_cc(s, tmp, OS_LONG);
}

DISAS_INSN(tst)
{
    int opsize;
    TCGv tmp;

    opsize = insn_opsize(insn, 6);
    SRC_EA(tmp, opsize, -1, NULL);
    gen_logic_cc(s, tmp, opsize);
}

DISAS_INSN(pulse)
{
  /* Implemented as a NOP.  */
}

DISAS_INSN(illegal)
{
    gen_exception(s, s->pc - 2, EXCP_ILLEGAL);
}

/* ??? This should be atomic.  */
DISAS_INSN(tas)
{
    TCGv dest;
    TCGv src1;
    TCGv addr;

    dest = tcg_temp_new();
    SRC_EA(src1, OS_BYTE, 1, &addr);
    gen_logic_cc(s, src1, OS_BYTE);
    tcg_gen_ori_i32(dest, src1, 0x80);
    DEST_EA(insn, OS_BYTE, dest, &addr);
}

DISAS_INSN(mull)
{
    uint16_t ext;
    TCGv reg;
    TCGv src1;
    TCGv dest;
    TCGv regh;

    /* The upper 32 bits of the product are discarded, so
       muls.l and mulu.l are functionally equivalent.  */
    ext = read_im16(s);
    if (ext & 0x400) {
       if (!m68k_feature(s->env, M68K_FEATURE_QUAD_MULDIV)) {
           gen_exception(s, s->pc - 4, EXCP_UNSUPPORTED);
           return;
       }
       reg = DREG(ext, 12);
       regh = DREG(ext, 0);
       SRC_EA(src1, OS_LONG, 0, NULL);
       dest = tcg_temp_new();
       if (ext & 0x800)
           gen_helper_muls64(dest, cpu_env, src1, reg);
       else
           gen_helper_mulu64(dest, cpu_env, src1, reg);
       tcg_gen_mov_i32(reg, dest);
       tcg_gen_mov_i32(regh, QREG_QUADH);
       s->cc_op = CC_OP_FLAGS;
       return;
    }
    reg = DREG(ext, 12);
    SRC_EA(src1, OS_LONG, 0, NULL);
    dest = tcg_temp_new();
    if (m68k_feature(s->env, M68K_FEATURE_M68000)) {
       if (ext & 0x800)
           gen_helper_muls32_cc(dest, cpu_env, src1, reg);
       else
           gen_helper_mulu32_cc(dest, cpu_env, src1, reg);
       s->cc_op = CC_OP_FLAGS;
    } else {
       tcg_gen_mul_i32(dest, src1, reg);
    }
    gen_logic_cc(s, dest, OS_LONG);
    tcg_gen_mov_i32(reg, dest);
}

DISAS_INSN(link)
{
    int16_t offset;
    TCGv reg;
    TCGv tmp;

    offset = read_im16(s);
    reg = AREG(insn, 0);
    tmp = tcg_temp_new();
    tcg_gen_subi_i32(tmp, QREG_SP, 4);
    gen_store(s, OS_LONG, tmp, reg);
    if ((insn & 7) != 7)
        tcg_gen_mov_i32(reg, tmp);
    tcg_gen_addi_i32(QREG_SP, tmp, offset);
}

DISAS_INSN(linkl)
{
    int32_t offset;
    TCGv reg;
    TCGv tmp;

    offset = read_im32(s);
    reg = AREG(insn, 0);
    tmp = tcg_temp_new();
    tcg_gen_subi_i32(tmp, QREG_SP, 4);
    gen_store(s, OS_LONG, tmp, reg);
    if ((insn & 7) != 7)
        tcg_gen_mov_i32(reg, tmp);
    tcg_gen_addi_i32(QREG_SP, tmp, offset);
}

DISAS_INSN(unlk)
{
    TCGv src;
    TCGv reg;
    TCGv tmp;

    src = tcg_temp_new();
    reg = AREG(insn, 0);
    tcg_gen_mov_i32(src, reg);
    tmp = gen_load(s, OS_LONG, src, 0);
    tcg_gen_mov_i32(reg, tmp);
    tcg_gen_addi_i32(QREG_SP, src, 4);
}

DISAS_INSN(nop)
{
}

DISAS_INSN(rts)
{
    TCGv tmp;

    tmp = gen_load(s, OS_LONG, QREG_SP, 0);
    tcg_gen_addi_i32(QREG_SP, QREG_SP, 4);
    gen_jmp(s, tmp);
}

DISAS_INSN(jump)
{
    TCGv tmp;

    /* Load the target address first to ensure correct exception
       behavior.  */
    tmp = gen_lea(s, insn, OS_LONG);
    if (IS_NULL_QREG(tmp)) {
        gen_addr_fault(s);
        return;
    }
    if ((insn & 0x40) == 0) {
        /* jsr */
        gen_push(s, tcg_const_i32(s->pc));
    }
    gen_jmp(s, tmp);
}

DISAS_INSN(addsubq)
{
    TCGv src1;
    TCGv src2;
    TCGv dest;
    int val;
    TCGv addr;
    int opsize;

    if ((insn & 070) == 010) {
        /* Operation on address register is always long.  */
        opsize = OS_LONG;
    } else
        opsize = insn_opsize(insn, 6);
    SRC_EA(src1, opsize, -1, &addr);
    val = (insn >> 9) & 7;
    if (val == 0)
        val = 8;
    dest = tcg_temp_new();
    tcg_gen_mov_i32(dest, src1);
    if ((insn & 0x38) == 0x08) {
        /* Don't update condition codes if the destination is an
           address register.  */
        if (insn & 0x0100) {
            tcg_gen_subi_i32(dest, dest, val);
        } else {
            tcg_gen_addi_i32(dest, dest, val);
        }
    } else {
        src2 = tcg_const_i32(val);
        if (insn & 0x0100) {
            SET_X_FLAG(opsize, dest, src2);
            tcg_gen_subi_i32(dest, dest, val);
            SET_CC_OP(opsize, SUB);
        } else {
            tcg_gen_addi_i32(dest, dest, val);
            SET_X_FLAG(opsize, dest, src2);
            SET_CC_OP(opsize, ADD);
        }
        gen_update_cc_add(dest, src2);
    }
    //DEST_EA(insn, OS_LONG, dest, &addr);
    DEST_EA(insn, opsize, dest, &addr);
}

DISAS_INSN(tpf)
{
    switch (insn & 7) {
    case 2: /* One extension word.  */
        s->pc += 2;
        break;
    case 3: /* Two extension words.  */
        s->pc += 4;
        break;
    case 4: /* No extension words.  */
        break;
    default:
        disas_undef(s, insn);
    }
}

DISAS_INSN(branch)
{
    int32_t offset;
    uint32_t base;
    int op;
    int l1;

    base = s->pc;
    op = (insn >> 8) & 0xf;
    offset = (int8_t)insn;
    if (offset == 0) {
        offset = ldsw_code(s->pc);
        s->pc += 2;
    } else if (offset == -1) {
        offset = read_im32(s);
    }
    if (op == 1) {
        /* bsr */
        gen_push(s, tcg_const_i32(s->pc));
    }
    gen_flush_cc_op(s);
    if (op > 1) {
        /* Bcc */
        l1 = gen_new_label();
        gen_jmpcc(s, ((insn >> 8) & 0xf) ^ 1, l1);
        gen_jmp_tb(s, 1, base + offset);
        gen_set_label(l1);
        gen_jmp_tb(s, 0, s->pc);
    } else {
        /* Unconditional branch.  */
        gen_jmp_tb(s, 0, base + offset);
    }
}

DISAS_INSN(moveq)
{
    uint32_t val;

    val = (int8_t)insn;
    tcg_gen_movi_i32(DREG(insn, 9), val);
    gen_logic_cc(s, tcg_const_i32(val), OS_LONG);
}

DISAS_INSN(mvzs)
{
    int opsize;
    TCGv src;
    TCGv reg;

    if (insn & 0x40)
        opsize = OS_WORD;
    else
        opsize = OS_BYTE;
    SRC_EA(src, opsize, (insn & 0x80) == 0, NULL);
    reg = DREG(insn, 9);
    tcg_gen_mov_i32(reg, src);
    gen_logic_cc(s, src, opsize);
}

DISAS_INSN(or)
{
    TCGv reg;
    TCGv dest;
    TCGv src;
    TCGv addr;
    int opsize;

    opsize = insn_opsize(insn, 6);
    reg = DREG(insn, 9);
    dest = tcg_temp_new();
    if (insn & 0x100) {
        SRC_EA(src, opsize, -1, &addr);
        tcg_gen_or_i32(dest, src, reg);
        DEST_EA(insn, opsize, dest, &addr);
    } else {
        SRC_EA(src, opsize, -1, NULL);
        tcg_gen_or_i32(dest, src, reg);
        gen_partset_reg(opsize, reg, dest);
    }
    gen_logic_cc(s, dest, opsize);
}

DISAS_INSN(suba)
{
    TCGv src;
    TCGv reg;

    SRC_EA(src, (insn & 0x100) ? OS_LONG : OS_WORD, -1, NULL);
    reg = AREG(insn, 9);
    tcg_gen_sub_i32(reg, reg, src);
}

DISAS_INSN(subx_reg)
{
    TCGv reg;
    TCGv src;
    int opsize;

    opsize = insn_opsize(insn, 6);

    gen_flush_flags(s);
    reg = DREG(insn, 9);
    src = DREG(insn, 0);
    switch(opsize) {
    case OS_BYTE:
        gen_helper_subx8_cc(reg, cpu_env, reg, src);
        break;
    case OS_WORD:
        gen_helper_subx16_cc(reg, cpu_env, reg, src);
        break;
    case OS_LONG:
        gen_helper_subx32_cc(reg, cpu_env, reg, src);
        break;
    }
    s->cc_op = CC_OP_FLAGS;
}

DISAS_INSN(subx_mem)
{
    TCGv src;
    TCGv addr_src;
    TCGv reg;
    TCGv addr_reg;
    int opsize;

    opsize = insn_opsize(insn, 6);

    gen_flush_flags(s);

    addr_src = AREG(insn, 0);
    tcg_gen_subi_i32(addr_src, addr_src, opsize);
    src = gen_load(s, opsize, addr_src, 0);

    addr_reg = AREG(insn, 9);
    tcg_gen_subi_i32(addr_reg, addr_reg, opsize);
    reg = gen_load(s, opsize, addr_reg, 0);

    switch(opsize) {
    case OS_BYTE:
        gen_helper_subx8_cc(reg, cpu_env, reg, src);
        break;
    case OS_WORD:
        gen_helper_subx16_cc(reg, cpu_env, reg, src);
        break;
    case OS_LONG:
        gen_helper_subx32_cc(reg, cpu_env, reg, src);
        break;
    }
    s->cc_op = CC_OP_FLAGS;

    gen_store(s, opsize, addr_reg, reg);
}

DISAS_INSN(mov3q)
{
    TCGv src;
    int val;

    val = (insn >> 9) & 7;
    if (val == 0)
        val = -1;
    src = tcg_const_i32(val);
    gen_logic_cc(s, src, OS_LONG);
    DEST_EA(insn, OS_LONG, src, NULL);
}

DISAS_INSN(cmpm)
{
    TCGv src;
    TCGv dest;
    TCGv result;
    int opsize;
    uint16_t insn2;

    insn2 = insn;
    opsize = insn_opsize(insn, 6);

    insn = (insn >> 9) & 7;
    insn |= 0x3 << 3; //AR with post increment
    SRC_EA(src, opsize, -1, NULL);

    insn = insn2;
    insn |= 0x3 << 3; //AR with post increment
    SRC_EA(dest, opsize, -1, NULL);

    result = tcg_temp_new();
    tcg_gen_sub_i32(result, dest, src);
    gen_update_cc_add(result, src);
    SET_CC_OP(opsize, SUB);
}

DISAS_INSN(cmp)
{
    TCGv src;
    TCGv reg;
    TCGv dest;
    int opsize;

    opsize = insn_opsize(insn, 6);
    SRC_EA(src, opsize, -1, NULL);
    reg = DREG(insn, 9);
    dest = tcg_temp_new();
    tcg_gen_sub_i32(dest, reg, src);
    gen_update_cc_add(dest, src);
    SET_CC_OP(opsize, SUB);
}

DISAS_INSN(cmpa)
{
    int opsize;
    TCGv src;
    TCGv reg;
    TCGv dest;

    if (insn & 0x100) {
        opsize = OS_LONG;
    } else {
        opsize = OS_WORD;
    }
    SRC_EA(src, opsize, 1, NULL);
    reg = AREG(insn, 9);
    dest = tcg_temp_new();
    tcg_gen_sub_i32(dest, reg, src);
    gen_update_cc_add(dest, src);
    SET_CC_OP(opsize, SUB);
}

DISAS_INSN(eor)
{
    TCGv src;
    TCGv reg;
    TCGv dest;
    TCGv addr;
    int opsize;

    opsize = insn_opsize(insn, 6);
    SRC_EA(src, opsize, -1, &addr);
    reg = DREG(insn, 9);
    dest = tcg_temp_new();
    tcg_gen_xor_i32(dest, src, reg);
    gen_logic_cc(s, dest, opsize);
    DEST_EA(insn, opsize, dest, &addr);
}

DISAS_INSN(exg)
{
    TCGv src;
    TCGv dest;
    TCGv tmp;

    tmp = tcg_temp_new();
    switch ((insn >> 3) & 0x1f) {
        case 0x08:
            src = DREG(insn, 9);
            dest = DREG(insn, 0);
            break;
        case 0x09:
            src = AREG(insn, 9);
            dest = AREG(insn, 0);
            break;
        case 0x11:
            src = DREG(insn, 9);
            dest = AREG(insn, 0);
            break;
        default:
            fprintf(stderr, "Invalid exg operands");
            abort();
    }
    tcg_gen_mov_i32(tmp, src);
    tcg_gen_mov_i32(src, dest);
    tcg_gen_mov_i32(dest, tmp);
}

DISAS_INSN(and)
{
    TCGv src;
    TCGv reg;
    TCGv dest;
    TCGv addr;
    int opsize;

    opsize = insn_opsize(insn, 6);
    reg = DREG(insn, 9);
    dest = tcg_temp_new();
    if (insn & 0x100) {
        SRC_EA(src, opsize, -1, &addr);
        tcg_gen_and_i32(dest, src, reg);
        DEST_EA(insn, opsize, dest, &addr);
    } else {
        SRC_EA(src, opsize, -1, NULL);
        tcg_gen_and_i32(dest, src, reg);
        gen_partset_reg(opsize, reg, dest);
    }
    gen_logic_cc(s, dest, opsize);
}

DISAS_INSN(adda)
{
    TCGv src;
    TCGv reg;

    SRC_EA(src, (insn & 0x100) ? OS_LONG : OS_WORD, -1, NULL);
    reg = AREG(insn, 9);
    tcg_gen_add_i32(reg, reg, src);
}

DISAS_INSN(addx_reg)
{
    TCGv reg;
    TCGv src;
    int opsize;

    opsize = insn_opsize(insn, 6);

    gen_flush_flags(s);
    reg = DREG(insn, 9);
    src = DREG(insn, 0);
    switch(opsize) {
    case OS_BYTE:
        gen_helper_addx8_cc(reg, cpu_env, reg, src);
        break;
    case OS_WORD:
        gen_helper_addx16_cc(reg, cpu_env, reg, src);
        break;
    case OS_LONG:
        gen_helper_addx32_cc(reg, cpu_env, reg, src);
        break;
    }
    s->cc_op = CC_OP_FLAGS;
}

DISAS_INSN(addx_mem)
{
    TCGv src;
    TCGv addr_src;
    TCGv reg;
    TCGv addr_reg;
    int opsize;

    opsize = insn_opsize(insn, 6);

    gen_flush_flags(s);

    addr_src = AREG(insn, 0);
    tcg_gen_subi_i32(addr_src, addr_src, opsize);
    src = gen_load(s, opsize, addr_src, 0);

    addr_reg = AREG(insn, 9);
    tcg_gen_subi_i32(addr_reg, addr_reg, opsize);
    reg = gen_load(s, opsize, addr_reg, 0);

    switch(opsize) {
    case OS_BYTE:
        gen_helper_addx8_cc(reg, cpu_env, reg, src);
        break;
    case OS_WORD:
        gen_helper_addx16_cc(reg, cpu_env, reg, src);
        break;
    case OS_LONG:
        gen_helper_addx32_cc(reg, cpu_env, reg, src);
        break;
    }
    s->cc_op = CC_OP_FLAGS;

    gen_store(s, opsize, addr_reg, reg);
}

/* TODO: This could be implemented without helper functions.  */
DISAS_INSN(shift8_im)
{
    TCGv reg;
    int tmp;
    TCGv shift;
    TCGv dest;

    reg = DREG(insn, 0);
    tmp = (insn >> 9) & 7;
    if (tmp == 0)
        tmp = 8;
    shift = tcg_const_i32(tmp);
    dest = tcg_temp_new_i32();
    /* No need to flush flags becuse we know we will set C flag.  */
    if (insn & 0x100) {
        if (insn & 8) {
            gen_helper_shl8_cc(dest, cpu_env, reg, shift);
        } else {
            gen_helper_sal8_cc(dest, cpu_env, reg, shift);
        }
    } else {
        if (insn & 8) {
            gen_helper_shr8_cc(dest, cpu_env, reg, shift);
        } else {
            gen_helper_sar8_cc(dest, cpu_env, reg, shift);
        }
    }
    s->cc_op = CC_OP_SHIFTB;
    gen_partset_reg(OS_BYTE, reg, dest);
}

/* TODO: This could be implemented without helper functions.  */
DISAS_INSN(shift16_im)
{
    TCGv reg;
    int tmp;
    TCGv shift;
    TCGv dest;

    reg = DREG(insn, 0);
    tmp = (insn >> 9) & 7;
    if (tmp == 0)
        tmp = 8;
    shift = tcg_const_i32(tmp);
    dest = tcg_temp_new_i32();
    /* No need to flush flags becuse we know we will set C flag.  */
    if (insn & 0x100) {
        if (insn & 8) {
            gen_helper_shl16_cc(dest, cpu_env, reg, shift);
        } else {
            gen_helper_sal16_cc(dest, cpu_env, reg, shift);
        }
    } else {
        if (insn & 8) {
            gen_helper_shr16_cc(dest, cpu_env, reg, shift);
        } else {
            gen_helper_sar16_cc(dest, cpu_env, reg, shift);
        }
    }
    s->cc_op = CC_OP_SHIFTW;
    gen_partset_reg(OS_WORD, reg, dest);
}


/* TODO: This could be implemented without helper functions.  */
DISAS_INSN(shift_im)
{
    TCGv reg;
    int tmp;
    TCGv shift;

    reg = DREG(insn, 0);
    tmp = (insn >> 9) & 7;
    if (tmp == 0)
        tmp = 8;
    shift = tcg_const_i32(tmp);
    /* No need to flush flags becuse we know we will set C flag.  */
    if (insn & 0x100) {
        if (insn & 8) {
            gen_helper_shl32_cc(reg, cpu_env, reg, shift);
        } else {
            gen_helper_sal32_cc(reg, cpu_env, reg, shift);
        }
    } else {
        if (insn & 8) {
            gen_helper_shr32_cc(reg, cpu_env, reg, shift);
        } else {
            gen_helper_sar32_cc(reg, cpu_env, reg, shift);
        }
    }
    s->cc_op = CC_OP_SHIFT;
}

DISAS_INSN(shift8_reg)
{
    TCGv reg;
    TCGv shift;
    TCGv dest;
    TCGv tmp;

    reg = DREG(insn, 0);
    shift = DREG(insn, 9);
    tmp = tcg_temp_new_i32();
    tcg_gen_andi_i32(tmp, shift, 63);
    dest = tcg_temp_new_i32();
    /* Shift by zero leaves C flag unmodified.   */
    gen_flush_flags(s);
    if (insn & 0x100) {
        if (insn & 8) {
            gen_helper_shl8_cc(dest, cpu_env, reg, tmp);
        } else {
            gen_helper_sal8_cc(dest, cpu_env, reg, tmp);
        }
    } else {
        if (insn & 8) {
            gen_helper_shr8_cc(dest, cpu_env, reg, tmp);
        } else {
            gen_helper_sar8_cc(dest, cpu_env, reg, tmp);
        }
    }
    s->cc_op = CC_OP_SHIFTB;
    gen_partset_reg(OS_BYTE, reg, dest);
}

DISAS_INSN(shift16_reg)
{
    TCGv reg;
    TCGv shift;
    TCGv dest;
    TCGv tmp;

    reg = DREG(insn, 0);
    shift = DREG(insn, 9);
    tmp = tcg_temp_new_i32();
    tcg_gen_andi_i32(tmp, shift, 63);
    dest = tcg_temp_new_i32();
    /* Shift by zero leaves C flag unmodified.   */
    gen_flush_flags(s);
    if (insn & 0x100) {
        if (insn & 8) {
            gen_helper_shl16_cc(dest, cpu_env, reg, tmp);
        } else {
            gen_helper_sal16_cc(dest, cpu_env, reg, tmp);
        }
    } else {
        if (insn & 8) {
            gen_helper_shr16_cc(dest, cpu_env, reg, tmp);
        } else {
            gen_helper_sar16_cc(dest, cpu_env, reg, tmp);
        }
    }
    s->cc_op = CC_OP_SHIFTW;
    gen_partset_reg(OS_WORD, reg, dest);
}

DISAS_INSN(shift_reg)
{
    TCGv reg;
    TCGv shift;

    reg = DREG(insn, 0);
    shift = DREG(insn, 9);
    /* Shift by zero leaves C flag unmodified.   */
    gen_flush_flags(s);
    if (insn & 0x100) {
        if (insn & 8) {
            gen_helper_shl32_cc(reg, cpu_env, reg, shift);
        } else {
            gen_helper_sal32_cc(reg, cpu_env, reg, shift);
        }
    } else {
        if (insn & 8) {
            gen_helper_shr32_cc(reg, cpu_env, reg, shift);
        } else {
            gen_helper_sar32_cc(reg, cpu_env, reg, shift);
        }
    }
    s->cc_op = CC_OP_SHIFT;
}

DISAS_INSN(shift_mem)
{
    TCGv src;
    TCGv dest;
    TCGv addr;
    TCGv shift;

    SRC_EA(src, OS_WORD, 0, &addr);
    dest = tcg_temp_new_i32();
    shift = tcg_const_i32(1);
    if (insn & 0x100) {
        gen_helper_shl16_cc(dest, cpu_env, src, shift);
    } else {
        if (insn & 8) {
            gen_helper_shr16_cc(dest, cpu_env, src, shift);
        } else {
            gen_helper_sar16_cc(dest, cpu_env, src, shift);
        }
    }
    s->cc_op = CC_OP_SHIFTW;
    DEST_EA(insn, OS_WORD, dest, &addr);
}

DISAS_INSN(rotate_im)
{
    TCGv reg;
    TCGv shift;
    int tmp;

    reg = DREG(insn, 0);
    tmp = (insn >> 9) & 7;
    if (tmp == 0)
       tmp = 8;
    shift = tcg_const_i32(tmp);
    if (insn & 8) {
       if (insn & 0x100) {
           gen_helper_rol32_cc(reg, cpu_env, reg, shift);
       } else {
           gen_helper_ror32_cc(reg, cpu_env, reg, shift);
       }
    } else {
        if (insn & 0x100) {
            gen_helper_roxl32_cc(reg, cpu_env, reg, shift);
        } else {
            gen_helper_roxr32_cc(reg, cpu_env, reg, shift);
        }
    }
    s->cc_op = CC_OP_FLAGS;
}

DISAS_INSN(rotate8_im)
{
    TCGv reg;
    TCGv dest;
    TCGv shift;
    int tmp;

    reg = DREG(insn, 0);
    tmp = (insn >> 9) & 7;
    if (tmp == 0)
       tmp = 8;
    dest = tcg_temp_new_i32();
    shift = tcg_const_i32(tmp);
    if (insn & 8) {
       if (insn & 0x100) {
           gen_helper_rol8_cc(dest, cpu_env, reg, shift);
       } else {
           gen_helper_ror8_cc(dest, cpu_env, reg, shift);
       }
    } else {
        if (insn & 0x100) {
            gen_helper_roxl8_cc(dest, cpu_env, reg, shift);
        } else {
            gen_helper_roxr8_cc(dest, cpu_env, reg, shift);
        }
    }
    s->cc_op = CC_OP_FLAGS;
    gen_partset_reg(OS_BYTE, reg, dest);
}

DISAS_INSN(rotate16_im)
{
    TCGv reg;
    TCGv dest;
    TCGv shift;
    int tmp;

    reg = DREG(insn, 0);
    tmp = (insn >> 9) & 7;
    if (tmp == 0)
       tmp = 8;
    dest = tcg_temp_new_i32();
    shift = tcg_const_i32(tmp);
    if (insn & 8) {
       if (insn & 0x100) {
           gen_helper_rol16_cc(dest, cpu_env, reg, shift);
       } else {
           gen_helper_ror16_cc(dest, cpu_env, reg, shift);
       }
    } else {
        if (insn & 0x100) {
            gen_helper_roxl16_cc(dest, cpu_env, reg, shift);
        } else {
            gen_helper_roxr16_cc(dest, cpu_env, reg, shift);
        }
    }
    s->cc_op = CC_OP_FLAGS;
    gen_partset_reg(OS_WORD, reg, dest);
}

DISAS_INSN(rotate_reg)
{
    TCGv reg;
    TCGv src;
    TCGv tmp;

    reg = DREG(insn, 0);
    src = DREG(insn, 9);
    tmp = tcg_temp_new_i32();
    tcg_gen_andi_i32(tmp, src, 63);
    if (insn & 8) {
       if (insn & 0x100) {
           gen_helper_rol32_cc(reg, cpu_env, reg, tmp);
       } else {
           gen_helper_ror32_cc(reg, cpu_env, reg, tmp);
       }
    } else {
        if (insn & 0x100) {
            gen_helper_roxl32_cc(reg, cpu_env, reg, tmp);
        } else {
            gen_helper_roxr32_cc(reg, cpu_env, reg, tmp);
        }
    }
    s->cc_op = CC_OP_FLAGS;
}

DISAS_INSN(rotate8_reg)
{
    TCGv reg;
    TCGv src;
    TCGv dest;
    TCGv tmp;

    reg = DREG(insn, 0);
    src = DREG(insn, 9);
    tmp = tcg_temp_new_i32();
    tcg_gen_andi_i32(tmp, src, 63);
    dest = tcg_temp_new_i32();
    if (insn & 8) {
       if (insn & 0x100) {
           gen_helper_rol8_cc(dest, cpu_env, reg, tmp);
       } else {
           gen_helper_ror8_cc(dest, cpu_env, reg, tmp);
       }
    } else {
        if (insn & 0x100) {
            gen_helper_roxl8_cc(dest, cpu_env, reg, tmp);
        } else {
            gen_helper_roxr8_cc(dest, cpu_env, reg, tmp);
        }
    }
    s->cc_op = CC_OP_FLAGS;
    gen_partset_reg(OS_BYTE, reg, dest);
}

DISAS_INSN(rotate16_reg)
{
    TCGv reg;
    TCGv src;
    TCGv dest;
    TCGv tmp;

    reg = DREG(insn, 0);
    src = DREG(insn, 9);
    tmp = tcg_temp_new_i32();
    tcg_gen_andi_i32(tmp, src, 63);
    dest = tcg_temp_new_i32();
    if (insn & 8) {
       if (insn & 0x100) {
           gen_helper_rol16_cc(dest, cpu_env, reg, tmp);
       } else {
           gen_helper_ror16_cc(dest, cpu_env, reg, tmp);
       }
    } else {
        if (insn & 0x100) {
            gen_helper_roxl16_cc(dest, cpu_env, reg, tmp);
        } else {
            gen_helper_roxr16_cc(dest, cpu_env, reg, tmp);
        }
    }
    s->cc_op = CC_OP_FLAGS;
    gen_partset_reg(OS_WORD, reg, dest);
}

DISAS_INSN(rotate_mem)
{
    TCGv src;
    TCGv dest;
    TCGv addr;
    TCGv shift;

    SRC_EA(src, OS_WORD, 0, &addr);
    dest = tcg_temp_new_i32();
    shift = tcg_const_i32(1);
    if (insn & 8) {
       if (insn & 0x100) {
           gen_helper_rol16_cc(dest, cpu_env, src, shift);
       } else {
           gen_helper_ror16_cc(dest, cpu_env, src, shift);
       }
    } else {
        if (insn & 0x100) {
            gen_helper_roxl16_cc(dest, cpu_env, src, shift);
        } else {
            gen_helper_roxr16_cc(dest, cpu_env, src, shift);
        }
    }
    s->cc_op = CC_OP_FLAGS;
    DEST_EA(insn, OS_WORD, dest, &addr);
}

static void bitfield_param(uint16_t ext, TCGv *offset, TCGv *width, TCGv *mask)
{
    TCGv tmp;

    /* offset */

    if (ext & 0x0800) {
        *offset = tcg_temp_new_i32();
        tcg_gen_mov_i32(*offset, DREG(ext, 6));
    } else {
        *offset = tcg_temp_new_i32();
        tcg_gen_movi_i32(*offset, (ext >> 6) & 31);
    }

    /* width */

    if (ext & 0x0020) {
        *width = tcg_temp_new_i32();
        tcg_gen_subi_i32(*width, DREG(ext, 0), 1);
        tcg_gen_andi_i32(*width, *width, 31);
        tcg_gen_addi_i32(*width, *width, 1);
    } else {
        *width = tcg_temp_new_i32();
        tcg_gen_movi_i32(*width, ((ext - 1) & 31) + 1);
    }

    /* mask */

    tmp = tcg_temp_new_i32();
    tcg_gen_sub_i32(tmp, tcg_const_i32(32), *width);
    *mask = tcg_temp_new_i32();
    tcg_gen_shl_i32(*mask, tcg_const_i32(0xffffffff), tmp);
}

DISAS_INSN(bitfield_reg)
{
    uint16_t ext;
    TCGv tmp;
    TCGv tmp1;
    TCGv reg;
    TCGv offset;
    TCGv width;
    int op;
    TCGv reg2;
    TCGv mask;

    reg = DREG(insn, 0);
    op = (insn >> 8) & 7;
    ext = read_im16(s);

    bitfield_param(ext, &offset, &width, &mask);

    if (ext & 0x0800)
        tcg_gen_andi_i32(offset, offset, 31);
    gen_helper_ror32(mask, mask, offset);

    /* reg & mask */

    tmp = tcg_temp_new_i32();
    tcg_gen_and_i32(tmp, reg, mask);

    tmp1 = tcg_temp_new_i32();
    gen_helper_rol32(tmp1, tmp, offset);
    gen_logic_cc(s, tmp1, OS_LONG);

    reg2 = DREG(ext, 12);
    switch (op) {
    case 0: /* bftst */
        break;
    case 1: /* bfextu */
        tcg_gen_add_i32(tmp1, offset, width);
        tcg_gen_andi_i32(tmp1, tmp1, 31);
        gen_helper_rol32(reg2, tmp, tmp1);
        break;
    case 2: /* bfchg */
        tcg_gen_xor_i32(reg, reg, mask);
        break;
    case 3: /* bfexts */
        gen_helper_rol32(reg2, tmp, offset);
        tcg_gen_sub_i32(width, tcg_const_i32(32), width);
        tcg_gen_sar_i32(reg2, reg2, width);
        break;
    case 4: /* bfclr */
        tcg_gen_and_i32(reg, reg, mask);
        break;
    case 5: /* bfffo */
        gen_helper_rol32(reg2, tmp, offset);
        gen_helper_bfffo(tmp, tmp, width);
        tcg_gen_add_i32(reg2, tmp, offset);
        break;
    case 6: /* bfset */
        tcg_gen_or_i32(reg, reg, mask);
        break;
    case 7: /* bfins */
        tcg_gen_shl_i32(tmp1, tcg_const_i32(1), width);
        tcg_gen_subi_i32(tmp1, tmp1, 1);
        tcg_gen_and_i32(tmp, reg2, tmp1);
        tcg_gen_add_i32(tmp1, offset, width);
        tcg_gen_andi_i32(tmp1, tmp1, 31);
        gen_helper_ror32(tmp, tmp, tmp1);
        tcg_gen_not_i32(mask, mask);
        tcg_gen_and_i32(reg, reg, mask);
        tcg_gen_or_i32(reg, reg, tmp);
        break;
    }
}

static TCGv gen_bitfield_cc(DisasContext *s,
                            TCGv offset, TCGv mask_cc, TCGv_i64 bitfield)
{
    TCGv dest;
    TCGv_i64 tmp64;

    /* move bitfield to a 32bit */

    tmp64 = tcg_temp_new_i64();

    tcg_gen_extu_i32_i64(tmp64, offset);

    /* tmp64 = bitfield << offset */

    tcg_gen_shl_i64(tmp64, bitfield, tmp64);

    /* tmp = (bitfield << offset) >> 32 */

    tcg_gen_shri_i64(tmp64, bitfield, 32ULL);
    dest = tcg_temp_new_i32();
    tcg_gen_trunc_i64_i32(dest, tmp64);

    /* compute cc */

    tcg_gen_and_i32(dest, dest, mask_cc);
    gen_logic_cc(s, dest, OS_LONG);

    return dest;
}

static TCGv_i64 gen_bitfield_mask(TCGv offset, TCGv width)
{
    TCGv tmp;
    TCGv_i64 mask;
    TCGv_i64 shift;

    mask = tcg_temp_new_i64();

    /* mask = (1u << width) - 1; */

    tcg_gen_extu_i32_i64(mask, width);
    tcg_gen_shl_i64(mask, tcg_const_i64(1), mask);
    tcg_gen_subi_i64(mask, mask, 1);

    /* shift = 64 - (width + offset); */

    tmp = tcg_temp_new_i32();
    tcg_gen_add_i32(tmp, offset, width);
    tcg_gen_sub_i32(tmp, tcg_const_i32(64), tmp);
    shift = tcg_temp_new_i64();
    tcg_gen_extu_i32_i64(shift, tmp);

    /* mask <<= shift */

    tcg_gen_shl_i64(mask, mask, shift);

    return mask;
}

static void gen_bitfield_ins(TCGv offset, TCGv width, TCGv src,
                                 TCGv_i64 val)
{
    TCGv_i64 insert;
    TCGv_i64 shift;
    TCGv tmp;

    tmp = tcg_temp_new_i32();

    /* tmp = (1u << width) - 1; */

    tcg_gen_shl_i32(tmp, tcg_const_i32(1), width);
    tcg_gen_subi_i32(tmp, tmp, 1);

    /* tmp = tmp & src; */

    tcg_gen_and_i32(tmp, tmp, src);

    /* insert = (i64)tmp; */

    insert = tcg_temp_new_i64();
    tcg_gen_extu_i32_i64(insert, tmp);

    /* tmp = 64 - (width + offset); */

    tcg_gen_add_i32(tmp, offset, width);
    tcg_gen_sub_i32(tmp, tcg_const_i32(64), tmp);
    shift = tcg_temp_new_i64();
    tcg_gen_extu_i32_i64(shift, tmp);

    /* insert <<= shift */

    tcg_gen_shl_i64(insert, insert, shift);

    /* val |=  select */

    tcg_gen_or_i64(val, val, insert);
}

DISAS_INSN(bitfield_mem)
{
    uint16_t ext;
    int op;
    TCGv_i64 bitfield;
    TCGv_i64 mask_bitfield;
    TCGv mask_cc;
    TCGv shift;
    TCGv val;
    TCGv src;
    TCGv offset;
    TCGv width;
    TCGv reg;
    TCGv tmp;

    op = (insn >> 8) & 7;
    ext = read_im16(s);
    src = gen_lea(s, insn, OS_LONG);
    if (IS_NULL_QREG(src)) {
       gen_addr_fault(s);
       return;
    }

    bitfield_param(ext, &offset, &width, &mask_cc);

    /* adjust src and offset */

    /* src += offset >> 3; */

    tmp = tcg_temp_new_i32();
    tcg_gen_shri_i32(tmp, offset, 3);
    tcg_gen_add_i32(src, src, tmp);

    /* offset &= 7; */

    tcg_gen_andi_i32(offset, offset, 7);

    /* load */

    bitfield = tcg_temp_new_i64();
    gen_helper_bitfield_load(bitfield, src, offset, width);

    /* compute CC and move bitfield into a 32bit */

    val = gen_bitfield_cc(s, offset, mask_cc, bitfield);

    /* execute operation */

    reg = DREG(ext, 12);
    switch (op) {
    case 0: /* bftst */
        break;
    case 1: /* bfextu */
        shift = tcg_temp_new_i32();
        tcg_gen_sub_i32(shift, tcg_const_i32(32), width);
        tcg_gen_shr_i32(reg, val, shift);
        break;
    case 2: /* bfchg */
        mask_bitfield = gen_bitfield_mask(offset, width);
        tcg_gen_xor_i64(bitfield, bitfield, mask_bitfield);
        gen_helper_bitfield_store(src, offset, width, bitfield);
        break;
    case 3: /* bfexts */
        shift = tcg_temp_new_i32();
        tcg_gen_sub_i32(shift, tcg_const_i32(32), width);
        tcg_gen_sar_i32(reg, val, shift);
        break;
    case 4: /* bfclr */
        mask_bitfield = gen_bitfield_mask(offset, width);
        tcg_gen_not_i64(mask_bitfield, mask_bitfield);
        tcg_gen_and_i64(bitfield, bitfield, mask_bitfield);
        gen_helper_bitfield_store(src, offset, width, bitfield);
        break;
    case 5: /* bfffo */
        gen_helper_bfffo(val, val, width);
        tcg_gen_add_i32(reg, val, offset);
        break;
    case 6: /* bfset */
        mask_bitfield = gen_bitfield_mask(offset, width);
        tcg_gen_or_i64(bitfield, bitfield, mask_bitfield);
        gen_helper_bitfield_store(src, offset, width, bitfield);
        break;
    case 7: /* bfins */
        /* clear */
        mask_bitfield = gen_bitfield_mask(offset, width);
        tcg_gen_not_i64(mask_bitfield, mask_bitfield);
        tcg_gen_and_i64(bitfield, bitfield, mask_bitfield);
        /* insert */
        gen_bitfield_ins(offset, width, reg, bitfield);
        gen_helper_bitfield_store(src, offset, width, bitfield);
        break;
    }
}

DISAS_INSN(ff1)
{
    TCGv reg;
    reg = DREG(insn, 0);
    gen_logic_cc(s, reg, OS_LONG);
    gen_helper_ff1(reg, reg);
}

DISAS_INSN(strldsr)
{
    uint16_t ext;
    uint32_t addr;

    addr = s->pc - 2;
    ext = read_im16(s);
    if (ext != 0x46FC) {
        gen_exception(s, addr, EXCP_UNSUPPORTED);
        return;
    }
    ext = read_im16(s);
    if (IS_USER(s) || (ext & SR_S) == 0) {
        gen_exception(s, addr, EXCP_PRIVILEGE);
        return;
    }
    gen_push(s, gen_get_sr(s));
    gen_set_sr_im(s, ext, 0);
}

DISAS_INSN(move_from_sr)
{
    TCGv reg;
    TCGv sr;

    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }
    sr = gen_get_sr(s);
    reg = DREG(insn, 0);
    gen_partset_reg(OS_WORD, reg, sr);
}

DISAS_INSN(move_to_sr)
{
    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }
    gen_set_sr(s, insn, 0);
    gen_lookup_tb(s);
}

DISAS_INSN(move_from_usp)
{
    TCGv reg;

    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }
    reg = AREG(insn, 0);
    gen_helper_get_usp(reg, cpu_env);
}

DISAS_INSN(move_to_usp)
{
    TCGv reg;

    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }
    reg = AREG(insn, 0);
    gen_helper_set_usp(cpu_env, reg);
}

DISAS_INSN(halt)
{
    gen_exception(s, s->pc, EXCP_HALT_INSN);
}

DISAS_INSN(stop)
{
    uint16_t ext;

    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }

    ext = read_im16(s);

    gen_set_sr_im(s, ext, 0);
    tcg_gen_movi_i32(QREG_HALTED, 1);
    gen_exception(s, s->pc, EXCP_HLT);
}

DISAS_INSN(rte)
{
    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }
    gen_exception(s, s->pc - 2, EXCP_RTE);
}

DISAS_INSN(movec)
{
    uint16_t ext;
    TCGv reg;

    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }

    ext = read_im16(s);

    if (ext & 0x8000) {
        reg = AREG(ext, 12);
    } else {
        reg = DREG(ext, 12);
    }
    gen_helper_movec(cpu_env, tcg_const_i32(ext & 0xfff), reg);
    gen_lookup_tb(s);
}

DISAS_INSN(intouch)
{
    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }
    /* ICache fetch.  Implement as no-op.  */
}

DISAS_INSN(cpushl)
{
    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }
    /* Cache push/invalidate.  Implement as no-op.  */
}

DISAS_INSN(wddata)
{
    gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
}

DISAS_INSN(wdebug)
{
    if (IS_USER(s)) {
        gen_exception(s, s->pc - 2, EXCP_PRIVILEGE);
        return;
    }
    /* TODO: Implement wdebug.  */
    qemu_assert(0, "WDEBUG not implemented");
}

DISAS_INSN(trap)
{
    gen_exception(s, s->pc - 2, EXCP_TRAP0 + (insn & 0xf));
}

/* ??? FP exceptions are not implemented.  Most exceptions are deferred until
   immediately before the next FP instruction is executed.  */
DISAS_INSN(fpu)
{
    uint16_t ext;
    uint8_t rom_offset;
    int opmode;
    TCGv_i64 src;
    TCGv_i64 dest;
    TCGv_i64 res;
    TCGv tmp32;
    TCGv reg;
    int round;
    int set_dest;
    int opsize;

    ext = read_im16(s);
    opmode = ext & 0x7f;
    switch ((ext >> 13) & 7) {
    case 0:
        break;
    case 1:
        goto undef;
    case 2:
        if ( insn == 0xf200 && (ext & 0xfc00) == 0x5c00) {
            /* fmovecr */
            rom_offset = ext & 0x7f;
            dest = FREG(ext, 7);
            gen_helper_const_f64(dest, cpu_env, tcg_const_i32(rom_offset));
            return;
        }
        break;
    case 3: /* fmove out */
        src = FREG(ext, 7);
        tmp32 = tcg_temp_new_i32();
        /* fmove */
        /* ??? TODO: Proper behavior on overflow.  */
        switch ((ext >> 10) & 7) {
        case 0: opsize = OS_LONG; break;
        case 1: opsize = OS_SINGLE; break;
        case 4: opsize = OS_WORD; break;
        case 5: opsize = OS_DOUBLE; break;
        case 6: opsize = OS_BYTE; break;
        default:
            goto undef;
        }
        if (opsize == OS_DOUBLE) {
            tmp32 = gen_lea(s, insn, opsize);
            if (IS_NULL_QREG(tmp32)) {
                gen_addr_fault(s);
                return;
            }
            gen_store64(s, tmp32, src);
            if ( ((insn >> 3) & 7) == 3) { /* post-increment */
                reg = AREG(insn, 0);
                tcg_gen_addi_i32(reg, reg, opsize_bytes(opsize));
            }
        } else {
            switch (opsize) {
            case OS_LONG:
            case OS_WORD:
            case OS_BYTE:
                gen_helper_f64_to_i32(tmp32, cpu_env, src);
                break;
            case OS_SINGLE:
                gen_helper_f64_to_f32(tmp32, cpu_env, src);
                break;
            }
            DEST_EA(insn, opsize, tmp32, NULL);
        }
        tcg_temp_free_i32(tmp32);
        return;
    case 4: /* fmove to control register.  */
        switch ((ext >> 10) & 7) {
        case 4: /* FPCR */
            /* Not implemented.  Ignore writes.  */
            break;
        case 1: /* FPIAR */
        case 2: /* FPSR */
        default:
            cpu_abort(NULL, "Unimplemented: fmove to control %d",
                      (ext >> 10) & 7);
        }
        break;
    case 5: /* fmove from control register.  */
        switch ((ext >> 10) & 7) {
        case 4: /* FPCR */
            /* Not implemented.  Always return zero.  */
            tmp32 = tcg_const_i32(0);
            break;
        case 1: /* FPIAR */
        case 2: /* FPSR */
        default:
            cpu_abort(NULL, "Unimplemented: fmove from control %d",
                      (ext >> 10) & 7);
            goto undef;
        }
        DEST_EA(insn, OS_LONG, tmp32, NULL);
        break;
    case 6: /* fmovem */
    case 7:
        {
            TCGv addr;
            int incr;
            uint16_t mask;
            int i;
            if ((ext & 0xf00) != 0 || (ext & 0xff) == 0)
                goto undef;
            if ((ext & 0x1000) == 0 && !m68k_feature(s->env, M68K_FEATURE_FPU))
                goto undef;
            if ((insn & 070) == 040)
                tmp32 = AREG(insn, 0);
            else {
                tmp32 = gen_lea(s, insn, OS_LONG);
                 if (IS_NULL_QREG(tmp32)) {
                     gen_addr_fault(s);
                     return;
                 }
            }
            addr = tcg_temp_new_i32();
            tcg_gen_mov_i32(addr, tmp32);
            mask = 0x80;
            if (m68k_feature(s->env, M68K_FEATURE_FPU))
                incr = 12;
            else
                incr = 8;
            if ((ext & (1 << 13)) && (insn & 070) == 040) {
                for (i = 0; i < 8; i++) {
                    if (ext & mask) {
                        s->is_mem = 1;
                        dest = FREG(i, 7);
                        tcg_gen_subi_i32(addr, addr, incr);
                        tcg_gen_mov_i32(AREG(insn, 0), addr);
                        tcg_gen_qemu_stf64(dest, addr, IS_USER(s));
                    }
                    mask >>= 1;
                }
                tcg_temp_free_i32(addr);
                return;
            }
            for (i = 0; i < 8; i++) {
                if (ext & mask) {
                    s->is_mem = 1;
                    dest = FREG(i, 0);
                    if (ext & (1 << 13)) {
                        /* store */
                        tcg_gen_qemu_stf64(dest, addr, IS_USER(s));
                    } else {
                        /* load */
                        tcg_gen_qemu_ldf64(dest, addr, IS_USER(s));
                    }
                    if (ext & (mask - 1) || (insn & 070) == 030) {
                        tcg_gen_addi_i32(addr, addr, incr);
                        if ((insn & 070) == 030)
                            tcg_gen_mov_i32(AREG(insn, 0), addr);
                    }
                }
                mask >>= 1;
            }
            tcg_temp_free_i32(addr);
        }
        return;
    }
    if (ext & (1 << 14)) {
        /* Source effective address.  */
        switch ((ext >> 10) & 7) {
        case 0: opsize = OS_LONG; break;
        case 1: opsize = OS_SINGLE; break;
        case 4: opsize = OS_WORD; break;
        case 5: opsize = OS_DOUBLE; break;
        case 6: opsize = OS_BYTE; break;
        default:
            goto undef;
        }
        if (opsize == OS_DOUBLE) {
            if ((insn & 0x3f) == 0x3c) {
                src = gen_load64(s, tcg_const_i32(s->pc));
                s->pc += 8;
            } else {
                tmp32 = gen_lea(s, insn, opsize);
                if (IS_NULL_QREG(tmp32)) {
                    gen_addr_fault(s);
                    return;
                }
                src = gen_load64(s, tmp32);
                if ( ((insn >> 3) & 7) == 3) { /* post-increment */
                    reg = AREG(insn, 0);
                    tcg_gen_addi_i32(reg, reg, opsize_bytes(opsize));
                }
            }
        } else {
            SRC_EA(tmp32, opsize, 1, NULL);
            src = tcg_temp_new_i64();
            switch (opsize) {
            case OS_LONG:
            case OS_WORD:
            case OS_BYTE:
                gen_helper_i32_to_f64(src, cpu_env, tmp32);
                break;
            case OS_SINGLE:
                gen_helper_f32_to_f64(src, cpu_env, tmp32);
                break;
            }
        }
    } else {
        /* Source register.  */
        src = FREG(ext, 10);
    }
    dest = FREG(ext, 7);
    res = tcg_temp_new_i64();
    if (opmode != 0x3a)
        tcg_gen_mov_f64(res, dest);
    round = 1;
    set_dest = 1;
    switch (opmode) {
    case 0: case 0x40: case 0x44: /* fmove */
        tcg_gen_mov_f64(res, src);
        break;
    case 1: /* fint */
        gen_helper_iround_f64(res, cpu_env, src);
        round = 0;
        break;
    case 3: /* fintrz */
        gen_helper_itrunc_f64(res, cpu_env, src);
        round = 0;
        break;
    case 4: case 0x41: case 0x45: /* fsqrt */
        gen_helper_sqrt_f64(res, cpu_env, src);
        break;
    case 0x18: case 0x58: case 0x5c: /* fabs */
        gen_helper_abs_f64(res, src);
        break;
    case 0x1a: case 0x5a: case 0x5e: /* fneg */
        gen_helper_chs_f64(res, src);
        break;
    case 0x20: case 0x60: case 0x64: /* fdiv */
        gen_helper_div_f64(res, cpu_env, res, src);
        break;
    case 0x22: case 0x62: case 0x66: /* fadd */
        gen_helper_add_f64(res, cpu_env, res, src);
        break;
    case 0x23: case 0x63: case 0x67: /* fmul */
        gen_helper_mul_f64(res, cpu_env, res, src);
        break;
    case 0x24:                      /* fsgldiv */
        gen_helper_div_f64(res, cpu_env, res, src);
        break;
    case 0x27:                      /* fsglmul */ 
        gen_helper_mul_f64(res, cpu_env, res, src);
        break; 
    case 0x28: case 0x68: case 0x6c: /* fsub */
        gen_helper_sub_f64(res, cpu_env, res, src);
        break;
    case 0x38: /* fcmp */
        gen_helper_sub_cmp_f64(res, cpu_env, res, src);
        set_dest = 0;
        round = 0;
        break;
    case 0x3a: /* ftst */
        tcg_gen_mov_f64(res, src);
        set_dest = 0;
        round = 0;
        break;
    default:
        goto undef;
    }
    if (ext & (1 << 14)) {
        tcg_temp_free_i64(src);
    }
    if (round) {
        if (opmode & 0x40) {
            if ((opmode & 0x4) != 0)
                round = 0;
        } else if ((s->fpcr & M68K_FPCR_PREC) == 0) {
            round = 0;
        }
    }
    if (round) {
        TCGv tmp = tcg_temp_new_i32();
        gen_helper_f64_to_f32(tmp, cpu_env, res);
        gen_helper_f32_to_f64(res, cpu_env, tmp);
        tcg_temp_free_i32(tmp);
    }
    tcg_gen_mov_f64(QREG_FP_RESULT, res);
    if (set_dest) {
        tcg_gen_mov_f64(dest, res);
    }
    tcg_temp_free_i64(res);
    return;
undef:
    /* FIXME: Is this right for offset addressing modes?  */
    s->pc -= 2;
    disas_undef_fpu(s, insn);
}

static void gen_fjmpcc(DisasContext *s, int cond, int l1)
{
    TCGv flag;

    /* TODO: Raise BSUN exception.  */
    flag = tcg_temp_new();
    gen_helper_compare_f64(flag, cpu_env, QREG_FP_RESULT);
    /* Jump to l1 if condition is true.  */
    switch (cond) {
    case 0: /* f */
        break;
    case 1: /* eq (=0) */
        tcg_gen_brcond_i32(TCG_COND_EQ, flag, tcg_const_i32(0), l1);
        break;
    case 2: /* ogt (=1) */
        tcg_gen_brcond_i32(TCG_COND_EQ, flag, tcg_const_i32(1), l1);
        break;
    case 3: /* oge (=0 or =1) */
        tcg_gen_brcond_i32(TCG_COND_LEU, flag, tcg_const_i32(1), l1);
        break;
    case 4: /* olt (=-1) */
        tcg_gen_brcond_i32(TCG_COND_LT, flag, tcg_const_i32(0), l1);
        break;
    case 5: /* ole (=-1 or =0) */
        tcg_gen_brcond_i32(TCG_COND_LE, flag, tcg_const_i32(0), l1);
        break;
    case 6: /* ogl (=-1 or =1) */
        tcg_gen_andi_i32(flag, flag, 1);
        tcg_gen_brcond_i32(TCG_COND_NE, flag, tcg_const_i32(0), l1);
        break;
    case 7: /* or (=2) */
        tcg_gen_brcond_i32(TCG_COND_EQ, flag, tcg_const_i32(2), l1);
        break;
    case 8: /* un (<2) */
        tcg_gen_brcond_i32(TCG_COND_LT, flag, tcg_const_i32(2), l1);
        break;
    case 9: /* ueq (=0 or =2) */
        tcg_gen_andi_i32(flag, flag, 1);
        tcg_gen_brcond_i32(TCG_COND_EQ, flag, tcg_const_i32(0), l1);
        break;
    case 10: /* ugt (>0) */
        tcg_gen_brcond_i32(TCG_COND_GT, flag, tcg_const_i32(0), l1);
        break;
    case 11: /* uge (>=0) */
        tcg_gen_brcond_i32(TCG_COND_GE, flag, tcg_const_i32(0), l1);
        break;
    case 12: /* ult (=-1 or =2) */
        tcg_gen_brcond_i32(TCG_COND_GEU, flag, tcg_const_i32(2), l1);
        break;
    case 13: /* ule (!=1) */
        tcg_gen_brcond_i32(TCG_COND_NE, flag, tcg_const_i32(1), l1);
        break;
    case 14: /* ne (!=0) */
        tcg_gen_brcond_i32(TCG_COND_NE, flag, tcg_const_i32(0), l1);
        break;
    case 15: /* t */
        tcg_gen_br(l1);
        break;
    }
}

DISAS_INSN(fbcc)
{
    uint32_t offset;
    uint32_t addr;
    int l1;

    addr = s->pc;
    offset = ldsw_code(s->pc);
    s->pc += 2;
    if (insn & (1 << 6)) {
        offset = (offset << 16) | lduw_code(s->pc);
        s->pc += 2;
    }

    l1 = gen_new_label();
    gen_fjmpcc(s, insn & 0xf, l1);
    gen_jmp_tb(s, 0, s->pc);
    gen_set_label(l1);
    gen_jmp_tb(s, 1, addr + offset);
}

DISAS_INSN(fscc_mem)
{
    int l1, l2;
    TCGv taddr;
    TCGv addr;
    uint16_t ext;

    ext = read_im16(s);

    taddr = gen_lea(s, insn, OS_BYTE);
    if (IS_NULL_QREG(taddr)) {
        gen_addr_fault(s);
        return;
    }
    addr = tcg_temp_local_new ();
    tcg_gen_mov_i32(addr, taddr);
    l1 = gen_new_label();
    l2 = gen_new_label();
    gen_fjmpcc(s, ext & 0xf, l1);
    gen_store(s, OS_BYTE, addr, tcg_const_i32(0x00));
    tcg_gen_br(l2);
    gen_set_label(l1);
    gen_store(s, OS_BYTE, addr, tcg_const_i32(0xff));
    gen_set_label(l2);
    tcg_temp_free(addr);
}

DISAS_INSN(fscc_reg)
{
    int l1;
    TCGv reg;
    uint16_t ext;

    ext = read_im16(s);
    
    reg = DREG(insn, 0);

    l1 = gen_new_label();
    tcg_gen_ori_i32(reg, reg, 0x000000ff);
    gen_fjmpcc(s, ext & 0xf, l1);
    tcg_gen_andi_i32(reg, reg, 0xffffff00);
    gen_set_label(l1);
}

DISAS_INSN(frestore)
{
    /* TODO: Implement frestore.  */
    qemu_assert(0, "FRESTORE not implemented");
}

DISAS_INSN(fsave)
{
    /* TODO: Implement fsave.  */
    qemu_assert(0, "FSAVE not implemented");
}

static inline TCGv gen_mac_extract_word(DisasContext *s, TCGv val, int upper)
{
    TCGv tmp = tcg_temp_new();
    if (s->env->macsr & MACSR_FI) {
        if (upper)
            tcg_gen_andi_i32(tmp, val, 0xffff0000);
        else
            tcg_gen_shli_i32(tmp, val, 16);
    } else if (s->env->macsr & MACSR_SU) {
        if (upper)
            tcg_gen_sari_i32(tmp, val, 16);
        else
            tcg_gen_ext16s_i32(tmp, val);
    } else {
        if (upper)
            tcg_gen_shri_i32(tmp, val, 16);
        else
            tcg_gen_ext16u_i32(tmp, val);
    }
    return tmp;
}

static void gen_mac_clear_flags(void)
{
    tcg_gen_andi_i32(QREG_MACSR, QREG_MACSR,
                     ~(MACSR_V | MACSR_Z | MACSR_N | MACSR_EV));
}

DISAS_INSN(mac)
{
    TCGv rx;
    TCGv ry;
    uint16_t ext;
    int acc;
    TCGv tmp;
    TCGv addr;
    TCGv loadval;
    int dual;
    TCGv saved_flags;

    if (!s->done_mac) {
        s->mactmp = tcg_temp_new_i64();
        s->done_mac = 1;
    }

    ext = read_im16(s);

    acc = ((insn >> 7) & 1) | ((ext >> 3) & 2);
    dual = ((insn & 0x30) != 0 && (ext & 3) != 0);
    if (dual && !m68k_feature(s->env, M68K_FEATURE_CF_EMAC_B)) {
        disas_undef(s, insn);
        return;
    }
    if (insn & 0x30) {
        /* MAC with load.  */
        tmp = gen_lea(s, insn, OS_LONG);
        addr = tcg_temp_new();
        tcg_gen_and_i32(addr, tmp, QREG_MAC_MASK);
        /* Load the value now to ensure correct exception behavior.
           Perform writeback after reading the MAC inputs.  */
        loadval = gen_load(s, OS_LONG, addr, 0);

        acc ^= 1;
        rx = (ext & 0x8000) ? AREG(ext, 12) : DREG(insn, 12);
        ry = (ext & 8) ? AREG(ext, 0) : DREG(ext, 0);
    } else {
        loadval = addr = NULL_QREG;
        rx = (insn & 0x40) ? AREG(insn, 9) : DREG(insn, 9);
        ry = (insn & 8) ? AREG(insn, 0) : DREG(insn, 0);
    }

    gen_mac_clear_flags();
#if 0
    l1 = -1;
    /* Disabled because conditional branches clobber temporary vars.  */
    if ((s->env->macsr & MACSR_OMC) != 0 && !dual) {
        /* Skip the multiply if we know we will ignore it.  */
        l1 = gen_new_label();
        tmp = tcg_temp_new();
        tcg_gen_andi_i32(tmp, QREG_MACSR, 1 << (acc + 8));
        gen_op_jmp_nz32(tmp, l1);
    }
#endif

    if ((ext & 0x0800) == 0) {
        /* Word.  */
        rx = gen_mac_extract_word(s, rx, (ext & 0x80) != 0);
        ry = gen_mac_extract_word(s, ry, (ext & 0x40) != 0);
    }
    if (s->env->macsr & MACSR_FI) {
        gen_helper_macmulf(s->mactmp, cpu_env, rx, ry);
    } else {
        if (s->env->macsr & MACSR_SU)
            gen_helper_macmuls(s->mactmp, cpu_env, rx, ry);
        else
            gen_helper_macmulu(s->mactmp, cpu_env, rx, ry);
        switch ((ext >> 9) & 3) {
        case 1:
            tcg_gen_shli_i64(s->mactmp, s->mactmp, 1);
            break;
        case 3:
            tcg_gen_shri_i64(s->mactmp, s->mactmp, 1);
            break;
        }
    }

    if (dual) {
        /* Save the overflow flag from the multiply.  */
        saved_flags = tcg_temp_new();
        tcg_gen_mov_i32(saved_flags, QREG_MACSR);
    } else {
        saved_flags = NULL_QREG;
    }

#if 0
    /* Disabled because conditional branches clobber temporary vars.  */
    if ((s->env->macsr & MACSR_OMC) != 0 && dual) {
        /* Skip the accumulate if the value is already saturated.  */
        l1 = gen_new_label();
        tmp = tcg_temp_new();
        gen_op_and32(tmp, QREG_MACSR, tcg_const_i32(MACSR_PAV0 << acc));
        gen_op_jmp_nz32(tmp, l1);
    }
#endif

    if (insn & 0x100)
        tcg_gen_sub_i64(MACREG(acc), MACREG(acc), s->mactmp);
    else
        tcg_gen_add_i64(MACREG(acc), MACREG(acc), s->mactmp);

    if (s->env->macsr & MACSR_FI)
        gen_helper_macsatf(cpu_env, tcg_const_i32(acc));
    else if (s->env->macsr & MACSR_SU)
        gen_helper_macsats(cpu_env, tcg_const_i32(acc));
    else
        gen_helper_macsatu(cpu_env, tcg_const_i32(acc));

#if 0
    /* Disabled because conditional branches clobber temporary vars.  */
    if (l1 != -1)
        gen_set_label(l1);
#endif

    if (dual) {
        /* Dual accumulate variant.  */
        acc = (ext >> 2) & 3;
        /* Restore the overflow flag from the multiplier.  */
        tcg_gen_mov_i32(QREG_MACSR, saved_flags);
#if 0
        /* Disabled because conditional branches clobber temporary vars.  */
        if ((s->env->macsr & MACSR_OMC) != 0) {
            /* Skip the accumulate if the value is already saturated.  */
            l1 = gen_new_label();
            tmp = tcg_temp_new();
            gen_op_and32(tmp, QREG_MACSR, tcg_const_i32(MACSR_PAV0 << acc));
            gen_op_jmp_nz32(tmp, l1);
        }
#endif
        if (ext & 2)
            tcg_gen_sub_i64(MACREG(acc), MACREG(acc), s->mactmp);
        else
            tcg_gen_add_i64(MACREG(acc), MACREG(acc), s->mactmp);
        if (s->env->macsr & MACSR_FI)
            gen_helper_macsatf(cpu_env, tcg_const_i32(acc));
        else if (s->env->macsr & MACSR_SU)
            gen_helper_macsats(cpu_env, tcg_const_i32(acc));
        else
            gen_helper_macsatu(cpu_env, tcg_const_i32(acc));
#if 0
        /* Disabled because conditional branches clobber temporary vars.  */
        if (l1 != -1)
            gen_set_label(l1);
#endif
    }
    gen_helper_mac_set_flags(cpu_env, tcg_const_i32(acc));

    if (insn & 0x30) {
        TCGv rw;
        rw = (insn & 0x40) ? AREG(insn, 9) : DREG(insn, 9);
        tcg_gen_mov_i32(rw, loadval);
        /* FIXME: Should address writeback happen with the masked or
           unmasked value?  */
        switch ((insn >> 3) & 7) {
        case 3: /* Post-increment.  */
            tcg_gen_addi_i32(AREG(insn, 0), addr, 4);
            break;
        case 4: /* Pre-decrement.  */
            tcg_gen_mov_i32(AREG(insn, 0), addr);
        }
    }
}

DISAS_INSN(from_mac)
{
    TCGv rx;
    TCGv_i64 acc;
    int accnum;

    rx = (insn & 8) ? AREG(insn, 0) : DREG(insn, 0);
    accnum = (insn >> 9) & 3;
    acc = MACREG(accnum);
    if (s->env->macsr & MACSR_FI) {
        gen_helper_get_macf(rx, cpu_env, acc);
    } else if ((s->env->macsr & MACSR_OMC) == 0) {
        tcg_gen_trunc_i64_i32(rx, acc);
    } else if (s->env->macsr & MACSR_SU) {
        gen_helper_get_macs(rx, acc);
    } else {
        gen_helper_get_macu(rx, acc);
    }
    if (insn & 0x40) {
        tcg_gen_movi_i64(acc, 0);
        tcg_gen_andi_i32(QREG_MACSR, QREG_MACSR, ~(MACSR_PAV0 << accnum));
    }
}

DISAS_INSN(move_mac)
{
    /* FIXME: This can be done without a helper.  */
    int src;
    TCGv dest;
    src = insn & 3;
    dest = tcg_const_i32((insn >> 9) & 3);
    gen_helper_mac_move(cpu_env, dest, tcg_const_i32(src));
    gen_mac_clear_flags();
    gen_helper_mac_set_flags(cpu_env, dest);
}

DISAS_INSN(from_macsr)
{
    TCGv reg;

    reg = (insn & 8) ? AREG(insn, 0) : DREG(insn, 0);
    tcg_gen_mov_i32(reg, QREG_MACSR);
}

DISAS_INSN(from_mask)
{
    TCGv reg;
    reg = (insn & 8) ? AREG(insn, 0) : DREG(insn, 0);
    tcg_gen_mov_i32(reg, QREG_MAC_MASK);
}

DISAS_INSN(from_mext)
{
    TCGv reg;
    TCGv acc;
    reg = (insn & 8) ? AREG(insn, 0) : DREG(insn, 0);
    acc = tcg_const_i32((insn & 0x400) ? 2 : 0);
    if (s->env->macsr & MACSR_FI)
        gen_helper_get_mac_extf(reg, cpu_env, acc);
    else
        gen_helper_get_mac_exti(reg, cpu_env, acc);
}

DISAS_INSN(macsr_to_ccr)
{
    tcg_gen_movi_i32(QREG_CC_X, 0);
    tcg_gen_andi_i32(QREG_CC_DEST, QREG_MACSR, 0xf);
    s->cc_op = CC_OP_FLAGS;
}

DISAS_INSN(to_mac)
{
    TCGv_i64 acc;
    TCGv val;
    int accnum;
    accnum = (insn >> 9) & 3;
    acc = MACREG(accnum);
    SRC_EA(val, OS_LONG, 0, NULL);
    if (s->env->macsr & MACSR_FI) {
        tcg_gen_ext_i32_i64(acc, val);
        tcg_gen_shli_i64(acc, acc, 8);
    } else if (s->env->macsr & MACSR_SU) {
        tcg_gen_ext_i32_i64(acc, val);
    } else {
        tcg_gen_extu_i32_i64(acc, val);
    }
    tcg_gen_andi_i32(QREG_MACSR, QREG_MACSR, ~(MACSR_PAV0 << accnum));
    gen_mac_clear_flags();
    gen_helper_mac_set_flags(cpu_env, tcg_const_i32(accnum));
}

DISAS_INSN(to_macsr)
{
    TCGv val;
    SRC_EA(val, OS_LONG, 0, NULL);
    gen_helper_set_macsr(cpu_env, val);
    gen_lookup_tb(s);
}

DISAS_INSN(to_mask)
{
    TCGv val;
    SRC_EA(val, OS_LONG, 0, NULL);
    tcg_gen_ori_i32(QREG_MAC_MASK, val, 0xffff0000);
}

DISAS_INSN(to_mext)
{
    TCGv val;
    TCGv acc;
    SRC_EA(val, OS_LONG, 0, NULL);
    acc = tcg_const_i32((insn & 0x400) ? 2 : 0);
    if (s->env->macsr & MACSR_FI)
        gen_helper_set_mac_extf(cpu_env, val, acc);
    else if (s->env->macsr & MACSR_SU)
        gen_helper_set_mac_exts(cpu_env, val, acc);
    else
        gen_helper_set_mac_extu(cpu_env, val, acc);
}

static disas_proc opcode_table[65536];

static void
register_opcode (disas_proc proc, uint16_t opcode, uint16_t mask)
{
  int i;
  int from;
  int to;

  /* Sanity check.  All set bits must be included in the mask.  */
  if (opcode & ~mask) {
      fprintf(stderr,
              "qemu internal error: bogus opcode definition %04x/%04x\n",
              opcode, mask);
      abort();
  }
  /* This could probably be cleverer.  For now just optimize the case where
     the top bits are known.  */
  /* Find the first zero bit in the mask.  */
  i = 0x8000;
  while ((i & mask) != 0)
      i >>= 1;
  /* Iterate over all combinations of this and lower bits.  */
  if (i == 0)
      i = 1;
  else
      i <<= 1;
  from = opcode & ~(i - 1);
  to = from + i;
  for (i = from; i < to; i++) {
      if ((i & mask) == opcode)
          opcode_table[i] = proc;
  }
}

/* Register m68k opcode handlers.  Order is important.
   Later insn override earlier ones.  */
void register_m68k_insns (CPUM68KState *env)
{
#define INSN(name, opcode, mask, feature) do { \
    if (m68k_feature(env, M68K_FEATURE_##feature)) \
        register_opcode(disas_##name, 0x##opcode, 0x##mask); \
    } while(0)
    INSN(undef,     0000, 0000, CF_ISA_A);
    INSN(undef,     0000, 0000, M68000);
    INSN(arith_im,  0080, fff8, CF_ISA_A);
    INSN(arith_im,  0000, ff00, M68000);
    INSN(arith_im_sr,  003c, ffbf, M68000);
    INSN(undef,     00c0, ffc0, M68000);
    INSN(bitrev,    00c0, fff8, CF_ISA_APLUSC);
    INSN(bitop_reg, 0100, f1c0, CF_ISA_A);
    INSN(bitop_reg, 0100, f1c0, M68000);
    INSN(bitop_reg, 0140, f1c0, CF_ISA_A);
    INSN(bitop_reg, 0140, f1c0, M68000);
    INSN(bitop_reg, 0180, f1c0, CF_ISA_A);
    INSN(bitop_reg, 0180, f1c0, M68000);
    INSN(bitop_reg, 01c0, f1c0, CF_ISA_A);
    INSN(bitop_reg, 01c0, f1c0, M68000);
    INSN(arith_im,  0280, fff8, CF_ISA_A);
    INSN(arith_im,  0200, ff00, M68000);
    INSN(arith_im_sr,  023c, ffbf, M68000);
    INSN(undef,     02c0, ffc0, M68000);
    INSN(byterev,   02c0, fff8, CF_ISA_APLUSC);
    INSN(arith_im,  0480, fff8, CF_ISA_A);
    INSN(arith_im,  0400, ff00, M68000);
    INSN(undef,     04c0, ffc0, M68000);
    INSN(arith_im,  0600, ff00, M68000);
    INSN(arith_im_sr,  063c, ffbf, M68000);
    INSN(undef,     06c0, ffc0, M68000);
    INSN(ff1,       04c0, fff8, CF_ISA_APLUSC);
    INSN(arith_im,  0680, fff8, CF_ISA_A);
    INSN(bitop_im,  0800, ffc0, CF_ISA_A);
    INSN(bitop_im,  0800, ffc0, M68000);
    INSN(bitop_im,  0840, ffc0, CF_ISA_A);
    INSN(bitop_im,  0840, ffc0, M68000);
    INSN(bitop_im,  0880, ffc0, CF_ISA_A);
    INSN(bitop_im,  0880, ffc0, M68000);
    INSN(cas,       08c0, f9c0, CAS);
    INSN(bitop_im,  08c0, ffc0, CF_ISA_A);
    INSN(bitop_im,  08c0, ffc0, M68000);
    INSN(arith_im,  0a80, fff8, CF_ISA_A);
    INSN(arith_im,  0a00, ff00, M68000);
    INSN(undef,     0ac0, ffc0, M68000);
    INSN(arith_im,  0c00, ff38, CF_ISA_A);
    INSN(arith_im,  0c00, ff00, M68000);
    INSN(undef,     0cc0, ffc0, M68000);
    INSN(move,      1000, f000, CF_ISA_A);
    INSN(move,      1000, f000, M68000);
    INSN(move,      2000, f000, CF_ISA_A);
    INSN(move,      2000, f000, M68000);
    INSN(move,      3000, f000, CF_ISA_A);
    INSN(move,      3000, f000, M68000);
    INSN(strldsr,   40e7, ffff, CF_ISA_APLUSC);
    INSN(negx,      4080, fff8, CF_ISA_A);
    INSN(negx,      4000, ff00, M68000);
    INSN(undef,     40c0, ffc0, M68000);
    INSN(move_from_sr, 40c0, fff8, CF_ISA_A);
    INSN(move_from_sr, 40c0, ffc0, M68000);
    INSN(lea,       41c0, f1c0, CF_ISA_A);
    INSN(lea,       41c0, f1c0, M68000);
    INSN(clr,       4200, ff00, CF_ISA_A);
    INSN(clr,       4200, ff00, M68000);
    INSN(undef,     42c0, ffc0, CF_ISA_A);
    INSN(undef,     42c0, ffc0, M68000);
    INSN(move_from_ccr, 42c0, fff8, CF_ISA_A);
    INSN(neg,       4480, fff8, CF_ISA_A);
    INSN(neg,       4400, ff00, M68000);
    INSN(undef,     44c0, ffc0, M68000);
    INSN(move_to_ccr, 44c0, ffc0, CF_ISA_A);
    INSN(move_to_ccr, 44c0, ffc0, M68000);
    INSN(not,       4680, fff8, CF_ISA_A);
    INSN(not,       4600, ff00, M68000);
    INSN(undef,     46c0, ffc0, M68000);
    INSN(move_to_sr, 46c0, ffc0, CF_ISA_A);
    INSN(move_to_sr, 46c0, ffc0, M68000);
    INSN(nbcd,      4800, ffc0, M68000);
    INSN(linkl,     4808, fff8, M68000);
    INSN(pea,       4840, ffc0, CF_ISA_A);
    INSN(pea,       4840, ffc0, M68000);
    INSN(swap,      4840, fff8, CF_ISA_A);
    INSN(swap,      4840, fff8, M68000);
    INSN(movem,     48c0, fbc0, CF_ISA_A);
    INSN(movem,     48c0, fbc0, M68000);
    INSN(ext,       4880, fff8, CF_ISA_A);
    INSN(ext,       4880, fff8, M68000);
    INSN(ext,       48c0, fff8, CF_ISA_A);
    INSN(ext,       48c0, fff8, M68000);
    INSN(ext,       49c0, fff8, CF_ISA_A);
    INSN(ext,       49c0, fff8, M68000);
    INSN(tst,       4a00, ff00, CF_ISA_A);
    INSN(tst,       4a00, ff00, M68000);
    INSN(tas,       4ac0, ffc0, CF_ISA_B);
    INSN(tas,       4ac0, ffc0, M68000);
    INSN(halt,      4ac8, ffff, CF_ISA_A);
    INSN(pulse,     4acc, ffff, CF_ISA_A);
    INSN(illegal,   4afc, ffff, CF_ISA_A);
    INSN(illegal,   4afc, ffff, M68000);
    INSN(mull,      4c00, ffc0, CF_ISA_A);
    INSN(mull,      4c00, ffc0, LONG_MULDIV);
    INSN(divl,      4c40, ffc0, CF_ISA_A);
    INSN(divl,      4c40, ffc0, LONG_MULDIV);
    INSN(sats,      4c80, fff8, CF_ISA_B);
    INSN(trap,      4e40, fff0, CF_ISA_A);
    INSN(trap,      4e40, fff0, M68000);
    INSN(link,      4e50, fff8, CF_ISA_A);
    INSN(link,      4e50, fff8, M68000);
    INSN(unlk,      4e58, fff8, CF_ISA_A);
    INSN(unlk,      4e58, fff8, M68000);
    INSN(move_to_usp, 4e60, fff8, USP);
    INSN(move_from_usp, 4e68, fff8, USP);
    INSN(nop,       4e71, ffff, CF_ISA_A);
    INSN(nop,       4e71, ffff, M68000);
    INSN(stop,      4e72, ffff, CF_ISA_A);
    INSN(stop,      4e72, ffff, M68000);
    INSN(rte,       4e73, ffff, CF_ISA_A);
    INSN(rte,       4e73, ffff, M68000);
    INSN(rts,       4e75, ffff, CF_ISA_A);
    INSN(rts,       4e75, ffff, M68000);
    INSN(movec,     4e7b, ffff, CF_ISA_A);
    INSN(jump,      4e80, ffc0, CF_ISA_A);
    INSN(jump,      4e80, ffc0, M68000);
    INSN(jump,      4ec0, ffc0, CF_ISA_A);
    INSN(jump,      4ec0, ffc0, M68000);
    INSN(addsubq,   5080, f0c0, CF_ISA_A);
    INSN(addsubq,   5000, f080, M68000);
    INSN(addsubq,   5080, f0c0, M68000);
    INSN(scc,       50c0, f0f8, CF_ISA_A);
    INSN(scc_mem,   50c0, f0c0, M68000);
    INSN(scc,       50c0, f0f8, M68000);
    INSN(dbcc,      50c8, f0f8, M68000);
    INSN(tpf,       51f8, fff8, CF_ISA_A);

    /* Branch instructions.  */
    INSN(branch,    6000, f000, CF_ISA_A);
    INSN(branch,    6000, f000, M68000);
    /* Disable long branch instructions, then add back the ones we want.  */
    INSN(undef,     60ff, f0ff, CF_ISA_A); /* All long branches.  */
    INSN(undef,     60ff, f0ff, M68000); /* All long branches.  */
    INSN(branch,    60ff, f0ff, CF_ISA_B);
    INSN(undef,     60ff, ffff, CF_ISA_B); /* bra.l */
    INSN(branch,    60ff, ffff, BRAL);
    INSN(branch,    60ff, f0ff, BCCL);

    INSN(moveq,     7000, f100, CF_ISA_A);
    INSN(moveq,     7000, f100, M68000);
    INSN(mvzs,      7100, f100, CF_ISA_B);
    INSN(or,        8000, f000, CF_ISA_A);
    INSN(or,        8000, f000, M68000);
    INSN(divw,      80c0, f0c0, CF_ISA_A);
    INSN(divw,      80c0, f0c0, M68000);
    INSN(sbcd_reg,  8100, f1f8, M68000);
    INSN(sbcd_mem,  8108, f1f8, M68000);
    INSN(addsub,    9000, f000, CF_ISA_A);
    INSN(addsub,    9000, f000, M68000);
    INSN(undef,     90c0, f0c0, CF_ISA_A);
    INSN(subx_reg,  9180, f1f8, CF_ISA_A);
    INSN(subx_reg,  9100, f138, M68000);
    INSN(subx_mem,  9108, f138, M68000);
    INSN(suba,      91c0, f1c0, CF_ISA_A);
    INSN(suba,      90c0, f0c0, M68000);

    INSN(undef_mac, a000, f000, CF_ISA_A);
    INSN(undef_mac, a000, f000, M68000);
    INSN(mac,       a000, f100, CF_EMAC);
    INSN(from_mac,  a180, f9b0, CF_EMAC);
    INSN(move_mac,  a110, f9fc, CF_EMAC);
    INSN(from_macsr,a980, f9f0, CF_EMAC);
    INSN(from_mask, ad80, fff0, CF_EMAC);
    INSN(from_mext, ab80, fbf0, CF_EMAC);
    INSN(macsr_to_ccr, a9c0, ffff, CF_EMAC);
    INSN(to_mac,    a100, f9c0, CF_EMAC);
    INSN(to_macsr,  a900, ffc0, CF_EMAC);
    INSN(to_mext,   ab00, fbc0, CF_EMAC);
    INSN(to_mask,   ad00, ffc0, CF_EMAC);

    INSN(mov3q,     a140, f1c0, CF_ISA_B);
    INSN(cmp,       b000, f1c0, CF_ISA_B); /* cmp.b */
    INSN(cmp,       b040, f1c0, CF_ISA_B); /* cmp.w */
    INSN(cmpa,      b0c0, f1c0, CF_ISA_B); /* cmpa.w */
    INSN(cmp,       b080, f1c0, CF_ISA_A);
    INSN(cmpa,      b1c0, f1c0, CF_ISA_A);
    INSN(cmp,       b000, f100, M68000);
    INSN(eor,       b100, f100, M68000);
    INSN(cmpm,      b108, f138, M68000);
    INSN(cmpa,      b0c0, f0c0, M68000);
    INSN(eor,       b180, f1c0, CF_ISA_A);
    INSN(and,       c000, f000, CF_ISA_A);
    INSN(and,       c000, f000, M68000);
    INSN(exg,       c100, f130, M68000);
    INSN(mulw,      c0c0, f0c0, CF_ISA_A);
    INSN(mulw,      c0c0, f0c0, M68000);
    INSN(abcd_reg,  c100, f1f8, M68000);
    INSN(abcd_mem,  c108, f1f8, M68000);
    INSN(addsub,    d000, f000, CF_ISA_A);
    INSN(addsub,    d000, f000, M68000);
    INSN(undef,     d0c0, f0c0, CF_ISA_A);
    INSN(addx_reg,      d180, f1f8, CF_ISA_A);
    INSN(addx_reg,  d100, f138, M68000);
    INSN(addx_mem,  d108, f138, M68000);
    INSN(adda,      d1c0, f1c0, CF_ISA_A);
    INSN(adda,      d0c0, f0c0, M68000);
    INSN(shift_im,  e080, f0f0, CF_ISA_A);
    INSN(shift_reg, e0a0, f0f0, CF_ISA_A);
    INSN(shift8_im, e000, f0f0, M68000);
    INSN(shift16_im, e040, f0f0, M68000);
    INSN(shift_im,  e080, f0f0, M68000);
    INSN(shift8_reg, e020, f0f0, M68000);
    INSN(shift16_reg, e060, f0f0, M68000);
    INSN(shift_reg, e0a0, f0f0, M68000);
    INSN(shift_mem, e0c0, fcc0, M68000);
    INSN(rotate_im, e090, f0f0, M68000);
    INSN(rotate8_im, e010, f0f0, M68000);
    INSN(rotate16_im, e050, f0f0, M68000);
    INSN(rotate_reg, e0b0, f0f0, M68000);
    INSN(rotate8_reg, e030, f0f0, M68000);
    INSN(rotate16_reg,e070, f0f0, M68000);
    INSN(rotate_mem, e4c0, fcc0, M68000);
    INSN(bitfield_mem,e8c0, f8c0, BITFIELD);
    INSN(bitfield_reg,e8c0, f8f8, BITFIELD);
    INSN(undef_fpu, f000, f000, CF_ISA_A);
    INSN(undef_fpu, f000, f000, M68000);
    INSN(fpu,       f200, ffc0, CF_FPU);
    INSN(fbcc,      f280, ffc0, CF_FPU);
    INSN(frestore,  f340, ffc0, CF_FPU);
    INSN(fsave,     f340, ffc0, CF_FPU);
    INSN(fpu,       f200, ffc0, FPU);
    INSN(fscc_mem,  f240, ffc0, FPU);
    INSN(fscc_reg,  f240, fff8, FPU);
    INSN(fbcc,      f280, ffc0, FPU);
    INSN(frestore,  f340, ffc0, FPU);
    INSN(fsave,     f340, ffc0, FPU);
    INSN(intouch,   f340, ffc0, CF_ISA_A);
    INSN(cpushl,    f428, ff38, CF_ISA_A);
    INSN(wddata,    fb00, ff00, CF_ISA_A);
    INSN(wdebug,    fbc0, ffc0, CF_ISA_A);
#undef INSN
}

/* ??? Some of this implementation is not exception safe.  We should always
   write back the result to memory before setting the condition codes.  */
static void disas_m68k_insn(CPUState * env, DisasContext *s)
{
    uint16_t insn;

    if (unlikely(qemu_loglevel_mask(CPU_LOG_TB_OP)))
        tcg_gen_debug_insn_start(s->pc);

    insn = read_im16(s);

    opcode_table[insn](s, insn);
}

/* generate intermediate code for basic block 'tb'.  */
static inline void
gen_intermediate_code_internal(CPUState *env, TranslationBlock *tb,
                               int search_pc)
{
    DisasContext dc1, *dc = &dc1;
    uint16_t *gen_opc_end;
    CPUBreakpoint *bp;
    int j, lj;
    target_ulong pc_start;
    int pc_offset;
    int num_insns;
    int max_insns;

    /* generate intermediate code */
    pc_start = tb->pc;

    dc->tb = tb;

    gen_opc_end = gen_opc_buf + OPC_MAX_SIZE;

    dc->env = env;
    dc->is_jmp = DISAS_NEXT;
    dc->pc = pc_start;
    dc->cc_op = CC_OP_DYNAMIC;
    dc->singlestep_enabled = env->singlestep_enabled;
    dc->fpcr = env->fpcr;
    dc->user = (env->sr & SR_S) == 0;
    dc->is_mem = 0;
    dc->done_mac = 0;
    lj = -1;
    num_insns = 0;
    max_insns = tb->cflags & CF_COUNT_MASK;
    if (max_insns == 0)
        max_insns = CF_COUNT_MASK;

    gen_icount_start();
    do {
        pc_offset = dc->pc - pc_start;
        gen_throws_exception = NULL;
        if (unlikely(!QTAILQ_EMPTY(&env->breakpoints))) {
            QTAILQ_FOREACH(bp, &env->breakpoints, entry) {
                if (bp->pc == dc->pc) {
                    gen_exception(dc, dc->pc, EXCP_DEBUG);
                    dc->is_jmp = DISAS_JUMP;
                    break;
                }
            }
            if (dc->is_jmp)
                break;
        }
        if (search_pc) {
            j = gen_opc_ptr - gen_opc_buf;
            if (lj < j) {
                lj++;
                while (lj < j)
                    gen_opc_instr_start[lj++] = 0;
            }
            gen_opc_pc[lj] = dc->pc;
            gen_opc_instr_start[lj] = 1;
            gen_opc_icount[lj] = num_insns;
        }
        if (num_insns + 1 == max_insns && (tb->cflags & CF_LAST_IO))
            gen_io_start();
        dc->insn_pc = dc->pc;
	disas_m68k_insn(env, dc);
        num_insns++;
    } while (!dc->is_jmp && gen_opc_ptr < gen_opc_end &&
             !env->singlestep_enabled &&
             !singlestep &&
             (pc_offset) < (TARGET_PAGE_SIZE - 32) &&
             num_insns < max_insns);

    if (tb->cflags & CF_LAST_IO)
        gen_io_end();
    if (unlikely(env->singlestep_enabled)) {
        /* Make sure the pc is updated, and raise a debug exception.  */
        if (!dc->is_jmp) {
            gen_flush_cc_op(dc);
            tcg_gen_movi_i32(QREG_PC, dc->pc);
        }
        gen_helper_raise_exception(tcg_const_i32(EXCP_DEBUG));
    } else {
        switch(dc->is_jmp) {
        case DISAS_NEXT:
            gen_flush_cc_op(dc);
            gen_jmp_tb(dc, 0, dc->pc);
            break;
        default:
        case DISAS_JUMP:
        case DISAS_UPDATE:
            gen_flush_cc_op(dc);
            /* indicate that the hash table must be used to find the next TB */
            tcg_gen_exit_tb(0);
            break;
        case DISAS_TB_JUMP:
            /* nothing more to generate */
            break;
        }
    }
    gen_icount_end(tb, num_insns);
    *gen_opc_ptr = INDEX_op_end;

#ifdef DEBUG_DISAS
    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
        qemu_log("----------------\n");
        qemu_log("IN: %s\n", lookup_symbol(pc_start));
        log_target_disas(pc_start, dc->pc - pc_start, 0);
        qemu_log("\n");
    }
#endif
    if (search_pc) {
        j = gen_opc_ptr - gen_opc_buf;
        lj++;
        while (lj <= j)
            gen_opc_instr_start[lj++] = 0;
    } else {
        tb->size = dc->pc - pc_start;
        tb->icount = num_insns;
    }

    //optimize_flags();
    //expand_target_qops();
}

void gen_intermediate_code(CPUState *env, TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, 0);
}

void gen_intermediate_code_pc(CPUState *env, TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, 1);
}

void cpu_dump_state(CPUState *env, FILE *f, fprintf_function cpu_fprintf,
                    int flags)
{
    int i;
    uint16_t sr;
    CPU_DoubleU u;
    for (i = 0; i < 8; i++)
      {
        u.d = env->fregs[i];
        cpu_fprintf (f, "D%d = %08x   A%d = %08x   F%d = %08x%08x (%12g)\n",
                     i, env->dregs[i], i, env->aregs[i],
                     i, u.l.upper, u.l.lower, *(double *)&u.d);
      }
    cpu_fprintf (f, "PC = %08x   ", env->pc);
    sr = env->sr;
    cpu_fprintf (f, "SR = %04x %c%c%c%c%c ", sr, (sr & 0x10) ? 'X' : '-',
                 (sr & CCF_N) ? 'N' : '-', (sr & CCF_Z) ? 'Z' : '-',
                 (sr & CCF_V) ? 'V' : '-', (sr & CCF_C) ? 'C' : '-');
    cpu_fprintf (f, "FPRESULT = %12g\n", *(double *)&env->fp_result);
}

void restore_state_to_opc(CPUState *env, TranslationBlock *tb, int pc_pos)
{
    env->pc = gen_opc_pc[pc_pos];
}
