/*
 *  M68K helper routines
 *
 *  Copyright (c) 2007 CodeSourcery
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include "exec.h"
#include "helpers.h"

#if defined(CONFIG_USER_ONLY)

void do_interrupt(CPUState *env1)
{
    env1->exception_index = -1;
}

void do_interrupt_m68k_hardirq(CPUState *env1)
{
}

#else

extern int semihosting_enabled;

#define MMUSUFFIX _mmu

#define SHIFT 0
#include "softmmu_template.h"

#define SHIFT 1
#include "softmmu_template.h"

#define SHIFT 2
#include "softmmu_template.h"

#define SHIFT 3
#include "softmmu_template.h"

/* Try to fill the TLB and return an exception if error. If retaddr is
   NULL, it means that the function was called in C code (i.e. not
   from generated code or from helper.c) */
/* XXX: fix it to restore all registers */
void tlb_fill (target_ulong addr, int is_write, int mmu_idx, void *retaddr)
{
    TranslationBlock *tb;
    CPUState *saved_env;
    unsigned long pc;
    int ret;

    /* XXX: hack to restore env in all cases, even if not called from
       generated code */
    saved_env = env;
    env = cpu_single_env;
    ret = cpu_m68k_handle_mmu_fault(env, addr, is_write, mmu_idx, 1);
    if (unlikely(ret)) {
        if (retaddr) {
            /* now we have a real cpu fault */
            pc = (unsigned long)retaddr;
            tb = tb_find_pc(pc);
            if (tb) {
                /* the PC is inside the translated code. It means that we have
                   a virtual CPU fault */
                cpu_restore_state(tb, env, pc);
            }
        }
        cpu_loop_exit(env);
    }
    env = saved_env;
}

static void do_rte(void)
{
    uint32_t sp;
    uint32_t fmt;
    int sr_size;

    sp = env->aregs[7];
    /* XXX Not valid for 68010+ */
    if (m68k_feature(env, M68K_FEATURE_M68000)) {
        sr_size = 2;
        fmt = lduw_kernel(sp);
    } else {
        sr_size = 4;
        fmt = ldl_kernel(sp);
    }
    env->pc = ldl_kernel(sp + sr_size);
    sp |= (fmt >> 28) & 3;
    env->sr = fmt & 0xffff;
    env->aregs[7] = sp + sr_size + 4;
    m68k_switch_sp(env);
}

static void do_interrupt_all(int is_hw)
{
    uint32_t sp;
    uint32_t sr;
    uint32_t fmt;
    uint32_t retaddr;
    uint32_t vector;

    fmt = 0;
    retaddr = env->pc;

    if (!is_hw) {
        switch (env->exception_index) {
        case EXCP_RTE:
            /* Return from an exception.  */
            do_rte();
            return;
        case EXCP_HALT_INSN:
            if (semihosting_enabled
                    && (env->sr & SR_S) != 0
                    && (env->pc & 3) == 0
                    && lduw_code(env->pc - 4) == 0x4e71
                    && ldl_code(env->pc) == 0x4e7bf000) {
                env->pc += 4;
                do_m68k_semihosting(env, env->dregs[0]);
                return;
            }
            env->halted = 1;
            env->exception_index = EXCP_HLT;
            cpu_loop_exit(env);
            return;
        }
        if (env->exception_index >= EXCP_TRAP0
            && env->exception_index <= EXCP_TRAP15) {
            /* Move the PC after the trap instruction.  */
            retaddr += 2;
        }
    }

    vector = env->exception_index << 2;

    sp = env->aregs[7];

    fmt |= 0x40000000;
    fmt |= (sp & 3) << 28;
    fmt |= vector << 16;
    fmt |= env->sr;

    sr = env->sr;
    env->sr |= SR_S;
    if (is_hw) {
        env->sr = (env->sr & ~SR_I) | (env->pending_level << SR_I_SHIFT);
        env->sr &= ~SR_M;
    }
    m68k_switch_sp(env);

    if (m68k_feature(env, M68K_FEATURE_M68000))
        sp = env->aregs[7]; //Put frame on supervisor stack
    else
        sp &= ~3;
    /* ??? This could cause MMU faults.  */
    /* XXX Not valid for 68010+ */
    if (m68k_feature(env, M68K_FEATURE_M68000)) {
        sp -= 4;
        stl_kernel(sp, retaddr);
        sp -= 2;
        stw_kernel(sp, sr);
    } else {
        sp -= 4;
        stl_kernel(sp, retaddr);
        sp -= 4;
        stl_kernel(sp, fmt);
    }
    env->aregs[7] = sp;
    /* Jump to vector.  */
    env->pc = ldl_kernel(env->vbr + vector);
}

void do_interrupt(CPUState *env1)
{
    CPUState *saved_env;

    saved_env = env;
    env = env1;
    do_interrupt_all(0);
    env = saved_env;
}

void do_interrupt_m68k_hardirq(CPUState *env1)
{
    CPUState *saved_env;

    saved_env = env;
    env = env1;
    do_interrupt_all(1);
    env = saved_env;
}
#endif

static void raise_exception(int tt)
{
    env->exception_index = tt;
    cpu_loop_exit(env);
}

void HELPER(raise_exception)(uint32_t tt)
{
    raise_exception(tt);
}

void HELPER(divu)(CPUState *env, uint32_t word)
{
    uint32_t num;
    uint32_t den;
    uint32_t quot;
    uint32_t rem;
    uint32_t flags;

    num = env->div1;
    den = env->div2;
    /* ??? This needs to make sure the throwing location is accurate.  */
    if (den == 0)
        raise_exception(EXCP_DIV0);
    quot = num / den;
    rem = num % den;
    flags = 0;
    /* Avoid using a PARAM1 of zero.  This breaks dyngen because it uses
       the address of a symbol, and gcc knows symbols can't have address
       zero.  */
    if (word && quot > 0xffff) {
	/* real 68040 keep Z and N on overflow,
         * whereas documentation says "undefined"
         */
        flags |= CCF_V | (env->cc_dest & (CCF_Z|CCF_N));
    } else {
        if (quot == 0)
            flags |= CCF_Z;
        else if ((int16_t)quot < 0)
            flags |= CCF_N;
    }

    env->div1 = quot;
    env->div2 = rem;
    env->cc_dest = flags;
}

void HELPER(divs)(CPUState *env, uint32_t word)
{
    int32_t num;
    int32_t den;
    int32_t quot;
    int32_t rem;
    int32_t flags;

    num = env->div1;
    den = env->div2;
    if (den == 0)
        raise_exception(EXCP_DIV0);
    quot = num / den;
    rem = num % den;
    flags = 0;
    if (word && quot != (int16_t)quot) {
	/* real 68040 keep Z and N on overflow,
         * whereas documentation says "undefined"
         */
        flags |= CCF_V | (env->cc_dest & (CCF_Z|CCF_N));
    } else {
        if (quot == 0)
            flags |= CCF_Z;
        else if ((int16_t)quot < 0)
            flags |= CCF_N;
    }

    env->div1 = quot;
    env->div2 = rem;
    env->cc_dest = flags;
}

void HELPER(divu64)(CPUState *env)
{
    uint32_t num;
    uint32_t den;
    uint32_t quot;
    uint32_t rem;
    uint32_t flags;

    num = env->div1;
    den = env->div2;
    /* ??? This needs to make sure the throwing location is accurate.  */
    if (den == 0)
        raise_exception(EXCP_DIV0);
    quot = (num | ((uint64_t)env->quadh << 32)) / den;
    rem = (num | ((uint64_t)env->quadh << 32)) % den;
    flags = 0;
    /* Avoid using a PARAM1 of zero.  This breaks dyngen because it uses
       the address of a symbol, and gcc knows symbols can't have address
       zero.  */
    if (quot > 0xffffffff)
        flags |= CCF_V;
    if (quot == 0)
        flags |= CCF_Z;
    else if ((int64_t)quot < 0)
        flags |= CCF_N;
    /* Don't modify destination if overflow occured.  */
    if ((flags & CCF_V) == 0) {
        env->div1 = quot;
        env->div2 = rem;
    }
    env->cc_dest = flags;
}

void HELPER(divs64)(CPUState *env)
{
    int32_t num;
    int32_t den;
    int32_t quot;
    int32_t rem;
    int32_t flags;

    num = env->div1;
    den = env->div2;
    if (den == 0)
        raise_exception(EXCP_DIV0);
    quot = (num | ((int64_t)env->quadh << 32)) / den;
    rem = (num | ((int64_t)env->quadh << 32)) % den;
    rem = num % den;
    flags = 0;
    if (quot != (int32_t)quot)
        flags |= CCF_V;
    if (quot == 0)
        flags |= CCF_Z;
    else if (quot < 0)
        flags |= CCF_N;
    /* Don't modify destination if overflow occured.  */
    if ((flags & CCF_V) == 0) {
        env->div1 = quot;
        env->div2 = rem;
    }
    env->cc_dest = flags;
}

uint32_t HELPER(mulu32_cc)(CPUState *env, uint32_t op1, uint32_t op2)
{
    uint64_t res = (uint32_t)op1 * op2;
    uint32_t flags;

    flags = 0;
    if (res >> 32)
       flags |= CCF_V;
    if ((uint32_t)res == 0)
       flags |= CCF_Z;
    if ((int32_t)res < 0)
       flags |= CCF_N;
    env->cc_dest = flags;

    return res;
}

uint32_t HELPER(muls32_cc)(CPUState *env, uint32_t op1, uint32_t op2)
{
    int64_t res = (int32_t)op1 * (int32_t)op2;
    uint32_t flags;

    flags = 0;
    if (res != (int64_t)(int32_t)res)
       flags |= CCF_V;
    if ((uint32_t)res == 0)
       flags |= CCF_Z;
    if ((int32_t)res < 0)
       flags |= CCF_N;
    env->cc_dest = flags;

    return res;
}

uint32_t HELPER(mulu64)(CPUState *env, uint32_t op1, uint32_t op2)
{
    uint64_t res = (uint64_t)op1 * op2;
    uint32_t flags;

    env->quadh = res >> 32;
    flags = 0;
    if (res == 0)
       flags |= CCF_Z;
    if ((int64_t)res < 0)
       flags |= CCF_N;
    env->cc_dest = flags;

    return res;
}

uint32_t HELPER(muls64)(CPUState *env, uint32_t op1, uint32_t op2)
{
    int64_t res = (uint64_t)(int32_t)op1 * (int32_t)op2;
    uint32_t flags;

    env->quadh = res >> 32;
    flags = 0;
    if (res == 0)
       flags |= CCF_Z;
    if (res < 0)
       flags |= CCF_N;
    env->cc_dest = flags;

    return res;
}
