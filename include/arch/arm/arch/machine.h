/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <machine.h>
#include <plat/machine/hardware.h>
#include <arch/types.h>
#include <util.h>

#ifndef __ASSEMBLER__

void map_kernel_devices(void);

void initL2Cache(void);

struct plat_getIRQTarget_ret {
    exception_t status;
    uint8_t target;
};
typedef struct plat_getIRQTarget_ret plat_getIRQTarget_ret_t;

void initIRQController(void);
void cpu_initLocalIRQController(void);
void plat_setIRQTrigger(irq_t irq, bool_t trigger);
void plat_setIRQTarget(irq_t irq, seL4_Word target);


/**
 *  The idea behind getIRQTarget is that we can read this to see if the target
 *  is us. In an ideal word, the user is smart enough to not mismanage the IRQs.
 *  The kernel only cares about PPI/SGI IRQs, and these don't touch the GICD
 *  interface. For masking interrupts, we'll only do those operations if
 *  the target is set correctly. If someone wants to exploit a TOCTOU issue
 *  where the target is changed between us checking it and reading it later,
 *  then sure, they can deal with maybe getting there IRQs overwritten.
 *  From the seL4 side, only one kernel has permissions to set the target core.
 *
 *  The GIC spec (at least the v3 version) specifies that reads/writes are
 *  atomic, so it's fine from that perspective.
 */

plat_getIRQTarget_ret_t plat_getIRQTarget(irq_t irq);
bool_t plat_SGITargetValid(word_t target);
void plat_sendSGI(word_t irq, word_t target);

static inline void plat_cleanL2Range(paddr_t start, paddr_t end);
static inline void plat_invalidateL2Range(paddr_t start, paddr_t end);
static inline void plat_cleanInvalidateL2Range(paddr_t start, paddr_t end);
static inline void plat_cleanInvalidateL2Cache(void);

void cleanInvalidateCacheRange_RAM(word_t start, word_t end, paddr_t pstart);
void cleanCacheRange_RAM(word_t start, word_t end, paddr_t pstart);
void cleanCacheRange_PoU(word_t start, word_t end, paddr_t pstart);
void invalidateCacheRange_RAM(word_t start, word_t end, paddr_t pstart);
void invalidateCacheRange_I(word_t start, word_t end, paddr_t pstart);
void branchFlushRange(word_t start, word_t end, paddr_t pstart);

void clean_D_PoU(void);
void cleanInvalidate_D_PoC(void);
void cleanInvalidate_L1D(void);
void cleanCaches_PoU(void);
void cleanInvalidateL1Caches(void);

/* Cleaning memory before user-level access. Does not flush cache. */
static inline void clearMemory(word_t *ptr, word_t bits)
{
    memzero(ptr, BIT(bits));
}

/* Cleaning memory before page table walker access */
static inline void clearMemory_PT(word_t *ptr, word_t bits)
{
    memzero(ptr, BIT(bits));
    cleanCacheRange_PoU((word_t)ptr, (word_t)ptr + BIT(bits) - 1,
                        addrFromPPtr(ptr));
}

#ifdef ENABLE_SMP_SUPPORT
static inline void arch_pause(void)
{
    /* TODO */
}
#endif /* ENABLE_SMP_SUPPORT */

/* Update the value of the actual register to hold the expected value */
static inline exception_t Arch_setTLSRegister(word_t tls_base)
{
    /* This register is saved and restored on kernel exit and entry so
     * we only update it in the saved context. */
    setRegister(NODE_STATE(ksCurThread), TLS_BASE, tls_base);
    return EXCEPTION_NONE;
}

#endif /* __ASSEMBLER__ */
