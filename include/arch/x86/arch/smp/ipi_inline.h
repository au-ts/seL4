/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <smp/ipi.h>

#ifdef ENABLE_SMP_SUPPORT
static inline void doRemoteStall(word_t cpu)
{
    doRemoteOp0Arg(IpiRemoteCall_Stall, cpu);
}

static inline void doRemoteswitchFpuOwner(user_fpu_state_t *new_owner, word_t cpu)
{
    doRemoteOp1Arg(IpiRemoteCall_switchFpuOwner, (word_t)new_owner, cpu);
}

static inline void doRemoteInvalidatePageStructureCacheHWASID(paddr_t root, hw_asid_t pcid, word_t mask)
{
    doRemoteMaskOp2Arg(IpiRemoteCall_InvalidatePageStructureCacheHWASID, root, (word_t)(pcid.v), mask);
}

static inline void doRemoteInvalidateTranslationSingle(vptr_t vptr, word_t mask)
{
    doRemoteMaskOp1Arg(IpiRemoteCall_InvalidateTranslationSingle, vptr, mask);
}

static inline void doRemoteInvalidateTranslationSingleHWASID(vptr_t vptr, hw_asid_t pcid, word_t mask)
{
    doRemoteMaskOp2Arg(IpiRemoteCall_InvalidateTranslationSingleHWASID, vptr, (word_t)(pcid.v), mask);
}

static inline void doRemoteInvalidateTranslationAll(word_t mask)
{
    doRemoteMaskOp0Arg(IpiRemoteCall_InvalidateTranslationAll, mask);
}

#ifdef CONFIG_VTX
static inline void doRemoteClearCurrentVCPU(word_t cpu)
{
    doRemoteOp0Arg(IpiRemoteCall_ClearCurrentVCPU, cpu);
}

static inline void doRemoteVMCheckBoundNotification(word_t cpu, tcb_t *tcb)
{
    doRemoteOp1Arg(IpiRemoteCall_VMCheckBoundNotification, (word_t)tcb, cpu);
}
#endif

#endif /* ENABLE_SMP_SUPPORT */
