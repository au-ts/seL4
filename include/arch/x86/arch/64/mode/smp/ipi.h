/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <types.h>
#include <plat/machine.h>
#include <smp/ipi.h>

#ifdef ENABLE_SMP_SUPPORT

typedef enum {
    IpiRemoteCall_InvalidatePCID = IpiNumArchRemoteCall,
    IpiRemoteCall_InvalidateASID,
    IpiNumModeRemoteCall
} IpiModeRemoteCall_t;

void Mode_handleRemoteCall(IpiModeRemoteCall_t call, word_t arg0, word_t arg1, word_t arg2);

static inline void doRemoteInvalidatePCID(word_t type, void *vaddr, hw_asid_t pcid, word_t mask)
{
    doRemoteMaskOp3Arg((IpiRemoteCall_t)IpiRemoteCall_InvalidatePCID, type, (word_t)vaddr, (word_t)pcid, mask);
}

/* see tlb.h invalidateLocalASID for why these are different */
static inline void doRemoteInvalidateASID(vspace_root_t *vspace, vspace_id_t vspaceId, word_t mask)
{
    doRemoteMaskOp2Arg((IpiRemoteCall_t)IpiRemoteCall_InvalidateASID, (word_t)vspace, (word_t)vspaceId, mask);
}

#endif /* ENABLE_SMP_SUPPORT */
