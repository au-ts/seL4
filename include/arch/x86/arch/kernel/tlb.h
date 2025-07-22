/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <arch/smp/ipi_inline.h>

static inline void invalidatePageStructureCacheASID(paddr_t root, vspace_id_t vspaceId, word_t mask)
{
    hw_asid_t pcid = (hw_asid_t)vspaceId;

    invalidateLocalPageStructureCacheHWASID(root, pcid);
    SMP_COND_STATEMENT(doRemoteInvalidatePageStructureCacheHWASID(root, pcid, mask));
}

static inline void invalidateTranslationSingle(vptr_t vptr, word_t mask)
{
    invalidateLocalTranslationSingle(vptr);
    SMP_COND_STATEMENT(doRemoteInvalidateTranslationSingle(vptr, mask));
}

static inline void invalidateTranslationSingleASID(vptr_t vptr, vspace_id_t vspaceId, word_t mask)
{
    hw_asid_t pcid = (hw_asid_t)vspaceId;

    invalidateLocalTranslationSingleHWASID(vptr, pcid);
    SMP_COND_STATEMENT(doRemoteInvalidateTranslationSingleHWASID(vptr, pcid, mask));
}

static inline void invalidateTranslationAll(word_t mask)
{
    invalidateLocalTranslationAll();
    SMP_COND_STATEMENT(doRemoteInvalidateTranslationAll(mask));
}
