/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <object/structures.h>

#ifdef CONFIG_VTX

struct EPTPDPTMapped_ret {
    ept_pml4e_t *pml4;
    ept_pml4e_t *pml4Slot;
    exception_t status;
};
typedef struct EPTPDPTMapped_ret EPTPDPTMapped_ret_t;

struct EPTPageDirectoryMapped_ret {
    ept_pml4e_t *pml4;
    ept_pdpte_t *pdptSlot;
    exception_t status;
};
typedef struct EPTPageDirectoryMapped_ret EPTPageDirectoryMapped_ret_t;

struct EPTPageTableMapped_ret {
    ept_pml4e_t *pml4;
    ept_pde_t *pdSlot;
    exception_t status;
};
typedef struct EPTPageTableMapped_ret EPTPageTableMapped_ret_t;

struct findEPTForVSpaceID_ret {
    exception_t status;
    ept_pml4e_t *ept;
};
typedef struct findEPTForVSpaceID_ret findEPTForVSpaceID_ret_t;

EPTPDPTMapped_ret_t EPTPDPTMapped(vspace_id_t vspaceId, vptr_t vptr, ept_pdpte_t *pdpt);
EPTPageDirectoryMapped_ret_t EPTPageDirectoryMapped(vspace_id_t vspaceId, vptr_t vaddr, ept_pde_t *pd);
EPTPageTableMapped_ret_t EPTPageTableMapped(vspace_id_t vspaceId, vptr_t vaddr, ept_pte_t *pt);
findEPTForVSpaceID_ret_t findEPTForVSpaceID(vspace_id_t vspaceId);

void deleteEPTVSpaceID(vspace_id_t vspaceId, ept_pml4e_t *ept);
exception_t decodeX86EPTInvocation(word_t invLabel, word_t length, cptr_t cptr, cte_t *cte, cap_t cap,
                                   word_t *buffer);
exception_t decodeX86EPTPDInvocation(word_t invLabel, word_t length, cte_t *cte, cap_t cap, word_t *buffer);
exception_t decodeX86EPTPTInvocation(word_t invLabel, word_t length, cte_t *cte, cap_t cap, word_t *buffer);
exception_t decodeX86EPTPageMap(word_t invLabel, word_t length, cte_t *cte, cap_t cap, word_t *buffer);
exception_t performX86EPTPageInvocationUnmap(cap_t cap, cte_t *ctSlot);
void unmapEPTPDPT(vspace_id_t vspaceId, vptr_t vaddr, ept_pdpte_t *pdpt);
void unmapEPTPageDirectory(vspace_id_t vspaceId, vptr_t vaddr, ept_pde_t *pd);
void unmapEPTPageTable(vspace_id_t vspaceId, vptr_t vaddr, ept_pte_t *pt);
void unmapEPTPage(vm_page_size_t page_size, vspace_id_t vspaceId, vptr_t vptr, void *pptr);

#endif /* CONFIG_VTX */
