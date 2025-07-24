/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <types.h>
#include <api/failures.h>
#include <object/structures.h>

/* PD slot reserved for storing the PD's allocated hardware ASID */
#define PD_ASID_SLOT (0xff000000 >> (PT_INDEX_BITS + PAGE_BITS))

enum pde_pte_tag {
    ME_PDE,
    ME_PTE
};
typedef word_t pde_pte_tag_t;

struct createMappingEntries_ret {
    exception_t status;
    pde_pte_tag_t tag;
    void *pde_pte_ptr;
    unsigned int offset;
    word_t size;
};
typedef struct createMappingEntries_ret createMappingEntries_ret_t;

struct findVSpaceForVSpaceId_ret {
    exception_t status;
    pde_t *pd;
};
typedef struct findVSpaceForVSpaceId_ret findVSpaceForVSpaceId_ret_t;

struct lookupPTSlot_ret {
    exception_t status;
    pte_t *ptSlot;
};
typedef struct lookupPTSlot_ret lookupPTSlot_ret_t;

#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT
hw_asid_t getHWASID(vspace_id_t vspaceId);
#endif
void copyGlobalMappings(pde_t *newPD);
findVSpaceForVSpaceId_ret_t findVSpaceForVSpaceId(vspace_id_t asid);
lookupPTSlot_ret_t lookupPTSlot(pde_t *pd, vptr_t vptr);
pde_t *CONST lookupPDSlot(pde_t *pd, vptr_t vptr);
void deleteVSpaceIdPool(vspace_id_t base, vspace_id_pool_t *pool);
void deleteVSpaceId(vspace_id_t vspaceId, pde_t *pd);
pde_t *pageTableMapped(vspace_id_t vspaceId, vptr_t vaddr, pte_t *pt);
void unmapPageTable(vspace_id_t vspaceId, vptr_t vaddr, pte_t *pt);
void unmapPage(vm_page_size_t page_size, vspace_id_t vspaceId, vptr_t vptr, void *pptr);
hw_asid_t getHWASID(vspace_id_t vspaceId);
hw_asid_t findFreeHWASID(void);
void flushPage(vm_page_size_t page_size, pde_t *pd, vspace_id_t vspaceId, word_t vptr);
void flushTable(pde_t *pd, vspace_id_t vspaceId, word_t vptr, pte_t *pt);
void flushSpace(vspace_id_t vspaceId);
void invalidateTLBByASID(vspace_id_t vspaceId);

bool_t CONST isIOSpaceFrameCap(cap_t cap);

/* Reserved memory ranges */
static const region_t BOOT_RODATA mode_reserved_region[] = {
    {
        (PD_ASID_SLOT + 0) << ARMSectionBits,
                           (PD_ASID_SLOT + 1) << ARMSectionBits
    }
};
#define MODE_RESERVED ARRAY_SIZE(mode_reserved_region)
