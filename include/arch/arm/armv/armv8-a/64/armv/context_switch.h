/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <arch/kernel/vspace.h>


static inline void armv_contextSwitch_HWASID(vspace_root_t *vspace, hw_asid_t hw_asid)
{
    setCurrentUserVSpaceRoot(ttbr_new(hw_asid.v, pptr_to_paddr(vspace)));
}

/*
 * In AARCH64, hardware and virtual asids are the same and are written
 * when updating the translation table base register.
 */
static inline void armv_contextSwitch(vspace_root_t *vspace, vspace_id_t vspaceId)
{
#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT
    hw_asid_t hw_asid = getHWASID(asid);
#else
    hw_asid_t hw_asid = (hw_asid_t){vspaceId};
#endif
    armv_contextSwitch_HWASID(vspace, hw_asid);
}
