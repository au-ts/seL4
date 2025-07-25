/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <machine/capdl.h>

#ifdef CONFIG_PRINTING

#ifdef CONFIG_TK1_SMMU
static inline void arm_obj_iospace_print_attrs(cap_t iospace_cap)
{
    printf("(armiospace: %lu)\n", (long unsigned int)cap_io_space_cap_get_capModuleID(iospace_cap));
}
#endif

static inline void obj_asidpool_print_attrs(cap_t asid_cap)
{
    printf("(asid_high: 0x%lx)\n", (long unsigned int)VSPACE_ID_HIGH(cap_vspace_id_pool_cap_get_capVSpaceIDBase(asid_cap)));
}

#endif /* CONFIG_PRINTING */
