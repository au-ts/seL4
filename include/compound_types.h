/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <stdint.h>
#include <api/types.h>
#include <object/structures.h>
#include <arch/types.h>

typedef cte_t *cte_ptr_t;

struct extra_caps {
    cte_ptr_t excaprefs[seL4_MsgMaxExtraCaps];
};
typedef struct extra_caps extra_caps_t;
