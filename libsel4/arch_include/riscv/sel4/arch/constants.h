/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4/config.h>

#ifdef CONFIG_RISCV_APLIC
#define seL4_RISCV_IRQEdgeRising    4
#define seL4_RISCV_IRQEdgeFalling   5
#define seL4_RISCV_IRQLevelHigh     6
#define seL4_RISCV_IRQLevelLow      7
#endif
