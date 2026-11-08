/*
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/* One record per IOMMU selected by /chosen/seL4,kernel-devices. */
typedef struct seL4_RISCV_IOMMUBootInfo {
    seL4_BootInfoHeader header;
    seL4_CPtr control; /* Root CNode slot, or seL4_CapNull if initialisation failed. */
    seL4_Word paddr;   /* Physical register base, identifying the IOMMU in the device tree. */
} seL4_RISCV_IOMMUBootInfo;
