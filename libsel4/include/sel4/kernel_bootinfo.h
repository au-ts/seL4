/*
 * Copyright 2025, UNSW
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4/config.h>
#include <sel4/macros.h>

/**
 *
 * The Kernel Boot Info structure looks like this:
 *
 *            +---------------------------+
 *            |                           |
 *            |  Kernel Boot Info Header  |
 *            |                           |
 *            +---------------------------+
*             |     Kernel Mem Region     |               (singular)
 *            +---------------------------+
 *            |        RAM Region 1       |
 *            +---------------------------+           -+
 *            |        RAM Region 2       |            |  fixed "RAM region" size
 *            +---------------------------+           -+
*             |     Root Task Region 1    |
 *            +---------------------------+           -+
 *            |     Root Task Region 2    |            |  (possibly) different fixed size
 *            +---------------------------+           -+
*             |     Reserved Region 1     |
 *            +---------------------------+
 *            |     Reserved Region 2     |
 *            +---------------------------+
 *
 * An alternative would be to repurpose the ELF header structure but it's a bit
 * complicated for this task.
 *
 *
 */

typedef struct seL4_KernelBootInfo {
    /* Should be SEL4_KERNEL_BOOT_INFO_MAGIC */
    seL4_Uint32 magic;
    /* The version of this seL4_KernelBootInfo protocol.
       For now, only SEL4_KERNEL_BOOT_INFO_VERSION_0 exists. */
    seL4_Uint8 version;
    // TODO: Might want 32-bit vs 64-bit? For now only 64-bit.
    seL4_Uint8 _padding0[3];

    /* Root task's entry point */
    seL4_Uint64 root_task_entry;

    // XXX: useful for domain partitioning later
    /** This represents the kernel ELF's loaded physical address range. Only one. */
    seL4_Uint8 num_kernel_regions;
    /** This represents physical memory ranges useable by the kernel. Can have multiple.
        On multikernel, this might not be all of available RAM.
        This will be exposed as "normal" untyped capabilities, excluding any reserved
        regions. */
    seL4_Uint8 num_ram_regions;
    /** This represents the physical memory address of the root task. */
    seL4_Uint8 num_root_task_regions;
    /** This represents reserved physical memory addresses.
        No untyped capabilities will be created for these regions; nor will
        seL4 try to write or read from these addreses.
        Used in multikernel to mark other kernels' memory as reserved. */
    seL4_Uint8 num_reserved_regions;
    seL4_Uint8 _padding[4];
} SEL4_PACKED seL4_KernelBootInfo;


#define SEL4_KERNEL_BOOT_INFO_MAGIC ((seL4_Uint32)0x73654c34)  /* "seL4" */

#define SEL4_KERNEL_BOOT_INFO_VERSION_0 ((seL4_Uint8)0)        /* Version 0 */


/* A region [start..end) of physical memory addresses.
   The fields addresses have architecture-specific alignment requirements. */
typedef struct seL4_KernelBoot_KernelRegion {
    seL4_Uint64 base;
    seL4_Uint64 end;
} SEL4_PACKED SEL4_ALIGNAS(seL4_Uint64) seL4_KernelBoot_KernelRegion;

/* A region [start..end) of physical memory addresses.
   The fields addresses have architecture-specific alignment requirements. */
typedef struct seL4_KernelBoot_RamRegion {
    seL4_Uint64 base;
    seL4_Uint64 end;
} SEL4_PACKED SEL4_ALIGNAS(seL4_Uint64) seL4_KernelBoot_RamRegion;

/* A region [start..end) of physical memory addresses.
   Then, the base virtual address of this region. The virtual end is implicitly
   defined based on the size of the physical region. */
typedef struct seL4_KernelBoot_RootTaskRegion {
    seL4_Uint64 paddr_base;
    seL4_Uint64 paddr_end;
    seL4_Uint64 vaddr_base;
    seL4_Uint8 _padding[8];
} SEL4_PACKED SEL4_ALIGNAS(seL4_Uint64) seL4_KernelBoot_RootTaskRegion;

/* A region [start..end) of physical memory addresses.
   The fields addresses have architecture-specific alignment requirements. */
typedef struct seL4_KernelBoot_ReservedRegion {
    seL4_Uint64 base;
    seL4_Uint64 end;
} SEL4_PACKED SEL4_ALIGNAS(seL4_Uint64) seL4_KernelBoot_ReservedRegion;


/* Make sure that modifications to the structure don't change these offsets */
SEL4_COMPILE_ASSERT(kernel_boot_header_size, sizeof(seL4_KernelBootInfo) == 0x18);
SEL4_COMPILE_ASSERT(kernel_boot_header_magic_offset, __builtin_offsetof(seL4_KernelBootInfo, magic) == 0x0);
SEL4_COMPILE_ASSERT(kernel_boot_header_version_offset, __builtin_offsetof(seL4_KernelBootInfo, version) == 0x4);
SEL4_COMPILE_ASSERT(kernel_boot_header_root_task_entry_offset, __builtin_offsetof(seL4_KernelBootInfo, root_task_entry) == 0x8);
SEL4_COMPILE_ASSERT(kernel_boot_header_num_kernel_regions_offset, __builtin_offsetof(seL4_KernelBootInfo, num_kernel_regions) == 0x10);
SEL4_COMPILE_ASSERT(kernel_boot_header_num_ram_regions_offset, __builtin_offsetof(seL4_KernelBootInfo, num_ram_regions) == 0x11);
SEL4_COMPILE_ASSERT(kernel_boot_header_num_root_task_regions_offset, __builtin_offsetof(seL4_KernelBootInfo, num_root_task_regions) == 0x12);
SEL4_COMPILE_ASSERT(kernel_boot_header_num_reserved_regions_offset, __builtin_offsetof(seL4_KernelBootInfo, num_reserved_regions) == 0x13);

SEL4_COMPILE_ASSERT(kernel_boot_kernel_region_size, sizeof(seL4_KernelBoot_KernelRegion) == 0x10);
SEL4_COMPILE_ASSERT(kernel_boot_kernel_region_align, _Alignof(seL4_KernelBoot_KernelRegion) == _Alignof(seL4_Uint64));

SEL4_COMPILE_ASSERT(kernel_boot_ram_region_size, sizeof(seL4_KernelBoot_RamRegion) == 0x10);
SEL4_COMPILE_ASSERT(kernel_boot_ram_region_align, _Alignof(seL4_KernelBoot_RamRegion) == _Alignof(seL4_Uint64));

SEL4_COMPILE_ASSERT(kernel_boot_root_task_region_size, sizeof(seL4_KernelBoot_RootTaskRegion) == 0x20);
SEL4_COMPILE_ASSERT(kernel_boot_root_task_region_align, _Alignof(seL4_KernelBoot_RootTaskRegion) == _Alignof(seL4_Uint64));

SEL4_COMPILE_ASSERT(kernel_boot_reserved_region_size, sizeof(seL4_KernelBoot_ReservedRegion) == 0x10);
SEL4_COMPILE_ASSERT(kernel_boot_reserved_region_align, _Alignof(seL4_KernelBoot_ReservedRegion) == _Alignof(seL4_Uint64));
