/*
 * Copyright 2025, UNSW
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4/config.h>
#include <sel4/macros.h>

typedef struct seL4_KernelBootInfo {
    /* Should be SEL4_KERNEL_BOOT_INFO_MAGIC */
    seL4_Uint32 magic;
    /* The version of this seL4_KernelBootInfo protocol.
       For now, only SEL4_KERNEL_BOOT_INFO_VERSION_0 exists. */
    seL4_Uint8 version;

    seL4_Uint8 _padding0[3];

    /* Root task's entry point */
    seL4_Uint64 root_task_entry;
    /* Root task's physical-to-virtual offset. vaddr = paddr - offset */
    seL4_Uint64 root_task_pv_offset;

    /* An &seL4_KernelBootInfo-relative offset to an array of memory descriptors
       i.e. if &seL4_KernelBootInfo == 0xe000 and memory descriptors at 0xf000
       then offset_of_memory_descriptors is 0x1000. The idea is that these
       are located nearby the KernelBootInfo such that they both could fit
       within a single page. */
    seL4_Uint64 offset_of_memory_descriptors;
    seL4_Uint32 number_of_memory_descriptors;

    seL4_Uint8 _padding1[28];
} SEL4_PACKED seL4_KernelBootInfo;


#define SEL4_KERNEL_BOOT_INFO_MAGIC ((seL4_Uint32)0x73654c34)  /* "seL4" */

#define SEL4_KERNEL_BOOT_INFO_VERSION_0 ((seL4_Uint8)0)        /* Version 0 */

/* A region [start..end) of memory addresses, and a type.
   The fields addresses have architecture-specific alignment requirements. */
typedef struct seL4_KernelBootMemoryDescriptor {
    seL4_Uint64 base;
    seL4_Uint64 end;
    seL4_Uint8  kind;
    seL4_Uint8 _padding[7];
} SEL4_PACKED seL4_KernelBootMemoryDescriptor;

/* Reserved so we can (hopefully) detect unitiliased descriptors */
#define SEL4_KERNEL_BOOT_MEMORY_DESCRIPTOR_KIND_INVALID ((seL4_Uint8)0)

/** This represents the kernel ELF's loaded physical address range. Only one. */
#define SEL4_KERNEL_BOOT_MEMORY_DESCRIPTOR_KIND_KERNEL ((seL4_Uint8)1)

/** This represents physical memory ranges useable by the kernel. Can have multiple.
    On multikernel, this might not be all of available RAM.
    This will be exposed as "normal" untyped capabilities, excluding any reserved
    regions.
*/
#define SEL4_KERNEL_BOOT_MEMORY_DESCRIPTOR_KIND_RAM ((seL4_Uint8)2)

/** This represents the physical memory address of the root task. */
#define SEL4_KERNEL_BOOT_MEMORY_DESCRIPTOR_KIND_ROOT_TASK ((seL4_Uint8)3)

/** This represents reserved physical memory addresses.
    No untyped capabilities will be created for these regions; nor will
    seL4 try to write or read from these addreses.
    Used in multikernel to mark other kernels' memory as reserved. */
#define SEL4_KERNEL_BOOT_MEMORY_DESCRIPTOR_KIND_RESERVED ((seL4_Uint8)4)


/* Make sure that modifications to the structure don't change these offsets */
SEL4_COMPILE_ASSERT(kernel_boot_info_size_60_bytes, sizeof(seL4_KernelBootInfo) == 0x40);
SEL4_COMPILE_ASSERT(kernel_boot_info_magic_offset, __builtin_offsetof(seL4_KernelBootInfo, magic) == 0x0);
SEL4_COMPILE_ASSERT(kernel_boot_info_root_task_entry_offset, __builtin_offsetof(seL4_KernelBootInfo, root_task_entry) == 0x8);
SEL4_COMPILE_ASSERT(kernel_boot_info_root_task_pv_offset_offset, __builtin_offsetof(seL4_KernelBootInfo, root_task_pv_offset) == 0x10);
SEL4_COMPILE_ASSERT(kernel_boot_info_offset_of_memory_descriptors_offset, __builtin_offsetof(seL4_KernelBootInfo, offset_of_memory_descriptors) == 0x18);
SEL4_COMPILE_ASSERT(kernel_boot_info_number_of_memory_descriptors_offset, __builtin_offsetof(seL4_KernelBootInfo, number_of_memory_descriptors) == 0x20);

SEL4_COMPILE_ASSERT(kernel_boot_memory_descriptor_size, sizeof(seL4_KernelBootMemoryDescriptor) == 0x18);
// Important: must be aligned if made into an array.
// XXX: ????
// SEL4_COMPILE_ASSERT(kernel_boot_memory_descriptor_align, _Alignof(seL4_KernelBootMemoryDescriptor) == _Alignof(seL4_Uint64));
SEL4_COMPILE_ASSERT(kernel_boot_memory_descriptor_base_offset, __builtin_offsetof(seL4_KernelBootMemoryDescriptor, base) == 0x0);
SEL4_COMPILE_ASSERT(kernel_boot_memory_descriptor_end_offset, __builtin_offsetof(seL4_KernelBootMemoryDescriptor, end) == 0x8);
SEL4_COMPILE_ASSERT(kernel_boot_memory_descriptor_type_offset, __builtin_offsetof(seL4_KernelBootMemoryDescriptor, kind) == 0x10);
