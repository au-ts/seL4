/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright 2015, 2016 Hesham Almatary <heshamelmatary@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4/config.h>
#include <sel4/macros.h>

#define seL4_WordBits           64
/* log 2 bits in a word */
#define seL4_WordSizeBits       3

#define seL4_SlotBits           5
#ifdef CONFIG_KERNEL_MCS
#define seL4_NotificationBits   6
#define seL4_ReplyBits          5
#else
#define seL4_NotificationBits   5
#endif
#define seL4_EndpointBits       4
#define seL4_IPCBufferSizeBits  10
#ifdef CONFIG_HAVE_FPU
#define seL4_TCBBits            11
#else
#define seL4_TCBBits            10
#endif

/* Sv39/Sv48 pages/ptes sizes */
#define seL4_PageTableEntryBits 3
#define seL4_PageTableIndexBits 9

#define seL4_PageBits          12
#define seL4_LargePageBits     21
#define seL4_HugePageBits      30
#define seL4_TeraPageBits      39
#define seL4_PageTableBits     12
#define seL4_VSpaceBits        seL4_PageTableBits

#ifdef CONFIG_RISCV_IOMMU
#define seL4_RootX4PageTableBits            (seL4_PageTableBits + 2)
#define seL4_RootX4PageTableIndexBits       (seL4_PageTableIndexBits + 2)
#define seL4_X4GPABits                      41

/* A 4 KiB ASID pool holds 256 entries of 16 bytes each. */
#define seL4_NumASIDPoolsBits               8
#define seL4_ASIDPoolIndexBits              8
#define seL4_ASIDEntryBits                  4

#define seL4_IOMMUDeviceBits                5
#define seL4_IOMMUDDTBits                   12
#define seL4_IOMMUDDTEntryBits              3
#define seL4_IOMMUDDTLeafEntryBits          5
#define seL4_IOMMUDDTLeafExtendedEntryBits  6
#define seL4_IOMMUDDTIndexBits              (seL4_IOMMUDDTBits - seL4_IOMMUDDTEntryBits)
#define seL4_IOMMUDDTLeafIndexBits          (seL4_IOMMUDDTBits - seL4_IOMMUDDTLeafEntryBits)
#define seL4_IOMMUDDTLeafExtendedIndexBits  (seL4_IOMMUDDTBits - seL4_IOMMUDDTLeafExtendedEntryBits)
#define seL4_IOMMUDeviceIDBits              24
#define seL4_IOMMUDDTMaxLevels              3
#define seL4_IOMMUGSCIDBits                 16
#define seL4_IOMMUPSCIDBits                 20
#define seL4_IOMMUFaultRecordBits           5

#define seL4_IOMMUInterruptMSI              0
#define seL4_IOMMUInterruptWSI              1
#define seL4_IOMMUInterruptBoth             2

/* ConfigureFirstStage kind and flags. */
#define seL4_IOMMUFirstStageDirect          0
#define seL4_IOMMUFirstStagePDT             1
#define seL4_IOMMUFirstStageDPE             (1 << 0)
#define seL4_IOMMUFirstStageAD              (1 << 2)

/* InvalidateFirstStage kind. */
#define seL4_IOMMUInvalidatePDT             0
#define seL4_IOMMUInvalidatePSCID           1
#define seL4_IOMMUInvalidateAllPSCID        2

/* ConfigureFirstStage mode. */
#define seL4_IOMMUModeBare                  0
#define seL4_IOMMUModeSv39                  8
#define seL4_IOMMUModeSv48                  9
#define seL4_IOMMUModeSv57                  10
#define seL4_IOMMUModePD8                   1
#define seL4_IOMMUModePD17                  2
#define seL4_IOMMUModePD20                  3

/* RWFaultQueue clearErrors and status bits. */
#define seL4_IOMMUFaultOverflow             (1 << 9)
#define seL4_IOMMUFaultMemoryError          (1 << 8)
#else
#define seL4_NumASIDPoolsBits   7
#define seL4_ASIDPoolIndexBits  9
#endif
#define seL4_ASIDPoolBits       12

/* Untyped size limits */
#define seL4_MinUntypedBits     4
#define seL4_MaxUntypedBits     38
#ifndef __ASSEMBLER__

typedef enum {
    seL4_VMFault_IP,
    seL4_VMFault_Addr,
    seL4_VMFault_PrefetchFault,
    seL4_VMFault_FSR,
    seL4_VMFault_Length,
} seL4_VMFault_Msg;

typedef enum {
    seL4_UnknownSyscall_FaultIP,
    seL4_UnknownSyscall_SP,
    seL4_UnknownSyscall_RA,
    seL4_UnknownSyscall_A0,
    seL4_UnknownSyscall_A1,
    seL4_UnknownSyscall_A2,
    seL4_UnknownSyscall_A3,
    seL4_UnknownSyscall_A4,
    seL4_UnknownSyscall_A5,
    seL4_UnknownSyscall_A6,
    seL4_UnknownSyscall_Syscall,
    seL4_UnknownSyscall_Length,
} seL4_UnknownSyscall_Msg;

typedef enum {
    seL4_UserException_FaultIP,
    seL4_UserException_SP,
    seL4_UserException_Number,
    seL4_UserException_Code,
    seL4_UserException_Length,
} seL4_UserException_Msg;

#ifdef CONFIG_KERNEL_MCS
typedef enum {
    seL4_TimeoutReply_FaultIP,
    seL4_TimeoutReply_LR,
    seL4_TimeoutReply_SP,
    seL4_TimeoutReply_GP,
    seL4_TimeoutReply_s0,
    seL4_TimeoutReply_s1,
    seL4_TimeoutReply_s2,
    seL4_TimeoutReply_s3,
    seL4_TimeoutReply_s4,
    seL4_TimeoutReply_s5,
    seL4_TimeoutReply_s6,
    seL4_TimeoutReply_s7,
    seL4_TimeoutReply_s8,
    seL4_TimeoutReply_s9,
    seL4_TimeoutReply_s10,
    seL4_TimeoutReply_s11,
    seL4_TimeoutReply_a0,
    seL4_TimeoutReply_a1,
    seL4_TimeoutReply_a2,
    seL4_TimeoutReply_a3,
    seL4_TimeoutReply_a4,
    seL4_TimeoutReply_a5,
    seL4_TimeoutReply_a6,
    seL4_TimeoutReply_a7,
    seL4_TimeoutReply_t0,
    seL4_TimeoutReply_t1,
    seL4_TimeoutReply_t2,
    seL4_TimeoutReply_t3,
    seL4_TimeoutReply_t4,
    seL4_TimeoutReply_t5,
    seL4_TimeoutReply_t6,
    seL4_TimeoutReply_TP,
    seL4_TimeoutReply_Length,
} seL4_TimeoutReply_Msg;

typedef enum {
    seL4_Timeout_Data,
    seL4_Timeout_Consumed,
    seL4_Timeout_Length,
} seL4_Timeout_Msg;
#endif
#endif /* __ASSEMBLER__ */

/* (Deprecated) First address in the virtual address space that is not accessible to user level */
#define seL4_UserTop 0x0000003ffffff000

/* Last address in the virtual address space that is accessible to user level */
#define seL4_UserVSpaceTop (seL4_UserTop - 1)

#ifdef CONFIG_ENABLE_BENCHMARKS
/* size of kernel log buffer in bytes */
#define seL4_LogBufferSize (LIBSEL4_BIT(20))
#endif /* CONFIG_ENABLE_BENCHMARKS */
