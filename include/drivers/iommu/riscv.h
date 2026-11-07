/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <arch/types.h>
#include <object/structures.h>
#include <stdint.h>
#include <util.h>
#include <drivers/iommu/riscv_gen.h>

#define IOHGATP_MODE_SV39X4 8

#define IOMMU_VERSION_MAJOR_SHIFT 4

#define IOMMU_DDTP_MODE_MASK MASK(4)
#define IOMMU_DDTP_BUSY      BIT(4)
#define IOMMU_DDTP_MODE_OFF  0
#define IOMMU_DDTP_MODE_1LVL 2

#define IOMMU_QCSR_EN    BIT(0)
#define IOMMU_QCSR_IE    BIT(1)
#define IOMMU_QCSR_ON    BIT(16)
#define IOMMU_QCSR_BUSY  BIT(17)
#define IOMMU_FQCSR_FQMF seL4_IOMMUFaultMemoryError
#define IOMMU_FQCSR_FQOF seL4_IOMMUFaultOverflow

#define IOMMU_IPSR_FIP          BIT(1)
#define IOMMU_IPSR_PENDING_MASK MASK(4)
#define IOMMU_ICVEC_VECTOR_MASK MASK(16)
#define IOMMU_MSI_VEC_CTL_MASK  BIT(0)
#define IOMMU_FQ_VECTOR         0

#define IOMMU_FSC_ADDR_BITS   56
#define IOMMU_FSC_MODE_CUSTOM 14

#define IOMMU_COMMAND_BITS  4
#define IOMMU_CQ_INDEX_BITS (CONFIG_RISCV_IOMMU_COMMAND_QUEUE_BITS - IOMMU_COMMAND_BITS)
#define IOMMU_IOTINVAL_GVMA 1
#define IOMMU_IOTINVAL_VMA  0

typedef struct iommu_command {
    uint64_t words[2];
} iommu_command_t;

typedef struct iommu_command_queue {
    iommu_command_t commands[BIT(IOMMU_CQ_INDEX_BITS)];
} ALIGN(BIT(CONFIG_RISCV_IOMMU_COMMAND_QUEUE_BITS)) iommu_command_queue_t;

typedef struct iommu_msi_regs {
    uint64_t addr;
    uint32_t data;
    uint32_t vec_ctl;
} riscv_iommu_msi_regs_t;

typedef struct riscv_iommu_regs {
    uint64_t capabilities;
    uint32_t fctl;
    uint32_t reserved_0;
    uint64_t ddtp;
    uint64_t cqb;
    uint32_t cqh;
    uint32_t cqt;
    uint64_t fqb;
    uint32_t fqh;
    uint32_t fqt;
    uint8_t reserved_1[16];
    uint32_t cqcsr;
    uint32_t fqcsr;
    uint32_t pqcsr;
    uint32_t ipsr;
    uint8_t reserved_2[672];
    uint64_t icvec;
    riscv_iommu_msi_regs_t msi_cfg_tbl[16];
    uint8_t reserved_3[3072];
} riscv_iommu_regs_t;

typedef volatile riscv_iommu_regs_t *riscv_iommu_regs_ptr_t;

void iommuIOFence(riscv_iommu_regs_ptr_t regs);
void iommuIOTINVAL(riscv_iommu_regs_ptr_t regs, word_t gscid, word_t function, bool_t av, vptr_t addr);
void iommuIODIR(riscv_iommu_regs_ptr_t regs, word_t deviceID);
void iommuInvalidateAllDDT(riscv_iommu_regs_ptr_t regs);
void riscvIOMMUUnmapFaultQueue(pptr_t iommu, pptr_t frame);
void riscvIOMMURetireContext(asid_entry_t *entry);
exception_t decodeRISCVIOMMUInvocation(word_t label, word_t length, cte_t *slot, cap_t cap, bool_t call,
                                       word_t *buffer);
void riscvIOMMUFinaliseCap(cap_t cap);
cap_t riscvIOMMUInit(pptr_t iommu, paddr_t commandQueue);
