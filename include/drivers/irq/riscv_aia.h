/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <plat/machine/devices_gen.h>
#include <sel4/arch/constants.h>
#include <util.h>

// *iselect values to access AIA CSRs
#define IMSIC_EIDELIVERY                0x70
#define IMSIC_EITHRESHOLD               0x72
#define IMSIC_EIP0                      0x80
#define IMSIC_EIE0                      0xc0
// *topei -> highest prio pending irq
#define IMSIC_TOPEI_ID_SHIFT            16

#ifdef CONFIG_RISCV_APLIC
#define APLIC_DOMAINCFG_BE              BIT(0)
#define APLIC_DOMAINCFG_DM              BIT(2)
#define APLIC_DOMAINCFG_IE              BIT(8)
#define APLIC_SOURCECFG_SM_MASK         MASK(3)
#define APLIC_SOURCECFG_INACTIVE        0
// support MSI delivery mode only
#define APLIC_TARGET_HART_INDEX_SHIFT   18

/* https://docs.riscv.org/reference/aia/v1.0/AdvPLIC.html */
typedef struct riscv_aplic_regs {
    uint32_t domaincfg;
    uint32_t sourcecfg[1023];
    uint8_t reserved_0[0xedc];
    uint32_t setienum;
    uint8_t reserved_1[0xfc];
    uint32_t clrienum;
    uint8_t reserved_2[0x20];
    uint32_t setipnum_le;
    uint8_t reserved_3[0x1000];
    uint32_t target[1023];
} riscv_aplic_regs_t;

// one hart is given control of the APLIC
#define aplic ((volatile riscv_aplic_regs_t *)APLIC_PPTR)

static inline void aplic_write(volatile uint32_t *reg, uint32_t value) {
    *reg = value;
    asm volatile("fence iorw, iorw" ::: "memory");
}
#endif

// indirect csr access via siselect and sireg
static inline void imsic_csr_write(word_t select, word_t value) {
    asm volatile("csrw 0x150, %0; csrw 0x151, %1" :: "r"(select), "r"(value) : "memory");
}

#ifdef CONFIG_RISCV_APLIC
static inline void aia_mask_irq(bool_t disable, irq_t irq, word_t source) {
#else
static inline void aia_mask_irq(bool_t disable, irq_t irq) {
#endif
    word_t select = IMSIC_EIE0 + (irq / 64) * 2;
    word_t mask = BIT(irq % 64);
    if (disable) {
#ifdef CONFIG_RISCV_APLIC
        if (source != 0) {
            aplic_write(&aplic->clrienum, source);
        }
#endif
        asm volatile("csrw 0x150, %0; csrc 0x151, %1" :: "r"(select), "r"(mask) : "memory");
    } else {
        asm volatile("csrw 0x150, %0; csrs 0x151, %1" :: "r"(select), "r"(mask) : "memory");
#ifdef CONFIG_RISCV_APLIC
        if (source != 0) {
            aplic_write(&aplic->setienum, source);
            word_t mode = aplic->sourcecfg[source - 1] & APLIC_SOURCECFG_SM_MASK;
            if (mode == seL4_RISCV_IRQLevelHigh || mode == seL4_RISCV_IRQLevelLow) {
                aplic_write(&aplic->setipnum_le, source);
            }
        }
#endif
    }
}

static inline void imsic_clear_pending(irq_t irq) {
    word_t select = IMSIC_EIP0 + (irq / 64) * 2;
    word_t mask = BIT(irq % 64);
    asm volatile("csrw 0x150, %0; csrc 0x151, %1" :: "r"(select), "r"(mask) : "memory");
}

#ifdef CONFIG_RISCV_APLIC
static inline void aia_delete_irq(irq_t irq, word_t source) {
#else
static inline void aia_delete_irq(irq_t irq) {
#endif
#ifdef CONFIG_RISCV_APLIC
    if (source != 0) {
        aplic_write(&aplic->sourcecfg[source - 1], APLIC_SOURCECFG_INACTIVE);
    }
#endif
    imsic_clear_pending(irq);
}

static inline irq_t aia_get_claim(void) {
    word_t value;
    asm volatile("csrrw %0, 0x15c, 0x0" : "=r"(value) :: "memory");
    return value >> IMSIC_TOPEI_ID_SHIFT;
}

#ifdef CONFIG_RISCV_APLIC
// MSI delivery mode 4.5.16.2 AIA spec
static inline bool_t aia_irq_configure(irq_t irq, word_t source, word_t trigger) {
    if (trigger < 4 || trigger > 7) {
        return false;
    }
    aplic_write(&aplic->sourcecfg[source - 1], trigger);
    if (aplic->sourcecfg[source - 1] != trigger) {
        aplic_write(&aplic->sourcecfg[source - 1], APLIC_SOURCECFG_INACTIVE);
        return false;
    }
    aplic_write(&aplic->target[source - 1], (IMSIC_HART_INDEX << APLIC_TARGET_HART_INDEX_SHIFT) | irq);
    return true;
}
#endif

static inline void aia_init_hart(void) {
    imsic_csr_write(IMSIC_EIDELIVERY, 0);
    for (word_t i = 0; i <= IMSIC_NUM_IDS; i += 64) {
        imsic_csr_write(IMSIC_EIE0 + (i / 64) * 2, 0);
        imsic_csr_write(IMSIC_EIP0 + (i / 64) * 2, 0);
    }
    imsic_csr_write(IMSIC_EITHRESHOLD, 0);
    imsic_csr_write(IMSIC_EIDELIVERY, 1);
}

static inline void aia_init_controller(void) {
#ifdef CONFIG_RISCV_APLIC
    aplic_write(&aplic->domaincfg, APLIC_DOMAINCFG_DM);
    for (word_t source = 1; source <= APLIC_NUM_SOURCES; source++) {
        aplic_write(&aplic->sourcecfg[source - 1], APLIC_SOURCECFG_INACTIVE);
    }

    word_t config = APLIC_DOMAINCFG_IE | APLIC_DOMAINCFG_DM;
    aplic_write(&aplic->domaincfg, config);
    if ((aplic->domaincfg & (config | APLIC_DOMAINCFG_BE)) != config) {
        fail("Supervisor APLIC does not support little-endian MSI delivery");
    }
#endif
}
