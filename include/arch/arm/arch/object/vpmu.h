#pragma once

#include <types.h>
#include <api/failures.h>
#include <object/structures.h>
#include <mode/machine/registerset.h>

#ifdef CONFIG_THREAD_LOCAL_PMU

typedef struct pmu_state {
    /* This state is the only things that userspace can read/access based
    on the kernel PMU access control capability's invocations. */
    uint64_t cycle_counter;
    /* The ARM spec allows for a maximum of 31 event counters */
    uint32_t event_counters[31];
    uint32_t event_type[31];
    /* PMU Control register */
    uint32_t pmcr;
    /* PMU Counter enable register */
    uint32_t pmcntenset;
    /* PMU Overflow flag status register */
    uint32_t pmovsclr;
    /* PMU Interrupt enable set register */
    uint32_t pmintenset;
} pmu_state_t;

typedef struct vpmu {
    pmu_state_t reg_state;
    cte_t virq_handler;
    // This field is to allow a virtualised irq ack
    uint8_t active_irq;
    #ifdef CONFIG_PROFILER_ENABLE
    uint64_t pc;
    uint64_t fp;
    #endif /* CONFIG_PROFILER_ENABLE */
} vpmu_t;

// If a VPMU is bound to the current running thread, and a valid
// vIRQ handler is set, we will deliver this IRQ to the endpoint of the vPMU
// and return 1. If no valid vIRQ is set, then we will return 0, and
// we will attempt to deliver the IRQ to an interrupt handler (if it exists).
uint8_t arm_vpmu_handle_irq(void);

exception_t decodeARMVPMUInvocation(word_t label, unsigned int length, cptr_t cptr,
                                         cte_t *srcSlot, cap_t cap,
                                         bool_t call, word_t *buffer);
#endif /* CONFIG_THREAD_LOCAL_PMU */
