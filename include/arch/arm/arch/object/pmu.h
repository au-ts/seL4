/* This is an intial header file to add the PMU cap to AARCH64 based systems. */

#pragma once

#include <types.h>
#include <api/failures.h>
#include <object/structures.h>

// @kwinter: This following enum only contains the registers that would normally be exported
// to EL0. Discuss if we also want to provide access to the privelleged EL1 registers.
// Also this is for FEAT_PMUv3_EXT64
// enum pmu_register {
//     // FEAT_PMUv3_EXT
//     pmevcntr0,
//     pmevcntr1,
//     pmevcntr2,
//     pmevcntr3,
//     pmevcntr4,
//     pmevcntr5,
//     pmevcntr6,
//     pmevcntr7,
//     pmevcntr8,
//     pmevcntr9,
//     pmevcntr10,
//     pmevcntr11,
//     pmevcntr12,
//     pmevcntr13,
//     pmevcntr14,
//     pmevcntr15,
//     pmevcntr16,
//     pmevcntr17,
//     pmevcntr18,
//     pmevcntr19,
//     pmevcntr20,
//     pmevcntr21,
//     pmevcntr22,
//     pmevcntr23,
//     pmevcntr24,
//     pmevcntr25,
//     pmevcntr26,
//     pmevcntr27,
//     pmevcntr28,
//     pmevcntr29,
//     pmevcntr30,
//     // FEAT_PMUv3_EXT
//     pmccntr,
//     // FEAT_PMUv3_ICNTR
//     pmicntr,
//     // FEAT_PMUv3_EXT and FEAT_PCSRv8p2
//     pmpcsr,
//     pmvcidsr,
//     pmcid1sr,
//     pmpcsr,
//     // FEAT_PMUv3_EXT and FEAT_PCSRv8p2 and EL2 is implemented
//     pmvidsr,
//     // FEAT_PMUv3_EXT
//     pmccidsr,
//     pmcid1sr,
//     pmevtyper0,
//     pmevtyper1,
//     pmevtyper2,
//     pmevtyper3,
//     pmevtyper4,
//     pmevtyper5,
//     pmevtyper6,
//     pmevtyper7,
//     pmevtyper8,
//     pmevtyper9,
//     pmevtyper10,
//     pmevtyper11,
//     pmevtyper12,
//     pmevtyper13,
//     pmevtyper14,
//     pmevtyper15,
//     pmevtyper16,
//     pmevtyper17,
//     pmevtyper18,
//     pmevtyper19,
//     pmevtyper20,
//     pmevtyper21,
//     pmevtyper22,
//     pmevtyper23,
//     pmevtyper24,
//     pmevtyper25,
//     pmevtyper26,
//     pmevtyper27,
//     pmevtyper28,
//     pmevtyper29,
//     pmevtyper30,
//     // @kwinter: add back the FEAT_PMUv3_SS features that are supposed to be here.
//     pmccfiltr,
//     // FEAT_PMUv3_ICNTR
//     pmicfiltr,

// };


enum pmu_register_el0 {
    // FEAT_PMUv3_EXT
    pmevcntr0,
    pmevcntr1,
    pmevcntr2,
    pmevcntr3,
    pmevcntr4,
    pmevcntr5,
    pmevcntr6,
    pmevcntr7,
    pmevcntr8,
    pmevcntr9,
    pmevcntr10,
    pmevcntr11,
    pmevcntr12,
    pmevcntr13,
    pmevcntr14,
    pmevcntr15,
    pmevcntr16,
    pmevcntr17,
    pmevcntr18,
    pmevcntr19,
    pmevcntr20,
    pmevcntr21,
    pmevcntr22,
    pmevcntr23,
    pmevcntr24,
    pmevcntr25,
    pmevcntr26,
    pmevcntr27,
    pmevcntr28,
    pmevcntr29,
    pmevcntr30,
    // FEAT_PMUv3_EXT
    pmccntr,
    // FEAT_PMUv3_ICNTR
    pmicntr,
    // FEAT_PMUv3_EXT
    pmevtyper0,
    pmevtyper1,
    pmevtyper2,
    pmevtyper3,
    pmevtyper4,
    pmevtyper5,
    pmevtyper6,
    pmevtyper7,
    pmevtyper8,
    pmevtyper9,
    pmevtyper10,
    pmevtyper11,
    pmevtyper12,
    pmevtyper13,
    pmevtyper14,
    pmevtyper15,
    pmevtyper16,
    pmevtyper17,
    pmevtyper18,
    pmevtyper19,
    pmevtyper20,
    pmevtyper21,
    pmevtyper22,
    pmevtyper23,
    pmevtyper24,
    pmevtyper25,
    pmevtyper26,
    pmevtyper27,
    pmevtyper28,
    pmevtyper29,
    pmevtyper30,
    pmccfiltr,
    // FEAT_PMUv3_ICNTR
    pmicfiltr,
    // FEAT_PMUv3_EXT
    pmcntenset,
    pmcntenclr,
    pmovsclr,
    // FEAT_PMUv3_EXT AND FEAT_PMUv3p9
    pmzr,
    // FEAT_PMUv3_EXT
    pmovsset,
    pmcr,
    // The following registers allow us to enable/disable interrupts.
    pmintenset,
    pmintenclr,
};

// Definitions of counters available on PMU
#define EVENT_CTR_0 0
#define EVENT_CTR_1 1
#define EVENT_CTR_2 2
#define EVENT_CTR_3 3
#define EVENT_CTR_4 4
#define EVENT_CTR_5 5
#define CYCLE_CTR 6

/* PMU Register Names */
#define PMU_CYCLE_CTR "pmccntr_el0"
#define PMCR_EL0 "pmcr_el0"
#define PMCNTENSET_EL0 "pmcntenset_el0"
#define PMOVSCLR_EL0 "pmovsclr_el0"
#define PMINTENSET_EL1 "pmintenset_el1"
/* Event select register */
#define PMSELR_EL0 "pmselr_el0"
#define PMXEVCNTR_EL0 "pmxevcntr_el0"
#define PMXEVTYPER_EL0 "pmxevtyper_el0"

exception_t decodePMUControlInvocation(word_t label, unsigned int length, cptr_t cptr,
                                         cte_t *srcSlot, cap_t cap,
                                         bool_t call, word_t *buffer);

#ifdef CONFIG_THREAD_LOCAL_PMU
/* Store the PMU state into the user context */
static inline void savePmuState(pmu_state_t *pmu_state)
{
    // pmu_state_t *pmuState = &thread->tcbArch.tcbContext.pmuState;
    /* @kwinter: Should we disable the PMU here, or further up in the context
    switching callchain?? Should we really disable at all? To disable or not to disable,
    that is the question. */
    MRS(PMU_CYCLE_CTR, pmu_state->cycle_counter);

    // Get the number of counters available on this platform
    uint32_t ctrl_reg;
    MRS(PMCR_EL0, ctrl_reg);
    uint32_t num_counters = (ctrl_reg >> 11) & 0x1f;

    for (int i = 0; i < num_counters; i++) {
        MSR(PMSELR_EL0, (1 << i));
        MRS(PMXEVCNTR_EL0, pmu_state->event_counters[i]);
        MRS(PMXEVTYPER_EL0, pmu_state->event_counters[i]);
    }
    MRS(PMCR_EL0, pmu_state->pmcr);
    MRS(PMCNTENSET_EL0, pmu_state->pmcntenset);
    MRS(PMOVSCLR_EL0, pmu_state->pmovsclr);
    MRS(PMINTENSET_EL1, pmu_state->pmintenset);
}

/* Load the PMU state from the user context into the PMU */
static inline void loadPmuState(pmu_state_t *pmu_state)
{
    /* @kwinter: Should we allow a write to enable the cycle counter here? Or disable
    it until we finish the context switching process. Would then have to keep
    track of even more state. */

    MSR(PMCR_EL0, pmu_state->pmcr);
    MSR(PMCNTENSET_EL0, pmu_state->pmcntenset);
    MSR(PMOVSCLR_EL0, pmu_state->pmovsclr);
    MSR(PMINTENSET_EL1, pmu_state->pmintenset);

    MSR(PMU_CYCLE_CTR, pmu_state->cycle_counter);

    // Get the number of counters available on this platform
    uint32_t ctrl_reg;
    MRS(PMCR_EL0, ctrl_reg);
    uint32_t num_counters = (ctrl_reg >> 11) & 0x1f;

    for (int i = 0; i < num_counters; i++) {
        MSR(PMSELR_EL0, (1 << i));
        MSR(PMXEVCNTR_EL0, pmu_state->event_counters[i]);
        MSR(PMXEVTYPER_EL0, pmu_state->event_counters[i]);
    }
}

static inline void restorePmuState(tcb_t *thread)
{
    if ((NODE_STATE(ksCurThread)->tcbArch.vpmu != NULL) &&
        (thread->tcbArch.vpmu != NULL)) {
        savePmuState(&NODE_STATE(ksCurThread)->tcbArch.vpmu->reg_state);
        loadPmuState(&NODE_STATE(ksCurThread)->tcbArch.vpmu->reg_state);
    } else if (NODE_STATE(ksCurThread)->tcbArch.vpmu != NULL) {
        // Transitioning from TCB doing per process monitoring to use global cpu
        // counters
        savePmuState(&NODE_STATE(ksCurThread)->tcbArch.vpmu->reg_state);
        loadPmuState(&ARCH_NODE_STATE(cpu_pmu_state).reg_state);
    } else if (thread->tcbFlags & seL4_TCBFlag_localPmuState) {
        // Transitioning from a TCB not doing per process monitoring to one that is
        savePmuState(&ARCH_NODE_STATE(cpu_pmu_state).reg_state);
        loadPmuState(&NODE_STATE(ksCurThread)->tcbArch.vpmu->reg_state);
    }
    /* In the case that we are transitioning between two TCB's not using per process
    PMU counters, then we don't need to alter any state, just continue to let the global
    counters persist. */

    /* @kwinter: There is an issue here related to how we set the TCB flags. As we can do
    this dynamically in this approach, then we should also do a simple save to the global counters
    every context switch. This is because if all threads are using the global PMU, then one decides
    to do per process monitoring, then all the global counters will reset back to 0, as the second
    case in the above if statement will be chosen, and the cpu_pmu_state will be unpopulated.
    We may need to change away from using these TCB flags. Is there a way to do this at build time? */
}
#endif /* CONFIG_THREAD_LOCAL_PMU */