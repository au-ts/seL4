#pragma once

#include <types.h>
#include <api/failures.h>
#include <object/structures.h>
#include <mode/machine/registerset.h>

#ifdef CONFIG_THREAD_LOCAL_PMU
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

exception_t decodeARMVPMUInvocation(word_t label, unsigned int length, cptr_t cptr,
                                         cte_t *srcSlot, cap_t cap,
                                         bool_t call, word_t *buffer);

// called during kernel entry. Saves the PMU related registers into the TCB's own state,
// and restarts itself.
static inline void trySavePmuState(tcb_t *thread) {
    if (thread->tcbArch.vpmu == NULL) return;
    pmu_state_t *pmu_state = &thread->tcbArch.vpmu->reg_state;
    MRS(PMCR_EL0, pmu_state->pmcr);
    MRS(PMU_CYCLE_CTR, pmu_state->cycle_counter);
    MRS(PMCNTENSET_EL0, pmu_state->pmcntenset);
    MRS(PMOVSCLR_EL0, pmu_state->pmovsclr);

    // disable the pmu
    uint32_t pmcr_dis = pmu_state->pmcr & (~1u);
    MSR(PMCR_EL0, pmcr_dis);
    isb();
}

// called before kernel exit. Writes all PMU related registers back (where they are writable).
// Only restores it if the vpmu is bound.
static inline void tryRestorePmuState(tcb_t *thread) {
    if (thread->tcbArch.vpmu == NULL) return;
    pmu_state_t *pmu_state = &thread->tcbArch.vpmu->reg_state;
    MSR(PMCR_EL0, pmu_state->pmcr);
    MSR(PMU_CYCLE_CTR, pmu_state->cycle_counter);
    MSR(PMCNTENSET_EL0, pmu_state->pmcntenset);
    MSR(PMOVSCLR_EL0, pmu_state->pmovsclr);
    isb();
}

// Initialises the vpmu if it has not been initialised (determined by vpmu->pmcr)
static inline void tryInitialiseVPMU(vpmu_t *vpmu) {
    if (vpmu->reg_state.pmcr != 0) return;
    pmu_state_t *pmu_state = &vpmu->reg_state;
    pmu_state->cycle_counter = 0;
    MRS(PMCR_EL0, pmu_state->pmcr);
    MRS(PMCNTENSET_EL0, pmu_state->pmcntenset);
    MRS(PMOVSCLR_EL0, pmu_state->pmovsclr);
}

#endif /* CONFIG_THREAD_LOCAL_PMU */
