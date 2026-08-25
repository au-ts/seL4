#include <config.h>
#include <arch/object/vpmu.h>
#include <mode/machine/registerset.h>

#ifdef CONFIG_THREAD_LOCAL_PMU

UP_STATE_DEFINE(vpmu_t, cpu_pmu_state);
UP_STATE_DEFINE(vpmu_t *, armCurVPMU);

#define PMCR_GET_NUM_CTRS(pmcr) ((pmcr >> 11) & 0x1f)

// returns true if suspended.
// can't do global variables because of possible in-kernel preemption.
static inline bool_t beginVpmuTransaction(vpmu_t *vpmu)
{
        switch (thread_state_get_tsType(vpmu->tcb->tcbState)) {
#ifdef CONFIG_VTX
            case ThreadState_RunningVM:
#endif
            case ThreadState_Running:
                // preempt the tcb. this should force it to save the registers
                invokeTCB_Suspend(vpmu->tcb);
                assert(thread_state_get_tsType(vpmu->tcb->tcbState) == ThreadState_Inactive);
                return true;
        }
        return false;
}
static inline void endVpmuTransaction(vpmu_t *vpmu, bool_t suspended)
{
    if (!suspended) return;
    invokeTCB_Resume(vpmu->tcb);
}

static exception_t decodeVPMUControl_ReadCycleCounter(vpmu_t *vpmu, bool_t call)
{
    tcb_t *thread = NODE_STATE(ksCurThread);
    setThreadState(thread, ThreadState_Restart);

    if (call) {
        bool_t sus = beginVpmuTransaction(vpmu);
        // if the vpmu is not the current thread then we cannot just read from pmu_regs, as that
        // would be the old value. In this we preempt the bound TCB to perform a ctxt switch,
        // which would update the registers.

        word_t *ipcBuffer = lookupIPCBuffer(true, thread);
        setRegister(thread, badgeRegister, 0);
        unsigned int length = setMR(thread, ipcBuffer, 0, vpmu->reg_state.cycle_counter);
        setRegister(thread, msgInfoRegister, wordFromMessageInfo(
                        seL4_MessageInfo_new(0, 0, 0, length)));
        endVpmuTransaction(vpmu, sus);
    }

    setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);

    return EXCEPTION_NONE;
}

static exception_t decodeVPMUControl_WriteCycleCounter(word_t *buffer, vpmu_t *vpmu)
{
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

    seL4_Word counter_value = getSyscallArg(0, buffer);


    bool_t sus = beginVpmuTransaction(vpmu);
    vpmu->reg_state.cycle_counter = counter_value;
    endVpmuTransaction(vpmu, sus);


    return EXCEPTION_NONE;
}


/* FEAT_PMUv3_EXT */
static exception_t decodeVPMUControl_CounterControl(word_t *buffer, vpmu_t *vpmu)
{
    seL4_Word cntl_val = getSyscallArg(0, buffer);

    if (cntl_val > 2) {
        userError("PMUControl_CounterControl: Invalid control value. Must be 0, 1 or 2.");
        current_syscall_error.type = seL4_InvalidArgument;
        return EXCEPTION_SYSCALL_ERROR;
    }

    // TODO: @0aids move error checking outside of this function.
    // not calling so no running
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

    bool_t sus = beginVpmuTransaction(vpmu);
    pmu_state_t *pmu_regs = &vpmu->reg_state; 

    uint32_t pmcr = 0;
    uint32_t pmcntenset = 0;
    pmcr = pmu_regs->pmcr;
    pmcntenset = pmu_regs->pmcntenset;

    switch(cntl_val) {
        case 0: {
            uint32_t mask = 0;

            /* Disable Performance Counter */
            mask = 0;
            mask |= (1 << 0);
            pmcr = (pmcr & ~mask);

            /* Disable cycle counter register */
            mask = 0;
            mask |= (1 << 31);
            pmcntenset = (pmcntenset & ~mask);
            
            break;
        }
        case 1: {
            pmcr |= BIT(0);
            pmcntenset = BIT(31);
            break;
        }
        case 2: {
            uint32_t mask = 0;
            mask |= (1 << 1); /* Cycle counter reset */
            mask |= (1 << 2); /* Reset all counters */
            pmcr = (pmcr & ~mask);
            pmu_regs->cycle_counter = 0;
            // event counters are not implemented.
            break;
        }
        default:
            break;
    }


    pmu_regs->pmcr = pmcr;
    pmu_regs->pmcntenset = pmcntenset;
    endVpmuTransaction(vpmu, sus);

    return EXCEPTION_NONE;
}

static exception_t decodeVPMUControl_NumCounters(cap_t cap, bool_t call)
{
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
	if (call)
	{
        // Find number of counters available on hardware, the VPMU will match this
        uint32_t ctrl_reg;
        MRS(PMCR_EL0, ctrl_reg);
        uint32_t num_counters = (ctrl_reg >> 11) & 0x1f;

        tcb_t *thread = NODE_STATE(ksCurThread);
        word_t *ipcBuffer = lookupIPCBuffer(true, thread);
        setRegister(thread, badgeRegister, 0);

        unsigned int length = setMR(thread, ipcBuffer, 0, num_counters);

        setRegister(thread, msgInfoRegister, wordFromMessageInfo(
                        seL4_MessageInfo_new(0, 0, 0, length)));
	}
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);

    return EXCEPTION_NONE;
}

// TODO: @0aids Error checking before invocations.
exception_t decodeARMVPMUInvocation(word_t label, unsigned int length, cptr_t cptr,
                                         cte_t *srcSlot, cap_t cap,
                                         bool_t call, word_t *buffer)
{
    vpmu_t *vpmu = VPMU_PTR(cap_vpmu_cap_get_capPMUPtr(cap));
    // initialise the vpmu if it is not.
    tryInitialiseVPMU(vpmu);

    switch(label) {
        case VPMUReadCycleCounter:
            return decodeVPMUControl_ReadCycleCounter(vpmu, call);
        case VPMUWriteCycleCounter:
            return decodeVPMUControl_WriteCycleCounter(buffer, vpmu);
        case VPMUCounterControl:
            return decodeVPMUControl_CounterControl(buffer, vpmu);
        case VPMUNumCounters:
            return decodeVPMUControl_NumCounters(cap, call);
        default:
            userError("PMUControl invocation: Illegal operation attempted.");
            current_syscall_error.type = seL4_IllegalOperation;
            return EXCEPTION_SYSCALL_ERROR;
    }

    return EXCEPTION_NONE;
}

#endif /* CONFIG_THREAD_LOCAL_PMU */
