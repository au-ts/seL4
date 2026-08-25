#include <config.h>
#include <arch/object/vpmu.h>
#include <mode/machine/registerset.h>

#ifdef CONFIG_THREAD_LOCAL_PMU

UP_STATE_DEFINE(vpmu_t, cpu_pmu_state);
UP_STATE_DEFINE(vpmu_t *, armCurVPMU);

#define PMCR_GET_NUM_CTRS(pmcr) ((pmcr >> 11) & 0x1f)

/* FEAT_PMUv3_EXT */
static exception_t decodeVPMUControl_ReadEventCounter(word_t *buffer, vpmu_t *vpmu, bool_t call)
{
    seL4_Word counter = getSyscallArg(0, buffer);

    pmu_state_t *pmu_regs = &vpmu->reg_state; 

    // Validate the counter is within range. We will match the number of counters available to the VPMU with that of hardware.
    uint32_t num_counters = PMCR_GET_NUM_CTRS(pmu_regs->pmcr);

    if (counter > num_counters && counter < 32) {
        userError("PMUControl_CounterControl: Invalid counter.");
        current_syscall_error.type = seL4_InvalidArgument;
        return EXCEPTION_SYSCALL_ERROR;
    }

    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

    if (call) {
		uint32_t res = 0;
        res = pmu_regs->event_counters[counter];
        tcb_t *thread = NODE_STATE(ksCurThread);

        word_t *ipcBuffer = lookupIPCBuffer(true, thread);
        setRegister(thread, badgeRegister, 0);
        unsigned int length = setMR(thread, ipcBuffer, 0, res);
        setRegister(thread, msgInfoRegister, wordFromMessageInfo(
                        seL4_MessageInfo_new(0, 0, 0, length)));
    }
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);

    return EXCEPTION_NONE;
}

/* FEAT_PMUv3_EXT */
static exception_t decodeVPMUControl_WriteEventCounter(word_t *buffer, vpmu_t *vpmu)
{
    userError("PMUControl_CounterControl: TODO");
    current_syscall_error.type = seL4_InvalidArgument;
    return EXCEPTION_SYSCALL_ERROR;
    // seL4_Word counter = getSyscallArg(0, buffer);
    // seL4_Word value = getSyscallArg(1, buffer);
    // seL4_Word event = getSyscallArg(2, buffer);

    // pmu_state_t *pmu_regs = &vpmu->reg_state; 

    // uint32_t num_counters = PMCR_GET_NUM_CTRS(pmu_regs->pmcr);

    // if (counter > num_counters && counter < 32) {
    //     userError("PMUControl_CounterControl: Invalid counter.");
    //     current_syscall_error.type = seL4_InvalidArgument;
    //     return EXCEPTION_SYSCALL_ERROR;
    // }

    // // TODO: @0aids move error checking outta here.^^

    // setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

    // if (ARCH_NODE_STATE(armCurVPMU) == vpmu) {
    //     uint32_t cnt_sel = 1 << counter;

    //     MSR(PMSELR_EL0, cnt_sel);
    //     MSR(PMXEVCNTR_EL0, value);
    //     isb();
    //     MSR(PMXEVTYPER_EL0, event);
    //     isb();
    // } else {
    //     pmu_regs->event_counters[counter] = value;
    //     pmu_regs->event_type[counter] = event;
    // }
    // return EXCEPTION_NONE;
}

static exception_t decodeVPMUControl_ReadCycleCounter(vpmu_t *vpmu, bool_t call)
{
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

    if (call) {
        pmu_state_t *pmu_regs = &vpmu->reg_state; 

        tcb_t *thread = NODE_STATE(ksCurThread);

        word_t *ipcBuffer = lookupIPCBuffer(true, thread);
        setRegister(thread, badgeRegister, 0);
        unsigned int length = setMR(thread, ipcBuffer, 0, pmu_regs->cycle_counter);
        setRegister(thread, msgInfoRegister, wordFromMessageInfo(
                        seL4_MessageInfo_new(0, 0, 0, length)));
    }

    setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);

    return EXCEPTION_NONE;
}

static exception_t decodeVPMUControl_WriteCycleCounter(word_t *buffer, vpmu_t *vpmu)
{
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

    pmu_state_t *pmu_regs = &vpmu->reg_state; 

    seL4_Word counter_value = getSyscallArg(0, buffer);

    pmu_regs->cycle_counter = counter_value;

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
            break;
        }
        default:
            break;
    }


    pmu_regs->pmcr = pmcr;
    pmu_regs->pmcntenset = pmcntenset;

    return EXCEPTION_NONE;
}

/* FEAT_PMUv3_EXT */
static exception_t decodeVPMUControl_ReadInterruptValue(vpmu_t *vpmu, bool_t call)
{
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

    if (call) {
        pmu_state_t *pmu_regs = &vpmu->reg_state; 

        // Get the interrupt flag from the PMU
        uint32_t irqFlag = 0;

        if (ARCH_NODE_STATE(armCurVPMU) == vpmu) {
            MRS(PMOVSCLR_EL0, irqFlag);
        } else {
            irqFlag = pmu_regs->pmovsclr;
        }
        tcb_t *thread = NODE_STATE(ksCurThread);

        word_t *ipcBuffer = lookupIPCBuffer(true, thread);
        setRegister(thread, badgeRegister, 0);
        unsigned int length = setMR(thread, ipcBuffer, 0, irqFlag);
        setRegister(thread, msgInfoRegister, wordFromMessageInfo(
                        seL4_MessageInfo_new(0, 0, 0, length)));
    }
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);

    return EXCEPTION_NONE;
}

static exception_t decodeVPMUControl_WriteInterruptValue(word_t *buffer, vpmu_t *vpmu)
{
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
    pmu_state_t *pmu_regs = &vpmu->reg_state; 

    seL4_Word interrupt_value = getSyscallArg(0, buffer);

    pmu_regs->pmovsclr = interrupt_value;

    return EXCEPTION_NONE;
}

static exception_t decodeVPMUControl_InterruptControl(word_t *buffer, vpmu_t *vpmu)
{
    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
    pmu_state_t *pmu_regs = &vpmu->reg_state; 

    // TODO: @0aids add wellformed-ness checks outside of this function.
    // For this specific argument?
    seL4_Word interrupt_ctl = getSyscallArg(0, buffer);

    pmu_regs->pmintenset = interrupt_ctl;

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

static exception_t decodeVPMUControl_SetVIRQ(word_t length, cap_t cap, word_t *buffer, vpmu_t *vpmu)
{
    cap_t ntfnCap;
    cte_t *slot;

    if (current_extra_caps.excaprefs[0] == NULL) {
        current_syscall_error.type = seL4_TruncatedMessage;
        return EXCEPTION_SYSCALL_ERROR;
    }

    ntfnCap = current_extra_caps.excaprefs[0]->cap;
    slot = current_extra_caps.excaprefs[0];

    if (cap_get_capType(ntfnCap) != cap_notification_cap ||
        !cap_notification_cap_get_capNtfnCanSend(ntfnCap)) {
        if (cap_get_capType(ntfnCap) != cap_notification_cap) {
            userError("VPMUSetVIRQ: provided cap is not an notification capability.");
        } else {
            userError("VPMUSetVIRQ: caller does not have send rights on the endpoint.");
        }
        current_syscall_error.type = seL4_InvalidCapability;
        current_syscall_error.invalidCapNumber = 0;
        return EXCEPTION_SYSCALL_ERROR;
    }

    cteDeleteOne(&vpmu->virq_handler);
    cteInsert(ntfnCap, slot, &vpmu->virq_handler);

    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
    return EXCEPTION_NONE;
}

static exception_t decodeVPMUControl_AckVIRQ(word_t length, cap_t cap, word_t *buffer, vpmu_t *vpmu)
{
    vpmu->active_irq = 0;
    return EXCEPTION_NONE;
}


#ifdef CONFIG_PROFILER_ENABLE
static exception_t decodeVPMUControl_GetProfilingInfo(word_t length, cap_t cap, word_t *buffer, vpmu_t *vpmu)
{
    setRegister(NODE_STATE(ksCurThread), msgRegisters[0], vpmu->fp);
    setRegister(NODE_STATE(ksCurThread), msgRegisters[1], vpmu->pc);
    return EXCEPTION_NONE;
}
#endif /* CONFIG_PROFILER_ENABLE */

// TODO: @0aids Error checking before invocations.
exception_t decodeARMVPMUInvocation(word_t label, unsigned int length, cptr_t cptr,
                                         cte_t *srcSlot, cap_t cap,
                                         bool_t call, word_t *buffer)
{
    vpmu_t *vpmu = VPMU_PTR(cap_vpmu_cap_get_capPMUPtr(cap));

    switch(label) {
        case VPMUReadEventCounter:
            return decodeVPMUControl_ReadEventCounter(buffer, vpmu, call);
        case VPMUWriteEventCounter:
            return decodeVPMUControl_WriteEventCounter(buffer, vpmu);
        case VPMUReadCycleCounter:
            return decodeVPMUControl_ReadCycleCounter(vpmu, call);
        case VPMUWriteCycleCounter:
            return decodeVPMUControl_WriteCycleCounter(buffer, vpmu);
        case VPMUCounterControl:
            return decodeVPMUControl_CounterControl(buffer, vpmu);
        case VPMUReadInterruptValue:
            return decodeVPMUControl_ReadInterruptValue(vpmu, call);
        case VPMUWriteInterruptValue:
            return decodeVPMUControl_WriteInterruptValue(buffer, vpmu);
        case VPMUInterruptControl:
            return decodeVPMUControl_InterruptControl(buffer, vpmu);
        case VPMUNumCounters:
            return decodeVPMUControl_NumCounters(cap, call);
        case VPMUSetVIRQ:
            return decodeVPMUControl_SetVIRQ(length, cap, buffer, vpmu);
        case VPMUAckVIRQ:
            return decodeVPMUControl_AckVIRQ(length, cap, buffer, vpmu);
        #ifdef CONFIG_PROFILER_ENABLE
        case VPMUGetProfilingInfo:
            return decodeVPMUControl_GetProfilingInfo(length, cap, buffer, vpmu);
        #endif /* CONFIG_PROFILER_ENABLE */
        default:
            userError("PMUControl invocation: Illegal operation attempted.");
            current_syscall_error.type = seL4_IllegalOperation;
            return EXCEPTION_SYSCALL_ERROR;
    }

    return EXCEPTION_NONE;
}

uint8_t arm_vpmu_handle_irq(void)
{
    vpmu_t *curr_vpmu = NODE_STATE(ksCurThread)->tcbArch.vpmu;
    // First check that there is a valid vPMU bound to this thread
    if (NODE_STATE(ksCurThread)->tcbArch.vpmu == NULL) {
        return 0;
    }

    // Make sure that a valid interrupt handler has been set
    if (cap_get_capType(curr_vpmu->virq_handler.cap) != cap_notification_cap) {
        return 0;
    }

    // If the active_irq field is still true, then we will mark this interrupt
    // as delivered, but drop it here.
    if (curr_vpmu->active_irq == 1) {
        return 1;
    }

    curr_vpmu->active_irq = 1;

    #ifdef CONFIG_PROFILER_ENABLE
    // Record the FP and SP
    NODE_STATE(ksCurThread)->tcbArch.vpmu->pc = getRegister(NODE_STATE(ksCurThread), FaultIP);
    // Read the x29 register for the address of the current frame pointer
    NODE_STATE(ksCurThread)->tcbArch.vpmu->fp = getRegister(NODE_STATE(ksCurThread), X29);
    #endif /* CONFIG_PROFILER_ENABLE */

    // We will now deliver to the vIRQ endpoint
    if (cap_notification_cap_get_capNtfnCanSend(curr_vpmu->virq_handler.cap)) {
        sendSignal(NTFN_PTR(cap_notification_cap_get_capNtfnPtr(curr_vpmu->virq_handler.cap)),
                       cap_notification_cap_get_capNtfnBadge(curr_vpmu->virq_handler.cap));
    }

    return 1;
}

#endif /* CONFIG_THREAD_LOCAL_PMU */
