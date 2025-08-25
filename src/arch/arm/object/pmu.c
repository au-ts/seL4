#include <config.h>

#include <arch/object/pmu.h>
#include <mode/machine/registerset.h>

#define ISB asm volatile("isb")

// @kwinter: The following functions are tagged with which ARM extension impelments the features.
// Future work can extend the functionality that is available via this API.

/* FEAT_PMUv3_EXT */
static exception_t decodePMUControl_ReadEventCounter(word_t length, cap_t cap, word_t *buffer, word_t badge)
{
    seL4_Word counter = getSyscallArg(0, buffer);

    // Validate the counter is within range
    uint32_t ctrl_reg;
    MRS(PMCR_EL0, ctrl_reg);
    uint32_t num_counters = (ctrl_reg >> 11) & 0x1f;

    if (counter > num_counters) {
        userError("PMUControl_CounterControl: Invalid counter.");
        current_syscall_error.type = seL4_InvalidArgument;
        return EXCEPTION_SYSCALL_ERROR;
    }

    uint32_t cnt_sel = 1 << counter;

    uint32_t counter_value;

    MSR(PMSELR_EL0, cnt_sel);
    MRS(PMXEVCNTR_EL0, counter_value);

    setRegister(NODE_STATE(ksCurThread), msgRegisters[0], counter_value);
    return EXCEPTION_NONE;
}

/* FEAT_PMUv3_EXT */
static exception_t decodePMUControl_WriteEventCounter(word_t length, cap_t cap, word_t *buffer, word_t badge)
{
    seL4_Word counter = getSyscallArg(0, buffer);
    seL4_Word value = getSyscallArg(1, buffer);
    seL4_Word event = getSyscallArg(2, buffer);

    // Validate the counter is within range
    uint32_t ctrl_reg;
    MRS(PMCR_EL0, ctrl_reg);
    uint32_t num_counters = (ctrl_reg >> 11) & 0x1f;

    if (counter > num_counters) {
        userError("PMUControl_CounterControl: Invalid counter.");
        current_syscall_error.type = seL4_InvalidArgument;
        return EXCEPTION_SYSCALL_ERROR;
    }

    uint32_t cnt_sel = 1 << counter;

    MSR(PMSELR_EL0, cnt_sel);
    MSR(PMXEVCNTR_EL0, value);
    MSR(PMXEVTYPER_EL0, event);

    return EXCEPTION_NONE;
}

static exception_t decodePMUControl_ReadCycleCounter(word_t length, cap_t cap, word_t *buffer, word_t badge)
{
    return EXCEPTION_NONE;
}

static exception_t decodePMUControl_WriteCycleCounter(word_t length, cap_t cap, word_t *buffer, word_t badge)
{
    return EXCEPTION_NONE;
}


/* FEAT_PMUv3_EXT */
static exception_t decodePMUControl_CounterControl(word_t length, cap_t cap, word_t *buffer, word_t badge)
{
    seL4_Word cntl_val = getSyscallArg(0, buffer);

    if (cntl_val > 2) {
        userError("PMUControl_CounterControl: Invalid control value. Must be 0, 1 or 2.");
        current_syscall_error.type = seL4_InvalidArgument;
        return EXCEPTION_SYSCALL_ERROR;
    }

    switch(cntl_val) {
        case 0: {
            uint32_t value = 0;
            uint32_t mask = 0;

            /* Disable Performance Counter */
            MRS(PMCR_EL0, value);
            mask = 0;
            mask |= (1 << 0); /* Enable */
            mask |= (1 << 1); /* Cycle counter reset */
            mask |= (1 << 2); /* Reset all counters */
            MSR(PMCR_EL0, (value & ~mask));

            /* Disable cycle counter register */
            MRS(PMCNTENSET_EL0, value);
            mask = 0;
            mask |= (1 << 31);
            MSR(PMCNTENSET_EL0, (value & ~mask));
            break;
        }
        case 1: {
            uint32_t val;
            MRS(PMCR_EL0, val);
            val |= BIT(0);
            ISB;
            MSR(PMCR_EL0, val);
            MSR(PMCNTENSET_EL0, (BIT(31)));
            break;
        }
        case 2: {
            uint32_t value = 0;
            uint32_t mask = 0;
            MRS(PMCR_EL0, value);
            mask = 0;
            mask |= (1 << 1); /* Cycle counter reset */
            mask |= (1 << 2); /* Reset all counters */
            MSR(PMCR_EL0, (value & ~mask));
            break;
        }
        default:
            break;
    }

    return EXCEPTION_NONE;
}

/* FEAT_PMUv3_EXT */
static exception_t decodePMUControl_ReadInterruptValue(word_t length, cap_t cap, word_t *buffer, word_t badge)
{
    // Get the interrupt flag from the PMU
    uint32_t irqFlag = 0;
    MRS(PMOVSCLR_EL0, irqFlag);

    setRegister(NODE_STATE(ksCurThread), msgRegisters[0], irqFlag);

    return EXCEPTION_NONE;
}

static exception_t decodePMUControl_WriteInterruptValue(word_t length, cap_t cap, word_t *buffer, word_t badge)
{
    seL4_Word interrupt_value = getSyscallArg(0, buffer);

    MSR(PMOVSCLR_EL0, interrupt_value);

    return EXCEPTION_NONE;
}

static exception_t decodePMUControl_InterruptControl(word_t length, cap_t cap, word_t *buffer, word_t badge)
{
    seL4_Word interrupt_ctl = getSyscallArg(0, buffer);

    MSR(PMINTENSET_EL1, interrupt_ctl);

    return EXCEPTION_NONE;
}

static exception_t decodePMUControl_NumCounters(word_t length, cap_t cap, word_t *buffer, word_t badge)
{
    // Find number of counters available on hardware
    uint32_t ctrl_reg;
    MRS(PMCR_EL0, ctrl_reg);
    uint32_t num_counters = (ctrl_reg >> 11) & 0x1f;

    setRegister(NODE_STATE(ksCurThread), msgRegisters[0], num_counters);

    return EXCEPTION_NONE;
}

exception_t decodePMUControlInvocation(word_t label, unsigned int length, cptr_t cptr,
                                          cte_t *srcSlot, cap_t cap, bool_t call, word_t *buffer)
{
    word_t badge = cap_pmu_control_cap_get_capPMUBadge(cap);

    switch(label) {
        case PMUReadEventCounter:
            return decodePMUControl_ReadEventCounter(length, cap, buffer, badge);
        case PMUWriteEventCounter:
            return decodePMUControl_WriteEventCounter(length, cap, buffer, badge);
        case PMUReadCycleCounter:
            return decodePMUControl_ReadCycleCounter(length, cap, buffer, badge);
        case PMUWriteCycleCounter:
            return decodePMUControl_WriteCycleCounter(length, cap, buffer, badge);
        case PMUCounterControl:
            return decodePMUControl_CounterControl(length, cap, buffer, badge);
        case PMUReadInterruptValue:
            return decodePMUControl_ReadInterruptValue(length, cap, buffer, badge);
        case PMUWriteInterruptValue:
            return decodePMUControl_WriteInterruptValue(length, cap, buffer, badge);
        case PMUInterruptControl:
            return decodePMUControl_InterruptControl(length, cap, buffer, badge);
        case PMUNumCounters:
            return decodePMUControl_NumCounters(length, cap, buffer, badge);
        default:
            userError("PMUControl invocation: Illegal operation attempted.");
            current_syscall_error.type = seL4_IllegalOperation;
            return EXCEPTION_SYSCALL_ERROR;
    }
}
