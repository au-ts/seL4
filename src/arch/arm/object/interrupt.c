/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "api/types.h"
#include "arch/machine.h"
#include <types.h>
#include <api/failures.h>
#include <config.h>

#include <arch/object/interrupt.h>

static exception_t Arch_invokeIRQControl(irq_t irq, cte_t *handlerSlot, cte_t *controlSlot, bool_t trigger)
{
#ifdef HAVE_SET_TRIGGER
    plat_setIRQTrigger(irq, trigger);
#endif
    return invokeIRQControl(irq, handlerSlot, controlSlot);
}


#ifndef CONFIG_ENABLE_SMP_SUPPORT

static exception_t Arch_invokeIssueSGISignal(word_t irq, word_t target, cte_t *sgiSlot, cte_t *controlSlot)
{
    cteInsert(cap_sgi_signal_cap_new(target, irq), controlSlot, sgiSlot);
    return EXCEPTION_NONE;
}

#endif

exception_t Arch_decodeIRQControlInvocation(word_t invLabel, word_t length,
                                            cte_t *srcSlot, word_t *buffer)
{
    if (invLabel == ARMIRQIssueIRQHandlerTrigger) {
        if (length < 4 || current_extra_caps.excaprefs[0] == NULL) {
            current_syscall_error.type = seL4_TruncatedMessage;
            return EXCEPTION_SYSCALL_ERROR;
        }

        if (!config_set(HAVE_SET_TRIGGER)) {
            userError("This platform does not support setting the IRQ trigger");
            current_syscall_error.type = seL4_IllegalOperation;
            return EXCEPTION_SYSCALL_ERROR;
        }

        word_t irq_w = getSyscallArg(0, buffer);
        irq_t irq = (irq_t) CORE_IRQ_TO_IRQT(0, irq_w);
        bool_t trigger = !!getSyscallArg(1, buffer);
        word_t index = getSyscallArg(2, buffer);
        word_t depth = getSyscallArg(3, buffer);

        cap_t cnodeCap = current_extra_caps.excaprefs[0]->cap;

        exception_t status = Arch_checkIRQ(irq_w);
        if (status != EXCEPTION_NONE) {
            return status;
        }

#if defined ENABLE_SMP_SUPPORT
        if (IRQ_IS_PPI(irq)) {
            userError("Trying to get a handler on a PPI: use GetTriggerCore.");
            current_syscall_error.type = seL4_IllegalOperation;
            return EXCEPTION_SYSCALL_ERROR;
        }
#endif
        if (isIRQActive(irq)) {
            current_syscall_error.type = seL4_RevokeFirst;
            userError("Rejecting request for IRQ %u. Already active.", (int)IRQT_TO_IRQ(irq));
            return EXCEPTION_SYSCALL_ERROR;
        }

        // This is just debugging. It's fine to set up an IRQ handler on any core at any time.
        // Whilst at the moment the setTrigger will however modify global state, and maybe
        // should be moved into a separate call?
        plat_getIRQTarget_ret_t target_ret = plat_getIRQTarget(irq);
        printf("getting trigger for irq %u with target set as %u (valid if 0: %lu)\n", (int)IRQT_TO_IRQ(irq), target_ret.target, target_ret.status);

        lookupSlot_ret_t lu_ret = lookupTargetSlot(cnodeCap, index, depth);
        if (lu_ret.status != EXCEPTION_NONE) {
            userError("Target slot for new IRQ Handler cap invalid: cap %lu, IRQ %u.",
                      getExtraCPtr(buffer, 0), (int)IRQT_TO_IRQ(irq));
            return lu_ret.status;
        }

        cte_t *destSlot = lu_ret.slot;

        status = ensureEmptySlot(destSlot);
        if (status != EXCEPTION_NONE) {
            userError("Target slot for new IRQ Handler cap not empty: cap %lu, IRQ %u.",
                      getExtraCPtr(buffer, 0), (int)IRQT_TO_IRQ(irq));
            return status;
        }

        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        return Arch_invokeIRQControl(irq, destSlot, srcSlot, trigger);
#ifndef CONFIG_ENABLE_SMP_SUPPORT
    } else if (invLabel == ARMIRQIssueSGISignal) {
        if (length < 4 || current_extra_caps.excaprefs[0] == NULL) {
            userError("IRQControl: IssueSGISignal: Truncated message.");
            current_syscall_error.type = seL4_TruncatedMessage;
            return EXCEPTION_SYSCALL_ERROR;
        }
        word_t irq = getSyscallArg(0, buffer);
        word_t target = getSyscallArg(1, buffer);
        word_t index = getSyscallArg(2, buffer);
        word_t depth = getSyscallArg(3, buffer);

        cap_t cnodeCap = current_extra_caps.excaprefs[0]->cap;

        if (irq >= NUM_SGIS) {
            current_syscall_error.type = seL4_RangeError;
            current_syscall_error.rangeErrorMin = 0;
            current_syscall_error.rangeErrorMax = NUM_SGIS - 1;
            userError("IRQControl: IssueSGISignal: Invalid SGI IRQ 0x%lx.", irq);
            return EXCEPTION_SYSCALL_ERROR;
        }

        if (!plat_SGITargetValid(target)) {
            current_syscall_error.type = seL4_InvalidArgument;
            current_syscall_error.invalidArgumentNumber = 1;
            userError("IRQControl: IssueSGISignal: Invalid SGI Target 0x%lx.", target);
            return EXCEPTION_SYSCALL_ERROR;
        }

        lookupSlot_ret_t lu_ret = lookupTargetSlot(cnodeCap, index, depth);
        if (lu_ret.status != EXCEPTION_NONE) {
            userError("IRQControl: IssueSGISignal: Target slot for new ARM_SGI_Signal cap invalid: cap %lu.",
                      getExtraCPtr(buffer, 0));
            return lu_ret.status;
        }
        cte_t *destSlot = lu_ret.slot;

        exception_t status = ensureEmptySlot(destSlot);
        if (status != EXCEPTION_NONE) {
            userError("IRQControl: IssueSGISignal: Target slot for new ARM_SGI_Signal cap not empty: cap %lu.",
                      getExtraCPtr(buffer, 0));
            return status;
        }
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        return Arch_invokeIssueSGISignal(irq, target, destSlot, srcSlot);

#endif
#ifdef ENABLE_SMP_SUPPORT
    } else if (invLabel == ARMIRQIssueIRQHandlerTriggerCore) {
        word_t irq_w = getSyscallArg(0, buffer);
        bool_t trigger = !!getSyscallArg(1, buffer);
        word_t index = getSyscallArg(2, buffer);
        word_t depth = getSyscallArg(3, buffer) & 0xfful;
        seL4_Word target = getSyscallArg(4, buffer);
        cap_t cnodeCap = current_extra_caps.excaprefs[0]->cap;
        exception_t status = Arch_checkIRQ(irq_w);
        irq_t irq = CORE_IRQ_TO_IRQT(target, irq_w);

        if (status != EXCEPTION_NONE) {
            return status;
        }

        if (target >= CONFIG_MAX_NUM_NODES) {
            current_syscall_error.type = seL4_InvalidArgument;
            userError("Target core %lu is invalid.", target);
            return EXCEPTION_SYSCALL_ERROR;
        }

        if (isIRQActive(irq)) {
            current_syscall_error.type = seL4_RevokeFirst;
            userError("Rejecting request for IRQ %u. Already active.", (int)IRQT_TO_IRQ(irq));
            return EXCEPTION_SYSCALL_ERROR;
        }

        lookupSlot_ret_t lu_ret = lookupTargetSlot(cnodeCap, index, depth);
        if (lu_ret.status != EXCEPTION_NONE) {
            userError("Target slot for new IRQ Handler cap invalid: cap %lu, IRQ %u.",
                      getExtraCPtr(buffer, 0), (int)IRQT_TO_IRQ(irq));
            return lu_ret.status;
        }

        cte_t *destSlot = lu_ret.slot;

        status = ensureEmptySlot(destSlot);
        if (status != EXCEPTION_NONE) {
            userError("Target slot for new IRQ Handler cap not empty: cap %lu, IRQ %u.",
                      getExtraCPtr(buffer, 0), (int)IRQT_TO_IRQ(irq));
            return status;
        }

        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

        /* If the IRQ is not a private interrupt, then the role of the syscall is to set
         * target core to which the shared interrupt will be physically delivered.
         */
        if (!IRQ_IS_PPI(irq)) {
            plat_setIRQTarget(irq, target);
        }
        return Arch_invokeIRQControl(irq, destSlot, srcSlot, trigger);
#endif /* ENABLE_SMP_SUPPORT */
#ifdef CONFIG_ENABLE_MULTIKERNEL_SUPPORT
    } else if (invLabel == ARMIRQSetIrqTargetCore) {
        word_t irq_w = getSyscallArg(0, buffer);
        seL4_Word target = getSyscallArg(1, buffer);

        exception_t status = Arch_checkIRQ(irq_w);
        if (status != EXCEPTION_NONE) {
            return status;
        }


        /* XXX: For now, we only support GICv2.
                It will probably work on GICv3 but I need to check the
                semantics
         */
        assert(!config_set(CONFIG_ARM_GIC_V3_SUPPORT));


        irq_t irq = CORE_IRQ_TO_IRQT(target, irq_w);
        if (IRQ_IS_PPI(irq) /* or SGI, implicitly */) {
            current_syscall_error.type = seL4_RangeError;
            current_syscall_error.rangeErrorMin = 0;
            current_syscall_error.rangeErrorMax = NUM_SGIS - 1;
            /* does this do anything */
            current_syscall_error.invalidArgumentNumber = 1;
            userError("IRQControl: SetIrqTargetCore: Invalid IRQ (not an SPI) 0x%lx.", irq);
            return EXCEPTION_SYSCALL_ERROR;
        }

        /* gic_infer_cpu_id or whatever */
        plat_getIRQTarget_ret_t self_target_ret = plat_getIRQTarget(0);
        assert(self_target_ret.status == EXCEPTION_NONE);
        uint8_t self_target = self_target_ret.status;


        plat_getIRQTarget_ret_t irq_target_ret = plat_getIRQTarget(irq);
        if (irq_target_ret.status != EXCEPTION_NONE || irq_target_ret.target != self_target) {
            userError("IRQControl: SetIrqTargetCore: target of the irq is not us, cannot change");
            // TODO.
            return EXCEPTION_SYSCALL_ERROR;
        }



/*

        Interrupt targets range from "CPU interface 0" to "CPU interface 7".

*/
        // IDK, validate this? can we?
        if (target >= 8) {
            current_syscall_error.type = seL4_InvalidArgument;
            userError("Target core %lu is invalid.", target);
            return EXCEPTION_SYSCALL_ERROR;
        }


// GICv2:
// The effect of changes to an GICD_ITARGETSR
// Software can write to an GICD_ITARGETSR at any time. Any change to a CPU targets field value:
// •Has no effect on any active interrupt. This means that removing a CPU interface from a targets list does not
// cancel an active state for that interrupt on that CPU interface.
// •Has an effect on any pending interrupts. This means:
// —adding a CPU interface to the target list of a pending interrupt makes that interrupt pending on that
// CPU interface
// —removing a CPU interface from the target list of a pending interrupt removes the pending state of that
// interrupt on that CPU interface.
// Note
// There is a small but finite time required for any change to take effect.
// •
// 4-108
// If it applies to an interrupt that is active and pending, does not change the interrupt targets until the active
// status is cleared
// In a uniprocessor implementation, all interrupts target the one processor, and the
// GICD_ITARGETSRs are RAZ/WI

        // ==> From an seL4 perspective, we can just change the GICD_ITARGETSRni
        //     at any time. For kernels processing those interrupts, they will
        //     look like they disappear.
        // TODO: We should prevent multiple kernels from writing to this;
        //       either just by only giving the cap for this (?) or just returning
        //       an error if the current cpu index is non-zero (or "primary").

        // Writes to other GICD being shared are prevented by migrated to using
        // the EOI GICC register. (TODO)

        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

        /* TODO: This target is the raw GIC interface number, which is not necessarily
                 the same as a logical core ID */
        plat_setIRQTarget(irq, target);
        return EXCEPTION_NONE;

#endif /* CONFIG_ENABLE_MULTIKERNEL_SUPPORT */
    } else {
        current_syscall_error.type = seL4_IllegalOperation;
        return EXCEPTION_SYSCALL_ERROR;
    }
}

#ifndef CONFIG_ENABLE_SMP_SUPPORT

static exception_t invokeSGISignalGenerate(word_t irq, word_t target)
{
    plat_sendSGI(irq, target);
    return EXCEPTION_NONE;
}


exception_t decodeSGISignalInvocation(word_t invLabel, word_t length,
                                      cap_t cap, word_t *buffer)
{

    word_t irq = cap_sgi_signal_cap_get_capSGIIRQ(cap);
    word_t target = cap_sgi_signal_cap_get_capSGITarget(cap);

    setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
    return invokeSGISignalGenerate(irq, target);
}
#endif /* !CONFIG_ENABLE_SMP_SUPPORT */
