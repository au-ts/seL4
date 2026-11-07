/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright 2015, 2016 Hesham Almatary <heshamelmatary@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <types.h>
#include <api/failures.h>

#include <arch/object/interrupt.h>
#ifdef CONFIG_RISCV_APLIC
#include <model/statedata.h>
#endif

exception_t Arch_checkIRQ(word_t irq)
{
#ifdef CONFIG_RISCV_AIA
    if (irq > IMSIC_NUM_IDS || irq == irqInvalid) {
#else
    if (irq > maxIRQ || irq == irqInvalid) {
#endif
        current_syscall_error.type = seL4_RangeError;
        current_syscall_error.rangeErrorMin = 1;
#ifdef CONFIG_RISCV_AIA
        current_syscall_error.rangeErrorMax = IMSIC_NUM_IDS;
#else
        current_syscall_error.rangeErrorMax = maxIRQ;
#endif
        userError("Rejecting request for IRQ %u. IRQ is out of range [1..%u].", (int)irq, maxIRQ);
        return EXCEPTION_SYSCALL_ERROR;
    }
    return EXCEPTION_NONE;
}


#if !defined(CONFIG_RISCV_AIA) || defined(CONFIG_RISCV_APLIC)
#ifdef CONFIG_RISCV_APLIC
static exception_t Arch_invokeIRQControl(irq_t irq, cte_t *handlerSlot, cte_t *controlSlot, word_t source)
#else
static exception_t Arch_invokeIRQControl(irq_t irq, cte_t *handlerSlot, cte_t *controlSlot, bool_t trigger)
#endif
{
#ifdef CONFIG_RISCV_APLIC
    intStateIRQTable[IRQT_TO_IDX(irq)].aplicSource = source;
#elif defined(HAVE_SET_TRIGGER)
    setIRQTrigger(irq, trigger);
#endif
    return invokeIRQControl(irq, handlerSlot, controlSlot);
}
#endif

exception_t Arch_decodeIRQControlInvocation(word_t invLabel, word_t length,
                                            cte_t *srcSlot, word_t *buffer)
{
#if !defined(CONFIG_RISCV_AIA) || defined(CONFIG_RISCV_APLIC)
#ifdef CONFIG_RISCV_APLIC
    if (invLabel == RISCVIRQIssueIRQHandlerAPLIC) {
        if (length < 5 || current_extra_caps.excaprefs[0] == NULL) {
#else
    if (invLabel == RISCVIRQIssueIRQHandlerTrigger) {
        if (length < 4 || current_extra_caps.excaprefs[0] == NULL) {
#endif
            current_syscall_error.type = seL4_TruncatedMessage;
            return EXCEPTION_SYSCALL_ERROR;
        }

#ifndef CONFIG_RISCV_AIA
        if (!config_set(HAVE_SET_TRIGGER)) {
            userError("This platform does not support setting the IRQ trigger");
            current_syscall_error.type = seL4_IllegalOperation;
            return EXCEPTION_SYSCALL_ERROR;
        }
#endif

        word_t irq_w = getSyscallArg(0, buffer);
        irq_t irq = (irq_t) irq_w;
#ifdef CONFIG_RISCV_APLIC
        word_t source  = getSyscallArg(1, buffer);
        word_t trigger = getSyscallArg(2, buffer);
        word_t index   = getSyscallArg(3, buffer);
        word_t depth   = getSyscallArg(4, buffer);
#else
        bool_t trigger = !!getSyscallArg(1, buffer);
        word_t index = getSyscallArg(2, buffer);
        word_t depth = getSyscallArg(3, buffer);
#endif

        cap_t cnodeCap = current_extra_caps.excaprefs[0]->cap;

        exception_t status = Arch_checkIRQ(irq_w);
        if (status != EXCEPTION_NONE) {
            return status;
        }

        if (isIRQActive(irq)) {
            current_syscall_error.type = seL4_RevokeFirst;
            userError("Rejecting request for IRQ %u. Already active.", (int)irq);
            return EXCEPTION_SYSCALL_ERROR;
        }

#ifdef CONFIG_RISCV_APLIC
        if (source == 0 || source > APLIC_NUM_SOURCES) {
            current_syscall_error.type = seL4_RangeError;
            current_syscall_error.rangeErrorMin = 1;
            current_syscall_error.rangeErrorMax = APLIC_NUM_SOURCES;
            return EXCEPTION_SYSCALL_ERROR;
        }
        if (aplic->sourcecfg[source - 1] != APLIC_SOURCECFG_INACTIVE) {
            current_syscall_error.type = seL4_RevokeFirst;
            userError("APLIC source %lu is already active.", source);
            return EXCEPTION_SYSCALL_ERROR;
        }
#endif
        lookupSlot_ret_t lu_ret = lookupTargetSlot(cnodeCap, index, depth);
        if (lu_ret.status != EXCEPTION_NONE) {
            userError("Target slot for new IRQ Handler cap invalid: cap %lu, IRQ %u.",
                      getExtraCPtr(buffer, 0), (int)irq);
            return lu_ret.status;
        }

        cte_t *destSlot = lu_ret.slot;

        status = ensureEmptySlot(destSlot);
        if (status != EXCEPTION_NONE) {
            userError("Target slot for new IRQ Handler cap not empty: cap %lu, IRQ %u.",
                      getExtraCPtr(buffer, 0), (int)irq);
            return status;
        }

#ifdef CONFIG_RISCV_APLIC
        if (!aia_irq_configure(irq, source, trigger)) {
            current_syscall_error.type = seL4_InvalidArgument;
            current_syscall_error.invalidArgumentNumber = 2;
            userError("Unsupported trigger %lu for IRQ %u, SOURCE %u.", trigger, (int)irq, (int)source);
            return EXCEPTION_SYSCALL_ERROR;
        }
#endif

        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
#ifdef CONFIG_RISCV_APLIC
        return Arch_invokeIRQControl(irq, destSlot, srcSlot, source);
#else
        return Arch_invokeIRQControl(irq, destSlot, srcSlot, trigger);
#endif
    }
#endif
    current_syscall_error.type = seL4_IllegalOperation;
    return EXCEPTION_SYSCALL_ERROR;
}
