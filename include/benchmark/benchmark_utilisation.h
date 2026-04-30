/*
 * Copyright 2016, General Dynamics C4 Systems
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <arch/benchmark.h>
#include <sel4/benchmark_utilisation_types.h>
#include <sel4/arch/constants.h>
#include <model/statedata.h>

#ifdef CONFIG_BENCHMARK_TRACK_UTILISATION
extern timestamp_t ksEnter;

void benchmark_track_utilisation_dump(void);

void benchmark_track_reset_utilisation(tcb_t *tcb);
/* Calculate and add the utilisation time from when the heir started to run i.e. scheduled
 * and until it's being kicked off
 */
static inline void benchmark_utilisation_switch(tcb_t *heir, tcb_t *next)
{
    /* Add heir thread utilisation */
    if (likely(NODE_STATE(benchmark_log_utilisation_enabled))) {

        /* Check if an overflow occurred while we have been in the kernel */
        if (likely(ksEnter > heir->benchmark.schedule_start_time)) {

            heir->benchmark.utilisation += (ksEnter - heir->benchmark.schedule_start_time);

        } else {
#ifdef CONFIG_ARM_ENABLE_PMU_OVERFLOW_INTERRUPT
            heir->benchmark.utilisation += (UINT32_MAX - heir->benchmark.schedule_start_time) + ksEnter;
            armv_handleOverflowIRQ();
#endif /* CONFIG_ARM_ENABLE_PMU_OVERFLOW_INTERRUPT */
        }

        /* Reset next thread utilisation */
        next->benchmark.schedule_start_time = ksEnter;
        next->benchmark.number_schedules++;
        NODE_STATE(benchmark_kernel_number_schedules)++;

#ifdef CONFIG_ARCH_AARCH64
        // assumes an Armv8.4 and earlier implementation that does not implement FEAT_PMUv3p5 and uses 32-bit event counters without overflows
        uint64_t pmu_events[6];
        uint64_t pmu_types[3];
        for (int i = 0; i < 6; i++) {
            asm volatile("msr PMSELR_EL0, %0" :: "r"(i));
            if (i % 2 == 1) {
                uint64_t type;
                asm volatile("mrs %0, PMXEVTYPER_EL0" : "=r"(type));
                pmu_types[i / 2] = type;
            }
            uint64_t value;
            asm volatile("mrs %0, PMXEVCNTR_EL0" : "=r"(value));
            pmu_events[i] = value;
        }

        for (int i = 0; i < 6; i++) {
            // for the chain event
            if (i % 2 == 0 && ((pmu_types[i / 2] & 0xFFFF) == 0x001E)) {
                uint64_t value = (heir->benchmark.pmu_events[i + 1] << 32) + heir->benchmark.pmu_events[i] + (pmu_events[i + 1] << 32) + pmu_events[i] - (heir->benchmark.pmu_events_start[i + 1] << 32) - heir->benchmark.pmu_events_start[i];
                heir->benchmark.pmu_events[i] = value & 0xFFFFFFFF;
                heir->benchmark.pmu_events[i + 1] = value >> 32;
                i += 1;
            } else {
                heir->benchmark.pmu_events[i] += pmu_events[i] - heir->benchmark.pmu_events_start[i];
            }
        }

        for (int i = 0; i < 6; i++) {
            next->benchmark.pmu_events_start[i] = pmu_events[i];
        }
#endif
    }
}

/* Add the time between the last thread got scheduled and when to stop
 * benchmarks
 */
static inline void benchmark_utilisation_finalise(void)
{
    /* Add the time between when NODE_STATE(ksCurThread), and benchmark finalise */
    benchmark_utilisation_switch(NODE_STATE(ksCurThread), NODE_STATE(ksIdleThread));

    NODE_STATE(benchmark_end_time) = ksEnter;
    NODE_STATE(benchmark_log_utilisation_enabled) = false;
}

#endif /* CONFIG_BENCHMARK_TRACK_UTILISATION */
