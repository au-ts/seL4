/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <drivers/iommu/riscv.h>
#include <arch/machine.h>
#include <arch/model/statedata.h>
#include <machine.h>
#include <api/failures.h>
#include <api/syscall.h>
#include <arch/api/invocation.h>
#include <arch/kernel/vspace.h>
#include <kernel/thread.h>
#include <object/cnode.h>
#include <plat/machine/devices_gen.h>

#define IOMMU_CMD_IOTINVAL 1
#define IOMMU_CMD_IOFENCE  2
#define IOMMU_CMD_IODIR    3

typedef struct iommu_ddt_lookup {
    ddte_t *table;
    /* The entry linking table into its parent, or NULL for the root. */
    ddte_t *parent;
    void *slot;
    /* Zero identifies a device-context slot; otherwise slot is a non-leaf entry. */
    word_t bitsLeft;
} iommu_ddt_lookup_t;

static word_t iommuSubmit(riscv_iommu_regs_ptr_t regs, const uint64_t words[2]);
static void iommuInvalidateFirstStagePT(riscv_iommu_regs_ptr_t regs, word_t gscid);
static void iommuInvalidateDC(riscv_iommu_regs_ptr_t regs, word_t deviceID, word_t gscid);
static bool_t iommuDeviceIDValid(word_t deviceID, word_t levels, word_t leafBits);
static void iommuUnmapDevice(iommu_device_t *binding);
static iommu_ddt_lookup_t iommuLookupDDT(riscv_iommu_regs_ptr_t regs, word_t deviceID, ddte_t *target);
static volatile uint64_t *iommuMatchingDC(iommu_device_t *binding);
static void iommuWait(volatile uint32_t *csr, uint32_t mask, uint32_t value);
static void iommuSetDDTP(riscv_iommu_regs_ptr_t regs, uint64_t value);
static void iommuUnmapDDT(cap_t cap);
static exception_t iommuError(seL4_Error error);
static exception_t iommuReply(bool_t call, const word_t *words, word_t count);

/* DMA is assumed cache coherent for IOMMU tables and queues. The fences below
 * order memory updates before the device observes a valid entry or queue tail.
 */
cap_t riscvIOMMUInit(pptr_t iommu, paddr_t commandQueue)
{
    riscv_iommu_regs_ptr_t regs = (void *)iommu;
    iommu_capabilities_t caps = {{regs->capabilities}};
    if ((iommu_capabilities_get_version(caps) >> IOMMU_VERSION_MAJOR_SHIFT) != 1) {
        return cap_null_cap_new();
    }
    iommuSetDDTP(regs, IOMMU_DDTP_MODE_OFF);
    iommuWait(&regs->cqcsr, IOMMU_QCSR_BUSY, 0);
    iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY, 0);
    iommuWait(&regs->pqcsr, IOMMU_QCSR_BUSY, 0);
    regs->cqcsr = 0;
    regs->fqcsr = 0;
    regs->pqcsr = 0;
    iommuWait(&regs->cqcsr, IOMMU_QCSR_BUSY | IOMMU_QCSR_ON, 0);
    iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY | IOMMU_QCSR_ON, 0);
    iommuWait(&regs->pqcsr, IOMMU_QCSR_BUSY | IOMMU_QCSR_ON, 0);
    regs->ipsr = IOMMU_IPSR_PENDING_MASK;

    iommu_fctl_t fctl = {{regs->fctl}};
    fctl = iommu_fctl_set_be(fctl, 0);
    fctl = iommu_fctl_set_gxl(fctl, 0);
    regs->fctl = fctl.words[0];
    fctl.words[0] = regs->fctl;
    if (iommu_fctl_get_be(fctl) || iommu_fctl_get_gxl(fctl)) {
        return cap_null_cap_new();
    }

    word_t igs = iommu_capabilities_get_igs(caps);
    if (igs > seL4_IOMMUInterruptBoth) {
        return cap_null_cap_new();
    }
    bool_t wsi = igs == seL4_IOMMUInterruptWSI;
#ifndef CONFIG_RISCV_AIA
    if (igs == seL4_IOMMUInterruptMSI) {
        return cap_null_cap_new();
    }
    wsi = true;
#endif
    regs->fctl = iommu_fctl_set_wsi(fctl, wsi).words[0];
    if (iommu_fctl_get_wsi((iommu_fctl_t){{regs->fctl}}) != wsi) {
        return cap_null_cap_new();
    }
    if (!iommu_capabilities_get_sv39x4(caps) || BIT(iommu_capabilities_get_pas(caps)) < CONFIG_PADDR_USER_DEVICE_TOP) {
        return cap_null_cap_new();
    }

    iommu_icvec_t icvec = {{regs->icvec & ~IOMMU_ICVEC_VECTOR_MASK}};
    regs->icvec = iommu_icvec_set_fiv(icvec, IOMMU_FQ_VECTOR).words[0];
    if (iommu_icvec_get_fiv((iommu_icvec_t){{regs->icvec}}) != IOMMU_FQ_VECTOR) {
        return cap_null_cap_new();
    }
    if (!wsi) {
        regs->msi_cfg_tbl[IOMMU_FQ_VECTOR].vec_ctl = IOMMU_MSI_VEC_CTL_MASK;
    }

    word_t levels;
    for (levels = seL4_IOMMUDDTMaxLevels; levels > 0; levels--) {
        word_t mode = levels + 1;
        iommuSetDDTP(regs, (regs->ddtp & ~IOMMU_DDTP_MODE_MASK) | mode);
        bool_t supported = (regs->ddtp & IOMMU_DDTP_MODE_MASK) == mode;
        iommuSetDDTP(regs, IOMMU_DDTP_MODE_OFF);
        if (supported) {
            break;
        }
    }
    if (levels == 0) {
        return cap_null_cap_new();
    }
    uint64_t cqb = iommu_cqb_new(commandQueue >> seL4_PageBits, IOMMU_CQ_INDEX_BITS - 1).words[0];
    regs->cqb = cqb;
    if (regs->cqb != cqb) {
        regs->cqb = 0;
        return cap_null_cap_new();
    }
    regs->cqt = 0;
    fence_iorw_iorw();
    regs->cqcsr = IOMMU_QCSR_EN;
    iommuWait(&regs->cqcsr, IOMMU_QCSR_BUSY | IOMMU_QCSR_ON, IOMMU_QCSR_ON);
    return cap_iommu_cap_new(iommu, levels);
}

exception_t decodeRISCVIOMMUInvocation(word_t label, word_t length, cte_t *slot, cap_t cap, bool_t call, word_t *buffer)
{
    word_t type = cap_get_capType(cap);
    word_t args[5] = {0};
    for (word_t i = 0; i < MIN(length, ARRAY_SIZE(args)); i++) {
        args[i] = getSyscallArg(i, buffer);
    }
    cte_t *extra0 = current_extra_caps.excaprefs[0];
    cte_t *extra1 = current_extra_caps.excaprefs[1];
    cap_t cap0 = extra0 ? extra0->cap : cap_null_cap_new();
    cap_t cap1 = extra1 ? extra1->cap : cap_null_cap_new();
    switch (label) {
    case RISCVIOMMUGetCapabilities: {
        if (type != cap_iommu_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        riscv_iommu_regs_ptr_t regs = (void *)cap_iommu_cap_get_capIOMMUBasePtr(cap);
        word_t reply[] = {regs->capabilities, cap_iommu_cap_get_capIOMMUDDTLevels(cap)};
        return iommuReply(call, reply, ARRAY_SIZE(reply));
    }
    case RISCVIOMMUConfigureFaultInterrupt: {
        if (type != cap_iommu_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        if (length < 2) {
            return iommuError(seL4_TruncatedMessage);
        }
        riscv_iommu_regs_ptr_t regs = (void *)cap_iommu_cap_get_capIOMMUBasePtr(cap);
        bool_t enable = args[0] != 0;
#ifdef CONFIG_RISCV_AIA
        word_t identity = args[1];
        bool_t wsi = iommu_fctl_get_wsi((iommu_fctl_t){{regs->fctl}});
        if (enable && !wsi && (identity == 0 || identity > IMSIC_NUM_IDS)) {
            current_syscall_error.type = seL4_InvalidArgument;
            current_syscall_error.invalidArgumentNumber = 1;
            return EXCEPTION_SYSCALL_ERROR;
        }
#endif
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
#ifdef CONFIG_RISCV_AIA
        if (!wsi) {
            regs->msi_cfg_tbl[IOMMU_FQ_VECTOR].vec_ctl = IOMMU_MSI_VEC_CTL_MASK;
        }
#endif
        iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY, 0);
        uint32_t fqen = regs->fqcsr & IOMMU_QCSR_EN;
        regs->fqcsr = fqen;
        iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY, 0);
        if (enable) {
#ifdef CONFIG_RISCV_AIA
            if (!wsi) {
                regs->msi_cfg_tbl[IOMMU_FQ_VECTOR].addr = IMSIC_S_FILE_PADDR;
                regs->msi_cfg_tbl[IOMMU_FQ_VECTOR].data = identity;
                fence_iorw_iorw();
                if (regs->msi_cfg_tbl[IOMMU_FQ_VECTOR].addr != IMSIC_S_FILE_PADDR ||
                    regs->msi_cfg_tbl[IOMMU_FQ_VECTOR].data != identity) {
                    return iommuError(seL4_IllegalOperation);
                }
                regs->msi_cfg_tbl[IOMMU_FQ_VECTOR].vec_ctl = 0;
                fence_iorw_iorw();
            }
#endif
            regs->fqcsr = fqen | IOMMU_QCSR_IE;
            iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY, 0);
        }
        regs->ipsr = IOMMU_IPSR_FIP;
        fence_iorw_iorw();
        return EXCEPTION_NONE;
    }
    case RISCVIOMMUMapFaultQueue: {
        if (type != cap_iommu_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        if (!extra0) {
            return iommuError(seL4_TruncatedMessage);
        }
        if (cap_get_capType(cap0) != cap_frame_cap || cap_frame_cap_get_capFVMRights(cap0) != VMReadWrite) {
            current_syscall_error.type = seL4_InvalidCapability;
            current_syscall_error.invalidCapNumber = 1;
            return EXCEPTION_SYSCALL_ERROR;
        }
        riscv_iommu_regs_ptr_t regs = (void *)cap_iommu_cap_get_capIOMMUBasePtr(cap);
        if (cap_frame_cap_get_capFMappedASID(cap0) != asidInvalid ||
            cap_frame_cap_get_capFMapType(cap0) == RISCVMapFQ ||
            (regs->fqcsr & (IOMMU_QCSR_EN | IOMMU_QCSR_ON | IOMMU_QCSR_BUSY))) {
            return iommuError(seL4_DeleteFirst);
        }
        paddr_t paddr = pptr_to_paddr((void *)cap_frame_cap_get_capFBasePtr(cap0));
        word_t indexBits = pageBitsForSize(cap_frame_cap_get_capFSize(cap0)) - seL4_IOMMUFaultRecordBits;
        uint64_t fqb = iommu_fqb_new(paddr >> seL4_PageBits, indexBits - 1).words[0];
        regs->fqb = fqb;
        if (regs->fqb != fqb) {
            regs->fqb = 0;
            return iommuError(seL4_IllegalOperation);
        }
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        regs->fqh = 0;
        regs->ipsr = IOMMU_IPSR_FIP;
        fence_iorw_iorw();
        regs->fqcsr = IOMMU_QCSR_EN | (regs->fqcsr & IOMMU_QCSR_IE);
        iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY | IOMMU_QCSR_ON, IOMMU_QCSR_ON);
        cap0 = cap_frame_cap_set_capFMappedAddress(cap0, (word_t)regs & MASK(seL4_X4GPABits));
        extra0->cap = cap_frame_cap_set_capFMapType(cap0, RISCVMapFQ);
        return EXCEPTION_NONE;
    }
    case RISCVIOMMURWFaultQueue: {
        if (type != cap_iommu_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        if (length < 2) {
            return iommuError(seL4_TruncatedMessage);
        }
        riscv_iommu_regs_ptr_t regs = (void *)cap_iommu_cap_get_capIOMMUBasePtr(cap);
        if (!(regs->fqcsr & IOMMU_QCSR_EN)) {
            return iommuError(seL4_IllegalOperation);
        }
        word_t clear = args[1] & (IOMMU_FQCSR_FQMF | IOMMU_FQCSR_FQOF);
        fence_iorw_iorw();
        regs->fqh = args[0];
        fence_iorw_iorw();
        if (clear) {
            iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY, 0);
            regs->fqcsr = IOMMU_QCSR_EN | (regs->fqcsr & IOMMU_QCSR_IE) | clear;
            iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY, 0);
        }
        regs->ipsr = IOMMU_IPSR_FIP;
        fence_iorw_iorw();
        word_t reply[] = {regs->fqt, regs->fqcsr};
        fence_iorw_iorw();
        return iommuReply(call, reply, ARRAY_SIZE(reply));
    }
    case RISCVIOMMUDDTMap: {
        if (type != cap_iommu_ddt_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        if (length < 1 || !extra0) {
            return iommuError(seL4_TruncatedMessage);
        }
        if (cap_get_capType(cap0) != cap_iommu_cap) {
            current_syscall_error.type = seL4_InvalidCapability;
            current_syscall_error.invalidCapNumber = 1;
            return EXCEPTION_SYSCALL_ERROR;
        }
        if (cap_iommu_ddt_cap_get_capDDTIsMapped(cap) || !isFinalCapability(slot)) {
            return iommuError(seL4_RevokeFirst);
        }
        riscv_iommu_regs_ptr_t regs = (void *)cap_iommu_cap_get_capIOMMUBasePtr(cap0);
        word_t ddt_base = cap_iommu_ddt_cap_get_capDDTBasePtr(cap);
        paddr_t ddt_paddr = pptr_to_paddr((void *)ddt_base);
        if (!(regs->ddtp & (IOMMU_DDTP_MODE_MASK | IOMMU_DDTP_BUSY))) {
            clearMemory((void *)ddt_base, seL4_IOMMUDDTBits);
            fence_iorw_iorw();
            uint64_t value =
                iommu_ddtp_new(ddt_paddr >> seL4_PageBits, 0, cap_iommu_cap_get_capIOMMUDDTLevels(cap0) + 1).words[0];
            iommuSetDDTP(regs, value);
            if (regs->ddtp != value) {
                iommuSetDDTP(regs, IOMMU_DDTP_MODE_OFF);
                iommuInvalidateAllDDT(regs);
                return iommuError(seL4_IllegalOperation);
            }
            slot->cap = cap_iommu_ddt_cap_new((word_t)regs, 0, true, ddt_base);
        } else {
            word_t deviceID = args[0];
            word_t levels = cap_iommu_cap_get_capIOMMUDDTLevels(cap0);
            iommu_capabilities_t caps = {{regs->capabilities}};
            word_t leafBits =
                iommu_capabilities_get_msi_flat(caps) ? seL4_IOMMUDDTLeafExtendedIndexBits : seL4_IOMMUDDTLeafIndexBits;
            if (!iommuDeviceIDValid(deviceID, levels, leafBits)) {
                current_syscall_error.type = seL4_InvalidArgument;
                current_syscall_error.invalidArgumentNumber = 0;
                return EXCEPTION_SYSCALL_ERROR;
            }
            iommu_ddt_lookup_t lookup = iommuLookupDDT(regs, deviceID, NULL);
            if (lookup.bitsLeft == 0) {
                return iommuError(seL4_DeleteFirst);
            }
            clearMemory((void *)ddt_base, seL4_IOMMUDDTBits);
            fence_iorw_iorw();
            *(ddte_t *)lookup.slot = ddte_new(ddt_paddr >> seL4_PageBits, 1);
            fence_iorw_iorw();
            slot->cap = cap_iommu_ddt_cap_new((word_t)regs, deviceID, true, ddt_base);
        }
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        return EXCEPTION_NONE;
    }
    case RISCVIOMMUDDTUnmap: {
        if (type != cap_iommu_ddt_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        if (!isFinalCapability(slot)) {
            return iommuError(seL4_RevokeFirst);
        }
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        iommuUnmapDDT(cap);
        clearMemory((void *)cap_iommu_ddt_cap_get_capDDTBasePtr(cap), seL4_IOMMUDDTBits);
        slot->cap = cap_iommu_ddt_cap_new(0, 0, false, cap_iommu_ddt_cap_get_capDDTBasePtr(cap));
        return EXCEPTION_NONE;
    }
    case RISCVIOMMUDeviceMap: {
        if (type != cap_iommu_device_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        if (length < 1 || !extra0 || !extra1) {
            return iommuError(seL4_TruncatedMessage);
        }
        if (cap_get_capType(cap0) != cap_iommu_cap) {
            current_syscall_error.type = seL4_InvalidCapability;
            current_syscall_error.invalidCapNumber = 1;
            return EXCEPTION_SYSCALL_ERROR;
        }
        if (cap_get_capType(cap1) != cap_page_table_cap || !cap_page_table_cap_get_capPTRootX4(cap1)) {
            current_syscall_error.type = seL4_InvalidCapability;
            current_syscall_error.invalidCapNumber = 2;
            return EXCEPTION_SYSCALL_ERROR;
        }
        iommu_device_t *device = (void *)cap_iommu_device_cap_get_capDevBasePtr(cap);
        if (iommu_device_ptr_get_asidEntryPtr(device) != 0) {
            return iommuError(seL4_IllegalOperation);
        }
        asid_entry_t *entry = lookupASIDEntry(cap_page_table_cap_get_capPTMappedASID(cap1));
        if (!entry || asid_entry_ptr_get_rootPtr(entry) != cap_page_table_cap_get_capPTBasePtr(cap1)) {
            current_syscall_error.type = seL4_InvalidCapability;
            current_syscall_error.invalidCapNumber = 2;
            return EXCEPTION_SYSCALL_ERROR;
        }
        riscv_iommu_regs_ptr_t regs = (void *)cap_iommu_cap_get_capIOMMUBasePtr(cap0);
        iommu_capabilities_t caps = {{regs->capabilities}};
        if ((regs->ddtp & IOMMU_DDTP_MODE_MASK) == IOMMU_DDTP_MODE_OFF) {
            return iommuError(seL4_IllegalOperation);
        }
        word_t deviceID = args[0];
        word_t leafBits =
            iommu_capabilities_get_msi_flat(caps) ? seL4_IOMMUDDTLeafExtendedIndexBits : seL4_IOMMUDDTLeafIndexBits;
        if (!iommuDeviceIDValid(deviceID, cap_iommu_cap_get_capIOMMUDDTLevels(cap0), leafBits)) {
            current_syscall_error.type = seL4_InvalidArgument;
            current_syscall_error.invalidArgumentNumber = 0;
            return EXCEPTION_SYSCALL_ERROR;
        }
        iommu_ddt_lookup_t lookup = iommuLookupDDT(regs, deviceID, NULL);
        volatile uint64_t *dc = lookup.slot;
        if (dc == NULL || lookup.bitsLeft != 0 || iommu_dc_tc_get_v((iommu_dc_tc_t){{dc[0]}})) {
            return iommuError(seL4_IllegalOperation);
        }
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        iommu_device_t *head = (iommu_device_t *)asid_entry_ptr_get_deviceHeadPtr(entry);
        iommu_device_t *next = head;
        while (next != NULL && iommu_device_ptr_get_iommuBasePtr(next) != (word_t)regs) {
            next = (void *)iommu_device_ptr_get_nextPtr(next);
        }
        if (next == NULL) {
            next = head;
        }
        iommu_device_t *prev = next ? (void *)iommu_device_ptr_get_prevPtr(next) : NULL;
        *device = iommu_device_new((word_t)regs, (word_t)next, (word_t)prev, deviceID, (word_t)entry);
        if (prev != NULL) {
            iommu_device_ptr_set_nextPtr(prev, (word_t)device);
        } else {
            asid_entry_ptr_set_deviceHeadPtr(entry, (word_t)device);
        }
        if (next != NULL) {
            iommu_device_ptr_set_prevPtr(next, (word_t)device);
        }
        dc[1] = iommu_iohgatp_new(IOHGATP_MODE_SV39X4, asid_entry_ptr_get_ASID(entry),
                                  pptr_to_paddr((void *)asid_entry_ptr_get_rootPtr(entry)) >> seL4_PageBits)
                    .words[0];
        dc[2] = 0;
        dc[3] = 0;
        if (iommu_capabilities_get_msi_flat(caps)) {
            dc[4] = 0;
            dc[5] = 0;
            dc[6] = 0;
            dc[7] = 0;
        }
        fence_iorw_iorw();
        dc[0] = iommu_dc_tc_set_v((iommu_dc_tc_t){0}, 1).words[0];
        fence_iorw_iorw();
        return EXCEPTION_NONE;
    }
    case RISCVIOMMUDeviceUnmap: {
        if (type != cap_iommu_device_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        iommuUnmapDevice((void *)cap_iommu_device_cap_get_capDevBasePtr(cap));
        return EXCEPTION_NONE;
    }
    case RISCVIOMMUDeviceConfigureFirstStage: {
        if (type != cap_iommu_device_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        if (length < 5) {
            return iommuError(seL4_TruncatedMessage);
        }
        word_t kind = args[0] & MASK(1);
        word_t mode = args[1];
        word_t rootGPA = args[2] & MASK(IOMMU_FSC_ADDR_BITS);
        word_t pscid = args[3] & MASK(seL4_IOMMUPSCIDBits);
        word_t flags = args[4];

        iommu_device_t *device = (void *)cap_iommu_device_cap_get_capDevBasePtr(cap);
        if (!iommu_device_ptr_get_asidEntryPtr(device)) {
            current_syscall_error.type = seL4_InvalidCapability;
            current_syscall_error.invalidCapNumber = 0;
            return EXCEPTION_SYSCALL_ERROR;
        }
        riscv_iommu_regs_ptr_t regs = (void *)iommu_device_ptr_get_iommuBasePtr(device);
        volatile uint64_t *dc = iommuMatchingDC(device);
        if (!dc) {
            return iommuError(seL4_IllegalOperation);
        }
        if (mode >= IOMMU_FSC_MODE_CUSTOM) {
            current_syscall_error.type = seL4_InvalidArgument;
            current_syscall_error.invalidArgumentNumber = 1;
            return EXCEPTION_SYSCALL_ERROR;
        }
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        asid_entry_t *entry = (void *)iommu_device_ptr_get_asidEntryPtr(device);
        dc[0] = 0;
        iommuInvalidateDC(regs, iommu_device_ptr_get_deviceID(device), asid_entry_ptr_get_ASID(entry));
        dc[2] = iommu_dc_ta_new(pscid).words[0];
        dc[3] = iommu_fsc_new(mode, rootGPA >> seL4_PageBits).words[0];
        iommu_dc_tc_t tc = {0};
        /* Hardware updates of first-stage accessed/dirty bits. */
        tc = iommu_dc_tc_set_sade(tc, (flags & seL4_IOMMUFirstStageAD) != 0);
        /* Enable the default process ID of zero. */
        tc = iommu_dc_tc_set_dpe(tc, (flags & seL4_IOMMUFirstStageDPE) != 0);
        tc = iommu_dc_tc_set_pdtv(tc, kind);
        fence_iorw_iorw();
        dc[0] = iommu_dc_tc_set_v(tc, 1).words[0];
        fence_iorw_iorw();
        return EXCEPTION_NONE;
    }
    case RISCVIOMMUDeviceInvalidateFirstStage: {
        if (type != cap_iommu_device_cap) {
            return iommuError(seL4_IllegalOperation);
        }
        if (length < 2) {
            return iommuError(seL4_TruncatedMessage);
        }
        word_t kind = args[0];
        word_t pscid = args[1] & MASK(seL4_IOMMUPSCIDBits);
        if (kind > seL4_IOMMUInvalidateAllPSCID) {
            current_syscall_error.type = seL4_InvalidArgument;
            current_syscall_error.invalidArgumentNumber = 0;
            return EXCEPTION_SYSCALL_ERROR;
        }
        iommu_device_t *device = (void *)cap_iommu_device_cap_get_capDevBasePtr(cap);
        if (!iommu_device_ptr_get_asidEntryPtr(device)) {
            current_syscall_error.type = seL4_InvalidCapability;
            current_syscall_error.invalidCapNumber = 0;
            return EXCEPTION_SYSCALL_ERROR;
        }

        riscv_iommu_regs_ptr_t regs = (void *)iommu_device_ptr_get_iommuBasePtr(device);
        if (kind == seL4_IOMMUInvalidatePDT && (regs->ddtp & IOMMU_DDTP_MODE_MASK) == IOMMU_DDTP_MODE_OFF) {
            return iommuError(seL4_IllegalOperation);
        }
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        if (kind == seL4_IOMMUInvalidatePDT) {
            iommuIODIR(regs, iommu_device_ptr_get_deviceID(device));
        } else {
            asid_entry_t *entry = (void *)iommu_device_ptr_get_asidEntryPtr(device);
            iommu_iotinval_t cmd =
                iommu_iotinval_new(0, asid_entry_ptr_get_ASID(entry), true, kind == seL4_IOMMUInvalidatePSCID, pscid,
                                   false, IOMMU_IOTINVAL_VMA, IOMMU_CMD_IOTINVAL);
            iommuSubmit(regs, cmd.words);
        }
        iommuIOFence(regs);
        return EXCEPTION_NONE;
    }
    default:
        return iommuError(seL4_IllegalOperation);
    }
}

static word_t iommuSubmit(riscv_iommu_regs_ptr_t regs, const uint64_t words[2])
{
    iommu_cqb_t cqb = {{regs->cqb}};
    word_t tail = regs->cqt;
    word_t next = (tail + 1) & MASK(IOMMU_CQ_INDEX_BITS);
    paddr_t base = iommu_cqb_get_ppn(cqb) << seL4_PageBits;
    iommu_command_queue_t *queue = paddr_to_pptr(base);
    while (regs->cqh == next) {
    }
    queue->commands[tail].words[0] = words[0];
    queue->commands[tail].words[1] = words[1];
    fence_iorw_iorw();
    regs->cqt = next;
    fence_iorw_iorw();
    return next;
}

void iommuIOFence(riscv_iommu_regs_ptr_t regs)
{
    iommu_iofence_t fence = {0};
    fence = iommu_iofence_set_opcode(fence, IOMMU_CMD_IOFENCE);
    fence = iommu_iofence_set_pr(fence, 1);
    fence = iommu_iofence_set_pw(fence, 1);
    word_t end = iommuSubmit(regs, fence.words);
    while (regs->cqh != end) {
    }
    fence_iorw_iorw();
}

void iommuIOTINVAL(riscv_iommu_regs_ptr_t regs, word_t gscid, word_t function, bool_t av, vptr_t addr)
{
    iommu_iotinval_t cmd = {0};
    cmd = iommu_iotinval_set_opcode(cmd, IOMMU_CMD_IOTINVAL);
    cmd = iommu_iotinval_set_func3(cmd, function);
    cmd = iommu_iotinval_set_gv(cmd, 1);
    cmd = iommu_iotinval_set_gscid(cmd, gscid);
    cmd = iommu_iotinval_set_av(cmd, av);
    cmd = iommu_iotinval_set_addr(cmd, av ? addr >> seL4_PageBits : 0);
    iommuSubmit(regs, cmd.words);
}

void iommuIODIR(riscv_iommu_regs_ptr_t regs, word_t deviceID)
{
    iommu_iodir_t cmd = {0};
    cmd = iommu_iodir_set_opcode(cmd, IOMMU_CMD_IODIR);
    cmd = iommu_iodir_set_dv(cmd, 1);
    cmd = iommu_iodir_set_did(cmd, deviceID);
    iommuSubmit(regs, cmd.words);
}

static void iommuInvalidateFirstStagePT(riscv_iommu_regs_ptr_t regs, word_t gscid)
{
    /* IOMMU specification 7.3.5 */
    iommuIOTINVAL(regs, gscid, IOMMU_IOTINVAL_VMA, false, 0);
    iommuIOFence(regs);
}

static void iommuInvalidateDC(riscv_iommu_regs_ptr_t regs, word_t deviceID, word_t gscid)
{
    /* IOMMU specification 7.3.1 */
    iommuIODIR(regs, deviceID);
    iommuIOFence(regs);
    iommuIOTINVAL(regs, gscid, IOMMU_IOTINVAL_GVMA, false, 0);
    iommuIOFence(regs);
    iommuInvalidateFirstStagePT(regs, gscid);
}

void iommuInvalidateAllDDT(riscv_iommu_regs_ptr_t regs)
{
    /* IOMMU specification 7.3.1 and 7.3.4 */
    iommu_iodir_t cmd = {0};
    cmd = iommu_iodir_set_opcode(cmd, IOMMU_CMD_IODIR);
    iommuSubmit(regs, cmd.words);
    iommuIOFence(regs);
}

static bool_t iommuDeviceIDValid(word_t deviceID, word_t levels, word_t leafBits)
{
    word_t deviceBits = MIN(seL4_IOMMUDeviceIDBits, leafBits + (levels - 1) * seL4_IOMMUDDTIndexBits);
    return deviceID < BIT(deviceBits);
}

static volatile uint64_t *iommuMatchingDC(iommu_device_t *binding)
{
    asid_entry_t *entry = (asid_entry_t *)iommu_device_ptr_get_asidEntryPtr(binding);
    riscv_iommu_regs_ptr_t regs = (riscv_iommu_regs_ptr_t)iommu_device_ptr_get_iommuBasePtr(binding);
    iommu_ddt_lookup_t lookup = iommuLookupDDT(regs, iommu_device_ptr_get_deviceID(binding), NULL);
    if (lookup.slot == NULL || lookup.bitsLeft != 0) {
        return NULL;
    }
    volatile uint64_t *dc = lookup.slot;
    iommu_dc_tc_t tc = {{dc[0]}};
    iommu_iohgatp_t hgatp = {{dc[1]}};
    if (!iommu_dc_tc_get_v(tc) || iommu_iohgatp_get_gscid(hgatp) != asid_entry_ptr_get_ASID(entry) ||
        iommu_iohgatp_get_ppn(hgatp) != (pptr_to_paddr((pte_t *)asid_entry_ptr_get_rootPtr(entry)) >> seL4_PageBits)) {
        return NULL;
    }
    return dc;
}

/* Stop at target, the first missing non-leaf link, or the device-context slot. */
static iommu_ddt_lookup_t iommuLookupDDT(riscv_iommu_regs_ptr_t regs, word_t deviceID, ddte_t *target)
{
    iommu_ddt_lookup_t lookup = {0};
    iommu_ddtp_t ddtp = {{regs->ddtp}};
    word_t mode = iommu_ddtp_get_iommu_mode(ddtp);
    if (mode < IOMMU_DDTP_MODE_1LVL) {
        return lookup;
    }
    iommu_capabilities_t caps = {{regs->capabilities}};
    word_t leafBits =
        iommu_capabilities_get_msi_flat(caps) ? seL4_IOMMUDDTLeafExtendedIndexBits : seL4_IOMMUDDTLeafIndexBits;
    lookup.table = paddr_to_pptr(iommu_ddtp_get_ppn(ddtp) << seL4_PageBits);
    for (word_t level = mode - IOMMU_DDTP_MODE_1LVL;; level--) {
        lookup.bitsLeft = level ? leafBits + (level - 1) * seL4_IOMMUDDTIndexBits : 0;
        word_t indexBits = level ? seL4_IOMMUDDTIndexBits : leafBits;
        word_t index = (deviceID >> lookup.bitsLeft) & MASK(indexBits);
        lookup.slot = (uint8_t *)lookup.table + (index << (seL4_IOMMUDDTBits - indexBits));
        if (lookup.table == target || level == 0 || !ddte_ptr_get_v(lookup.slot)) {
            return lookup;
        }
        lookup.parent = lookup.slot;
        lookup.table = paddr_to_pptr(ddte_ptr_get_ppn(lookup.parent) << seL4_PageBits);
    }
}

static void iommuUnmapDevice(iommu_device_t *binding)
{
    asid_entry_t *entry = (asid_entry_t *)iommu_device_ptr_get_asidEntryPtr(binding);
    if (entry == NULL) {
        return;
    }
    riscv_iommu_regs_ptr_t regs = (riscv_iommu_regs_ptr_t)iommu_device_ptr_get_iommuBasePtr(binding);
    word_t gscid = asid_entry_ptr_get_ASID(entry);
    volatile uint64_t *dc = iommuMatchingDC(binding);
    if (dc != NULL) {
        iommu_dc_tc_t tc = {{dc[0]}};
        dc[0] = iommu_dc_tc_set_v(tc, 0).words[0];
        iommuInvalidateDC(regs, iommu_device_ptr_get_deviceID(binding), gscid);
    } else {
        iommuIOTINVAL(regs, gscid, IOMMU_IOTINVAL_GVMA, false, 0);
        iommuIOFence(regs);
        iommuInvalidateFirstStagePT(regs, gscid);
    }
    iommu_device_t *prev = (iommu_device_t *)iommu_device_ptr_get_prevPtr(binding);
    iommu_device_t *next = (iommu_device_t *)iommu_device_ptr_get_nextPtr(binding);
    if (prev != NULL) {
        iommu_device_ptr_set_nextPtr(prev, (word_t)next);
    } else {
        asid_entry_ptr_set_deviceHeadPtr(entry, (word_t)next);
    }
    if (next != NULL) {
        iommu_device_ptr_set_prevPtr(next, (word_t)prev);
    }
    *binding = (iommu_device_t){0};
}

static void iommuWait(volatile uint32_t *csr, uint32_t mask, uint32_t value)
{
    while ((*csr & mask) != value) {
    }
    fence_iorw_iorw();
}

static void iommuSetDDTP(riscv_iommu_regs_ptr_t regs, uint64_t value)
{
    while (regs->ddtp & IOMMU_DDTP_BUSY) {
    }
    if (value == IOMMU_DDTP_MODE_OFF) {
        value = regs->ddtp & ~IOMMU_DDTP_MODE_MASK;
    }
    regs->ddtp = value;
    fence_iorw_iorw();
    while (regs->ddtp & IOMMU_DDTP_BUSY) {
    }
}

void riscvIOMMUUnmapFaultQueue(pptr_t iommu, pptr_t frame)
{
    riscv_iommu_regs_ptr_t regs = (void *)iommu;
    iommu_fqb_t fqb = {{regs->fqb}};
    if (!(regs->fqcsr & IOMMU_QCSR_EN) || iommu_fqb_get_ppn(fqb) != (pptr_to_paddr((void *)frame) >> seL4_PageBits)) {
        return;
    }
    if (!iommu_fctl_get_wsi((iommu_fctl_t){{regs->fctl}})) {
        regs->msi_cfg_tbl[IOMMU_FQ_VECTOR].vec_ctl = IOMMU_MSI_VEC_CTL_MASK;
    }
    iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY, 0);
    regs->fqcsr = 0;
    iommuWait(&regs->fqcsr, IOMMU_QCSR_BUSY | IOMMU_QCSR_ON, 0);
    regs->ipsr = IOMMU_IPSR_FIP;
    regs->fqb = 0;
    fence_iorw_iorw();
}

void riscvIOMMURetireContext(asid_entry_t *entry)
{
    while (asid_entry_ptr_get_deviceHeadPtr(entry)) {
        iommuUnmapDevice((void *)asid_entry_ptr_get_deviceHeadPtr(entry));
    }
}

static void iommuUnmapDDT(cap_t cap)
{
    if (!cap_iommu_ddt_cap_get_capDDTIsMapped(cap)) {
        return;
    }
    riscv_iommu_regs_ptr_t regs = (void *)cap_iommu_ddt_cap_get_capDDTIOMMUBasePtr(cap);
    ddte_t *target = (void *)cap_iommu_ddt_cap_get_capDDTBasePtr(cap);
    iommu_ddt_lookup_t lookup = iommuLookupDDT(regs, cap_iommu_ddt_cap_get_capDDTDeviceID(cap), target);
    if (lookup.table != target) {
        return;
    }
    if (lookup.parent == NULL) {
        iommuSetDDTP(regs, IOMMU_DDTP_MODE_OFF);
    } else {
        *lookup.parent = (ddte_t){0};
    }
    iommuInvalidateAllDDT(regs);
}

void riscvIOMMUFinaliseCap(cap_t cap)
{
    switch (cap_get_capType(cap)) {
    case cap_iommu_ddt_cap:
        iommuUnmapDDT(cap);
        break;
    case cap_iommu_device_cap:
        iommuUnmapDevice((void *)cap_iommu_device_cap_get_capDevBasePtr(cap));
        break;
    default:
        break;
    }
}

static exception_t iommuError(seL4_Error error)
{
    current_syscall_error.type = error;
    return EXCEPTION_SYSCALL_ERROR;
}

static exception_t iommuReply(bool_t call, const word_t *words, word_t count)
{
    tcb_t *thread = NODE_STATE(ksCurThread);
    if (call) {
        word_t *ipcBuffer = lookupIPCBuffer(true, thread);
        word_t length = 0;
        for (word_t i = 0; i < count; i++) {
            length = setMR(thread, ipcBuffer, i, words[i]);
        }
        setRegister(thread, badgeRegister, 0);
        setRegister(thread, msgInfoRegister, wordFromMessageInfo(seL4_MessageInfo_new(0, 0, 0, length)));
    }
    setThreadState(thread, ThreadState_Running);
    return EXCEPTION_NONE;
}
