#
# SPDX-License-Identifier: GPL-2.0-only
#

config_string(KernelRiscvIOMMUCommandQueueBits
                RISCV_IOMMU_COMMAND_QUEUE_BITS
                "Size bits of each kernel IOMMU command queue"
                DEFAULT 12
                UNDEF_DISABLED UNQUOTE DEPENDS "KernelRiscvIOMMU")
add_bf_source_old("KernelRiscvIOMMU" "riscv.bf" "include" "drivers/iommu")
add_sources(DEP "KernelRiscvIOMMU" CFILES src/drivers/iommu/riscv.c)
