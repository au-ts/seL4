--
-- SPDX-License-Identifier: GPL-2.0-only
--

base 64

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_registers.html#CAP
block iommu_capabilities {
    padding                                         26
    field       pas                                 6
    padding                                         2
    field       igs                                 2
    padding                                         5
    field       msi_flat                            1
    padding                                         4
    field       sv39x4                              1
    padding                                         9
    field       version                             8
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_registers.html#DDTP
block iommu_ddtp {
    padding                                         10
    field       ppn                                 44
    padding                                         5
    field       busy                                1
    field       iommu_mode                          4
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_registers.html#CQB
block iommu_cqb {
    padding                                         10
    field       ppn                                 44
    padding                                         5
    field       logsz_1                             5
}

block iommu_fqb {
    padding                                         10
    field       ppn                                 44
    padding                                         5
    field       logsz_1                             5
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_data_structures.html#first-stage-context-fsc
block iommu_fsc {
    field       mode                                4
    padding                                         16
    field       ppn                                 44
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_data_structures.html#DC_TA
block iommu_dc_ta {
    padding                                         32
    field       pscid                               20
    padding                                         12
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_data_structures.html#io-hypervisor-guest-address-translation-and-protection-iohgatp
block iommu_iohgatp {
    field       mode                                4
    field       gscid                               16
    field       ppn                                 44
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_data_structures.html#translation-control-tc
block iommu_dc_tc {
    padding                                         54
    field       dpe                                 1
    field       sade                                1
    padding                                         2
    field       pdtv                                1
    padding                                         4
    field       v                                   1
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_data_structures.html#non-leaf-ddt-entry
block ddte {
    padding                                         10
    field       ppn                                 44
    padding                                         9
    field       v                                   1
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_in_memory_queues.html#iommu-page-table-cache-invalidation-commands
block iommu_iotinval {
    padding                                         2
    field       addr                                52
    padding                                         10

    padding                                         4
    field       gscid                               16
    padding                                         10
    field       gv                                  1
    field       pscv                                1
    field       pscid                               20
    padding                                         1
    field       av                                  1
    field       func3                               3
    field       opcode                              7
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_in_memory_queues.html#iommu-directory-cache-invalidation-commands
block iommu_iodir {
    padding                                         64
    field       did                                 24
    padding                                         6
    field       dv                                  1
    padding                                         26
    field       opcode                              7
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_in_memory_queues.html#iommu-command-queue-fence-commands
block iommu_iofence {
    padding                                         64
    padding                                         50
    field       pw                                  1
    field       pr                                  1
    padding                                         5
    field       opcode                              7
}

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_registers.html#ICVEC
block iommu_icvec {
    padding                                         56
    field       fiv                                 4
    padding                                         4
}

base 32

-- https://docs.riscv.org/reference/iommu/v20260222/iommu_registers.html#FCTRL
block iommu_fctl {
    padding                                         29
    field       gxl                                 1
    field       wsi                                 1
    field       be                                  1
}
