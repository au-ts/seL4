# SPDX-License-Identifier: GPL-2.0-only

'''Read AIA geometry for /chosen/seL4,boot-cpu and its kernel device list.'''

from hardware.utils import align_up


def word(node, name, default=None):
    if not node.has_prop(name) and default is not None:
        return default
    return node.get_prop(name).words[0]


def compatible(node, name):
    return node.has_prop('compatible') and name in node.get_prop('compatible').strings


def imsic_layout(imsic, file_index):
    '''Return the boot hart's S-mode MSI page address, APLIC hart index and ID count IMSIC minor IDs.

    AIA 1.0 section 3.6 gives the supervisor interrupt-file address as:
        paddr = g * 2**E + B + h * 2**D

    B is the supervisor base address, g the group number, and h the hart number
    within that group. D and E place the hart and group fields in the address;
    k and j are their respective widths. In terms of the DTS properties:
        D = 12 + riscv,guest-index-bits
        E = riscv,group-index-shift
        k = riscv,hart-index-bits
        j = riscv,group-index-bits
    The supervisor file has guest index zero; guest-file pages follow it at
    4-KiB intervals within the hart's 2**D-byte allocation.

    file_index is the boot hart's zero-based entry in interrupts-extended,
    across all groups. The reg ranges locate its page without needing B as a
    separate input. We then recover g and h from that page address. Section
    4.9.1 encodes these in the APLIC target Hart Index as (g << k) | h, with
    LHXW = k and HHXW = j. This index need not equal the CPU's mhartid.

    Return paddr (physical page address), hart (APLIC target index), and ids
    (the riscv,num-ids value).

    RISC-V AIA 1.0, sections 3.6 and 4.9.1:
    https://docs.riscv.org/reference/aia/_attachments/riscv-interrupts.pdf
    '''
    # Each entry contains two cells: a CPU interrupt-controller phandle and IRQ.
    cells = imsic.get_prop('interrupts-extended').words
    regions = imsic.get_regions()
    D = 12 + word(imsic, 'riscv,guest-index-bits', 0)
    E = word(imsic, 'riscv,group-index-shift', 24)
    # If omitted, use enough hart bits to index all interrupts-extended entries.
    k = word(imsic, 'riscv,hart-index-bits', (len(cells) // 2 - 1).bit_length())
    j = word(imsic, 'riscv,group-index-bits', 0)

    # Locate the selected supervisor page using the spec's 2**D hart stride.
    offset = file_index * 2**D
    for region in regions:
        if offset < region.size:
            page = region.base + offset
            break
        # Count whole hart allocations, including any unused guest slots.
        offset -= align_up(region.size, D)

    # Recover the spec's group number g and group-relative hart number h.
    g = (page >> E) & (2**j - 1)
    h = (page >> D) & (2**k - 1)
    hart = (g << k) | h
    return dict(paddr=page, hart=hart, ids=word(imsic, 'riscv,num-ids'))


def parse(tree, enabled):
    if not enabled:
        return None

    # retrieve the device-tree nodes listed in /chosen/seL4,kernel-devices
    devices = tree.get_kernel_devices()
    # check if any devices are the aplic
    aplics = [node for node in devices if compatible(node, 'riscv,aplic')]

    # Locate the boot CPU's supervisor IMSIC file.
    boot = tree.get_phandle(tree.get_boot_cpu())
    for node in tree.by_path.values():
        if not compatible(node, 'riscv,imsics'):
            continue
        cells = node.get_prop('interrupts-extended').words
        for index in range(0, len(cells), 2):
            if cells[index + 1] == 9 and tree.get_phandle(cells[index]).parent == boot:
                layout = imsic_layout(node, index // 2)
                layout['sources'] = word(aplics[0], 'riscv,num-sources') if aplics else 0
                return layout
