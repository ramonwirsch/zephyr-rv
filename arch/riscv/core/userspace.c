/*
 * Copyright (c) 2022 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Physical Memory Protection (PMP) is RISC-V parlance for an MPU.
 *
 * The PMP is comprized of a number of entries or slots. This number depends
 * on the hardware design. For each slot there is an address register and
 * a configuration register. While each address register is matched to an
 * actual CSR register, configuration registers are small and therefore
 * several of them are bundled in a few additional CSR registers.
 *
 * PMP slot configurations are updated in memory to avoid read-modify-write
 * cycles on corresponding CSR registers. Relevant CSR registers are always
 * written in batch from their shadow copy in RAM for better efficiency.
 *
 * In the stackguard case we keep an m-mode copy for each thread. Each user
 * mode threads also has a u-mode copy. This makes faster context switching
 * as precomputed content just have to be written to actual registers with
 * no additional processing.
 *
 * Thread-specific m-mode and u-mode PMP entries start from the PMP slot
 * indicated by global_pmp_end_index. Lower slots are used by global entries
 * which are never modified.
 */

#include <zephyr/kernel.h>
#include <kernel_internal.h>
#include <zephyr/linker/linker-defs.h>
#include <pmp.h>
#include <zephyr/sys/arch_interface.h>
#include <zephyr/arch/riscv/csr.h>

#define LOG_LEVEL CONFIG_MPU_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mpu);


#define IS_WITHIN(inner_start, inner_size, outer_start, outer_size) \
	((inner_start) >= (outer_start) && (inner_size) <= (outer_size) && \
	 ((inner_start) - (outer_start)) <= ((outer_size) - (inner_size)))

int arch_buffer_validate(void *addr, size_t size, int write)
{
	uintptr_t start = (uintptr_t)addr;
	int ret = -1;

	/* Check if this is on the stack */
	if (IS_WITHIN(start, size,
		      _current->stack_info.start, _current->stack_info.size)) {
		return 0;
	}

	/* Check if this is within the global read-only area */
	if (!write) {
		uintptr_t ro_start = (uintptr_t)__rom_region_start;
		size_t ro_size = (size_t)__rom_region_size;

		if (IS_WITHIN(start, size, ro_start, ro_size)) {
			return 0;
		}
	}

	/* Look for a matching partition in our memory domain */
	struct k_mem_domain *domain = _current->mem_domain_info.mem_domain;
	int p_idx, remaining_partitions;
	k_spinlock_key_t key = k_spin_lock(&z_mem_domain_lock);

	remaining_partitions = domain->num_partitions;
	for (p_idx = 0; remaining_partitions > 0; p_idx++) {
		struct k_mem_partition *part = &domain->partitions[p_idx];

		if (part->size == 0) {
			/* unused partition */
			continue;
		}

		remaining_partitions--;

		if (!IS_WITHIN(start, size, part->start, part->size)) {
			/* unmatched partition */
			continue;
		}

		/* partition matched: determine access result */
		if ((part->attr.pmp_attr & (write ? PMP_W : PMP_R)) != 0) {
			ret = 0;
		}
		break;
	}

	k_spin_unlock(&z_mem_domain_lock, key);
	return ret;
}

#if !defined(CONFIG_RISCV_PMP)

int arch_mem_domain_max_partitions_get(void)
{
	return 16;
}

#endif