/*
 * Copyright (c) 2021 Katsuhiro Suzuki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief QEMU RISC-V virt machine hardware depended interface
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>


void sys_arch_reboot(int type)
{
	

	ARG_UNUSED(type);
}
