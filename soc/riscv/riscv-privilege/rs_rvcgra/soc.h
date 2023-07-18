/*
 * Copyright (c) 2020 Cobham Gaisler AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __RISCV_RS_RVCGRA_SOC_H_
#define __RISCV_RS_RVCGRA_SOC_H_

#include <soc_common.h>

// Magic  register with which SiFive seems to communicate exit, start and failures to qemu or some kind of HW watchdog
//#define SIFIVE_SYSCON_TEST           0x00100000
#define RISCV_MTIME_BASE             0x60001000
#define RISCV_MTIMECMP_BASE          0x60001008
// MSIP is the machine-mode software interrupt enable bit.
// It is a mmapped register for other HARTs (cores) to send a software-interrupt to this core. Unused by us
//#define RISCV_MSIP_BASE              0x02000000

#endif
