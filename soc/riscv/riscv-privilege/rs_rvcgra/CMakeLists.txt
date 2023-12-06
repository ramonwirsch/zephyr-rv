# SPDX-License-Identifier: Apache-2.0

zephyr_sources(soc.c)

zephyr_sources_ifdef(CONFIG_RISCV_SOC_CONTEXT_SAVE soc_context.S)