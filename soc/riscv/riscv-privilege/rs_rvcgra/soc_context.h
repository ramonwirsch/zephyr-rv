#pragma once

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE

#define SOC_ESF_MEMBERS	\
	uint32_t gp;		\
	uint32_t tp

extern uint32_t __global_pointer$;

#define SOC_ESF_INIT	\
	(uint32_t) &__global_pointer$,	\
	0

#endif /* CONFIG_RISCV_SOC_CONTEXT_SAVE */