#pragma once

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
/*
 * Ensure offset macros are available in <offsets.h>.
 */
#define GEN_SOC_OFFSET_SYMS()				\
	GEN_OFFSET_SYM(soc_esf_t, gp);			\
	GEN_OFFSET_SYM(soc_esf_t, tp)
#else


#endif /* CONFIG_RISCV_SOC_CONTEXT_SAVE */