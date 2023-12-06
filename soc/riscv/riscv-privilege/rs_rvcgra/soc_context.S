
#include <zephyr/toolchain.h>
#include <offsets.h>

GTEXT(__soc_save_context)
GTEXT(__soc_restore_context)

// __soc_save_context(*ptrToSocSaveStruct, org scratched stackptr (only non-zero when coming from user-space), org tp-ptr)
SECTION_FUNC(exception.other, __soc_save_context)
	beqz a1, 1f
#if defined(CONFIG_RISCV_GP)
	// we entered from a user thread. Save users gp and restore. Ignoring tp for now, has already been overwritten
	sw gp, __soc_esf_t_gp_OFFSET(a0)
	.option push
	.option norelax // with relaxing, this will be replaced by gp as source
	la gp, __global_pointer$
	.option pop
#endif
	sw a2, __soc_esf_t_tp_OFFSET(a0) // cannot restore it right now, waste of time to save it then
	// will be restored by isr_wrapper anyway if USERSPACE is enabled
	
1:
	ret

// __soc_restore_context(*ptrTpSocSaveStruct, (0 if going into usermode))
SECTION_FUNC(exception.other, __soc_restore_context)

	bnez a1, 1f // skip over everything if not going into usermode
	lw gp, __soc_esf_t_gp_OFFSET(a0)
	lw tp, __soc_esf_t_tp_OFFSET(a0)
1:
	ret