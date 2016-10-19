	.global 	Vector_Init
	.balign		4
	.code		32

	/* Import exception handlers */
	.equ		T_bit, 0x20
	.extern		vPortSVCHandler
	.extern		vPortInterruptContext
	.extern		Undefined_Handler_Panic
	.extern		Prefetch_Handler_Panic
	.extern		Abort_Handler_Panic
	.extern		__stack
	.extern		_init

	/* Install exception vector table */
	.section	.isr_vector, "x"

Vector_Init:
	LDR		PC, Reset_Addr
	LDR		PC, Undefined_Addr
	LDR		PC, SWI_Addr
	LDR		PC, Prefetch_Addr
	LDR		PC, Abort_Addr
	NOP
	LDR		PC, IRQ_Addr
	LDR		PC, FIQ_Addr

Reset_Addr:		.word	Reset_Handler
Undefined_Addr:		.word	Undefined_Handler
SWI_Addr:		.word	SWI_Handler
Prefetch_Addr:		.word	Prefetch_Handler
Abort_Addr:		.word	Abort_Handler
IRQ_Addr:		.word	IRQ_Handler
FIQ_Addr:		.word	FIQ_Handler

/**
 * Define Exception Handlers declared above
 */
Reset_Handler:
	LDR		SP, Stack_Addr
	B		_init

Stack_Addr		.word	__stack

Undefined_Handler:
	STMDB		SP, {R0-LR}^
	NOP
	SUB		SP, SP, #60
	MRS		R0, SPSR
	SUB		R1, LR, #2
	TST		R0, #T_bit
	SUBEQ		R1, R1, #2
	STMDB		SP!, {R1}
	STMDB		SP!, {R0}
	NOP
	MOV		R0, SP
	BL,		Undefined_Handler_Panic

Prefetch_Handler:
	/* Put system/user mode R0-R14, exception address (LR - 4) and SPSR on stack */
	STMDB		SP, {R0-LR}^
	NOP
	SUB		SP, SP, #60
	SUB		R0, LR, #4
	STMDB		SP!, {R0}
	MRS		R0, SPSR
	STMDB		SP!, {R0}
	NOP
	MOV		R0, SP
	BL,		Prefetch_Handler_Panic

/* Software Interrupt Handler */
SWI_Handler:
	B		vPortSVCHandler

FIQ_Handler:
IRQ_Handler:
	B		vPortInterruptContext

	.end
