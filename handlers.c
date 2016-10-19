/* This file defines exception handler routines for this platform */

#include <stdio.h>
#include <string.h>

/* Memory Map */
#define portPERIPH_BASE					0x1F000000UL
#define portGIC_PRIVATE_BASE				0x1F000100UL
#define portSYSTICK_BASE				0x1F000600UL
#define portGIC_DISTRIBUTOR_BASE			0x1F001000UL
#define portEXCEPTION_BASE				0x01000000UL

/* General Macros */
#define	portGIC_READ(address)				( *( ( unsigned long* volatile )( address ) ) )
#define portGIC_WRITE(address, value)			( *( ( unsigned long* volatile )( address ) ) = ( value ) )
#define portGIC_ICCIAR(x)				(  ( ( unsigned long ) ( x ) ) + 0x0CUL )
#define portGIC_ICCEOIR(x)				(  ( ( unsigned long ) ( x ) ) + 0x10UL )

/* Timer Macros */
#define portSYSTICK_LOAD				(  ( volatile unsigned long* )( portSYSTICK_BASE + 0x00 ) )
#define portSYSTICK_CONTROL				(  ( volatile unsigned long* )( portSYSTICK_BASE + 0x08 ) )
#define portSYSTICK_INTERRUPT_STATUS			(  ( volatile unsigned long* )( portSYSTICK_BASE + 0x0C ) )
#define portSYSTICK_CTRL_ENABLE_PERIODIC_INTERRUPTS	( 0x00000007UL )
#define portSYSTICK_PRESCALE				( 99UL )
#define portSYSTICK_VECTOR_ID				( 29UL )

/* Declare handler prototypes */
void	vPortSVCHandler(void) __attribute__ (( naked ));
void	vPortInterruptContext(void) __attribute__ (( naked ));
void	Undefined_Handler_Panic(void);
void	Prefetch_Handler_Panic(void);
void	Abort_Handler_Panic(void);

/* Declare systick handler routine */
void	vPortSysTickHandler(void);

/* Declare initialization routine */
void	_init(void);

/* Declare the main routine */
void	main(void);

/* Define  Exception Handler Routines */

/**
 * vPortSVCHandler
 * Routine for handling interrupts raised by software i.e. SWI_Handler
 * It is used to change processor mode to supervisor and it is usually
 * used for context switching. Currenlty, it is just a dummy function
 */
void vPortSVCHandler(void)
{
	while(1);
}

/**
 * vPortInterruptContext
 * This routine is invoked when an IRQ or FIQ is triggered
 */
void vPortInterruptContext(void)
{
	__asm volatile(
			"sub	lr, lr, #4				\n"
			"srsdb	SP, #31					\n"
			"stmdb	SP, {SP}^				\n"
			"sub	SP, SP, #4				\n"
			"ldmia	SP!, {lr}				\n"
			"sub	LR, LR, #8				\n"
			"stmdb	LR, {r0-lr}^				\n"
			"sub	LR, LR, #60				\n"
			"ldr	r9, pxCurrentTCBConst2			\n"	/* FIXME */
			"ldr	r8, [r9]				\n"
			"str	lr, [r8]				\n"
			"bl	vPortGICInterruptHandler		\n"	/* FIXME */
			"ldr	r8, [r9]				\n"
			"ldr	lr, [r8]				\n"
			"ldmia	lr, {r0-lr}^				\n"
			"add	lr, lr, #60				\n"
			"rfeia	lr					\n"
			"nop						\n"
			);
}

/**
 * vPortGICInterruptHandler
 * Interrupt handler routine for GIC invoked by IRQ or FIQ context
 */
void vPortGICInterruptHandler(void)
{
	unsigned long ulVector = 0UL;
	unsigned long ulGICBaseAddress = portGIC_PRIVATE_BASE;

	/* Query the private address first */
	ulVector = portGIC_READ(portGIC_ICCIAR(ulGICBaseAddress));

	if (ulVector == portSYSTICK_VECTOR_ID) {
		/* Invoke the associated interrupt handler */
		vPortSysTickHandler();

		/* Acknowledge the interrupt */
		portGIC_WRITE(portGIC_ICCEOIR(ulGICBaseAddress), ulVector);
	} else {
		/* Spurious interrupt - Nothing to do */
	}

	return;
}

/**
 * vPortSysTickHandler
 * Routine to handle systick timer interrupt
 */
void vPortSysTickHandler(void)
{
	/* Clear the interrupt */
	*(portSYSTICK_INTERRUPT_STATUS) = 0x01UL;

	/* Increment the tick counter */
	vTaskIncrementTick();							/* FIXME */

	/* All done here */
	return;
}

/**
 * Undefined_Handler_Panic
 * Stub for handling undefined exception
 */
void Undefined_Handler_Panic (void)
{
	while(1);
}

/**
 * Prefetch_Handler_Panic
 * Stub for handling prefetch exception
 */
void Prefetch_Handler_Panic (void)
{
	while(1);
}

/**
 * Abort_Handler_Panic
 * Stub for handling abort exception
 */
void Abort_Handler_Panic (void)
{
	while(1);
}

/**
 * _init
 * Primary hardware intialization function
 */
void _init(void)
{
	int i;
	unsigned long 		*pulSrc, *pulDst;
	volatile unsigned long 	ulSCTLR = 0UL;
	extern unsigned long 	__isr_vector_start;				/* FIXME */	
	extern unsigned long 	__isr_vector_end;				/* FIXME */	
	extern unsigned long 	_bss;						/* FIXME */	
	extern unsigned long 	_ebss;						/* FIXME */	

	/* Zero out the bss section */
	for (pulDst = &_bss; pulDst < &_ebss; ) {
		*pulDst++ = 0;
	}

	/* Configure the stack pointer for processor modes */
	__asm volatile (
			"cps	#17			\n"
			"nop				\n"
			"mov	SP, %[fiqsp]		\n"
			"nop				\n"
			"cps	#18			\n"
			"nop				\n"
			"mov	SP, %[irqsp]		\n"
			"nop				\n"
			"cps	#23			\n"
			"nop				\n"
			"mov	SP, %[abtsp]		\n"
			"nop				\n"
			"cps	#27			\n"
			"nop				\n"
			"mov	SP, %[abtsp]		\n"
			"nop				\n"
			"cps	#19			\n"
			"nop				\n"
			"mov	SP, %[svcsp]		\n"
			"nop				\n"
			: :	[fiqsp] "r" (puxFIQStackPointer),		/* FIXME */
				[irqsp] "r" (puxIRQStackPointer),		/* FIXME */
				[abtsp] "r" (puxAbortStackPointer),		/* FIXME */
				[svcsp] "r" (puxSVCStackPointer)		/* FIXME */
				: );

	/* Copy the exception vector table */
	pulSrc = (unsigned long*)&__isr_vector_start;
	pulDst = (unsigned long*)&__portEXCEPTION_VECTORS_BASE;

	for ( ; pulSrc < &__isr_vector_end; ) {
		*pulDst++ = *pulSrc++;
	}

	/* Modify vbar to point to the new base address */
	pulDst = (unsigned long*)&__portEXCEPTION_VECTORS_BASE;

	__asm volatile (
			"mcr	p15, 0, %[vbar], c12, c0, 0	\n"
			: :	[vbar] "r" (pulDst)
				: );

	/* Modify SCTLR to change the vector table address */
	ulSCTLR = ReadSCTLR();
	ulSCTLR = ~(1 << 13);
	WriteSCTLR(uSCTLR);

	/* Invoke the main funciton */
	main();

	/* Nothing more to do */
	return;
}

void main(void)
{
	/* Disable Interrupts */
	portDISABLE_INTERRUPTS();						/* FIXME */

	/* Zero out the tick count */
	xTickCount = 0UL;							/* FIXME */

	/* Setup timer interrupt */
	prvSetupTimerInterrupt();						/* FIXME */

	/* Configure GIC to pass interrupts to the processor */
	portGIC_WRITE( portGIC_ICDDCR( portGIC_DISTRIBUTOR_BASE ), 0x01UL );
	portGIC_WRITE( portGIC_ICCBPR( portGIC_PRIVATE_BASE ), 0x00UL );
	portGIC_WRITE( portGIC_ICCPMR( portGIC_PRIVATE_BASE ), configLOWEST_INTERRUPT_PRIORITY );
	portGIC_WRITE( portGIC_ICCICR( portGIC_PRIVATE_BASE ), 0x01UL);

	/* All done */
	return;
}
