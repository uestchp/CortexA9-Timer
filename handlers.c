/* This file defines exception handler routines for this platform */

#include <stdio.h>
#include <string.h>

/* Memory Map */
#define portPERIPH_BASE					0x1F000000UL
#define portGIC_PRIVATE_BASE				0x1F000100UL
#define portSYSTICK_BASE				0x1F000600UL
#define portGIC_DISTRIBUTOR_BASE			0x1F001000UL
#define portEXCEPTION_VECTORS_BASE			0x00000000UL

/* Port Macros */
#define	portGIC_READ(address)				( *( ( unsigned long* volatile )( address ) ) )
#define portGIC_WRITE(address, value)			( *( ( unsigned long* volatile )( address ) ) = ( value ) )
#define portGIC_ICCICR(x)				(  ( ( unsigned long ) ( x ) ) + 0x00UL )
#define portGIC_ICCPMR(x)				(  ( ( unsigned long ) ( x ) ) + 0x04UL )
#define portGIC_ICCBPR(x)				(  ( ( unsigned long ) ( x ) ) + 0x08UL )
#define portGIC_ICCIAR(x)				(  ( ( unsigned long ) ( x ) ) + 0x0CUL )
#define portGIC_ICCEOIR(x)				(  ( ( unsigned long ) ( x ) ) + 0x10UL )
#define portGIC_ICDDCR(x)				(  ( ( unsigned long ) ( x ) ) + 0x00UL )

/* Timer Macros */
#define portSYSTICK_LOAD				(  ( volatile unsigned long* )( portSYSTICK_BASE + 0x00 ) )
#define portSYSTICK_CONTROL				(  ( volatile unsigned long* )( portSYSTICK_BASE + 0x08 ) )
#define portSYSTICK_INTERRUPT_STATUS			(  ( volatile unsigned long* )( portSYSTICK_BASE + 0x0C ) )
#define portSYSTICK_CTRL_ENABLE_PERIODIC_INTERRUPTS	( 0x00000007UL )
#define portSYSTICK_PRESCALE				( 99UL )
#define portSYSTICK_VECTOR_ID				( 29UL )

#define configCPU_CLOCK_HZ				( ( unsigned long ) 2000000 )
#define configTICK_RATE_HZ				( 1000 )

/* General Macros */
#define configLOWEST_INTERRUPT_PRIORITY			( 0xFF )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY		( 0xA0 )

#define portFIQ_STACK_SIZE				( 256 )
#define portIRQ_STACK_SIZE				( 256 )
#define portABT_STACK_SIZE				( 256 )
#define portSVC_STACK_SIZE				( 256 )

/* Global Data */
static unsigned long xTickCount = 0;
void * volatile pxCurrentTCB = NULL;

static unsigned long puxFIQStack[ portFIQ_STACK_SIZE ];
static unsigned long puxIRQStack[ portIRQ_STACK_SIZE ];
static unsigned long puxABTStack[ portABT_STACK_SIZE ];
static unsigned long puxSVCStack[ portSVC_STACK_SIZE ];
static unsigned long *puxFIQStackPointer = &(puxFIQStack[ portFIQ_STACK_SIZE - 1 ]);
static unsigned long *puxIRQStackPointer = &(puxIRQStack[ portIRQ_STACK_SIZE - 1 ]);
static unsigned long *puxABTStackPointer = &(puxABTStack[ portIRQ_STACK_SIZE - 1 ]);
static unsigned long *puxSVCStackPointer = &(puxSVCStack[ portSVC_STACK_SIZE - 1 ]);

/* Declare handler prototypes */
void	vPortSVCHandler(void) __attribute__ (( naked ));
void	vPortInterruptContext(void) __attribute__ (( naked ));
void 	vPortGICInterruptHandler(void);
void 	vPortSysTickHandler(void);
void	Undefined_Handler_Panic(void);
void	Prefetch_Handler_Panic(void);
void	Abort_Handler_Panic(void);
void 	_init(void);
void	prvSetupTimerInterrupt(void);
void	portDISABLE_INTERRUPTS(void);
void 	portENABLE_INTERRUPTS(void);

static unsigned long ReadSCTLR();
static void WriteSCTLR(unsigned long SCTLR);

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
	__asm volatile(
			"ldr 	r9, pxCurrentTCBConst2			\n"
			"ldr 	r8, [r9]				\n"
			"ldr 	lr, [r8]				\n"
			"ldmia	lr, {r0-lr}^				\n"
			"add	lr, lr, #60				\n"
			"nop						\n"
			"rfeia	lr					\n"
			"nop						\n"
			"						\n"
			"	.align 2				\n"
			"pxCurrentTCBConst2: .word pxCurrentTCB		\n"
			);
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
			"ldr	r9, pxCurrentTCBConst2			\n"
			"ldr	r8, [r9]				\n"
			"str	lr, [r8]				\n"
			"bl	vPortGICInterruptHandler		\n"
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
	xTickCount++;

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
	extern unsigned long 	__isr_vector_start;	
	extern unsigned long 	__isr_vector_end;
	extern unsigned long 	_bss;
	extern unsigned long 	_ebss;

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
			: :	[fiqsp] "r" (puxFIQStackPointer),
				[irqsp] "r" (puxIRQStackPointer),
				[abtsp] "r" (puxABTStackPointer),
				[svcsp] "r" (puxSVCStackPointer)
				: );

	/* Copy the exception vector table */
	pulSrc = (unsigned long*)&__isr_vector_start;
	pulDst = (unsigned long*)portEXCEPTION_VECTORS_BASE;

	for ( ; pulSrc < &__isr_vector_end; ) {
		*pulDst++ = *pulSrc++;
	}

	/* Modify vbar to point to the new base address */
	pulDst = (unsigned long*)portEXCEPTION_VECTORS_BASE;

	__asm volatile (
			"mcr	p15, 0, %[vbar], c12, c0, 0	\n"
			: :	[vbar] "r" (pulDst)
				: );

	/* Modify SCTLR to change the vector table address */
	ulSCTLR = ReadSCTLR();
	ulSCTLR &= ~(1 << 13);
	WriteSCTLR(ulSCTLR);

	/* Invoke the main funciton */
	main();

	/* Nothing more to do */
	return;
}

void main(void)
{
	/* Disable Interrupts */
	portDISABLE_INTERRUPTS();

	/* Zero out the tick count */
	xTickCount = 0UL;

	/* Setup timer interrupt */
	prvSetupTimerInterrupt();

	/* Configure GIC to pass interrupts to the processor */
	portGIC_WRITE( portGIC_ICDDCR( portGIC_DISTRIBUTOR_BASE ), 0x01UL );
	portGIC_WRITE( portGIC_ICCBPR( portGIC_PRIVATE_BASE ), 0x00UL );
	portGIC_WRITE( portGIC_ICCPMR( portGIC_PRIVATE_BASE ), configLOWEST_INTERRUPT_PRIORITY );
	portGIC_WRITE( portGIC_ICCICR( portGIC_PRIVATE_BASE ), 0x01UL);

	/* Create a dummy task for kicks */
	tskTCB *pxNewTCB;						/* FIXME */

	/* Allocate storage for the TCB and stack of the dummy task */
	pxNewTCB = prvAllocateTCBAndStack( 128, NULL ); 		/* FIXME */

	portSTACK_TYPE *pxTopOfStack;

	pxTopOfStack = pxNewTCB->pxStack + ( 128 - (unsigned short)1 );
	pxTopOfStack = ( portStack_TYPE *) ( ( ( portPOINTER_SIZE_TYPE ) pxTopOfStack ) & ( ( portPOINTER_SIZE_TYPE ) ~portBYTE_ALIGNMENT_MASK ) );				/* FIXME */

	pxNewTCB->pxTopOfStack = pxPortInitialiseStack( pxTopOfStack, pxTaskCode, NULL ); /*FIXME */

	pxCurrentTCB = pxNewTCB

	/* Goto SVC Mode */
	__asm volatile(
			"mov SP, %[svcsp]		\n"
			"svc	0			\n"
			"nop				\n"
			: : [svcsp] "r" (puxSVCStackPointer) :);

	/* All done */
	return;
}

void prvSetupTimerInterrupt(void)
{
	*(portSYSTICK_LOAD) = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	*(portSYSTICK_CONTROL) = ( portSYSTICK_PRESCALE << 8 ) | portSYSTICK_CTRL_ENABLE_PERIODIC_INTERRUPTS;

	/* Setup complete */
	return;
}

void portDISABLE_INTERRUPTS(void)
{
	portGIC_WRITE( portGIC_ICCPMR(portGIC_PRIVATE_BASE), configMAX_SYSCALL_INTERRUPT_PRIORITY );

	/* Interrupts disabled */
	return;
}

void portENABLE_INTERRUPTS(void)
{
	portGIC_WRITE( portGIC_ICCPMR(portGIC_PRIVATE_BASE), configLOWEST_INTERRUPT_PRIORITY );
	__asm__ __volatile__ ( "nop" );

	return;
}

static unsigned long ReadSCTLR()
{
	unsigned long SCTLR;
	__asm volatile("mrc p15, 0, %[sctlr], c1, c0, 0"
			:[sctlr] "=r" (SCTLR)::);

	return SCTLR;
}

static void WriteSCTLR(unsigned long SCTLR)
{
	__asm volatile("mcr p15, 0, %[sctlr], c1, c0, 0"
			::[sctlr] "r" (SCTLR):);

	return;
}
