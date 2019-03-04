        PUBLIC SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
		THUMB
SysTick_Handler
	EXTERN	SysTick_C_Handler
	PRESERVE8
	tst lr, #4 
    ite eq 
	mrseq r0, msp
	mrsne r0, psp
	push   {lr}
	bl SysTick_C_Handler
	pop    {lr}
	bx	   lr
	
	END
