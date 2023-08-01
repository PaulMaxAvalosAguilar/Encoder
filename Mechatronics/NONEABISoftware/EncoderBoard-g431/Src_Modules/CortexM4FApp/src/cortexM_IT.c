#include "FreeRTOS.h"
#include "task.h"
#include "stm32g431xx.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void) __attribute__((naked));
void DebugMon_Handler(void);
void PendSV_Handler(void) __attribute__((naked));
void SysTick_Handler(void);
void TIM2_IRQHandler(void);

void NMI_Handler(void)
{
    while (1)
    {
    }
}

// This function handles Hard fault interrupt.
void HardFault_Handler(void)
{
    while (1)
    {
    }
}

// This function handles Memory management fault.
void MemManage_Handler(void)
{
    while (1)
    {
    }
}

// This function handles Prefetch fault, memory access fault.
void BusFault_Handler(void)
{
    while (1)
    {
    }
}

// This function handles Undefined instruction or illegal state.
void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

// This function handles System service call via SWI instruction.
void SVC_Handler(void)
{
    __asm volatile(
        "	ldr	r3, pxCurrentTCBConst2		\n" /* Restore the context. */
        "	ldr r1, [r3]					\n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
        "	ldr r0, [r1]					\n" /* The first item in pxCurrentTCB is the task top of stack. */
        "	ldmia r0!, {r4-r11, r14}		\n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
        "	msr psp, r0						\n" /* Restore the task stack pointer. */
        "	isb								\n"
        "	mov r0, #0 						\n"
        "	msr	basepri, r0					\n"
        "	bx r14							\n"
        "									\n"
        "	.align 4						\n"
        "pxCurrentTCBConst2: .word pxCurrentTCB				\n");
}

// This function handles Debug monitor.
void DebugMon_Handler(void)
{
}

// This function handles Pendable request for system service.
void PendSV_Handler(void)
{
    /* This is a naked function. */

    __asm volatile(
        "	mrs r0, psp							\n"
        "	isb									\n"
        "										\n"
        "	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
        "	ldr	r2, [r3]						\n"
        "										\n"
        "	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, push high vfp registers. */
        "	it eq								\n"
        "	vstmdbeq r0!, {s16-s31}				\n"
        "										\n"
        "	stmdb r0!, {r4-r11, r14}			\n" /* Save the core registers. */
        "	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
        "										\n"
        "	stmdb sp!, {r0, r3}					\n"
        "	mov r0, %0 							\n"
        "	msr basepri, r0						\n"
        "	dsb									\n"
        "	isb									\n"
        "	bl vTaskSwitchContext				\n"
        "	mov r0, #0							\n"
        "	msr basepri, r0						\n"
        "	ldmia sp!, {r0, r3}					\n"
        "										\n"
        "	ldr r1, [r3]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
        "	ldr r0, [r1]						\n"
        "										\n"
        "	ldmia r0!, {r4-r11, r14}			\n" /* Pop the core registers. */
        "										\n"
        "	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, pop the high vfp registers too. */
        "	it eq								\n"
        "	vldmiaeq r0!, {s16-s31}				\n"
        "										\n"
        "	msr psp, r0							\n"
        "	isb									\n"
        "										\n"
#ifdef WORKAROUND_PMU_CM001 /* XMC4000 specific errata workaround. */
#if WORKAROUND_PMU_CM001 == 1
        "			push { r14 }				\n"
        "			pop { pc }					\n"
#endif
#endif
        "										\n"
        "	bx r14								\n"
        "										\n"
        "	.align 4							\n"
        "pxCurrentTCBConst: .word pxCurrentTCB	\n" ::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY));
}

// This function handles System tick timer.
void SysTick_Handler(void)
{
    /* The SysTick runs at the lowest interrupt priority, so when this interrupt
executes all interrupts must be unmasked.  There is therefore no need to
save and then restore the interrupt mask value as its value is already
known. */
    portDISABLE_INTERRUPTS();
    {
        /* Increment the RTOS tick. */
        if (xTaskIncrementTick() != pdFALSE)
        {
            /* A context switch is required.  Context switching is performed in
            the PendSV interrupt.  Pend the PendSV interrupt. */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
    }
    portENABLE_INTERRUPTS();
}

void TIM2_IRQHandler(void)
{
    __ucHAL_Encoder_function_ITAddEncoderValues();
}