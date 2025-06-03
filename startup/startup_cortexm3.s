    .syntax unified     // Use unified ARM/Thumb syntax
    .arch   armv7-m     // Target ARMv7-M architecture (Cortex-M3/M4)
    .thumb              // Use Thumb-2 instructions (default for Cortex-M)

    // Define global symbols that the linker will need or C code might see
    .global Reset_Handler
    .global _estack         // Symbol for the end of the stack (top of stack)

    // External symbols defined in C
    .global g_current_task
    .global g_next_scheduled_task
    .extern SysTick_Handler // Defined in rtos_kernel.c
    .extern main            // Entry point to C code

    // --- Vector Table ---
    // This section must be placed at the beginning of memory by the linker script.
    .section .isr_vector, "a", %progbits // "a"=allocatable, "x"=executable, %progbits=contains program data
    .type   g_pfnVectors, %object       // Mark as an object (data)
    .size   g_pfnVectors, . - g_pfnVectors // Optional: size of the vector table

g_pfnVectors:
    .long   _estack         // Initial Stack Pointer (MSP)
    .long   Reset_Handler   // Reset Handler
    .long   NMI_Handler     // NMI Handler
    .long   HardFault_Handler // Hard Fault Handler
    .long   MemManage_Handler // MPU Fault Handler
    .long   BusFault_Handler  // Bus Fault Handler
    .long   UsageFault_Handler// Usage Fault Handler
    .long   0               // Reserved
    .long   0               // Reserved
    .long   0               // Reserved
    .long   0               // Reserved
    .long   SVC_Handler     // SVCall Handler
    .long   DebugMon_Handler// Debug Monitor Handler
    .long   0               // Reserved
    .long   PendSV_Handler  // PendSV Handler
    .long   SysTick_Handler // SysTick Handler

    // Add more device-specific interrupt handlers here if needed,
    // otherwise, they can all point to Default_Handler.
    // For this minimal example, we'll just define a Default_Handler
    // for the core exceptions.

// --- Reset Handler ---
.section .text.Reset_Handler // Place Reset_Handler code in .text section
    .type   Reset_Handler, %function
Reset_Handler:
    // Set the stack pointer.
    // The _estack symbol is defined in the linker script and marks the top of the stack.
    ldr     r0, =_estack
    mov     sp, r0

    // Optionally, copy .data section from Flash to RAM and zero .bss section.
    // For this minimal example, we will skip this for simplicity.
    // If your main() uses global/static variables that are initialized,
    // or uninitialized (and expects them to be zero), this step is crucial.
    // A more complete startup file would call a function like SystemInit here
    // (often a weak symbol) for early hardware setup (e.g., clocks).

    // Branch to C an C++ library initialisation (if using it) - Not in this minimal example.
    // Branch to the main C function.
    bl      main

    // If main ever returns (it shouldn't in a bare-metal embedded system),
    // loop indefinitely to prevent running off into undefined memory.
LoopForever:
    b       LoopForever
.size Reset_Handler, . - Reset_Handler


// --- Default Exception Handlers ---
// These are simple infinite loops. In a real application, they'd do something more useful.
.section .text.Default_Handler
    .type Default_Handler, %function
Default_Handler:
    b       . // Infinite loop
.size Default_Handler, . - Default_Handler

// Assign the Default_Handler to the other core exception handlers by default.
// You can override these by defining specific handlers for them.
    .weak   NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak   HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak   MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler

    .weak   BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler

    .weak   UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler

    .weak   SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak   DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler

    .weak   PendSV_Handler  // This will be replaced by our strong definition below
    // .thumb_set PendSV_Handler, Default_Handler // Remove this line as we provide a full PendSV_Handler

    // SysTick_Handler is defined in C, so the linker will pick that strong symbol.
    // The weak definition below is just a fallback if C definition is not found.
    .weak   SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler


    // --- PendSV Handler (Context Switch) ---
    .section .text.PendSV_Handler, "ax", %progbits
    .global PendSV_Handler      // Ensure it's global
    .type PendSV_Handler, %function
PendSV_Handler:
    CPSID   I                               // Disable interrupts to ensure atomicity

    // Save context of the current task (g_current_task)
    LDR     R2, =g_current_task             // R2 = &g_current_task
    LDR     R3, [R2]                        // R3 = g_current_task (TCB pointer)
    CBZ     R3, PendSV_RestoreContextOnly   // If g_current_task is NULL, skip saving context (first switch)

    MRS     R0, PSP                         // R0 = Process Stack Pointer of current task
    STMDB   R0!, {R4-R11}                   // Save R4-R11 onto the task's stack, update R0
                                            // (Assumes ARM EABI where R4-R11 are callee-saved)
    STR     R0, [R3]                        // Store new PSP back into current_task_tcb->stack_pointer
                                            // (Assuming stack_pointer is the first field, offset 0)

PendSV_RestoreContextOnly:
    // Load context of the next task (g_next_scheduled_task)
    LDR     R0, =g_next_scheduled_task      // R0 = &g_next_scheduled_task
    LDR     R1, [R0]                        // R1 = g_next_scheduled_task (TCB pointer for task to switch to)

    LDR     R2, =g_current_task             // R2 = &g_current_task
    STR     R1, [R2]                        // g_current_task = g_next_scheduled_task (update g_current_task)

    LDR     R0, [R1]                        // R0 = new_current_task_tcb->stack_pointer
                                            // (Assuming stack_pointer is the first field, offset 0)
    LDMIA   R0!, {R4-R11}                   // Restore R4-R11 from the new task's stack, update R0
    MSR     PSP, R0                         // Set Process Stack Pointer to the new task's stack top

    CPSIE   I                               // Re-enable interrupts

    // Return from exception. This will use the EXC_RETURN value automatically
    // loaded into LR when the PendSV exception was entered.
    // For tasks, this should be 0xFFFFFFFD (return to Thread mode, use PSP).
    // The hardware will automatically unstack the remaining registers (xPSR, PC, LR, R12, R3, R2, R1, R0).
    BX      LR
    .size PendSV_Handler, . - PendSV_Handler

    // The .stack section is not strictly necessary to define in the startup file
    // if the linker script defines _estack at the top of RAM.
    // However, some startup files define a nominal stack area here.
    // We rely on the linker script to place _estack correctly.
    