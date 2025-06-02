    .syntax unified     // Use unified ARM/Thumb syntax
    .arch   armv7-m     // Target ARMv7-M architecture (Cortex-M3/M4)
    .thumb              // Use Thumb-2 instructions (default for Cortex-M)

    // Define global symbols that the linker will need or C code might see
    .global Reset_Handler
    .global _estack         // Symbol for the end of the stack (top of stack)

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

    .weak   PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    .weak   SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    // The .stack section is not strictly necessary to define in the startup file
    // if the linker script defines _estack at the top of RAM.
    // However, some startup files define a nominal stack area here.
    // We rely on the linker script to place _estack correctly.
    