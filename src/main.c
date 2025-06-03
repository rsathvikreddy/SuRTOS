// src/main.c
#include "rtos_kernel.h" // Include RTOS header for task creation and scheduler
#include <stddef.h>     // For NULL definition

/*
 * For QEMU emulating an LM3S6965EVB, UART0 is typically available.
 * The Data Register (DR) for UART0 on this chip is at address 0x4000C000.
 * We define a pointer to this memory-mapped register.
 * 'volatile' is important to ensure the compiler doesn't optimize away accesses.
 */
#define UART0_DR (*((volatile unsigned int *)0x4000C000))

/*
 * A simple function to print a single character to UART0.
 * For this basic QEMU test, we just write to the data register.
 * A real UART driver would check status flags (e.g., if the transmitter is busy).
 */
void print_char_uart0(char c) {
    UART0_DR = c;
}

/*
 * A simple function to print a null-terminated string to UART0.
 */
void print_str_uart0(const char* str) {
    while (*str != '\0') {
        print_char_uart0(*str);
        str++;
    }
}

/*
 * The entry point of our C code, called by Reset_Handler from startup_cortexm3.S
 */

// --- Task Definitions ---
void task1_function(void *args) {
    (void)args; // Unused parameter
    while (1) {
        print_str_uart0("Task 1 is running!\n\r");
        // Simple busy-wait delay
        for (volatile int i = 0; i < 100000; i++);
    }
}

void task2_function(void *args) {
    (void)args; // Unused parameter
    while (1) {
        print_str_uart0("Task 2 is running!\n\r");
        // Simple busy-wait delay
        for (volatile int i = 0; i < 150000; i++); // Slightly different delay
    }
}

int main(void) {
    // Create tasks
    // The stack_size parameter (e.g., 256) is checked by rtos_task_create,
    // but the actual allocation uses DEFAULT_STACK_SIZE from rtos_kernel.c.
    // Ensure the passed value meets MIN_STACK_SIZE and MAX_ALLOWED_STACK_SIZE.
    int task1_ret = rtos_task_create(task1_function, NULL, 256);
    int task2_ret = rtos_task_create(task2_function, NULL, 256);

    if (task1_ret != 0 || task2_ret != 0) {
        print_str_uart0("Error creating tasks!\n\r");
        // Handle error, perhaps loop indefinitely
        while(1);
    }

    print_str_uart0("Tasks created. Starting scheduler...\n\r");
    rtos_start_scheduler(); // This function should not return

    // Should never be reached
    return 0;
}