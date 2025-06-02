// src/main.c

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
int main(void) {
    // Print a simple message.
    // "\n" is newline, "\r" is carriage return. Both are often needed for
    // proper console display.
    print_str_uart0("Bare-metal test on QEMU lm3s6965evb: SUCCESS!\n\r");

    // Loop forever. In a bare-metal system, main() should not return.
    while (1) {
        // You could put a simple delay here and print something periodically
        // in a more advanced test. For now, just loop.
    }

    return 0; // This line should never be reached.
}