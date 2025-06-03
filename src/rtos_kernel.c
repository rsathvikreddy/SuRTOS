#include "rtos_kernel.h"
#include <stdint.h>
#include <stddef.h> // For NULL

// --- Register Definitions (ARM Cortex-M SysTick and SCB) ---
#define SYSTICK_STCSR       (*((volatile uint32_t *)0xE000E010)) // SysTick Control and Status Register
#define SYSTICK_STRVR       (*((volatile uint32_t *)0xE000E014)) // SysTick Reload Value Register
#define SYSTICK_STCVR       (*((volatile uint32_t *)0xE000E018)) // SysTick Current Value Register

#define ICSR                (*((volatile uint32_t *)0xE000ED04)) // Interrupt Control and State Register
#define SHPR3               (*((volatile uint32_t *)0xE000ED20)) // System Handler Priority Register 3

// SysTick Control and Status Register bits
#define SYSTICK_CSR_ENABLE_POS  0   // Bit 0: ENABLE
#define SYSTICK_CSR_TICKINT_POS 1   // Bit 1: TICKINT (enable interrupt)
#define SYSTICK_CSR_CLKSRC_POS  2   // Bit 2: CLKSOURCE (1=processor clock, 0=external)
#define SYSTICK_CSR_COUNTFLAG_POS 16 // Bit 16: COUNTFLAG (set if timer counted to 0 since last read)

// ICSR bits
#define PENDSVSET_BIT       (1UL << 28) // Bit 28: PendSV set-pending bit

// System Clock Frequency - specific to the target (e.g., QEMU lm3s6965evb)
#define SYSTEM_CLOCK_FREQ   50000000UL // 50 MHz


// --- Global Variables ---
rtos_tcb_t *g_current_task = NULL;    /**< Pointer to the currently running task's TCB */
rtos_tcb_t *g_task_list_head = NULL;  /**< Pointer to the head of the task list (circular) */
rtos_tcb_t *g_next_scheduled_task = NULL; /**< Pointer to the TCB of the next task to be scheduled */
volatile uint32_t g_system_ticks = 0; /**< System tick counter, incremented by SysTick_Handler */

// --- Static RTOS Resources ---
#define MAX_TASKS 4
#define DEFAULT_STACK_SIZE 256 // Bytes, ensure it's a multiple of 4 for uint32_t alignment
#define MIN_STACK_SIZE 128
#define MAX_ALLOWED_STACK_SIZE 1024 // Max stack size the create function will accept

//NOTE: sizeof() is a compile-time operator, not a preprocessor operator.
//This check should be done manually or with static_assert in C11+
//#if (DEFAULT_STACK_SIZE % sizeof(uint32_t) != 0)
//#error "DEFAULT_STACK_SIZE must be a multiple of sizeof(uint32_t)"
//#endif

static rtos_tcb_t g_task_tcbs[MAX_TASKS];
static uint32_t g_task_stacks[MAX_TASKS][DEFAULT_STACK_SIZE / sizeof(uint32_t)];
static uint8_t g_tcb_in_use[MAX_TASKS] = {0}; // 0 for not in use, 1 for in use

// --- Task ID Generation ---
static uint32_t g_next_task_id = 0;

// --- Error Codes ---
#define RTOS_OK 0
#define RTOS_ERROR_INVALID_STACK_SIZE -1
#define RTOS_ERROR_MAX_TASKS_REACHED -2
#define RTOS_ERROR_NULL_POINTER -3

/**
 * @brief Generates a unique task ID.
 * @return uint32_t The next available task ID.
 */
static uint32_t rtos_generate_task_id(void) {
    return g_next_task_id++;
}

/**
 * @brief Creates a new task and adds it to the scheduler.
 *
 * @param task_function Pointer to the function that the task will execute.
 * @param args Pointer to arguments to be passed to the task function.
 * @param stack_size The desired stack size in bytes for the new task.
 *                   Note: In this static allocation version, all tasks get DEFAULT_STACK_SIZE.
 *                   This parameter is checked against MIN_STACK_SIZE and MAX_ALLOWED_STACK_SIZE.
 * @return int RTOS_OK (0) on success, or a negative error code on failure.
 */
int rtos_task_create(void (*task_function)(void *args), void *args_param, uint32_t stack_size) {
    if (task_function == NULL) {
        return RTOS_ERROR_NULL_POINTER;
    }

    // Check stack_size parameter, though we use DEFAULT_STACK_SIZE for allocation
    if (stack_size < MIN_STACK_SIZE || stack_size > MAX_ALLOWED_STACK_SIZE) {
        return RTOS_ERROR_INVALID_STACK_SIZE;
    }

    int tcb_idx = -1;
    // Find an unused TCB and stack
    for (int i = 0; i < MAX_TASKS; ++i) {
        if (!g_tcb_in_use[i]) {
            tcb_idx = i;
            break;
        }
    }

    if (tcb_idx == -1) {
        return RTOS_ERROR_MAX_TASKS_REACHED; // No free TCBs
    }

    rtos_tcb_t *new_tcb = &g_task_tcbs[tcb_idx];
    uint32_t *stack_bottom = g_task_stacks[tcb_idx]; // Base of the allocated stack for this task
    uint32_t *stack_top = &stack_bottom[DEFAULT_STACK_SIZE / sizeof(uint32_t)]; // Point just beyond the stack

    // Initialize TCB
    new_tcb->task_id = rtos_generate_task_id();
    new_tcb->current_state = RTOS_TASK_READY;
    // stack_pointer will be set after stack initialization

    // Initialize the task's stack for context switching (ARM Cortex-M specific)
    // The stack grows downwards.
    // The layout must match what PendSV_Handler expects:
    // High address
    //  xPSR
    //  PC (task_function)
    //  LR (EXC_RETURN: 0xFFFFFFFD for Thread mode, MSP, no FPU)
    //  R12
    //  R3
    //  R2
    //  R1
    //  R0 (args_param)
    //  R11
    //  R10
    //  R9
    //  R8
    //  R7
    //  R6
    //  R5
    //  R4
    // Low address (this is what new_tcb->stack_pointer will point to)

    uint32_t *sp = stack_top; // Start at the very top (end) of the allocated stack memory

    *(--sp) = 0x01000000;  // xPSR (Thumb state)
    *(--sp) = (uint32_t)task_function; // PC
    *(--sp) = 0xFFFFFFFD;  // LR (EXC_RETURN to thread mode, using MSP, no FPU)
    *(--sp) = 0x12121212;  // R12
    *(--sp) = 0x03030303;  // R3
    *(--sp) = 0x02020202;  // R2
    *(--sp) = 0x01010101;  // R1
    *(--sp) = (uint32_t)args_param; // R0 (task argument)

    // Remaining registers (R4-R11) - often called "callee-saved" by AAPCS
    // but PendSV must save/restore them.
    *(--sp) = 0x11111111;  // R11
    *(--sp) = 0x10101010;  // R10
    *(--sp) = 0x09090909;  // R9
    *(--sp) = 0x08080808;  // R8
    *(--sp) = 0x07070707;  // R7
    *(--sp) = 0x06060606;  // R6
    *(--sp) = 0x05050505;  // R5
    *(--sp) = 0x04040404;  // R4

    new_tcb->stack_pointer = sp; // This is the task's current SP

    // Add to circular linked list
    if (g_task_list_head == NULL) {
        g_task_list_head = new_tcb;
        new_tcb->next_task = new_tcb; // Points to itself
        g_current_task = new_tcb;     // First task is initially the current task
        g_next_scheduled_task = new_tcb; // And also the next scheduled task
    } else {
        // Find the tail of the list and insert
        rtos_tcb_t *tail = g_task_list_head;
        while (tail->next_task != g_task_list_head) {
            tail = tail->next_task;
        }
        tail->next_task = new_tcb;
        new_tcb->next_task = g_task_list_head;
    }

    g_tcb_in_use[tcb_idx] = 1; // Mark TCB/stack as used

    return RTOS_OK;
}

/**
 * @brief Selects the next task to run based on a round-robin policy.
 *
 * This function iterates through the task list to find the next available
 * task in the RTOS_TASK_READY state. The selected task is stored in
 * `g_next_scheduled_task`. This function does NOT perform the context switch.
 *
 * If no tasks are ready, `g_next_scheduled_task` might not change, or could
 * point to an idle task if implemented.
 */
void rtos_schedule(void) {
    if (g_task_list_head == NULL || g_current_task == NULL) {
        // No tasks in the system or RTOS not fully started
        // g_next_scheduled_task might already be NULL or point to an idle task if we had one.
        // For now, if g_current_task is NULL, there's nothing to schedule from.
        // If g_task_list_head is NULL, there are no tasks at all.
        if(g_task_list_head != NULL) { // If there are tasks, but current is NULL (e.g. before first run)
            g_next_scheduled_task = g_task_list_head; // Default to head, assuming it could be ready
            // Iterate to find a ready one from head if list_head isn't guaranteed ready
            rtos_tcb_t* candidate = g_task_list_head;
            do {
                if (candidate->current_state == RTOS_TASK_READY) {
                    g_next_scheduled_task = candidate;
                    return;
                }
                candidate = candidate->next_task;
            } while (candidate != g_task_list_head);
            // If no task is ready, g_next_scheduled_task remains as it was (e.g. pointing to list_head which is not ready)
            // or could be set to NULL if we want to signify no task is schedulable.
            // For now, let PendSV handle if g_next_scheduled_task is not ready or same as current.
        } else {
             g_next_scheduled_task = NULL; // No tasks at all
        }
        return;
    }

    rtos_tcb_t *search_task = g_current_task->next_task;
    rtos_tcb_t *first_searched = search_task; // To detect full loop

    // Iterate to find the next ready task
    do {
        if (search_task->current_state == RTOS_TASK_READY) {
            g_next_scheduled_task = search_task;
            return; // Found a ready task
        }
        search_task = search_task->next_task;
    } while (search_task != first_searched);

    // If we looped through all tasks and didn't find a *different* ready task,
    // check if the current task itself is still ready.
    if (g_current_task->current_state == RTOS_TASK_READY) {
        g_next_scheduled_task = g_current_task;
    }
    // If no task (neither other tasks nor the current task) is READY,
    // g_next_scheduled_task remains unchanged from its previous value before this call.
    // This means if the current task was running and became blocked, and no other task
    // is ready, g_next_scheduled_task would still point to the (now blocked) current task.
    // The PendSV logic would then need to decide what to do (e.g., run an idle loop or re-evaluate).
    // A more robust system would explicitly switch to an idle task here if no tasks are ready.
    // For now, we assume PendSV will handle if g_current_task == g_next_scheduled_task or if g_next_scheduled_task->current_state != RTOS_TASK_READY
}

/**
 * @brief Initializes the SysTick timer for the RTOS.
 *
 * Configures the SysTick timer to generate interrupts at the specified frequency.
 * This interrupt is used as the system tick for scheduling.
 *
 * @param freq_hz The desired frequency of the SysTick interrupt in Hz.
 */
void rtos_init_systick(uint32_t freq_hz) {
    if (freq_hz == 0) {
        // Avoid division by zero and disable SysTick if freq is 0
        SYSTICK_STCSR = 0; // Disable SysTick
        return;
    }

    // Calculate the reload value for the SysTick timer
    // Reload Value = (Clock Frequency / Desired Interrupt Frequency) - 1
    // -1 because the counter counts from RELOAD_VALUE down to 0, and then reloads.
    uint32_t reload_value = (SYSTEM_CLOCK_FREQ / freq_hz) - 1;

    // Configure SysTick registers:
    // 1. Disable SysTick timer before configuration
    SYSTICK_STCSR = 0;

    // 2. Set the Reload Value Register (STRVR)
    // The value must be in the range [0x00000001, 0x00FFFFFF]
    if (reload_value > 0x00FFFFFF) {
        reload_value = 0x00FFFFFF; // Cap at maximum value
    }
    SYSTICK_STRVR = reload_value;

    // 3. Set the Current Value Register (STCVR) to any value to clear it
    SYSTICK_STCVR = 0; // Clears current value and COUNTFLAG in STCSR

    // 4. Configure the SysTick Control and Status Register (STCSR)
    //    - Enable SysTick interrupt (TICKINT = 1, bit 1)
    //    - Select processor clock as clock source (CLKSOURCE = 1, bit 2)
    //    - Enable the SysTick timer (ENABLE = 1, bit 0)
    SYSTICK_STCSR = (1 << SYSTICK_CSR_CLKSRC_POS) |
                    (1 << SYSTICK_CSR_TICKINT_POS) |
                    (1 << SYSTICK_CSR_ENABLE_POS);

    // Optional: Set SysTick interrupt priority (e.g., lowest)
    // SysTick is exception number 15. Priority is in SHPR3 bits [31:24]
    // Lower numerical values represent higher priorities.
    // Example: SHPR3 = (SHPR3 & 0x00FFFFFF) | (0xFF << 24); // Set to lowest priority
    // For now, relying on default priority.
}

/**
 * @brief SysTick Interrupt Service Routine (ISR).
 *
 * This function is called at every SysTick interrupt. It increments the
 * global system tick counter and calls the scheduler. If the scheduler
 * decides that a different task should run, it triggers a PendSV exception
 * to perform the context switch.
 */
void SysTick_Handler(void) {
    // Increment the system tick counter
    g_system_ticks++;

    // Call the scheduler to determine the next task
    rtos_schedule();

    // Check if a context switch is needed
    // A context switch is needed if:
    // 1. There is a task selected to run (g_next_scheduled_task is not NULL)
    // 2. The selected task is different from the currently running task
    if (g_next_scheduled_task != NULL && g_next_scheduled_task != g_current_task) {
        // Trigger PendSV interrupt to perform the context switch
        // PendSV is set pending by writing 1 to bit 28 (PENDSVSET) of ICSR
        ICSR = PENDSVSET_BIT;
    }
    // If g_next_scheduled_task is NULL or same as g_current_task, PendSV is not triggered.
    // If g_next_scheduled_task points to a task that is not READY, the PendSV handler
    // should ideally handle this (e.g., by not switching or switching to an idle task).
}

/**
 * @brief Starts the RTOS scheduler and the first task.
 *
 * This function initializes the SysTick timer for periodic interrupts,
 * sets up the environment for the first context switch by PendSV,
 * and enables global interrupts. It assumes that tasks have already been created.
 * This function does not return, as control is transferred to the scheduled tasks.
 */
void rtos_start_scheduler(void) {
    // Ensure at least one task is ready to be scheduled.
    // g_next_scheduled_task should have been set when the first task was created
    // or by rtos_schedule if it was called prior (though not typical before start).
    if (g_next_scheduled_task == NULL) {
        // Error state: No tasks created or scheduler cannot determine the first task.
        // In a real system, this might trigger an error handler, log, or reset.
        // For now, we'll enter an infinite loop to halt further execution.
        while(1) {
            // Cannot start scheduler without tasks.
        }
    }

    // Initialize the SysTick timer. Let's aim for a 100 Hz tick rate.
    rtos_init_systick(100); // Tick frequency in Hz

    // Manually set the Process Stack Pointer (PSP) to 0.
    // The first PendSV will switch from Handler mode (MSP) to Thread mode (PSP).
    // The PendSV_Handler will then load the task's actual PSP.
    // Initializing PSP to 0 here is a safeguard to ensure it's not some random
    // value from the MSP if read by PendSV before the first task's PSP is loaded.
    __asm volatile ("MSR PSP, %0" : : "r" (0U));

    // Trigger PendSV interrupt to perform the first context switch and start the first task.
    // g_current_task is likely NULL at this point. PendSV_Handler is designed
    // to skip saving context if g_current_task is NULL and directly proceed to
    // restoring the context of g_next_scheduled_task.
    ICSR = PENDSVSET_BIT;

    // Enable global interrupts.
    // This allows SysTick and PendSV interrupts to be processed.
    __asm volatile ("CPSIE i");

    // The scheduler has now started. This function should not be returned from.
    // An infinite loop here could catch unexpected returns, though ideally unreachable.
    while(1) {
        // Should not be reached.
    }
}


// Further RTOS functions (scheduler, context switch, etc.) would go here
// For example:
// void rtos_scheduler_start(void);
// void rtos_yield(void);
// PendSV_Handler (assembly or naked C function)
