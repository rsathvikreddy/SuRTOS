#ifndef RTOS_KERNEL_H_
#define RTOS_KERNEL_H_

#include <stdint.h>

// Forward declaration for rtos_tcb_s needed for the next_task pointer
struct rtos_tcb_s;

// --- Global Variables ---
extern volatile uint32_t g_system_ticks; // System tick counter

// --- Function Declarations ---
/**
 * @brief Creates a new task and adds it to the scheduler.
 *
 * @param task_function Pointer to the function that the task will execute.
 * @param args Pointer to arguments to be passed to the task function.
 * @param stack_size The desired stack size in bytes for the new task.
 * @return int 0 on success, or a negative error code on failure.
 */
int rtos_task_create(void (*task_function)(void *args), void *args_param, uint32_t stack_size);

/**
 * @brief Defines the possible states of a task in the RTOS.
 */
typedef enum {
    RTOS_TASK_READY,    /**< Task is ready to run */
    RTOS_TASK_RUNNING,  /**< Task is currently running */
    RTOS_TASK_BLOCKED,  /**< Task is blocked, waiting for an event */
    RTOS_TASK_SUSPENDED /**< Task is suspended and not scheduled */
} rtos_task_state_t;

/**
 * @brief Structure representing a Task Control Block (TCB).
 *
 * Each task in the system will have a TCB that stores its context
 * and other relevant information for scheduling and management.
 */
typedef struct rtos_tcb_s {
    uint32_t *stack_pointer;     /**< Pointer to the current top of the task's stack */
    struct rtos_tcb_s *next_task; /**< Pointer to the next TCB in the scheduler's list */
    uint32_t task_id;            /**< Unique identifier for the task */
    rtos_task_state_t current_state; /**< Current state of the task */
    // Additional fields can be added here, e.g.:
    // - void (*task_handler)(void *); /**< Pointer to the task function */
    // - void *task_parameters;       /**< Parameters to pass to the task function */
    // - uint32_t priority;           /**< Task priority */
    // - uint32_t timeslice;          /**< Time slice allocated to the task (for round-robin) */
    // - void *stack_base;            /**< Pointer to the base of the task's stack for overflow detection */
    // - uint32_t stack_size;         /**< Total size of the task's stack */
} rtos_tcb_t;

/**
 * @brief Initializes the SysTick timer for generating periodic interrupts.
 *
 * @param freq_hz The desired frequency of the SysTick interrupt in Hz.
 */
void rtos_init_systick(uint32_t freq_hz);

/**
 * @brief Starts the RTOS scheduler.
 *
 * Initializes the system tick, sets up the first task context switch via PendSV,
 * and enables global interrupts. This function does not return.
 */
void rtos_start_scheduler(void);

// Note: SysTick_Handler is an ISR and typically not declared in a header,
// as its address is placed directly in the vector table.

#endif /* RTOS_KERNEL_H_ */
