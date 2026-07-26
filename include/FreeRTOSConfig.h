#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    #define configENABLE_FPU                         1
    #define configENABLE_MPU                         0
    #define configENABLE_TRUSTZONE                   0
    #define configRUN_FREERTOS_SECURE_ONLY           0
#endif

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     1
#define configUSE_TICK_HOOK                     1
#define configCPU_CLOCK_HZ                      (600000000UL)
#define configTICK_RATE_HZ                      (1000)
#define configMAX_PRIORITIES                    (8)
#define configMINIMAL_STACK_SIZE                (128)
#define configTOTAL_HEAP_SIZE                   ((size_t)(128 * 1024))
#define configMAX_TASK_NAME_LEN                 (16)
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES            1
#define configQUEUE_REGISTRY_SIZE               10
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  0
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0

#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1

#define configUSE_TASK_NOTIFICATIONS            1

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (2)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            (256)

#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1

#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define configASSERT(x) if((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configstatsTASK_NAME                    1

#define configUSE_TICKLESS_IDLE                 0

#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         (2)

#define configMAX_API_CALL_INTERRUPT_PRIORITY   5

#define xPortPendSVHandler                      PendSV_Handler
#define vPortSVCHandler                         SVC_Handler

#define CONTROL_LOOP_TICK_MS                    1
#define CONTROL_LOOP_PRIORITY                   (configMAX_PRIORITIES - 2)

#define SAFETY_LOOP_TICK_MS                     10
#define SAFETY_LOOP_PRIORITY                    (configMAX_PRIORITIES - 3)

#define TELEMETRY_LOOP_TICK_MS                  10
#define TELEMETRY_LOOP_PRIORITY                 (configMAX_PRIORITIES - 5)

#define TASK_CONTROL_STACK_SIZE                 512
#define TASK_SAFETY_STACK_SIZE                  256
#define TASK_TELEMETRY_STACK_SIZE               512

#define COMMAND_QUEUE_SIZE                      10
#define TELEMETRY_QUEUE_SIZE                    20

#define WATCHDOG_TIMEOUT_MS                     1000

#endif
