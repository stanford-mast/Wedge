/*****************************************************************************
 * Spindle
 *   Multi-platform topology-aware thread control library.
 *   Distributes a set of synchronized tasks over cores in the system.
 *****************************************************************************
 * Authored by Samuel Grossman
 * Department of Electrical Engineering, Stanford University
 * Copyright (c) 2016-2017
 *************************************************************************//**
 * @file spindle.h
 *   Declaration of external API functions.
 *   Top-level header file for this library, to be included externally.
 *****************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>


// -------- CONSTANTS ------------------------------------------------------ //

/// In the `numThreads` field of #SSpindleTaskSpec, specifies to use all available threads on a NUMA node.
#define kSpindleTaskSpecAllAvailableThreads     0

/// In the `numThreads` field of #SSpindleTaskSpec, specifies to use the same number of threads for the current task as for the previous task.
/// When spawning threads, Spindle processes task specifications in index order (0 to `taskCount - 1`).
/// If `taskSpec[i].numThreads` is equal to this value, then the number of threads will be set to whatever Spindle used or calculated for `taskSpec[i-1]`.
/// If there are insufficient threads left on the current NUMA node, then this will result in an error.
#define kSpindleTaskSpecThreadsSameAsPrevious   UINT32_MAX


// -------- TYPE DEFINITIONS ----------------------------------------------- //

/// Signature of the starting function of each thread.
/// Must return nothing and accept a single parameter.
typedef void (* TSpindleFunc)(void* arg);

/// Enumerates supported SMT thread assignment policies.
/// Each policy specifies how Spindle should order its assignment of threads to cores, where each core may have multiple logical threads (by means of simultaneous multithreading, or SMT).
/// As an example, consider a task with 7 threads to be assigned to 4 physical cores, each supporting 2 logical cores (hardware threads).
/// Preferring physical cores would assign threads in the order P0L0, P1L0, P2L0, P3L0, P0L1, P1L1, and finally P2L1.
/// Preferring logical cores would assign threads in the order P0L0, P0L1, P1L0, P1L1, P2L0, P2L1, and finally P3L0.
/// The correct policy depends largely on the tasks themselves and how each thread shares data with other threads.
/// Regardless of the SMT policy, separate tasks are always affinitized to different physical cores.
typedef enum ESpindleSMTPolicy
{
    SpindleSMTPolicyDisableSMT,                                             ///< Disable SMT completely. Reserve one physical core per thread and affinitize each thread to a different physical core.
    SpindleSMTPolicyPreferPhysical,                                         ///< When assigning threads to cores, assign consecutive threads to different physical cores.
    SpindleSMTPolicyPreferLogical                                           ///< When assigning threads to cores, saturate each physical core (by assigning a thread to all logical cores) before moving onto the next one.
} ESpindleSMTPolicy;

/// Specifies a Spindle task that can be created and assigned to threads.
typedef struct SSpindleTaskSpec
{
    TSpindleFunc func;                                                      ///< Starting function to call for each thread.
    void* arg;                                                              ///< Argument to pass to the starting function.
    uint32_t numaNode;                                                      ///< Zero-based index of the NUMA node on which to create the threads.
    uint32_t numThreads;                                                    ///< Number of threads to create, or 0 to use all remaining threads available.
    ESpindleSMTPolicy smtPolicy;                                            ///< Specifies the policy for distributing threads among cores that may each have multiple hardware threads.
} SSpindleTaskSpec;


// -------- FUNCTIONS ------------------------------------------------------ //
#ifdef __cplusplus
extern "C" {
#endif

/// Checks whether execution is happening within a Spindle parallelized region.
/// Only one such region can exist at any time.
/// @return `true` if so, `false` otherwise.
bool spindleIsInParallelRegion(void);

/// Spawns threads according to the provided task specification.
/// NUMA node indices must appear in monotonically increasing order in the array, and only the last entry per NUMA node may specify 0 (automatically-determined) threads.
/// @param [in] taskSpec Task specifications, as an array.
/// @param [in] taskCount Number of tasks specified.
/// @param [in] useCurrentThread `true` if the calling thread should be used as a worker (can improve performance), `false` otherwise.
/// @return 0 once all spawned threads have terminated, or nonzero in the event of an error.
uint32_t spindleThreadsSpawn(SSpindleTaskSpec* taskSpec, uint32_t taskCount, bool useCurrentThread);

/// Retrieves the current thread's local ID within its task.
/// Undefined return value if called outside the context of a code region parallelized by this library.
/// @return Current thread's local ID.
uint32_t spindleGetLocalThreadID(void);

/// Retrieves the current thread's global ID, unique among all spawned threads.
/// Undefined return value if called outside the context of a code region parallelized by this library.
/// @return Current thread's global ID.
uint32_t spindleGetGlobalThreadID(void);

/// Retrieves the current thread's logical task number.
/// Undefined return value if called outside the context of a code region parallelized by this library.
/// @return Current thread's task ID.
uint32_t spindleGetTaskID(void);

/// Retrieves the number of threads in the current thread's logical task.
/// Undefined return value if called outside the context of a code region parallelized by this library.
/// @return Number of threads in the current thread's task.
uint32_t spindleGetLocalThreadCount(void);

/// Retrieves the total number of threads spawned globally.
/// Undefined return value if called outside the context of a code region parallelized by this library.
/// @return Total number of threads spawned globally.
uint32_t spindleGetGlobalThreadCount(void);

/// Retrieves the total number of tasks.
/// Undefined return value if called outside the context of a code region parallelized by this library.
/// @return Total number of tasks.
uint32_t spindleGetTaskCount(void);

/// Sets the value of the current thread's 64-bit per-thread variable.
/// This variable can be used for any purpose and is valid only within the context of a code region parallelized by this library.
/// @param [in] value Value to set.
void spindleSetLocalVariable(uint64_t value);

/// Retrieves the value of the current thread's 64-bit per-thread variable.
/// This variable can be used for any purpose and is valid only within the context of a code region parallelized by this library.
/// @return Value of the current thread's per-thread variable.
uint64_t spindleGetLocalVariable(void);

/// Provides a barrier that no thread can pass until all threads in the current task have reached this point in the execution.
/// Useful for synchronization.
void spindleBarrierLocal(void);

/// Provides a barrier that no thread can pass until all threads have reached this point in the execution.
/// Useful for synchronization.
void spindleBarrierGlobal(void);

/// Provides a barrier that no thread can pass until all threads in the current task have reached this point in the execution.
/// Useful for synchronization, but this version measures the time a thread spends waiting.
/// @return Number of cycles the calling thread spent waiting, captured using the `rdtsc` instruction.
uint64_t spindleTimedBarrierLocal(void);

/// Provides a barrier that no thread can pass until all threads have reached this point in the execution.
/// Useful for synchronization, but this version measures the time a thread spends waiting.
/// @return Number of cycles the calling thread spent waiting, captured using the `rdtsc` instruction.
uint64_t spindleTimedBarrierGlobal(void);

/// Shares a 64-bit data item with other threads in the same Spindle task.
/// Only one thread in the task should call this function.
/// @param [in] data Quantity that is to be shared.
void spindleDataShareSendLocal(uint64_t data);

/// Shares a 64-bit data item with all other Spindle-created threads.
/// Only one thread should call this function.
/// @param [in] data Quantity that is to be shared.
void spindleDataShareSendGlobal(uint64_t data);

/// Receives a 64-bit data item shared by another thread in the same Spindle task.
/// All threads in the same task except the sender should call this function.
/// @return [out] Shared data item.
uint64_t spindleDataShareReceiveLocal(void);

/// Receives a 64-bit data item shared by another Spindle-created thread.
/// All threads except the sender should call this function.
/// @return [out] Shared data item.
uint64_t spindleDataShareReceiveGlobal(void);


#ifdef __cplusplus
}
#endif
