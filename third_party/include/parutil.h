/*****************************************************************************
 * Parutil
 *   Multi-platform library of parallelized utility functions.
 *****************************************************************************
 * Authored by Samuel Grossman
 * Department of Electrical Engineering, Stanford University
 * Copyright (c) 2016-2017
 *************************************************************************//**
 * @file parutil.h
 *   Declaration of external API functions.
 *   Top-level header file for this library, to be included externally.
 *****************************************************************************/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// -------- TYPE DEFINITIONS ----------------------------------------------- //

/// Communicates static schedule information to applications that use Parutil's scheduling assistance functionality.
typedef struct SParutilStaticSchedule
{
    uint64_t startUnit;                                                     ///< First unit of work assigned to the current thread.
    uint64_t endUnit;                                                       ///< One-past-last unit of work assigned to the current thread.
    uint64_t increment;                                                     ///< Number of units of work between units of work assigned to the current thread.
} SParutilStaticSchedule;

/// Enumerates the different types of static schedulers Parutil implements.
/// Used along with scheduling assistance functions to identify which type of static scheduler to use.
typedef enum EParutilStaticScheduler
{
    ParutilStaticSchedulerChunked,                                          ///< Chunked scheduler, which creates one continuous chunk of work per thread.
} EParutilStaticScheduler;


#ifdef __cplusplus
extern "C" {
#endif

// -------- FUNCTIONS: ATOMIC ---------------------------------------------- //

/// Performs an atomic add operation with 8-bit operands.
/// The specified memory location is updated with its current value plus the supplied value.
/// @param [in,out] ptr Memory location of the value to add. This location is updated with the result of the summation.
/// @param [in] incr Value to add to the specified memory location.
void parutilAtomicAdd8(uint8_t* const ptr, const uint8_t incr);

/// Performs an atomic add operation with 16-bit operands.
/// The specified memory location is updated with its current value plus the supplied value.
/// @param [in,out] ptr Memory location of the value to add. This location is updated with the result of the summation.
/// @param [in] incr Value to add to the specified memory location.
void parutilAtomicAdd16(uint16_t* const ptr, const uint16_t incr);

/// Performs an atomic add operation with 32-bit operands.
/// The specified memory location is updated with its current value plus the supplied value.
/// @param [in,out] ptr Memory location of the value to add. This location is updated with the result of the summation.
/// @param [in] incr Value to add to the specified memory location.
void parutilAtomicAdd32(uint32_t* const ptr, const uint32_t incr);

/// Performs an atomic add operation with 64-bit operands.
/// The specified memory location is updated with its current value plus the supplied value.
/// @param [in,out] ptr Memory location of the value to add. This location is updated with the result of the summation.
/// @param [in] incr Value to add to the specified memory location.
void parutilAtomicAdd64(uint64_t* const ptr, const uint64_t incr);

/// Performs an atomic exchange operation with 8-bit operands.
/// The specified memory location is exchanged with the specified value.
/// @param [in,out] ptr Memory location of the value to be exchanged.
/// @param [in] value Value to be written to the memory location.
/// @return Value at the memory location prior to the exchange operation.
uint8_t parutilAtomicExchange8(uint8_t* const ptr, const uint8_t value);

/// Performs an atomic exchange operation with 16-bit operands.
/// The specified memory location is exchanged with the specified value.
/// @param [in,out] ptr Memory location of the value to be exchanged.
/// @param [in] value Value to be written to the memory location.
/// @return Value at the memory location prior to the exchange operation.
uint16_t parutilAtomicExchange16(uint16_t* const ptr, const uint16_t value);

/// Performs an atomic exchange operation with 32-bit operands.
/// The specified memory location is exchanged with the specified value.
/// @param [in,out] ptr Memory location of the value to be exchanged.
/// @param [in] value Value to be written to the memory location.
/// @return Value at the memory location prior to the exchange operation.
uint32_t parutilAtomicExchange32(uint32_t* const ptr, const uint32_t value);

/// Performs an atomic exchange operation with 64-bit operands.
/// The specified memory location is exchanged with the specified value.
/// @param [in,out] ptr Memory location of the value to be exchanged.
/// @param [in] value Value to be written to the memory location.
/// @return Value at the memory location prior to the exchange operation.
uint64_t parutilAtomicExchange64(uint64_t* const ptr, const uint64_t value);

/// Performs an atomic exchange-and-add operation with 8-bit operands.
/// The specified memory location is updated with its current value plus the supplied value, and the old value of the memory location is returned.
/// @param [in,out] ptr Memory location of the value to add. This location is updated with the result of the summation.
/// @param [in] incr Value to add to the specified memory location.
/// @return Value at the memory location prior to the addition operation.
uint8_t parutilAtomicExchangeAdd8(uint8_t* const ptr, const uint8_t incr);

/// Performs an atomic exchange-and-add operation with 16-bit operands.
/// The specified memory location is updated with its current value plus the supplied value, and the old value of the memory location is returned.
/// @param [in,out] ptr Memory location of the value to add. This location is updated with the result of the summation.
/// @param [in] incr Value to add to the specified memory location.
/// @return Value at the memory location prior to the addition operation.
uint16_t parutilAtomicExchangeAdd16(uint16_t* const ptr, const uint16_t incr);

/// Performs an atomic exchange-and-add operation with 32-bit operands.
/// The specified memory location is updated with its current value plus the supplied value, and the old value of the memory location is returned.
/// @param [in,out] ptr Memory location of the value to add. This location is updated with the result of the summation.
/// @param [in] incr Value to add to the specified memory location.
/// @return Value at the memory location prior to the addition operation.
uint32_t parutilAtomicExchangeAdd32(uint32_t* const ptr, const uint32_t incr);

/// Performs an atomic exchange-and-add operation with 64-bit operands.
/// The specified memory location is updated with its current value plus the supplied value, and the old value of the memory location is returned.
/// @param [in,out] ptr Memory location of the value to add. This location is updated with the result of the summation.
/// @param [in] incr Value to add to the specified memory location.
/// @return Value at the memory location prior to the addition operation.
uint64_t parutilAtomicExchangeAdd64(uint64_t* const ptr, const uint64_t incr);


// -------- FUNCTIONS: MEMORY ---------------------------------------------- //

/// Copies `num` bytes of memory at `source` to memory at `destination`.
/// Intended to be a drop-in replacement for the standard `memcpy()` function.
/// It is the caller's responsibility to ensure that `source` and `destination` regions do not overlap.
/// If called from within a Spindle parallelized region, every thread in the same task must invoke this function with the same arguments.
/// If not, uses all available hardware threads on the NUMA node of the destination buffer.
/// Reverts to standard `memcpy()` if `num` is small enough.
/// @param [in] destination Target memory buffer.
/// @param [in] source Source memory buffer.
/// @param [in] num Number of bytes to copy.
/// @return `destination` is returned upon completion.
void* parutilMemoryCopy(void* destination, const void* source, size_t num);

/// Filters `num` bytes of memory at `buffer` by performing bitwise-and with the value specified by `value`.
/// If called from within a Spindle parallelized region, every thread in the same task must invoke this function with the same arguments.
/// If not, uses all available hardware threads on the NUMA node of the destination buffer.
/// Reverts to standard `memset()` if `num` is small enough.
/// @param [in] buffer Target memory buffer.
/// @param [in] value Byte-sized value to write to the target memory buffer.
/// @param [in] num Number of bytes to filter.
/// @return `buffer` is returned upon completion.
void* parutilMemoryFilter(void* buffer, uint8_t value, size_t num);

/// Sets `num` bytes of memory at `buffer` to the value specified by `value`.
/// Intended to be a drop-in replacement for the standard `memset()` function.
/// If called from within a Spindle parallelized region, every thread in the same task must invoke this function with the same arguments.
/// If not, uses all available hardware threads on the NUMA node of the destination buffer.
/// Reverts to standard `memset()` if `num` is small enough.
/// @param [in] buffer Target memory buffer.
/// @param [in] value Byte-sized value to write to the target memory buffer.
/// @param [in] num Number of bytes to initialize.
/// @return `buffer` is returned upon completion.
void* parutilMemorySet(void* buffer, uint8_t value, size_t num);


// -------- FUNCTIONS: SCHEDULER ------------------------------------------- //

/// Uses a static scheduler of the specified type to provide the caller with information on assigned work.
/// Intended to be called within a Spindle parallelized region and will fail otherwise.
/// Each thread within a Spindle task should call this function with the same values for the first two parameters.
/// The information provided via the output #SParutilStaticSchedule can be used to determine which units of parallel work should be performed by each thread.
/// For example, when parallelizing a `for` loop, the thread could start with `startUnit`, compare for less-than with `endUnit`, and increment by `increment`.
/// @param [in] type Type of static scheduler to use.
/// @param [in] units Total number of units of work that need to be scheduled.
/// @param [out] schedule Scheduling information, provided as output.
/// @return `true` if successful (i.e. in a parallel region, output parameter is not `NULL`, and so on), `false` otherwise.
bool parutilSchedulerStatic(const EParutilStaticScheduler type, const uint64_t units, SParutilStaticSchedule* const schedule);

/// Initializes a dynamic scheduler to assign work to the caller and allocates required memory.
/// A dynamic scheduler assigns work to threads in the order that they become available to take on more work.
/// Intended to be called within a Spindle parallelized region and will fail otherwise.
/// Each thread within a Spindle task should call this function with the same value for the first parameter.
/// @param [in] numUnits Total number of units of work that need to be scheduled.
/// @param [out] schedule Pointer to a variable that will hold the dynamic scheduler handle used to identify the scheduler instance, or `NULL` on failure.
/// @return First unit of work to be performed by the caller, or `UINT64_MAX` if no work is available.
uint64_t parutilSchedulerDynamicInit(const uint64_t numUnits, void** schedule);

/// Resets a dynamic scheduler back to a state as if it has not yet assigned any work.
/// Intended to be called within a Spindle parallelized region and will fail otherwise.
/// Each thread within a Spindle task should call this function with the same value as received back from #parutilSchedulerDynamicInit.
/// @param [in] Handle used to identify the dynamic scheduler instance.
/// @return First unit of work to be performed by the caller, or `UINT64_MAX` if no work is available.
uint64_t parutilSchedulerDynamicReset(void* schedule);

/// Obtains the next unit of work to be assigned to the calling thread.
/// Intended to be called within a Spindle parallelized region and will fail otherwise.
/// Threads within a Spindle task should call this function with the same value as received back from #parutilSchedulerDynamicInit.
/// @param [in] Handle used to identify the dynamic scheduler instance.
/// @return Next unit of work to be performed by the caller, or `UINT64_MAX` if no work is available.
uint64_t parutilSchedulerDynamicGetWork(void* schedule);

/// Destroys the specified dynamic scheduler once no further work is available.
/// Intended to be called within a Spindle parallelized region and will fail otherwise.
/// Each thread within a Spindle task should call this function with the same value as received back from #parutilSchedulerDynamicInit.
/// All required synchronization is handled internally; there is no need for the caller to do any synchronization.
/// @param [in] Handle used to identify the dynamic scheduler instance.
void parutilSchedulerDynamicExit(void* schedule);


#ifdef __cplusplus
}
#endif
