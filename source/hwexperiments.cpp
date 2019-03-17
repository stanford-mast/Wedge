/*****************************************************************************
* Wedge
*      High performance, hardware-optimized graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* hwexperiments.cpp
*      Abstraction layer implementation for performing experiments that use
*      hardware counters.
*****************************************************************************/

#ifdef ENABLE_HARDWARE_EXPERIMENTS

#include <intelpcm/cpucounters.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#include "benchmark.h"


/* -------- LOCALS --------------------------------------------------------- */

// Captures the time elapsed during a measurement interval.
static double time_measurement_interval = 0.0;
    
// Captures the state of the system at the end of a measurement interval, one element per socket.
static ServerUncorePowerState* uncore_state_after = NULL;
    
// Captures the state of the system at the start of a measurement interval, one element per socket.
static ServerUncorePowerState* uncore_state_before = NULL;


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "hwexperiments.h" for documentation.

uint32_t hwexperiments_get_num_sockets(void)
{
    return PCM::getInstance()->getNumSockets();
}

// --------

double hwexperiments_get_measurement_time(void)
{
    return time_measurement_interval;
}

// --------

double hwexperiments_get_socket_mb_read(const uint32_t socket)
{
    double mb_read = 0.0;
    
    PCM* const pcm = PCM::getInstance();
    
    for (uint32_t channel = 0; channel < pcm->getMCChannelsPerSocket(); ++channel)
        mb_read += (((double)getMCCounter(channel, 0, uncore_state_before[socket], uncore_state_after[socket])) * 64.0 / 1024.0 / 1024.0);
    
    return mb_read;
}

// --------

double hwexperiments_get_socket_mb_write(const uint32_t socket)
{
    double mb_write = 0.0;
    
    PCM * pcm = PCM::getInstance();
    
    for (uint32_t channel = 0; channel < pcm->getMCChannelsPerSocket(); ++channel)
        mb_write += (((double)getMCCounter(channel, 1, uncore_state_before[socket], uncore_state_after[socket])) * 64.0 / 1024.0 / 1024.0);
    
    return mb_write;
}

// --------

void hwexperiments_initialize(void)
{
    PCM* pcm = PCM::getInstance();
    pcm->resetPMU();
    pcm->disableJKTWorkaround();
    
    PCM::ErrorCode status = pcm->programServerUncoreMemoryMetrics(-1, -1);
    switch (status)
    {
    case PCM::Success:
        break;
    case PCM::MSRAccessDenied:
        printf("Access to Intel(r) Performance Counter Monitor has denied (no MSR or PCI CFG space access).\n");
        exit(EXIT_FAILURE);
    case PCM::PMUBusy:
        printf("Access to Intel(r) Performance Counter Monitor has denied (Performance Monitoring Unit is occupied by other application). Try to stop the application that uses PMU.\n");
        exit(EXIT_FAILURE);
    default:
        printf("Access to Intel(r) Performance Counter Monitor has denied (Unknown error).\n");
        exit(EXIT_FAILURE);
    }
    
    uncore_state_after = new ServerUncorePowerState[pcm->getNumSockets()];
    uncore_state_before = new ServerUncorePowerState[pcm->getNumSockets()];
}

// --------

void hwexperiments_measure_start(void)
{
    PCM * pcm = PCM::getInstance();
    
    for (uint32_t i = 0; i < pcm->getNumSockets(); ++i)
        uncore_state_before[i] = pcm->getServerUncorePowerState(i);
    
    benchmark_start(&time_measurement_interval);
}

// --------

void hwexperiments_measure_stop(void)
{
    time_measurement_interval = benchmark_stop(&time_measurement_interval);
    
    PCM * pcm = PCM::getInstance();
    
    for (uint32_t i = 0; i < pcm->getNumSockets(); ++i)
        uncore_state_after[i] = pcm->getServerUncorePowerState(i);
}

// --------

void hwexperiments_uninitialize(void)
{
    PCM::getInstance()->cleanup();
    
    if (NULL != uncore_state_after)
    {
        delete[] uncore_state_after;
        uncore_state_after = NULL;
    }
    
    if (NULL != uncore_state_before)
    {
        delete[] uncore_state_before;
        uncore_state_before = NULL;
    }
}


#ifdef __cplusplus
}
#endif

#endif //ENABLE_HARDWARE_EXPERIMENTS
