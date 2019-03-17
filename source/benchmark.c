/*****************************************************************************
* Wedge
*      High performance, hardware-optimized graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* benchmark.c
*      Platform-dependent implementation of functions related to obtaining
*      consistent wall-clock benchmark times, measured at the resolution of
*      milliseconds.
*****************************************************************************/

#include "versioninfo.h"

#ifdef GRAZELLE_WINDOWS
#include <Windows.h>
#else
#include <time.h>
#endif


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "benchmark.h" for documentation.

void benchmark_start(double* const time_counter)
{
#ifdef GRAZELLE_WINDOWS
    *time_counter = (double)GetTickCount64();
#else
    struct timespec time_val;
    clock_gettime(CLOCK_MONOTONIC_RAW, &time_val);
    *time_counter = ((double)time_val.tv_sec * 1000.0) + ((double)time_val.tv_nsec / 1000000.0);
#endif
}

// ---------

double benchmark_stop(const double* const time_counter)
{
    double time_elapsed = 0.0;

#ifdef GRAZELLE_WINDOWS
    time_elapsed = (double)GetTickCount64() - *time_counter;
#else
    struct timespec time_val;
    clock_gettime(CLOCK_MONOTONIC_RAW, &time_val);
    time_elapsed = ((double)time_val.tv_sec * 1000.0) + ((double)time_val.tv_nsec / 1000000.0) - *time_counter;
#endif

    return time_elapsed;
}
