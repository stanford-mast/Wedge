/*****************************************************************************
* Wedge
*      High performance, hardware-optimized graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* benchmark.h
*      Declaration of functions related to obtaining consistent wall-clock
*      benchmark times, generally at the resolution of milliseconds. These
*      declarations abstract away all platform-specific differences in the
*      actual mechanisms for obtaining these timing figures.
*****************************************************************************/

#ifndef __GRAZELLE_BENCHMARK_H
#define __GRAZELLE_BENCHMARK_H


/* -------- FUNCTIONS ------------------------------------------------------ */

// Starts the benchmark. Sets the internal time counter to the current time.
void benchmark_start(double* const time_counter);

// Stops the benchmark. Can only be called after benchmarking has been started.
// Returns the number of milliseconds that have passed since starting and resets the internal time counter.
double benchmark_stop(const double* const time_counter);


#endif //__GRAZELLE_BENCHMARK_H
