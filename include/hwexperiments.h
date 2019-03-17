/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* hwexperiments.h
*      Abstraction layer implementation for performing experiments that use
*      hardware counters.
*****************************************************************************/

#ifndef __GRAZELLE_HWEXPERIMENTS_H
#define __GRAZELLE_HWEXPERIMENTS_H

#ifdef ENABLE_HARDWARE_EXPERIMENTS

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* -------- FUNCTIONS ------------------------------------------------------ */

// Retrieves and returns the number of sockets in the system, as reported by the hardware experiments subsystem
uint32_t hwexperiments_get_num_sockets(void);

// Retrieves and returns the time interval, in milliseconds, between starting and stopping the measurements.
// Divide megabytes read or written per socket by this number to compute average bandwidth.
double hwexperiments_get_measurement_time(void);

// Retrieves and returns the total number of megabytes of data read for the specified socket during the last measurement interval.
double hwexperiments_get_socket_mb_read(const uint32_t socket);

// Retrieves and returns the total number of megabytes of data written for the specified socket during the last measurement interval.
double hwexperiments_get_socket_mb_write(const uint32_t socket);

// Initializes the hardware experiments subsystem.
// Must be called prior to beginning hardware experiments.
// Will cause the program to exit if initialization fails.
void hwexperiments_initialize(void);

// Starts a measurement interval.
void hwexperiments_measure_start(void);

// Stops a measurement interval.
void hwexperiments_measure_stop(void);

// Cleans up the hardware experiments subsystem.
// Should be called once measuring is finished.
void hwexperiments_uninitialize(void);

// Calculates the read memory bandwidth utilization for the specified socket during the previous measurement interval.
// Result is measured in GB/sec.
inline static double hwexperiments_get_bandwidth_read(const uint32_t socket)
{
    return hwexperiments_get_socket_mb_read(socket) / hwexperiments_get_measurement_time() / 1.024;
}

// Calculates the write memory bandwidth utilization for the specified socket during the previous measurement interval.
// Result is measured in GB/sec.
inline static double hwexperiments_get_bandwidth_write(const uint32_t socket)
{
    return hwexperiments_get_socket_mb_write(socket) / hwexperiments_get_measurement_time() / 1.024;
}


#ifdef __cplusplus
}
#endif

#endif //ENABLE_HARDWARE_EXPERIMENTS

#endif //__GRAZELLE_HWEXPERIMENTS_H
