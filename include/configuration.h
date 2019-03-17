/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* configuration.h
*      Global compile-time configuration parameters.
*****************************************************************************/

#ifndef __GRAZELLE_CONFIGURATION_H
#define __GRAZELLE_CONFIGURATION_H


// Frontier precision (tuning parameter, modifiable using experiment flags)
#ifdef EXPERIMENT_FRONTIER_PRECISION
#define CONFIG_LOG2_FRONTIER_PRECISION                                      EXPERIMENT_FRONTIER_PRECISION
#else
#define CONFIG_LOG2_FRONTIER_PRECISION                                      2
#endif
#define CONFIG_FRONTIER_PRECISION                                           (1 << CONFIG_LOG2_FRONTIER_PRECISION)

// Maximum number of supported NUMA nodes
#define CONFIG_NUMA_MAX_NODES                                               4


#endif //__GRAZELLE_CONFIGURATION_H
