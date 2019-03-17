/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* wedge.h
*      Declartion of Wedge frontier manipulation functionality.
*****************************************************************************/

#ifndef __GRAZELLE_WEDGE_H
#define __GRAZELLE_WEDGE_H


#include <stdint.h>


/* -------- FUNCTIONS ------------------------------------------------------ */

// Activates a single vertex in the local NUMA node's Wedge frontier.
void wedge_activate_single_vertex(const uint64_t vertex);

// Uses the global source-oriented vertex frontier to produce a Wedge frontier on the local NUMA node.
void wedge_generate_frontier(void);


#endif //__GRAZELLE_WEDGE_H
