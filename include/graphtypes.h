/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* graphtypes.h
*      Declaration of common types used to represent a graph, and its parts,
*      in memory.
*****************************************************************************/

#ifndef __GRAZELLE_GRAPHTYPES_H
#define __GRAZELLE_GRAPHTYPES_H


#include "intrinhelper.h"

#include <stdint.h>


/* -------- TYPE DEFINITIONS ----------------------------------------------- */

// Specifies multiple different access methods for vertex property data structures.
typedef union vertexprop_t
{
    double d;
    uint64_t u;
} vertexprop_t;

// Defines the type of an element in the merge buffers, which are used to merge overlapped vertex updates to a collapsed accumulator
typedef struct mergeaccum_t
{
    uint64_t initial_vertex_id;                             // first target vertex ID written by the corresponding thread
    uint64_t final_vertex_id;                               // final target vertex ID written by the corresponding thread
    vertexprop_t final_partial_value;                       // final partial value to be written by the corresponding thread
    
    uint64_t __padding__;                                   // nothing, just pads the size of each entry to be 32 bytes
} mergeaccum_t;


#endif //__GRAZELLE_GRAPHTYPES_H
