/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* constants.h
*      Declarations of commonly-used constants.
*****************************************************************************/

#ifndef __GRAZELLE_CONSTANTS_H
#define __GRAZELLE_CONSTANTS_H


#include "intrinhelper.h"

#include <math.h>
#include <stdint.h>


/* -------- CONSTANTS ------------------------------------------------------ */

// an aligned, 256-bit constant that contains the value +INFINITY four times, to be loaded into an AVX register
#define const_infinity (_mm256_set1_pd((double)INFINITY))

// an aligned, 256-bit constant representing the mask for bitwise-AND to obtain the spread-encoding vertex identifier
#define const_vid_and_mask (_mm256_set_epi64x(0x0007000000000000, 0x7fff000000000000, 0x7fff000000000000, 0x7fff000000000000))

// an aligned, 256-bit constant representing the mask for bitwise-AND to obtain the neighbor list, either sources or destinations, from an edge vector
#define const_edge_list_and_mask (_mm256_set1_epi64x(0x0000ffffffffffff))

// an aligned, 256-bit constant representing the mask for bitwise-AND to obtain the edge validity mask from an edge vector
#define const_edge_mask_and_mask (_mm256_set1_epi64x(0x8000000000000000))

// an aligned, 256-bit constant holding the value of the damping factor for use with PageRank
#define const_damping_factor (_mm256_set1_pd(0.875))


#endif //__GRAZELLE_CONSTANTS_H
