/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* phases.h
*      Declaration of functions that implement execution phases and related
*      operations.
*****************************************************************************/

#ifndef __GRAZELLE_PHASES_H
#define __GRAZELLE_PHASES_H


#include "graphtypes.h"
#include "intrinhelper.h"

#include <stdint.h>


/* -------- COMMON PHASE OPERATORS ----------------------------------------- */

// Combines partial values of a global variable, using summation, from the specified reduce buffer.
// This version treats the contents of the reduce buffer as 64-bit unsigned integers.
// Returns the fully-combined result.
uint64_t phase_op_combine_global_var_from_buf(uint64_t* reduce_buffer);

// Combines partial values of a global variable, using summation, from the specified reduce buffer.
// This version treats the contents of the reduce buffer as double-precision floating-point values.
// Returns the fully-combined result.
double phase_op_combine_global_var_from_buf_double(double* reduce_buffer);



/* -------- EDGE-PULL ENGINE OPERATORS ------------------------------------- */

// Performs a merge to the accumulators based on all the entries in the merge buffer.
// This version is simple, does not use the initial vertex ID record, and should be used with an idempotent merge operator.
// Should only be called by a single thread on each NUMA node, but it does not matter which thread.
// The "count" parameter refers to the "merge_buffer" array; "vertex_accumulators" is indexed based on the contents of "merge_buffer" and is assumed to have enough entries.
// This function is written in C.
void edge_pull_op_idempotent_merge_with_merge_buffer(mergeaccum_t* merge_buffer, uint64_t count, vertexprop_t* vertex_accumulators, vertexprop_t (*scalar_reduce_op)(vertexprop_t, vertexprop_t));

// Performs a merge to the accumulators based on all the entries in the merge buffer.
// This version is more complicated and should be used when initial vertex IDs are tracked due to a non-idempotent merge operator.
// Should only be called by a single thread on each NUMA node, but it does not matter which thread.
// The "count" parameter refers to the "merge_buffer" array; "vertex_accumulators" is indexed based on the contents of "merge_buffer" and is assumed to have enough entries.
// This function is written in C.
void edge_pull_op_merge_with_merge_buffer(mergeaccum_t* merge_buffer, uint64_t count, vertexprop_t* vertex_accumulators, vertexprop_t(*scalar_reduce_op)(vertexprop_t, vertexprop_t));


/* -------- PHASE CONTROL FUNCTIONS ---------------------------------------- */

// Performs the Edge-Pull phase for BFS.
// Specify the address and size (measured in 256-bit elements) of the edge vector list.
void perform_edge_pull_phase_bfs(const __m256i* const edge_list, const uint64_t edge_list_count);

// Performs the Edge-Pull phase for BFS while making use of the Wedge frontier.
// Specify the address and size (measured in 256-bit elements) of the edge vector list.
void perform_edge_pull_phase_bfs_with_wedge_frontier(const __m256i* const edge_list, const uint64_t edge_list_count, uint64_t* const wedge_frontier, const uint64_t wedge_frontier_count);

// Performs the Edge-Pull phase for CC.
// Specify the address and size (measured in 256-bit elements) of the edge vector list.
void perform_edge_pull_phase_cc(const __m256i* const edge_list, const uint64_t edge_list_count);

// Performs the Edge-Pull phase for CC while making use of the Wedge frontier.
// Specify the address and size (measured in 256-bit elements) of the edge vector list.
void perform_edge_pull_phase_cc_with_wedge_frontier(const __m256i* const edge_list, const uint64_t edge_list_count, uint64_t* const wedge_frontier, const uint64_t wedge_frontier_count);

// Performs the Edge-Pull phase for PR.
// Specify the address and size (measured in 256-bit elements) of the edge vector list.
void perform_edge_pull_phase_pr(const __m256i* const edge_list, const uint64_t edge_list_count);

// Performs the Edge-Pull phase for SSSP.
// Specify the address and size (measured in 256-bit elements) of the edge vector list.
void perform_edge_pull_phase_sssp(const __m256i* const edge_list, const uint64_t edge_list_count);

// Performs the Edge-Pull phase for SSSP while making use of the Wedge frontier.
// Specify the address and size (measured in 256-bit elements) of the edge vector list.
void perform_edge_pull_phase_sssp_with_wedge_frontier(const __m256i* const edge_list, const uint64_t edge_list_count, uint64_t* const wedge_frontier, const uint64_t wedge_frontier_count);

// Performs the Vertex phase for PR.
// Specify the range of vertices to process (as a starting vertex ID and count).
void perform_vertex_phase_pr(const uint64_t vertex_start, const uint64_t vertex_count);


#endif //__GRAZELLE_PHASES_H
