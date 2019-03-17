/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* graphdata.h
*      Declaration of operations used to represent a graph in memory,
*      including reading it from a properly-formatted file and exporting it
*      back.
*****************************************************************************/

#ifndef __GRAZELLE_GRAPHDATA_H
#define __GRAZELLE_GRAPHDATA_H


#include "graphtypes.h"
#include "intrinhelper.h"
#include "versioninfo.h"

#include <stdbool.h>
#include <stdint.h>


/* -------- DATA STRUCTURES ------------------------------------------------ */

// Number of vertices in the graph
extern uint64_t graph_num_vertices;

// Number of edges in the graph
extern uint64_t graph_num_edges;

// Number of edge vectors in the graph
extern uint64_t graph_num_edge_vectors;

// Number of iterations of statically-convergent algorithms to run.
extern uint64_t graph_param_num_static_iterations;

// Start vertex for search/traversal algorithms.
extern uint64_t graph_param_root_vertex;

// Frontier threshold for determining whether or not to run Wedge.
extern uint64_t graph_param_frontier_threshold_pct;

// Collection of vertex ranks
extern vertexprop_t* graph_vertex_props;

// Collection of vertex accumulators, used between the Edge and Vertex phases
extern vertexprop_t* graph_vertex_accumulators;

// Collection of vertex outdegrees
extern vertexprop_t* graph_vertex_outdegrees;

// Vertex merge buffers
extern mergeaccum_t* graph_vertex_merge_buffer;

// Per-NUMA-node base pointers into the merge buffers
extern mergeaccum_t** const graph_vertex_merge_buffer_baseptr_numa;

// Edge gather list buffer pointers, NUMA-aware
extern __m256i** const graph_edges_gather_list_bufs_numa;

// Edge gather list index, NUMA-aware
extern uint64_t** const graph_edges_gather_list_index_numa;

// Vector count for the edge gather list, NUMA-aware
extern uint64_t* const graph_edges_gather_list_counts_numa;

// First destination vertex assignments for each NUMA node
extern uint64_t* const graph_vertex_first_numa;

// Last destination vertex assignments for each NUMA node
extern uint64_t* const graph_vertex_last_numa;

// Number of vertex assignments for each NUMA node
extern uint64_t* const graph_vertex_count_numa;

// Dynamic scheduler counter pointers, one per NUMA node. Used to implement per-node local dynamic scheduling during the Processing phase.
extern uint64_t** const graph_scheduler_dynamic_counter_numa;

// Wedge frontier data structure, one bit per edge element, one per NUMA node.
extern uint64_t** graph_wedge_frontier_numa;

// Number of 64-bit elements in the Wedge frontier data structure, one per NUMA node.
extern uint64_t* graph_wedge_frontier_count_numa;

// Wedge CSRI index, one per NUMA node.
extern uint64_t** graph_edges_wedge_csri_index_numa;

// Wedge CSRI edge array, one per NUMA node.
extern uint64_t** graph_edges_wedge_csri_edges_numa;

// Traditional frontier data structure for active vertices, one bit per vertex.
extern uint64_t* graph_frontier_active_vertices;

// Traditional frontier data structure for active vertices, one bit per vertex.
// This is intended to be swapped each iteration with the non-snapshot version each iteration, so one is consumed as the other is produced.
extern uint64_t* graph_frontier_active_vertices_snapshot;

// Traditional frontier data structure for unconverged vertices, one bit per vertex.
extern uint64_t* graph_frontier_unconverged_vertices;

// Number of 64-bit elements in the traditional frontier data structures.
extern uint64_t graph_frontier_count;

// Reduce buffer, used for inter-thread communication.
extern uint64_t* graph_reduce_buffer;


/* -------- FUNCTIONS ------------------------------------------------------ */

// Allocates and initializes all graph data structures needed to support the selected application.
// To be invoked once a graph has been read successfully from a file.
void graph_data_initialize_data_structures(const uint32_t num_threads, const uint32_t num_numa_nodes, const uint32_t* const numa_nodes);

// Reads graph data from a properly-formatted file and fills in the graph data structures.
// A file name and NUMA node information are all required.
// Will print status messages to the standard output console.
// To be invoked after an application has been set.
bool graph_data_read_from_file(const char* const filename, const uint32_t num_numa_nodes, const uint32_t* const numa_nodes);


#endif //__GRAZELLE_GRAPHDATA_H
