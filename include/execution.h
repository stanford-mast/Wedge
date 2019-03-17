/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* execution.h
*      Declaration of the top-level functions executed by this program.
*****************************************************************************/

#ifndef __GRAZELLE_EXECUTION_H
#define __GRAZELLE_EXECUTION_H


#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


/* -------- GLOBALS -------------------------------------------------------- */

// Iteration statistics.
extern uint64_t total_iterations_executed;
extern uint64_t total_wedge_iterations_executed;

// Processing time statistics.
extern double total_processing_time_nowedge;
extern double total_processing_time_wedge;

// Activation time statistics.
extern double total_activation_time;

// Specifies if the algorithm produces a special execution statistic that should be output.
extern bool execution_has_special_output_stat;

// Specifies if the algorithm being executed needs a vertex accumulator.
extern bool execution_needs_vertex_accumulator;

// Specifies if the algorithm being executed uses a Wedge frontier data structure.
extern bool execution_needs_wedge_frontier;

// Specifies if the algorithm being executed uses traditional frontier data structures.
extern bool execution_needs_traditional_frontiers;

// Specifies if the algorithm uses edge weights.
extern bool execution_uses_edge_weights;

// Retrieves the final vertex property value in string form for the specified vertex.
// To be invoked only after the application has converged.
extern void (*execution_get_vertex_prop_string)(const uint64_t, char* const, const size_t);

// Retrieves the final vertex property value for the specified vertex.
// To be invoked only after the application has converged.
extern double (*execution_get_vertex_prop_val)(const uint64_t);

// Implements the Wedge initialization pass. Invoked immediately before running the application.
extern void(*execution_initialize)(void);

// Implements the algorithm that is to be run. This function acts as a driver and sequences other operations by calling their respective functions.
extern void (*execution_impl)(void);

// Provides the name of the special output statistic that should be displayed upon application completion.
extern const char*(*execution_special_stat_name)(void);

// Computes the value of the special output statistic that should be displays and fills the specified string.
extern void (*execution_special_stat_string)(char* const, const size_t);

// String name of the selected application.
extern const char* execution_app_name;


/* -------- FUNCTIONS ------------------------------------------------------ */

// Retrieves the final vertex property value in string form for the specified vertex, for Breadth-First Search.
void execution_get_vertex_prop_string_bfs(const uint64_t vertex, char* const string, const size_t size);

// Retrieves the final vertex property value in string form for the specified vertex, for Connected Components.
void execution_get_vertex_prop_string_cc(const uint64_t vertex, char* const string, const size_t size);

// Retrieves the final vertex property value in string form for the specified vertex, for PageRank.
void execution_get_vertex_prop_string_pr(const uint64_t vertex, char* const string, const size_t size);

// Retrieves the final vertex property value in string form for the specified vertex, for Single-Source Shortest Path.
void execution_get_vertex_prop_string_sssp(const uint64_t vertex, char* const string, const size_t size);

// Retrieves the final vertex property value for the specified vertex, for Breadth-First Search.
double execution_get_vertex_prop_val_bfs(const uint64_t vertex);

// Retrieves the final vertex property value for the specified vertex, for Connected Components.
double execution_get_vertex_prop_val_cc(const uint64_t vertex);

// Retrieves the final vertex property value for the specified vertex, for PageRank.
double execution_get_vertex_prop_val_pr(const uint64_t vertex);

// Retrieves the final vertex property value for the specified vertex, for Single-Source Shortest Path.
double execution_get_vertex_prop_val_sssp(const uint64_t vertex);

// Retrieves the name of the PageRank Sum statistic that PageRank produces.
const char* execution_special_stat_name_pr(void);

// Retrieves a string representation of the PageRank sum that PageRank produces.
void execution_special_stat_string_pr(char* const string, const size_t size);

// Implements the Wedge initialization pass, for Breadth-First Search.
void execution_initialize_bfs(void);

// Implements the Wedge initialization pass, for Connected Components.
void execution_initialize_cc(void);

// Implements the Wedge initialization pass, for PageRank.
void execution_initialize_pr(void);

// Implements the Wedge initialization pass, for Single-Source Shortest Path.
void execution_initialize_sssp(void);

// Implements the Breadth-First Search algorithm. This function acts as a driver and sequences other operations by calling their respective functions.
void execution_impl_bfs(void);

// Implements the Connected Components algorithm. This function acts as a driver and sequences other operations by calling their respective functions.
void execution_impl_cc(void);

// Implements the PageRank algorithm. This function acts as a driver and sequences other operations by calling their respective functions.
void execution_impl_pr(void);

// Implements the Single-Source Shortest Path algorithm. This function acts as a driver and sequences other operations by calling their respective functions.
void execution_impl_sssp(void);


#endif //__GRAZELLE_EXECUTION_H
