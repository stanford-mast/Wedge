/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* execution.c
*      Implementation of the top-level functions executed by this program.
*      Defines common variables used across algorithms.
*****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


/* -------- GLOBALS -------------------------------------------------------- */
// See "execution.h" for documentation.

uint64_t total_iterations_executed = 0ull;
uint64_t total_wedge_iterations_executed = 0ull;
double total_processing_time_nowedge = 0.0;
double total_processing_time_wedge = 0.0;
double total_activation_time = 0.0;
bool execution_has_special_output_stat = false;
bool execution_needs_vertex_accumulator = false;
bool execution_needs_wedge_frontier = false;
bool execution_needs_traditional_frontiers = false;
bool execution_uses_edge_weights = false;
void (*execution_get_vertex_prop_string)(const uint64_t, char* const, const size_t) = NULL;
double (*execution_get_vertex_prop_val)(const uint64_t) = NULL;
void(*execution_initialize)(void) = NULL;
void(*execution_impl)(void*) = NULL;
const char*(*execution_special_stat_name)(void) = NULL;
void (*execution_special_stat_string)(char* const, const size_t) = NULL;
const char* execution_app_name = NULL;
