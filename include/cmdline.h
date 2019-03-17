/*****************************************************************************
* Wedge
*      High performance, hardware-optimized graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* cmdline.h
*      Declaration of types and functions for parsing command-line arguments.
*****************************************************************************/

#ifndef __GRAZELLE_CMDLINE_H
#define __GRAZELLE_CMDLINE_H


#include "configuration.h"

#include <stdint.h>


/* -------- CONSTANTS ------------------------------------------------------ */

// Default values for the command-line settings; any option not specified here has no default
#define CMDLINE_DEFAULT_NUM_THREADS             0
#define CMDLINE_DEFAULT_NUM_NUMA_NODES          1
#define CMDLINE_DEFAULT_NUM_ITERATIONS          1
#define CMDLINE_DEFAULT_ROOT_VERTEX             0
#define CMDLINE_DEFAULT_FRONTIER_THRESHOLD_PCT  20


/* -------- TYPE DEFINITIONS ----------------------------------------------- */

// Contains the values for each possible command-line option.
typedef struct cmdline_opts_t
{
    char graph_input_filename[1024];                        // 'i' -> required; filename of the graph input file
    char* graph_application_output_filename;                // 'o' -> optional; filename of the output file that should contain final values for each vertex
    
    char application[64];                                   // 'a' -> required; application to run

    uint32_t num_iterations;                                // 'N' -> optional; number of iterations of the algorithm to execute
    
    uint32_t num_threads;                                   // 'n' -> optional; number of worker threads to use while executing
	uint32_t num_numa_nodes;								// 'u' -> optional; number of NUMA nodes to use, inferred from the list
    uint32_t numa_nodes[CONFIG_NUMA_MAX_NODES+1];           // 'u' -> optional; list of NUMA nodes to use
    
    uint64_t root_vertex;                                   // 'r' -> optional; root vertex for searches and traversals
    
    uint64_t frontier_threshold_pct;                        // 't' -> optional; frontier fullness threshold for using or not using Wedge, expressed as a percentage
} cmdline_opts_t;


/* -------- FUNCTIONS ------------------------------------------------------ */

// Accepts and parses an argc and argv[] combination, passed from main().
// Fills the command-line options structure, validates it, and returns on success.
// If there is a problem, prints an appropriate message and terminates the program.
void cmdline_parse_options_or_die(int argc, char* argv[]);

// Retrieves, by reference, the current settings that are in effect.
const cmdline_opts_t* const cmdline_get_current_settings();


#endif //__GRAZELLE_CMDLINE_H
