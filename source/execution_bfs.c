/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* execution_bfs.c
*      Implementation of the algorithm control flow for Breadth-First Search.
*****************************************************************************/

#include "benchmark.h"
#include "configuration.h"
#include "execution.h"
#include "graphdata.h"
#include "hwexperiments.h"
#include "phases.h"
#include "wedge.h"

#include <parutil.h>
#include <silo.h>
#include <spindle.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>


static inline void set_bit(void* const bitmask, const uint64_t bitindex)
{
    uint8_t* const addr = &(((uint8_t*)bitmask)[bitindex >> 3ull]);
    const uint8_t mask = (1 << ((uint8_t)bitindex & 7));
    
    if (0 == (*addr & mask))
    {
        __asm__("lock or BYTE PTR [%1], %2" : "=m"(*addr) : "r"(addr), "r"(mask));
    }
}

static inline void clear_bit(void* const bitmask, const uint64_t bitindex)
{
    uint8_t* const addr = &(((uint8_t*)bitmask)[bitindex >> 3ull]);
    const uint8_t mask = ~(1 << ((uint8_t)bitindex & 7));
    
    if (255 == (*addr | mask))
    {
        __asm__("lock and BYTE PTR [%1], %2" : "=m"(*addr) : "r"(addr), "r"(mask));
    }
}


#ifdef EXPERIMENT_ITERATION_STATS
/* -------- LOCALS --------------------------------------------------------- */

// Per-iteration statistics captured as part of experiments.
static double stat_pull_time = 0.0;
static double stat_wedge_time = 0.0;
static bool stat_used_wedge = false;
static uint64_t stat_num_edges_active;

#endif


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "execution.h" for documentation.

void execution_get_vertex_prop_string_bfs(const uint64_t vertex, char* const string, const size_t size)
{
    const long long int value = (long long int)graph_vertex_props[vertex].u;
    snprintf(string, size, "%lld", value);
}

// ---------

double execution_get_vertex_prop_val_bfs(const uint64_t vertex)
{
    return (double)graph_vertex_props[vertex].u;
}

// ---------

void execution_initialize_bfs(void)
{
    // initialize vertex properties and mark any vertices having in-degree 0 as having already converged
    SParutilStaticSchedule vertexInitSchedule;
    parutilSchedulerStatic(ParutilStaticSchedulerChunked, graph_vertex_count_numa[spindleGetTaskID()], &vertexInitSchedule);
    
    for (uint64_t vertex = (graph_vertex_first_numa[spindleGetTaskID()] + vertexInitSchedule.startUnit); vertex < (graph_vertex_first_numa[spindleGetTaskID()] + vertexInitSchedule.endUnit); ++vertex)
    {
        uint64_t vertex_index_min = UINT64_MAX;
        
        for (uint32_t i = 0; i < spindleGetTaskCount(); ++i)
        {
            if (graph_edges_gather_list_index_numa[i][vertex] < vertex_index_min)
            {
                vertex_index_min = 0ull;
                break;
            }
        }
        
        if (UINT64_MAX == vertex_index_min)
            clear_bit(graph_frontier_unconverged_vertices, vertex);
        
        graph_vertex_props[vertex].u = UINT64_MAX;
    }
    
    // activate and visit the search root
    if (0 == spindleGetGlobalThreadID())
    {
        set_bit(graph_frontier_active_vertices_snapshot, graph_param_root_vertex);
        clear_bit(graph_frontier_unconverged_vertices, graph_param_root_vertex);
    }
    
    if (0 == spindleGetLocalThreadID())
        wedge_activate_single_vertex(graph_param_root_vertex);
}

// ---------

void execution_impl_bfs(void)
{
#ifdef EXPERIMENT_MEASURE_LOAD_BALANCE
    double pull_wait_time = 0.0;
    double wedge_wait_time = 0.0;
#endif
    double iteration_processing_time = 0.0;
    uint64_t ctr = 0ull;
    uint64_t ctr_wedge = 0ull;
    
    const uint64_t active_edges_threshold = (graph_num_edges * graph_param_frontier_threshold_pct) / 100ull;
    uint64_t num_edges_active = graph_vertex_outdegrees[graph_param_root_vertex].u;
    
    total_processing_time_nowedge = 0.0;
    total_processing_time_wedge = 0.0;
    total_activation_time = 0.0;
    
    while(1)
    {
        const bool use_nowedge_pull = (num_edges_active > active_edges_threshold);
        ctr += 1ull;
        
#ifdef EXPERIMENT_ITERATION_STATS
        if (0 == spindleGetGlobalThreadID())
        {
            stat_used_wedge = (use_nowedge_pull ? false : true);
            stat_num_edges_active = num_edges_active;
        }
        
        spindleBarrierGlobal();
#endif
        
        if (0 == spindleGetGlobalThreadID())
            benchmark_start(&iteration_processing_time);
        
        
        /* Edge Phase */
        
#ifdef EXPERIMENT_HW_MEMBW_PULL
        if (0 == spindleGetGlobalThreadID())
            hwexperiments_measure_start();
        
        spindleBarrierGlobal();
#endif
        
        // perform the Edge-Pull phase
        if (use_nowedge_pull)
        {
            perform_edge_pull_phase_bfs(graph_edges_gather_list_bufs_numa[spindleGetTaskID()], graph_edges_gather_list_counts_numa[spindleGetTaskID()]);
        }
        else
        {
            ctr_wedge += 1ull;
            perform_edge_pull_phase_bfs_with_wedge_frontier(graph_edges_gather_list_bufs_numa[spindleGetTaskID()], graph_edges_gather_list_counts_numa[spindleGetTaskID()], graph_wedge_frontier_numa[spindleGetTaskID()], graph_wedge_frontier_count_numa[spindleGetTaskID()]);
        }
        
#ifdef EXPERIMENT_HW_MEMBW_PULL
        spindleBarrierGlobal();
        
        if (0 == spindleGetGlobalThreadID())
        {
            hwexperiments_measure_stop();
            
            fprintf(stderr, "%lf,%llu,%llu", hwexperiments_get_measurement_time(), (long long unsigned int)hwexperiments_get_socket_mb_read(0), (long long unsigned int)hwexperiments_get_socket_mb_write(0));
            
            for (uint32_t i = 1; i < hwexperiments_get_num_sockets(); ++i)
                fprintf(stderr, ",%llu,%llu", (long long unsigned int)hwexperiments_get_socket_mb_read(i), (long long unsigned int)hwexperiments_get_socket_mb_write(i));
            
#ifdef EXPERIMENT_HW_MEMBW_WEDGE
            fprintf(stderr, ",");
#else
            fprintf(stderr, "\n");
#endif
        }
#endif
        
#ifdef EXPERIMENT_MEASURE_LOAD_BALANCE
        {
            double wait_time = 0.0;
            benchmark_start(&wait_time);
            
            spindleBarrierGlobal();
            
            pull_wait_time += benchmark_stop(&wait_time);
        }
#else
        spindleBarrierGlobal();
#endif
        
        
        if (0 == spindleGetGlobalThreadID())
        {
            iteration_processing_time = benchmark_stop(&iteration_processing_time);
            
#ifdef EXPERIMENT_ITERATION_STATS
            stat_pull_time += iteration_processing_time;
#endif
            
            if (use_nowedge_pull)
                total_processing_time_nowedge += iteration_processing_time;
            else
                total_processing_time_wedge += iteration_processing_time;
        }
        
        
#ifdef EXPERIMENT_ITERATION_STATS
        if (0 == spindleGetGlobalThreadID())
        {
            fprintf(stderr, "%llu,%s,%lf,%lf\n", (long long unsigned int)stat_num_edges_active, stat_used_wedge ? "true" : "false", stat_pull_time, stat_wedge_time);
        }
        
        spindleBarrierGlobal();
#endif
        
        
        /* Termination Check */
        
        // figure out how many edges were activated this past iteration
        num_edges_active = phase_op_combine_global_var_from_buf(graph_reduce_buffer);
        
        // if no edges were activated, the algorithm is complete
        if (0 == num_edges_active)
        {
#if defined(EXPERIMENT_HW_MEMBW_PULL) && defined(EXPERIMENT_HW_MEMBW_WEDGE)
            if (0 == spindleGetGlobalThreadID())
                fprintf(stderr, "\n");
#endif
            
            break;
        }
        
        
        /* Wedge Phase */
        
        // if needed, produce the Wedge frontier
        spindleBarrierGlobal();
        
#ifdef EXPERIMENT_HW_MEMBW_WEDGE
        if (0 == spindleGetGlobalThreadID())
        {
            fprintf(stderr, "%s,", (num_edges_active <= active_edges_threshold) ? "true" : "false");
            hwexperiments_measure_start();
        }
        
        spindleBarrierGlobal();
#endif
        
        if (0 == spindleGetGlobalThreadID())
            benchmark_start(&iteration_processing_time);
        
        if (num_edges_active <= active_edges_threshold)
        {    
            wedge_generate_frontier();
        }
        
#ifdef EXPERIMENT_MEASURE_LOAD_BALANCE
        {
            double wait_time = 0.0;
            benchmark_start(&wait_time);
                
            spindleBarrierGlobal();
            
            wedge_wait_time += benchmark_stop(&wait_time);
        }
#else
        spindleBarrierGlobal();
#endif
            
        if (0 == spindleGetGlobalThreadID())
        {
            iteration_processing_time = benchmark_stop(&iteration_processing_time);
            
#ifdef EXPERIMENT_ITERATION_STATS
            stat_wedge_time = iteration_processing_time;
#endif
            
            total_activation_time += iteration_processing_time;
        }
        
#ifdef EXPERIMENT_HW_MEMBW_WEDGE
        spindleBarrierGlobal();
        
        if (0 == spindleGetGlobalThreadID())
        {
            hwexperiments_measure_stop();
            
            fprintf(stderr, "%lf,%llu,%llu", hwexperiments_get_measurement_time(), (long long unsigned int)hwexperiments_get_socket_mb_read(0), (long long unsigned int)hwexperiments_get_socket_mb_write(0));
            
            for (uint32_t i = 1; i < hwexperiments_get_num_sockets(); ++i)
                fprintf(stderr, ",%llu,%llu", (long long unsigned int)hwexperiments_get_socket_mb_read(i), (long long unsigned int)hwexperiments_get_socket_mb_write(i));
            
            fprintf(stderr, "\n");
        }
#endif
        
        
        /* Vertex Phase */
        
        // one thread swaps current and next active vertex frontiers, and then all threads collaboratively clear for next iteration
        spindleBarrierGlobal();
        
        if (0 == spindleGetGlobalThreadID())
            benchmark_start(&iteration_processing_time);
        
        if (0 == spindleGetGlobalThreadID())
        {
            uint64_t* const temp = graph_frontier_active_vertices;
            graph_frontier_active_vertices = graph_frontier_active_vertices_snapshot;
            graph_frontier_active_vertices_snapshot = temp;
        }
        
        spindleBarrierGlobal();
        
        if (0ull == spindleGetTaskID())
            parutilMemorySet((void*)graph_frontier_active_vertices, 0, graph_frontier_count << 3ull);
        
        spindleBarrierGlobal();
        
        if (0 == spindleGetGlobalThreadID())
        {
            iteration_processing_time = benchmark_stop(&iteration_processing_time);
            
#ifdef EXPERIMENT_ITERATION_STATS
            stat_pull_time = iteration_processing_time;
#endif
            
            if (use_nowedge_pull)
                total_processing_time_nowedge += iteration_processing_time;
            else
                total_processing_time_wedge += iteration_processing_time;
        }
    }
    
    // algorithm complete, record the number of iterations
    if (0 == spindleGetGlobalThreadID())
    {
        total_iterations_executed = ctr;
        total_wedge_iterations_executed = ctr_wedge;
    }
    
#ifdef EXPERIMENT_MEASURE_LOAD_BALANCE
    fprintf(stderr, "%lf,%lf\n", pull_wait_time, wedge_wait_time);
#endif
}
