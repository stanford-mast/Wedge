/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* execution_cc.c
*      Implementation of the algorithm control flow for Connected Components.
*****************************************************************************/

#include "benchmark.h"
#include "configuration.h"
#include "execution.h"
#include "graphdata.h"
#include "graphtypes.h"
#include "hwexperiments.h"
#include "intrinhelper.h"
#include "phases.h"
#include "wedge.h"

#include <parutil.h>
#include <spindle.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>


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

void execution_get_vertex_prop_string_cc(const uint64_t vertex, char* const string, const size_t size)
{
    const double value = graph_vertex_props[vertex].d;
    snprintf(string, size, "%.0lf", value);
}

// ---------

double execution_get_vertex_prop_val_cc(const uint64_t vertex)
{
    return graph_vertex_props[vertex].d;
}

// ---------

vertexprop_t execution_scalar_reduce_op_cc(vertexprop_t a, vertexprop_t b)
{
    return (a.d < b.d ? a : b);
}

// ---------

void execution_initialize_cc(void)
{
    // initialize vertex properties
    SParutilStaticSchedule vertexInitSchedule;
    parutilSchedulerStatic(ParutilStaticSchedulerChunked, graph_vertex_count_numa[spindleGetTaskID()], &vertexInitSchedule);
    
    for (uint64_t vertex = (graph_vertex_first_numa[spindleGetTaskID()] + vertexInitSchedule.startUnit); vertex < (graph_vertex_first_numa[spindleGetTaskID()] + vertexInitSchedule.endUnit); ++vertex)
    {
        graph_vertex_props[vertex].d = (double)vertex;
    }
}

// ---------

void execution_impl_cc(void)
{
#ifdef EXPERIMENT_MEASURE_LOAD_BALANCE
    double pull_wait_time = 0.0;
    double wedge_wait_time = 0.0;
#endif
    
    double iteration_processing_time = 0.0;
    uint64_t ctr = 0ull;
    uint64_t ctr_wedge = 0ull;
    
    const uint64_t active_edges_threshold = (graph_num_edges * graph_param_frontier_threshold_pct) / 100ull;
    uint64_t num_edges_active = graph_num_edges;
    
    total_processing_time_nowedge = 0.0;
    total_processing_time_wedge = 0.0;
    total_activation_time = 0.0;
    
    while(1)
    {
        const bool use_nowedge_pull = ((num_edges_active > active_edges_threshold) || (0ull == ctr));
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
            perform_edge_pull_phase_cc(graph_edges_gather_list_bufs_numa[spindleGetTaskID()], graph_edges_gather_list_counts_numa[spindleGetTaskID()]);
        }
        else
        {
            ctr_wedge += 1ull;
            perform_edge_pull_phase_cc_with_wedge_frontier(graph_edges_gather_list_bufs_numa[spindleGetTaskID()], graph_edges_gather_list_counts_numa[spindleGetTaskID()], graph_wedge_frontier_numa[spindleGetTaskID()], graph_wedge_frontier_count_numa[spindleGetTaskID()]);
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
    
        // first thread performs the actual merge operation between potentially-overlapping properties
        if (0 == spindleGetGlobalThreadID())
        {
            edge_pull_op_idempotent_merge_with_merge_buffer(graph_vertex_merge_buffer, (uint64_t)spindleGetGlobalThreadCount() * 32ull, graph_vertex_props, execution_scalar_reduce_op_cc);
        }
        
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
        // otherwise, reset the bit-mask of activated vertices
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
            
            if (0 == spindleGetTaskID())
            {
                SParutilStaticSchedule schedule;
                parutilSchedulerStatic(ParutilStaticSchedulerChunked, graph_frontier_count >> 2ull, &schedule);
                
                for (uint64_t i = schedule.startUnit; i < schedule.endUnit; ++i)
                {
                    const uint64_t frontier_idx = (i << 2ull);
                    
                    if (0 == _mm256_testc_si256(_mm256_setzero_si256(), *((__m256i*)(&graph_frontier_active_vertices[frontier_idx]))))
                        _mm256_store_si256((__m256i*)(&graph_frontier_active_vertices[frontier_idx]), _mm256_setzero_si256());
                }
            }
        }
        else
        {
            parutilMemorySet((void*)graph_frontier_active_vertices, 0, graph_frontier_count << 3ull);
        }
        
        spindleBarrierGlobal();
            
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
        
        // nothing to do here
        spindleBarrierGlobal();
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
