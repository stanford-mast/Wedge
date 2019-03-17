/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* execution_pr.c
*      Implementation of the algorithm control flow for PageRank.
*****************************************************************************/

#include "benchmark.h"
#include "execution.h"
#include "graphdata.h"
#include "hwexperiments.h"
#include "phases.h"

#include <parutil.h>
#include <spindle.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "execution.h" for documentation.

void execution_get_vertex_prop_string_pr(const uint64_t vertex, char* const string, const size_t size)
{
    const double value = graph_vertex_props[vertex].d * (0.0 == graph_vertex_outdegrees[vertex].d ? (double)graph_num_vertices : graph_vertex_outdegrees[vertex].d);
    snprintf(string, size, "%.4le", value);
}

// ---------

double execution_get_vertex_prop_val_pr(const uint64_t vertex)
{
    return graph_vertex_props[vertex].d * (0.0 == graph_vertex_outdegrees[vertex].d ? (double)graph_num_vertices : graph_vertex_outdegrees[vertex].d);
}

// ---------

vertexprop_t execution_scalar_reduce_op_pr(vertexprop_t a, vertexprop_t b)
{
    vertexprop_t result;
    result.d = a.d + b.d;
    
    return result;
}

// ---------

const char* execution_special_stat_name_pr(void)
{
    return "PageRank Sum";
}

// ---------

void execution_special_stat_string_pr(char* const string, const size_t size)
{
    double rank_sum = 0.0;
    
    for (uint64_t i = 0ull; i < graph_num_vertices; ++i)
        rank_sum += graph_vertex_props[i].d * (0.0 == graph_vertex_outdegrees[i].d ? (double)graph_num_vertices : graph_vertex_outdegrees[i].d);
    
    snprintf(string, size, "%.8lf", rank_sum);
}

void execution_initialize_pr(void)
{
    // initialize vertex properties and accumulators
    SParutilStaticSchedule vertexInitSchedule;
    parutilSchedulerStatic(ParutilStaticSchedulerChunked, graph_vertex_count_numa[spindleGetTaskID()], &vertexInitSchedule);
    
    for (uint64_t vertex = (graph_vertex_first_numa[spindleGetTaskID()] + vertexInitSchedule.startUnit); vertex < (graph_vertex_first_numa[spindleGetTaskID()] + vertexInitSchedule.endUnit); ++vertex)
    {
        graph_vertex_outdegrees[vertex].d = (double)graph_vertex_outdegrees[vertex].u;
        graph_vertex_accumulators[vertex].d = 0.0;
        graph_vertex_props[vertex].d = (1.0 / (double)graph_num_vertices) / (0.0 == graph_vertex_outdegrees[vertex].d ? (double)graph_num_vertices : graph_vertex_outdegrees[vertex].d);
    }
}

// ---------

void execution_impl_pr(void)
{
#ifdef EXPERIMENT_MEASURE_LOAD_BALANCE
    double pull_wait_time = 0.0;
    double wedge_wait_time = 0.0;
#endif
    double iteration_processing_time = 0.0;
    uint64_t ctr = 0ull;
    uint64_t ctr_wedge = 0ull;
    
    total_processing_time_nowedge = 0.0;
    total_processing_time_wedge = 0.0;
    total_activation_time = 0.0;
    
    for (ctr = 0; ctr < graph_param_num_static_iterations; ++ctr)
    {
        if (0 == spindleGetGlobalThreadID())
            benchmark_start(&iteration_processing_time);
        
        
        /* Edge Phase */
        
#ifdef EXPERIMENT_HW_MEMBW_PULL
        if (0 == spindleGetGlobalThreadID())
            hwexperiments_measure_start();
        
        spindleBarrierGlobal();
#endif
        
        // perform the Edge-Pull phase
        perform_edge_pull_phase_pr(graph_edges_gather_list_bufs_numa[spindleGetTaskID()], graph_edges_gather_list_counts_numa[spindleGetTaskID()]);
        spindleBarrierGlobal();
        
        // first thread performs the actual merge operation between potentially-overlapping accumulators
        if (0 == spindleGetGlobalThreadID())
        {
            edge_pull_op_merge_with_merge_buffer(graph_vertex_merge_buffer, (uint64_t)spindleGetGlobalThreadCount() * 32ull, graph_vertex_accumulators, execution_scalar_reduce_op_pr);
        }
        
#ifdef EXPERIMENT_HW_MEMBW_PULL
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
        
        
        /* Vertex Phase */
        
        // perform the Vertex phase
        perform_vertex_phase_pr(graph_vertex_first_numa[spindleGetTaskID()], graph_vertex_count_numa[spindleGetTaskID()]);
        spindleBarrierGlobal();
        
        
        if (0 == spindleGetGlobalThreadID())
            total_processing_time_nowedge += benchmark_stop(&iteration_processing_time);
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
