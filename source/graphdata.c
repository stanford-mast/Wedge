/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* graphdata.c
*      Implementation of operations used to represent a graph in memory,
*      including reading it from a properly-formatted file and exporting it
*      back.
*****************************************************************************/

#include "benchmark.h"
#include "configuration.h"
#include "execution.h"
#include "graphtypes.h"
#include "intrinhelper.h"
#include "phases.h"

#include <math.h>
#include <parutil.h>
#include <silo.h>
#include <spindle.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


static inline void atomic_add(uint64_t* const ptr, const uint64_t val)
{
    __asm__("lock add QWORD PTR [%1], %2" : "=m"(*ptr) : "r"(ptr), "r"(val));
}

static inline uint64_t atomic_exchange_add(uint64_t* const ptr, const uint64_t val)
{
    return __sync_fetch_and_add(ptr, val);
}


/* -------- MACROS --------------------------------------------------------- */

// Combines, gets, and returns the shared (spread-encoding) vertex ID from its piecewise representation, given an edge vector.
#define graph_data_macro_get_shared_vertex(__m256i_grecord)                                         \
    (                                                                                               \
        ((_mm256_extract_epi64(__m256i_grecord, 0) & 0x7fff000000000000ull) >> 48) |                \
        ((_mm256_extract_epi64(__m256i_grecord, 1) & 0x7fff000000000000ull) >> 33) |                \
        ((_mm256_extract_epi64(__m256i_grecord, 2) & 0x7fff000000000000ull) >> 18) |                \
        ((_mm256_extract_epi64(__m256i_grecord, 3) & 0x0007000000000000ull) >>  3)                  \
    )

    
/* -------- LOCALS --------------------------------------------------------- */

// Statically-allocated buffer to hold all NUMA-aware data structure base values
static uint64_t graph_numa_buf[16 * CONFIG_NUMA_MAX_NODES];

// Statically-allocated buffer to hold all temporary NUMA-aware data structure base values
static uint64_t graph_numa_temp_buf[8 * CONFIG_NUMA_MAX_NODES];

// Temporary buffer for reading from files, plus size and position information
// Size must be divisible by 3 to account for possibly loading edge-weighted graphs, which will have tuples of 3 64-bit integers
static uint64_t* graph_temp_edges_read_buffer[2] = { NULL, NULL };
static uint64_t graph_temp_edges_read_buffer_max_count = 3ull*1024ull*1024ull/sizeof(uint64_t);
static uint64_t graph_temp_edges_read_buffer_count[2] = { 0ull, 0ull };

// Size of an edge tuple to be read from the file
// Filled during initialization based on whether or not edges are weighted in the input graph
static uint64_t graph_temp_edges_read_tuple_size = 0ull;

// Temporary buffer for storing vertex in-degrees during read
// Used during the first read pass to figure out how many vectors are needed
uint64_t* graph_temp_vertex_indegrees = NULL;

// Temporary buffer for storing per-NUMA-node out-degrees during initialization
// Used to assist with CSRI generation
uint64_t** const graph_temp_vertex_outdegrees_numa = (uint64_t**)&graph_numa_temp_buf[0 * CONFIG_NUMA_MAX_NODES];


/* -------- DATA STRUCTURES ------------------------------------------------ */
// See "graphdata.h" for documentation.

uint64_t graph_num_vertices = 0ull;
uint64_t graph_num_edges = 0ull;
uint64_t graph_num_edge_vectors = 0ull;
uint64_t graph_param_num_static_iterations = 0ull;
uint64_t graph_param_root_vertex = 0ull;
uint64_t graph_param_frontier_threshold_pct = 0ull;
vertexprop_t* graph_vertex_props = NULL;
vertexprop_t* graph_vertex_accumulators = NULL;
vertexprop_t* graph_vertex_outdegrees = NULL;
mergeaccum_t* graph_vertex_merge_buffer = NULL;
mergeaccum_t** const graph_vertex_merge_buffer_baseptr_numa = (mergeaccum_t**)&graph_numa_buf[0 * CONFIG_NUMA_MAX_NODES];
__m256i** const graph_edges_gather_list_bufs_numa = (__m256i**)&graph_numa_buf[1 * CONFIG_NUMA_MAX_NODES];
uint64_t** const graph_edges_gather_list_index_numa = (uint64_t**)&graph_numa_buf[2 * CONFIG_NUMA_MAX_NODES];
uint64_t* const graph_edges_gather_list_counts_numa = (uint64_t*)&graph_numa_buf[3 * CONFIG_NUMA_MAX_NODES];
uint64_t* const graph_vertex_first_numa = (uint64_t*)&graph_numa_buf[4 * CONFIG_NUMA_MAX_NODES];
uint64_t* const graph_vertex_last_numa = (uint64_t*)&graph_numa_buf[5 * CONFIG_NUMA_MAX_NODES];
uint64_t* const graph_vertex_count_numa = (uint64_t*)&graph_numa_buf[6 * CONFIG_NUMA_MAX_NODES];
uint64_t** const graph_scheduler_dynamic_counter_numa = (uint64_t**)&graph_numa_buf[7 * CONFIG_NUMA_MAX_NODES];
uint64_t** graph_wedge_frontier_numa = (uint64_t**)&graph_numa_buf[8 * CONFIG_NUMA_MAX_NODES];
uint64_t* graph_wedge_frontier_count_numa = (uint64_t*)&graph_numa_buf[9 * CONFIG_NUMA_MAX_NODES];
uint64_t** graph_edges_wedge_csri_index_numa = (uint64_t**)&graph_numa_buf[10 * CONFIG_NUMA_MAX_NODES];
uint64_t** graph_edges_wedge_csri_edges_numa = (uint64_t**)&graph_numa_buf[11 * CONFIG_NUMA_MAX_NODES];
uint64_t* graph_frontier_active_vertices = NULL;
uint64_t* graph_frontier_active_vertices_snapshot = NULL;
uint64_t* graph_frontier_unconverged_vertices = NULL;
uint64_t graph_frontier_count = 0ull;
uint64_t* graph_reduce_buffer = NULL;


/* -------- TYPE DEFINITIONS ----------------------------------------------- */

// Defines a multi-threaded graph ingress operation.
typedef struct ingress_ctrl_t
{
    FILE* graph_file;                                       // file from which to read
    void(*consumer_func)(void);                             // consumer function to invoke
} ingress_ctrl_t;



/* -------- INTERNAL FUNCTIONS --------------------------------------------- */

// Allocates and initializes merge buffers used for Scheduler Awareness.
void graph_data_internal_allocate_merge_buffers(const uint32_t num_threads, const uint32_t num_numa_nodes, const uint32_t* const numa_nodes)
{
    const uint64_t num_blocks_per_thread = 32ull;
    const uint64_t num_blocks = num_blocks_per_thread * (uint64_t)num_threads;
    const uint64_t num_blocks_per_node = num_blocks / (uint64_t)num_numa_nodes;
    
    graph_vertex_merge_buffer = (mergeaccum_t*)siloSimpleBufferAlloc(sizeof(mergeaccum_t) * num_blocks, numa_nodes[0]);
    
    for (uint64_t i = 0; i < num_blocks; ++i)
    {
        graph_vertex_merge_buffer[i].initial_vertex_id = UINT64_MAX;
        graph_vertex_merge_buffer[i].final_vertex_id = UINT64_MAX;
        graph_vertex_merge_buffer[i].final_partial_value.u = 0ull;
    }
    
    for (uint64_t i = 0; i < num_numa_nodes; ++i)
    {
        graph_vertex_merge_buffer_baseptr_numa[i] = &graph_vertex_merge_buffer[i * num_blocks_per_node];
    }
}

// Allocates temporary buffers used for graph ingress.
// Sets up other miscellaneous read-related data structures as well.
static bool graph_data_internal_alloc_temp_read_buffers(const uint32_t numa_node)
{
    for (size_t i = 0; i < (sizeof(graph_temp_edges_read_buffer) / sizeof(graph_temp_edges_read_buffer[0])); ++i)
    {
        graph_temp_edges_read_buffer[i] = (uint64_t*)siloSimpleBufferAlloc(sizeof(uint64_t) * graph_temp_edges_read_buffer_max_count, numa_node);
        
        if (NULL == graph_temp_edges_read_buffer[i])
            return false;
    }
    
    if (execution_uses_edge_weights)
        graph_temp_edges_read_tuple_size = 3ull;
    else
        graph_temp_edges_read_tuple_size = 2ull;
    
    return true;
}

// Frees temporary buffers used for graph ingress.
static void graph_data_internal_free_temp_read_buffers(void)
{
    for (size_t i = 0; i < (sizeof(graph_temp_edges_read_buffer) / sizeof(graph_temp_edges_read_buffer[0])); ++i)
    {
        if (NULL != graph_temp_edges_read_buffer[i])
        {
            siloFree((void*)graph_temp_edges_read_buffer[i]);
            graph_temp_edges_read_buffer[i] = NULL;
        }
    }
}

// Retrieves the next edge from a temporary edge reading buffer.
// Places the source and destination vertex information into the specified locations.
// Returns zero on failure (no edges left in buffer), the next position on success.
static uint64_t graph_data_internal_ingress_helper_retrieve_next_edge_from_buf(uint64_t* const out_edge_source, uint64_t* const out_edge_dest, uint64_t* const out_edge_weight, const uint64_t posidx, const uint32_t bufidx)
{
    // check for remaining edges in the buffer
    if (posidx >= graph_temp_edges_read_buffer_count[bufidx])
    {
        return 0;
    }
    
    *out_edge_source = graph_temp_edges_read_buffer[bufidx][posidx+0ull];
    *out_edge_dest = graph_temp_edges_read_buffer[bufidx][posidx+1ull];
    
    if (execution_uses_edge_weights)
    {
        *out_edge_weight = graph_temp_edges_read_buffer[bufidx][posidx+2ull];
        return (posidx + 3ull);
    }
    else
    {
        return (posidx + 2ull);
    }
}

// Composes and writes an edge vector, given a shared vertex ID, individual vertex IDs, and edge weights.
// Returns the number of AVX vectors produced, which itself depends on whether or not weights were written out.
uint64_t graph_data_internal_ingress_helper_write_edge_vector(const uint64_t shared_vertex_id, const uint64_t* const individual_vertex_ids, const uint64_t* const edge_weights, const uint64_t individual_vertex_id_count, __m256i* const dest_vec_buf)
{
    // compose the destination ID by splitting it into pieces
    const uint64_t edge_shared_vertex_pieces[4] = {
        (shared_vertex_id & 0x0000000000007fffull) >> 0,    // bits 14:0
        (shared_vertex_id & 0x000000003fff8000ull) >> 15,   // bits 29:15
        (shared_vertex_id & 0x00001fffc0000000ull) >> 30,   // bits 44:30
        (shared_vertex_id & 0x0000e00000000000ull) >> 45    // bits 47:45
    };

    // create and write the in-edge list record
    // upper bit is the "valid" bit, the next 15 bits are parts of the destination vertex ID as pieced out above, and the lower 48 bits are source vertex IDs
    // when gathering, the destination vertex ID will be recovered from this piecewise representation, the "valid" bit is a mask, and the lower 48 bits are used as gather indices
    dest_vec_buf[0] = _mm256_set_epi64x(
        ((individual_vertex_id_count > 3 ? 1ull : 0ull) << 63) | (edge_shared_vertex_pieces[3] << 48) | (individual_vertex_ids[3]),
        ((individual_vertex_id_count > 2 ? 1ull : 0ull) << 63) | (edge_shared_vertex_pieces[2] << 48) | (individual_vertex_ids[2]),
        ((individual_vertex_id_count > 1 ? 1ull : 0ull) << 63) | (edge_shared_vertex_pieces[1] << 48) | (individual_vertex_ids[1]),
        ((individual_vertex_id_count > 0 ? 1ull : 0ull) << 63) | (edge_shared_vertex_pieces[0] << 48) | (individual_vertex_ids[0])
    );
    
    if (!execution_uses_edge_weights)
        return 1;
    
    dest_vec_buf[1] = _mm256_set_epi64x(
        (individual_vertex_id_count > 3 ? edge_weights[3] : 0ull),
        (individual_vertex_id_count > 2 ? edge_weights[2] : 0ull),
        (individual_vertex_id_count > 1 ? edge_weights[1] : 0ull),
        (individual_vertex_id_count > 0 ? edge_weights[0] : 0ull)
    );
    
    return 2;
}

// One of the functions that supports graph ingress.
// Consumer: reads from a buffer and produces edge vectors.
static void graph_data_internal_ingress_data_consumer_edge_vectors(void)
{
    // information about buffer and position within it
    uint32_t bufidx = 0;
    uint64_t posidx = 0ull;

    // information about the current record that has been read from the file
    uint64_t edge_source = 0ull;
    uint64_t edge_dest = 0ull;
    uint64_t edge_weight = 0ull;
    
    // stash for holding information while building edge vectors
    uint64_t edgevec_stash_srcids[4] = { 0ull, 0ull, 0ull, 0ull };
    uint64_t edgevec_stash_weights[4] = { 0ull, 0ull, 0ull, 0ull };
    uint64_t edgevec_stash_dstid = 0ull;
    uint32_t edgevec_stash_count = 0;
    
    // buffer address control information
    uint32_t numa_index = 0;
    uint64_t write_index = 0ull;
    uint64_t num_vectors_written = 0ull;
    
    while(1)
    {
        spindleBarrierLocal();
        
        if (graph_temp_edges_read_buffer_count[bufidx] < graph_temp_edges_read_tuple_size)
        {
            break;
        }
        
        posidx = graph_data_internal_ingress_helper_retrieve_next_edge_from_buf(&edge_source, &edge_dest, &edge_weight, posidx, bufidx);
        
        while (0 != posidx)
        {
            // if stash is not empty and the just-read destination is different, or if the stash is full, flush the stash
            // note that the stash size is 4 to correspond to the number of packed doubles that fit into a 256-bit AVX register
            if ((0 != edgevec_stash_count && edgevec_stash_dstid != edge_dest) || (4 == edgevec_stash_count))
            {
                // compose a record and write out the stash
                write_index += graph_data_internal_ingress_helper_write_edge_vector(edgevec_stash_dstid, edgevec_stash_srcids, edgevec_stash_weights, edgevec_stash_count, &graph_edges_gather_list_bufs_numa[numa_index][write_index]);
                num_vectors_written += 1ull;
                
                if (graph_edges_gather_list_counts_numa[numa_index] == num_vectors_written)
                {
                    numa_index += 1;
                    write_index = 0ull;
                    num_vectors_written = 0ull;
                }
                
                // reinitialize the stash to empty
                edgevec_stash_count = 0;
            }
            
            // add the new vertex into the stash
            edgevec_stash_dstid = edge_dest;
            edgevec_stash_srcids[edgevec_stash_count] = edge_source;
            edgevec_stash_weights[edgevec_stash_count] = edge_weight;
            edgevec_stash_count += 1;
            
            posidx = graph_data_internal_ingress_helper_retrieve_next_edge_from_buf(&edge_source, &edge_dest, &edge_weight, posidx, bufidx);
        }
        
        bufidx = (bufidx + 1) & 1;
    }
    
    // write the final vector
    if (0 != edgevec_stash_count)
    {
        graph_data_internal_ingress_helper_write_edge_vector(edgevec_stash_dstid, edgevec_stash_srcids, edgevec_stash_weights, edgevec_stash_count, &graph_edges_gather_list_bufs_numa[numa_index][write_index]);
        num_vectors_written += 1ull;
    }
}

// One of the functions that supports graph ingress.
// Consumer: reads from a buffer and computes vertex in-degrees.
static void graph_data_internal_ingress_data_consumer_vertex_indegree(void)
{
    // information about buffer and position within it
    uint32_t bufidx = 0;
    uint64_t posidx = 0ull;

    // information about the current record that has been read from the file
    uint64_t edge_source = 0ull;
    uint64_t edge_dest = 0ull;
    uint64_t edge_weight = 0ull;
    
    while(1)
    {
        spindleBarrierLocal();
        
        if (graph_temp_edges_read_buffer_count[bufidx] < graph_temp_edges_read_tuple_size)
        {
            break;
        }
        
        posidx = graph_data_internal_ingress_helper_retrieve_next_edge_from_buf(&edge_source, &edge_dest, &edge_weight, posidx, bufidx);
        
        while (0 != posidx)
        {
            graph_temp_vertex_indegrees[edge_dest] += 1ull;
            posidx = graph_data_internal_ingress_helper_retrieve_next_edge_from_buf(&edge_source, &edge_dest, &edge_weight, posidx, bufidx);
        }
        
        bufidx = (bufidx + 1) & 1;
    }
}

// One of the functions that supports graph ingress.
// Producer: reads from the file into a buffer.
static void graph_data_internal_ingress_data_producer(FILE* const graph_file)
{
    uint32_t bufidx = 0;
    
    while(1)
    {
        graph_temp_edges_read_buffer_count[bufidx] = fread((void*)graph_temp_edges_read_buffer[bufidx], sizeof(graph_temp_edges_read_buffer[0][0]), graph_temp_edges_read_buffer_max_count, graph_file);
        
        if (graph_temp_edges_read_buffer_count[bufidx] < graph_temp_edges_read_tuple_size)
        {
            break;
        }
        
        bufidx = (bufidx + 1) & 1;
        
        spindleBarrierLocal();
    }
    
    spindleBarrierLocal();
}

// Spindle entry point for controlling multi-threaded graph ingress operations.
static void graph_data_internal_ingress_multithread_control(void* arg)
{
    ingress_ctrl_t* ctrl = (ingress_ctrl_t*)arg;
    
    switch (spindleGetLocalThreadID())
    {
    case 0:
        graph_data_internal_ingress_data_producer(ctrl->graph_file);
        break;
        
    default:
        ctrl->consumer_func();
        break;
    }
}

// Spindle entry point for performing application-specific initialization.
static void graph_data_internal_initialize_application_specific(void* arg)
{
    execution_initialize();
}

// Spindle entry point for initializing vertex indices.
static void graph_data_internal_initialize_vertex_index(void* arg)
{
    const __m256i* const edge_vectors = graph_edges_gather_list_bufs_numa[spindleGetTaskID()];
    const uint64_t num_edge_vectors = graph_edges_gather_list_counts_numa[spindleGetTaskID()];
    const uint64_t edge_index_shift = (execution_uses_edge_weights ? 1ull : 0ull);
    
    uint64_t* const vertex_index = graph_edges_gather_list_index_numa[spindleGetTaskID()];
    uint64_t last_vertex_seen = UINT64_MAX;
    
    SParutilStaticSchedule schedule;
    parutilSchedulerStatic(ParutilStaticSchedulerChunked, num_edge_vectors, &schedule);
    
    if (schedule.startUnit > 0)
        last_vertex_seen = graph_data_macro_get_shared_vertex(_mm256_load_si256(&edge_vectors[(schedule.startUnit - 1ull) << edge_index_shift]));
    
    for (uint64_t i = schedule.startUnit; i < schedule.endUnit; ++i)
    {
        const uint64_t current_vertex = graph_data_macro_get_shared_vertex(_mm256_load_si256(&edge_vectors[i << edge_index_shift]));
        
        if (current_vertex != last_vertex_seen)
        {
            vertex_index[current_vertex] = i;
            last_vertex_seen = current_vertex;
        }
    }
}

// Spindle entry point for initializing vertex out-degrees.
static void graph_data_internal_initialize_vertex_outdegrees(void* arg)
{
    const __m256i* const edge_vectors = graph_edges_gather_list_bufs_numa[spindleGetTaskID()];
    const uint64_t num_edge_vectors = graph_edges_gather_list_counts_numa[spindleGetTaskID()];
    const uint64_t edge_index_shift = (execution_uses_edge_weights ? 1ull : 0ull);
    
    const bool needs_local_outdegrees = (execution_needs_wedge_frontier && (spindleGetTaskCount() > 1));
    uint64_t* const local_outdegrees = graph_temp_vertex_outdegrees_numa[spindleGetTaskID()];
    
    SParutilStaticSchedule schedule;
    parutilSchedulerStatic(ParutilStaticSchedulerChunked, num_edge_vectors, &schedule);
    
    for (uint64_t i = schedule.startUnit; i < schedule.endUnit; ++i)
    {
        const __m256i edgevec = _mm256_load_si256(&edge_vectors[i << edge_index_shift]);
        
        {
            const uint64_t edge = _mm256_extract_epi64(edgevec, 0);
            
            if (edge & 0x8000000000000000ull)
            {
                const uint64_t vertex = edge & 0x0000ffffffffffffull;
                atomic_add(&graph_vertex_outdegrees[vertex].u, 1ull);
                if (needs_local_outdegrees) atomic_add(&local_outdegrees[vertex], 1ull);
            }
        }
        
        {
            const uint64_t edge = _mm256_extract_epi64(edgevec, 1);
            
            if (edge & 0x8000000000000000ull)
            {
                const uint64_t vertex = edge & 0x0000ffffffffffffull;
                atomic_add(&graph_vertex_outdegrees[vertex].u, 1ull);
                if (needs_local_outdegrees) atomic_add(&local_outdegrees[vertex], 1ull);
            }
        }
        
        {
            const uint64_t edge = _mm256_extract_epi64(edgevec, 2);
            
            if (edge & 0x8000000000000000ull)
            {
                const uint64_t vertex = edge & 0x0000ffffffffffffull;
                atomic_add(&graph_vertex_outdegrees[vertex].u, 1ull);
                if (needs_local_outdegrees) atomic_add(&local_outdegrees[vertex], 1ull);
            }
        }
        
        {
            const uint64_t edge = _mm256_extract_epi64(edgevec, 3);
            
            if (edge & 0x8000000000000000ull)
            {
                const uint64_t vertex = edge & 0x0000ffffffffffffull;
                atomic_add(&graph_vertex_outdegrees[vertex].u, 1ull);
                if (needs_local_outdegrees) atomic_add(&local_outdegrees[vertex], 1ull);
            }
        }
    }
}

// Spindle entry point for initializing Wedge CSRI data structures.
static void graph_data_internal_initialize_wedge_csri(void* arg)
{
    uint64_t* const wedge_csri_index = graph_edges_wedge_csri_index_numa[spindleGetTaskID()];
    uint64_t* const wedge_csri_edges = graph_edges_wedge_csri_edges_numa[spindleGetTaskID()];
    const uint64_t* const local_outdegrees = ((spindleGetTaskCount() > 1) ? graph_temp_vertex_outdegrees_numa[spindleGetTaskID()] : &graph_vertex_outdegrees[0].u);
    
    // build the index using local outdegrees
    if (0 == spindleGetLocalThreadID())
    {
        uint64_t last_position = 0ull;
        
        for (uint64_t i = 0ull; i < graph_num_vertices; ++i)
        {
            wedge_csri_index[i] = last_position;
            last_position += local_outdegrees[i];
        }
        
        wedge_csri_index[graph_num_vertices] = last_position;
    }
    
    spindleBarrierLocal();
    
    // fill the edge array
    {
        const __m256i* const edge_vectors = graph_edges_gather_list_bufs_numa[spindleGetTaskID()];
        const uint64_t num_edge_vectors = graph_edges_gather_list_counts_numa[spindleGetTaskID()];
        const uint64_t edge_index_shift = (execution_uses_edge_weights ? 1ull : 0ull);
        
        SParutilStaticSchedule schedule;
        parutilSchedulerStatic(ParutilStaticSchedulerChunked, num_edge_vectors, &schedule);
        
        for (uint64_t i = schedule.startUnit; i < schedule.endUnit; ++i)
        {
            const __m256i edgevec = _mm256_load_si256(&edge_vectors[i << edge_index_shift]);
            
            {
                const uint64_t edge = _mm256_extract_epi64(edgevec, 0);
                
                if (edge & 0x8000000000000000ull)
                {
                    const uint64_t vertex = edge & 0x0000ffffffffffffull;
                    wedge_csri_edges[atomic_exchange_add(&wedge_csri_index[vertex], 1ull)] = (i >> CONFIG_LOG2_FRONTIER_PRECISION);
                }
            }
            
            {
                const uint64_t edge = _mm256_extract_epi64(edgevec, 1);
                
                if (edge & 0x8000000000000000ull)
                {
                    const uint64_t vertex = edge & 0x0000ffffffffffffull;
                    wedge_csri_edges[atomic_exchange_add(&wedge_csri_index[vertex], 1ull)] = (i >> CONFIG_LOG2_FRONTIER_PRECISION);
                }
            }
            
            {
                const uint64_t edge = _mm256_extract_epi64(edgevec, 2);
                
                if (edge & 0x8000000000000000ull)
                {
                    const uint64_t vertex = edge & 0x0000ffffffffffffull;
                    wedge_csri_edges[atomic_exchange_add(&wedge_csri_index[vertex], 1ull)] = (i >> CONFIG_LOG2_FRONTIER_PRECISION);
                }
            }
            
            {
                const uint64_t edge = _mm256_extract_epi64(edgevec, 3);
                
                if (edge & 0x8000000000000000ull)
                {
                    const uint64_t vertex = edge & 0x0000ffffffffffffull;
                    wedge_csri_edges[atomic_exchange_add(&wedge_csri_index[vertex], 1ull)] = (i >> CONFIG_LOG2_FRONTIER_PRECISION);
                }
            }
        }
    }
    
    spindleBarrierLocal();
    
    // rebuild the index using local outdegrees
    if (0 == spindleGetLocalThreadID())
    {
        uint64_t last_position = 0ull;
        
        for (uint64_t i = 0ull; i < graph_num_vertices; ++i)
        {
            wedge_csri_index[i] = last_position;
            last_position += local_outdegrees[i];
        }
        
        wedge_csri_index[graph_num_vertices] = last_position;
    }
}

// Opens the graph that is to be read and sets top-level metadata.
// Also retrieves the file size of the graph.
static FILE* graph_data_internal_open_file_and_extract_metadata(const char* const filename, size_t* const filesize)
{
    FILE* const graph_file = fopen(filename, "rb");
    
    if (NULL != graph_file)
    {
        uint64_t graph_info[2];
        long pos;
        long fsize;
        
        // extract the number of vertices and edges
        if (2 != fread((void *)graph_info, sizeof(uint64_t), 2, graph_file))
        {
            fclose(graph_file);
            return NULL;
        }
        
        // figure out the file size
        pos = ftell(graph_file);
        
        if (0 > pos)
        {
            fclose(graph_file);
            return NULL;
        }
        
        if (0 != fseek(graph_file, 0, SEEK_END))
        {
            fclose(graph_file);
            return NULL;
        }
        
        fsize = ftell(graph_file);
        
        if (0 > fsize)
        {
            fclose(graph_file);
            return NULL;
        }
        
        if (0 != fseek(graph_file, pos, SEEK_SET))
        {
            fclose(graph_file);
            return NULL;
        }
        
        // write out retrieved metadata
        graph_num_vertices = graph_info[0];
        graph_num_edges = graph_info[1];
        *filesize = (size_t)fsize;
    }
    
    return graph_file;
}


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "graphdata.h" for documentation.

void graph_data_initialize_data_structures(const uint32_t num_threads, const uint32_t num_numa_nodes, const uint32_t* const numa_nodes)
{
    double time_elapsed = 0.0;
    benchmark_start(&time_elapsed);
    
    // allocate per-vertex data structures
    {
        // round the number of vertices up to the nearest multiple of the cache line size (64 bytes)
        const uint64_t num_vertices_to_allocate = graph_num_vertices + (8ull - (graph_num_vertices & 7ull));
        
        // assign vertices to each NUMA node based on the range of destination vertices in each node's part of the edge list
        // modify the boundaries between nodes to be on an 512-vertex boundary to avoid issues with vertex phases that operate on bits rather than on vertex properties
        for (uint32_t i = 0; i < num_numa_nodes; ++i)
        {
            if (i > 0)
            {
                graph_vertex_first_numa[i] = graph_vertex_last_numa[i - 1] + 1ull;
            }
            else
            {
                graph_vertex_first_numa[i] = 0ull;
            }
            
            if (i < (num_numa_nodes - 1ull))
            {
                if (execution_uses_edge_weights)
                    graph_vertex_last_numa[i] = graph_data_macro_get_shared_vertex(graph_edges_gather_list_bufs_numa[i][(graph_edges_gather_list_counts_numa[i] - 1ull) << 1ull]);
                else
                    graph_vertex_last_numa[i] = graph_data_macro_get_shared_vertex(graph_edges_gather_list_bufs_numa[i][graph_edges_gather_list_counts_numa[i] - 1ull]);
                
                graph_vertex_last_numa[i] += 511ull - (graph_vertex_last_numa[i] & 511ull);
            }
            else
            {
                graph_vertex_last_numa[i] = graph_num_vertices - 1ull;
            }
            
            if (graph_vertex_last_numa[i] >= graph_num_vertices)
                graph_vertex_last_numa[i] = graph_num_vertices - 1ull;
            
            graph_vertex_count_numa[i] = graph_vertex_last_numa[i] - graph_vertex_first_numa[i] + 1ull;
        }
        
        // print edge and vertex assignments for each node
        if (num_numa_nodes > 1)
        {
            for (uint32_t i = 0; i < num_numa_nodes; ++i)
            {
                printf("NUMA %llu:      %llu vectors and %llu vertices (%llu to %llu)\n", (long long unsigned int)numa_nodes[i], (long long unsigned int)graph_edges_gather_list_counts_numa[i], (long long unsigned int)graph_vertex_count_numa[i], (long long unsigned int)graph_vertex_first_numa[i], (long long unsigned int)graph_vertex_last_numa[i]);
            }
        }
        
        // allocate and initialize vertex property arrays
        {
            SSiloMemorySpec vertex_alloc_spec[CONFIG_NUMA_MAX_NODES];
            bool successful_multinode_alloc = true;
            
            for (uint32_t i = 0; i < (num_numa_nodes - 1); ++i)
            {
                vertex_alloc_spec[i].size = sizeof(vertexprop_t) * graph_vertex_count_numa[i];
                vertex_alloc_spec[i].numaNode = numa_nodes[i];
            }
            
            vertex_alloc_spec[num_numa_nodes - 1].size = sizeof(vertexprop_t) * (num_vertices_to_allocate - graph_vertex_first_numa[num_numa_nodes - 1]);
            vertex_alloc_spec[num_numa_nodes - 1].numaNode = numa_nodes[num_numa_nodes - 1];
            
            graph_vertex_props = (vertexprop_t*)siloMultinodeArrayAlloc(num_numa_nodes, vertex_alloc_spec);
            
            if (NULL == graph_vertex_props)
            {
                successful_multinode_alloc = false;
                graph_vertex_props = (vertexprop_t*)siloSimpleBufferAlloc(sizeof(vertexprop_t) * num_vertices_to_allocate, numa_nodes[0]);
            }
            
            parutilMemorySet((void*)graph_vertex_props, 0, sizeof(graph_vertex_props[0]) * num_vertices_to_allocate);
            
            graph_vertex_outdegrees = (vertexprop_t*)siloMultinodeArrayAlloc(num_numa_nodes, vertex_alloc_spec);
            
            if (NULL == graph_vertex_outdegrees)
            {
                successful_multinode_alloc = false;
                graph_vertex_outdegrees = (vertexprop_t*)siloSimpleBufferAlloc(sizeof(vertexprop_t) * num_vertices_to_allocate, numa_nodes[0]);
            }
            
            parutilMemorySet((void*)graph_vertex_outdegrees, 0, sizeof(graph_vertex_outdegrees[0]) * num_vertices_to_allocate);
            
            if (execution_needs_vertex_accumulator)
            {
                graph_vertex_accumulators = (vertexprop_t*)siloMultinodeArrayAlloc(num_numa_nodes, vertex_alloc_spec);
                
                if (NULL == graph_vertex_accumulators)
                {
                    successful_multinode_alloc = false;
                    graph_vertex_accumulators = (vertexprop_t*)siloSimpleBufferAlloc(sizeof(vertexprop_t) * num_vertices_to_allocate, numa_nodes[0]);
                }
                
                parutilMemorySet((void*)graph_vertex_accumulators, 0, sizeof(graph_vertex_accumulators[0]) * num_vertices_to_allocate);
            }
            
            if (!successful_multinode_alloc && num_numa_nodes > 1)
            {
                printf("NUMA:        WARNING: graph is too small to distribute vertices across nodes\n");
            }
        }
        
        // allocate and initialize temporary vertex property arrays, if needed
        if (num_numa_nodes > 1)
        {
            for (uint32_t i = 0; i < num_numa_nodes; ++i)
            {
                graph_temp_vertex_outdegrees_numa[i] = (uint64_t*)siloSimpleBufferAlloc(sizeof(graph_temp_vertex_outdegrees_numa[0][0]) * graph_num_vertices, numa_nodes[i]);
                parutilMemorySet((void*)graph_temp_vertex_outdegrees_numa[i], 0, sizeof(graph_temp_vertex_outdegrees_numa[0][0]) * graph_num_vertices);
            }
        }
        
        // allocate and initialize the vertex indices
        for (uint32_t i = 0; i < num_numa_nodes; ++i)
        {
            graph_edges_gather_list_index_numa[i] = (uint64_t*)siloSimpleBufferAlloc(sizeof(graph_edges_gather_list_index_numa[i][0]) * num_vertices_to_allocate, numa_nodes[i]);
            parutilMemorySet((void*)graph_edges_gather_list_index_numa[i], 255, sizeof(graph_edges_gather_list_index_numa[i][0]) * num_vertices_to_allocate);
        }
        
        // build the per-NUMA-node vertex indices
        {
            SSpindleTaskSpec vertex_index_init_task_spec[CONFIG_NUMA_MAX_NODES];
            
            for (uint32_t i = 0; i < num_numa_nodes; ++i)
            {
                vertex_index_init_task_spec[i].func = &graph_data_internal_initialize_vertex_index;
                vertex_index_init_task_spec[i].arg = NULL;
                vertex_index_init_task_spec[i].numaNode = numa_nodes[i];
                vertex_index_init_task_spec[i].numThreads = 0;
                vertex_index_init_task_spec[i].smtPolicy = SpindleSMTPolicyPreferPhysical;
            }
            
            spindleThreadsSpawn(vertex_index_init_task_spec, num_numa_nodes, true);
        }
    }
    
    // initialize vertex out-degrees
    {
        SSpindleTaskSpec outdegree_init_task_spec[CONFIG_NUMA_MAX_NODES];
        
        for (uint32_t i = 0; i < num_numa_nodes; ++i)
        {
            outdegree_init_task_spec[i].func = &graph_data_internal_initialize_vertex_outdegrees;
            outdegree_init_task_spec[i].arg = NULL;
            outdegree_init_task_spec[i].numaNode = numa_nodes[i];
            outdegree_init_task_spec[i].numThreads = 0;
            outdegree_init_task_spec[i].smtPolicy = SpindleSMTPolicyPreferPhysical;
        }
        
        spindleThreadsSpawn(outdegree_init_task_spec, num_numa_nodes, true);
    }
    
    // allocate frontier data structures
    if (execution_needs_wedge_frontier)
    {
        // allocate and initialize Wedge frontier data structures
        for (uint32_t i = 0; i < num_numa_nodes; ++i)
        {
            // a frontier block is a group of 512 frontier elements (one or more edge vectors, per the frontier precision), enough to fit in a cache line
            const uint64_t frontier_required_edge_block_count = (graph_edges_gather_list_counts_numa[i] >> (9ull + CONFIG_LOG2_FRONTIER_PRECISION)) + 1ull;
            const uint64_t frontier_required_edge_bit_count = frontier_required_edge_block_count << 9ull;
            
            // round up to align properly within the cache
            const uint64_t frontier_allocate_edge_bit_bytes = ((frontier_required_edge_bit_count >> 9ull) + ((frontier_required_edge_bit_count & 511ull) ? 1ull : 0ull)) << 6ull;
            
            // allocate and initialize the current NUMA node's Wedge frontier
            graph_wedge_frontier_numa[i] = (uint64_t*)siloSimpleBufferAlloc((size_t)frontier_allocate_edge_bit_bytes, numa_nodes[i]);
            graph_wedge_frontier_count_numa[i] = frontier_allocate_edge_bit_bytes >> 3ull;
            parutilMemorySet((void*)graph_wedge_frontier_numa[i], 0, frontier_allocate_edge_bit_bytes);
            
            // allocate and initialize Wedge CSRI data structures
            graph_edges_wedge_csri_index_numa[i] = (uint64_t*)siloSimpleBufferAlloc(sizeof(graph_edges_wedge_csri_index_numa[0][0]) * (1ull + graph_num_vertices), numa_nodes[i]);
            graph_edges_wedge_csri_edges_numa[i] = (uint64_t*)siloSimpleBufferAlloc(sizeof(graph_edges_wedge_csri_edges_numa[0][0]) * (graph_edges_gather_list_counts_numa[i] << 2ull), numa_nodes[i]);
            parutilMemorySet((void*)graph_edges_wedge_csri_index_numa[i], 255, sizeof(graph_edges_wedge_csri_index_numa[0][0]) * (1ull + graph_num_vertices));
            parutilMemorySet((void*)graph_edges_wedge_csri_edges_numa[i], 0, sizeof(graph_edges_wedge_csri_edges_numa[0][0]) * (graph_edges_gather_list_counts_numa[i] << 2ull));
        }
        
        // build the Wedge CSRI data structures
        {
            SSpindleTaskSpec csri_init_task_spec[CONFIG_NUMA_MAX_NODES];
            
            for (uint32_t i = 0; i < num_numa_nodes; ++i)
            {
                csri_init_task_spec[i].func = &graph_data_internal_initialize_wedge_csri;
                csri_init_task_spec[i].arg = NULL;
                csri_init_task_spec[i].numaNode = numa_nodes[i];
                csri_init_task_spec[i].numThreads = 0;
                csri_init_task_spec[i].smtPolicy = SpindleSMTPolicyPreferPhysical;
            }
            
            spindleThreadsSpawn(csri_init_task_spec, num_numa_nodes, true);
        }
        
        // allocate and initialize traditional frontier data structures
        {
            // for traditional frontiers, one bit is required for each vertex
            const uint64_t traditional_frontier_bit_count = graph_num_vertices;
            
            // round up to align properly within the cache
            const uint64_t traditional_frontier_allocate_bytes = ((traditional_frontier_bit_count >> 9ull) + ((traditional_frontier_bit_count & 511ull) ? 1ull : 0ull)) << 6ull;
            
            // define the multi-node allocation of bit-mask frontier data structures
            SSiloMemorySpec frontier_alloc_spec[CONFIG_NUMA_MAX_NODES];
            uint64_t frontier_alloc_bytes_left = traditional_frontier_allocate_bytes;
            
            for (uint32_t i = 0; i < (num_numa_nodes - 1); ++i)
            {
                frontier_alloc_spec[i].size = graph_vertex_count_numa[i] >> 3ull;
                frontier_alloc_spec[i].numaNode = numa_nodes[i];
                
                frontier_alloc_bytes_left -= frontier_alloc_spec[i].size;
            }
            
            frontier_alloc_spec[num_numa_nodes - 1].size = frontier_alloc_bytes_left;
            frontier_alloc_spec[num_numa_nodes - 1].numaNode = numa_nodes[num_numa_nodes - 1];
            
            // allocate and initialize all required traditional frontier data structures
            // Wedge requires the active vertex frontier, and applications might require the full set of traditional frontier data structures
            graph_frontier_active_vertices = (uint64_t*)siloMultinodeArrayAlloc(num_numa_nodes, frontier_alloc_spec);
            
            if (NULL == graph_frontier_active_vertices)
                graph_frontier_active_vertices = (uint64_t*)siloSimpleBufferAlloc((size_t)traditional_frontier_allocate_bytes, numa_nodes[0]);
            
            parutilMemorySet((void*)graph_frontier_active_vertices, 0, traditional_frontier_allocate_bytes);
            
            if (execution_needs_traditional_frontiers)
            {
                graph_frontier_active_vertices_snapshot = (uint64_t*)siloMultinodeArrayAlloc(num_numa_nodes, frontier_alloc_spec);
                graph_frontier_unconverged_vertices = (uint64_t*)siloMultinodeArrayAlloc(num_numa_nodes, frontier_alloc_spec);
                
                if (NULL == graph_frontier_active_vertices_snapshot)
                    graph_frontier_active_vertices_snapshot = (uint64_t*)siloSimpleBufferAlloc((size_t)traditional_frontier_allocate_bytes, numa_nodes[0]);
                
                if (NULL == graph_frontier_unconverged_vertices)
                    graph_frontier_unconverged_vertices = (uint64_t*)siloSimpleBufferAlloc((size_t)traditional_frontier_allocate_bytes, numa_nodes[0]);
                
                parutilMemorySet((void*)graph_frontier_active_vertices_snapshot, 0, traditional_frontier_allocate_bytes);
                parutilMemorySet((void*)graph_frontier_unconverged_vertices, 255, traditional_frontier_allocate_bytes);
            }
            
            graph_frontier_count = traditional_frontier_allocate_bytes >> 3ull;
        }
    }
    
    // allocate miscellaneous support data structures
    {
        // calculate the number of inter-thread reduce buffer entries to allocate, for things like exchanging convergence information or other global variables
        // this is just 64 bits (8 bytes) per threads rounded up to the nearest multiple of the cache line size (64 bytes)
        const uint64_t num_reduce_buffers_to_allocate = num_threads + (8ull - (num_threads & 7ull));
        
        graph_reduce_buffer = (uint64_t*)siloSimpleBufferAlloc(sizeof(uint64_t) * num_reduce_buffers_to_allocate, numa_nodes[0]);
        parutilMemorySet((void*)graph_reduce_buffer, 0, sizeof(graph_reduce_buffer[0]) * num_reduce_buffers_to_allocate);
        
        graph_data_internal_allocate_merge_buffers(num_threads, num_numa_nodes, numa_nodes);
        
        for (uint32_t i = 0; i < num_numa_nodes; ++i)
        {
            graph_scheduler_dynamic_counter_numa[i] = (uint64_t*)siloSimpleBufferAlloc(sizeof(uint64_t), numa_nodes[i]);
        }
    }
    
    // free any previously-allocated temporary data structures
    {
        if (num_numa_nodes > 1)
        {
            for (uint32_t i = 0; i < num_numa_nodes; ++i)
            {
                siloFree((void*)graph_temp_vertex_outdegrees_numa[i]);
                graph_temp_vertex_outdegrees_numa[i] = NULL;
            }
        }
    }
    
    // allow the application to perform initialization steps of its own
    {
        SSpindleTaskSpec app_init_task_spec[CONFIG_NUMA_MAX_NODES];
        
        for (uint32_t i = 0; i < num_numa_nodes; ++i)
        {
            app_init_task_spec[i].func = &graph_data_internal_initialize_application_specific;
            app_init_task_spec[i].arg = NULL;
            app_init_task_spec[i].numaNode = numa_nodes[i];
            app_init_task_spec[i].numThreads = 0;
            app_init_task_spec[i].smtPolicy = SpindleSMTPolicyPreferPhysical;
        }
        
        spindleThreadsSpawn(app_init_task_spec, num_numa_nodes, true);
    }
    
    time_elapsed = benchmark_stop(&time_elapsed);
    printf("Graph:       initialization completed in %.2lf msec\n", time_elapsed);
}

// --------

bool graph_data_read_from_file(const char* const filename, const uint32_t num_numa_nodes, const uint32_t* const numa_nodes)
{
    size_t graph_filesize = 0;
    FILE* const graph_file = graph_data_internal_open_file_and_extract_metadata(filename, &graph_filesize);
    const long graph_file_beginpos = (NULL == graph_file) ? 0 : ftell(graph_file);
    
    printf("Graph:       reading \"%s\"\n", filename);
    
    if ((NULL == graph_file) || (-1 == graph_file_beginpos))
    {
        printf("Graph:       ERROR: unable to open \"%s\"\n", filename);
        return false;
    }
    
    printf("Graph:       contains %llu vertices and %llu edges\n", (long long unsigned int)graph_num_vertices, (long long unsigned int)graph_num_edges);
    
    // verify graph file size versus what is expected
    {
        size_t graph_expected_filesize = 0;
        
        // expected filesize is two 64-bit unsigned integers plus the size of each edge, which is two or three 64-bit unsigned integers depending if weights are needed
        if (execution_uses_edge_weights)
            graph_expected_filesize = (2ull * sizeof(uint64_t)) + (graph_num_edges * 3ull * sizeof(uint64_t));
        else
            graph_expected_filesize = (2ull * sizeof(uint64_t)) + (graph_num_edges * 2ull * sizeof(uint64_t));
        
        // verify that the graph filesize matches what is expected
        if (graph_expected_filesize != graph_filesize)
        {
            fclose(graph_file);
            printf("Graph:       ERROR: file is %llu bytes, expecting %llu bytes\n", (long long unsigned int)graph_filesize, (long long unsigned int)graph_expected_filesize);
            return false;
        }
    }
    
    // set up temporary graph read data structures
    graph_temp_vertex_indegrees = (uint64_t*)siloSimpleBufferAlloc(sizeof(uint64_t) * graph_num_vertices, numa_nodes[0]);
    
    if ((false == graph_data_internal_alloc_temp_read_buffers(numa_nodes[0])) || (NULL == graph_temp_vertex_indegrees))
    {
        if (NULL != graph_temp_vertex_indegrees)
        {
            siloFree((void*)graph_temp_vertex_indegrees);
            graph_temp_vertex_indegrees = NULL;
        }
        
        graph_data_internal_free_temp_read_buffers();
        fclose(graph_file);
        printf("Graph:       ERROR: unable to allocate temporary buffers for reading\n");
        return false;
    }
    
    parutilMemorySet((void*)graph_temp_vertex_indegrees, 0, sizeof(uint64_t) * graph_num_vertices);
    
    // read from the graph file
    {
        double time_elapsed = 0.0;
        benchmark_start(&time_elapsed);
        
        // first pass: figure out the in-degree of every vertex, which in turn determines the number of edge vectors needed
        {
            ingress_ctrl_t first_pass_ctrl;
            SSpindleTaskSpec read_task_spec;
            
            first_pass_ctrl.graph_file = graph_file;
            first_pass_ctrl.consumer_func = &graph_data_internal_ingress_data_consumer_vertex_indegree;
            
            read_task_spec.func = &graph_data_internal_ingress_multithread_control;
            read_task_spec.arg = (void*)&first_pass_ctrl;
            read_task_spec.numaNode = numa_nodes[0];
            read_task_spec.numThreads = 2;
            read_task_spec.smtPolicy = SpindleSMTPolicyDisableSMT;
            
            if (0 != spindleThreadsSpawn(&read_task_spec, 1, true))
            {
                siloFree((void*)graph_temp_vertex_indegrees);
                graph_temp_vertex_indegrees = NULL;
                
                graph_data_internal_free_temp_read_buffers();
                fclose(graph_file);
                printf("Graph:       ERROR: unable to spawn threads during read\n");
                return false;
            }
        }
        
        // compute the number of edge vectors needed
        for (uint64_t i = 0ull; i < graph_num_vertices; ++i)
            graph_num_edge_vectors += (graph_temp_vertex_indegrees[i] >> 2ull) + ((graph_temp_vertex_indegrees[i] & 3ull) ? 1ull : 0ull);
        
        printf("Graph:       creating %llu edge vectors, efficiency = %.1lf%%\n", (long long unsigned int)graph_num_edge_vectors, ((double)graph_num_edges / (double)graph_num_edge_vectors / 4.0 * 100.0));
        
        // free unnecessary data structures and reset the file position
        siloFree((void*)graph_temp_vertex_indegrees);
        graph_temp_vertex_indegrees = NULL;
        
        if (0 != fseek(graph_file, graph_file_beginpos, SEEK_SET))
        {
            graph_data_internal_free_temp_read_buffers();
            fclose(graph_file);
            printf("Graph:       ERROR: file I/O error during read\n");
            return false;
        }
        
        // prepare edge vector list data structures
        {
            // divide the number of vectors as evenly as possible by the number of NUMA nodes in use
            const uint64_t num_edge_vectors_per_node = graph_num_edge_vectors / num_numa_nodes;
            
            // assign vectors to NUMA nodes
            {
                uint64_t num_edge_vectors_left_to_assign = graph_num_edge_vectors;
                
                for (uint32_t i = 0; i < (num_numa_nodes - 1); ++i)
                {
                    graph_edges_gather_list_counts_numa[i] = num_edge_vectors_per_node;
                    num_edge_vectors_left_to_assign -= num_edge_vectors_per_node;
                }
                
                graph_edges_gather_list_counts_numa[num_numa_nodes - 1] = num_edge_vectors_left_to_assign;
            }
            
            // allocate buffer space for each NUMA node's edge vector list
            for (uint32_t i = 0; i < num_numa_nodes; ++i)
            {
                // round the number of edge vectors up to the nearest multiple of the frontier precision
                const uint64_t num_edge_vectors_to_allocate = graph_edges_gather_list_counts_numa[i] + ((1ull << (uint64_t)CONFIG_LOG2_FRONTIER_PRECISION) - (graph_edges_gather_list_counts_numa[i] & ((1ull << (uint64_t)CONFIG_LOG2_FRONTIER_PRECISION) - 1ull)));
                
                // allocate the edge vector array and initialize the tail part that will not be filled during ingress
                if (execution_uses_edge_weights)
                {
                    graph_edges_gather_list_bufs_numa[i] = (__m256i*)siloSimpleBufferAlloc(2ull * sizeof(__m256i) * num_edge_vectors_to_allocate, numa_nodes[i]);
                    
                    for (uint64_t j = (graph_edges_gather_list_counts_numa[i] << 1ull); j < (num_edge_vectors_to_allocate << 1ull); j += 2ull)
                    {
                        const uint64_t zeroes[] = { 0ull, 0ull, 0ull, 0ull };
                        graph_data_internal_ingress_helper_write_edge_vector(graph_num_vertices - 1ull, zeroes, zeroes, 0ull, &graph_edges_gather_list_bufs_numa[i][j]);
                    }
                }
                else
                {
                    graph_edges_gather_list_bufs_numa[i] = (__m256i*)siloSimpleBufferAlloc(sizeof(__m256i) * num_edge_vectors_to_allocate, numa_nodes[i]);
                    
                    for (uint64_t j = graph_edges_gather_list_counts_numa[i]; j < num_edge_vectors_to_allocate; j += 1ull)
                    {
                        const uint64_t zeroes[] = { 0ull, 0ull, 0ull, 0ull };
                        graph_data_internal_ingress_helper_write_edge_vector(graph_num_vertices - 1ull, zeroes, zeroes, 0ull, &graph_edges_gather_list_bufs_numa[i][j]);
                    }
                }
            }
        }
        
        // second pass: fill the vertex edge vector arrays
        {
            ingress_ctrl_t second_pass_ctrl;
            SSpindleTaskSpec read_task_spec;
            
            second_pass_ctrl.graph_file = graph_file;
            second_pass_ctrl.consumer_func = &graph_data_internal_ingress_data_consumer_edge_vectors;
            
            read_task_spec.func = &graph_data_internal_ingress_multithread_control;
            read_task_spec.arg = (void*)&second_pass_ctrl;
            read_task_spec.numaNode = numa_nodes[0];
            read_task_spec.numThreads = 2;
            read_task_spec.smtPolicy = SpindleSMTPolicyDisableSMT;
            
            if (0 != spindleThreadsSpawn(&read_task_spec, 1, true))
            {
                siloFree((void*)graph_temp_vertex_indegrees);
                graph_temp_vertex_indegrees = NULL;
                
                siloFree((void*)graph_edges_gather_list_bufs_numa[0]);
                graph_edges_gather_list_bufs_numa[0] = NULL;
                
                graph_data_internal_free_temp_read_buffers();
                fclose(graph_file);
                printf("Graph:       ERROR: unable to spawn threads during read\n");
                return false;
            }
        }
        
        time_elapsed = benchmark_stop(&time_elapsed);
        printf("Graph:       loading completed in %.2lf msec\n", time_elapsed);
    }
    
    graph_data_internal_free_temp_read_buffers();
    fclose(graph_file);
    return true;
}
