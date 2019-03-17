/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* wedge.c
*      Implementation of Wedge frontier manipulation functionality.
*****************************************************************************/

#include "graphdata.h"

#include <immintrin.h>
#include <parutil.h>
#include <spindle.h>
#include <stdint.h>


static inline void set_bit(void* const bitmask, const uint64_t bitindex)
{
    uint8_t* const addr = &(((uint8_t*)bitmask)[bitindex >> 3ull]);
    const uint8_t mask = (1 << ((uint8_t)bitindex & 7));
    
    if (0 == (*addr & mask))
    {
        __asm__("lock or BYTE PTR [%1], %2" : "=m"(*addr) : "r"(addr), "r"(mask));
    }
}


/* -------- INTERNAL FUNCTIONS --------------------------------------------- */

// Activates a single vertex in the specified Wedge frontier using the specified data structures as input.
static inline void wedge_internal_activate_vertex(uint64_t* const wedge_frontier, const uint64_t vertex, const uint64_t* const csri_index, const uint64_t* const csri_edges)
{
    const uint64_t* const out_edges = &csri_edges[csri_index[vertex]];
    const uint64_t out_edge_count = csri_index[vertex + 1ull] - csri_index[vertex];
    
    for (uint64_t i = 0ull; i < out_edge_count; ++i)
    {
        set_bit(wedge_frontier, out_edges[i]);
    }
}


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "wedge.h" for documentation.

void wedge_activate_single_vertex(const uint64_t vertex)
{
    uint64_t* const wedge_frontier = graph_wedge_frontier_numa[spindleGetTaskID()];
    const uint64_t* const csri_index = graph_edges_wedge_csri_index_numa[spindleGetTaskID()];
    const uint64_t* const csri_edges = graph_edges_wedge_csri_edges_numa[spindleGetTaskID()];
    
    wedge_internal_activate_vertex(wedge_frontier, vertex, csri_index, csri_edges);
}

// --------

void wedge_generate_frontier(void)
{
    uint64_t* const wedge_frontier = graph_wedge_frontier_numa[spindleGetTaskID()];
    const uint64_t* const csri_index = graph_edges_wedge_csri_index_numa[spindleGetTaskID()];
    const uint64_t* const csri_edges = graph_edges_wedge_csri_edges_numa[spindleGetTaskID()];
    
    const uint64_t schedule_num_units = (uint64_t)spindleGetLocalThreadCount() << 5ull;
    const uint64_t schedule_work_per_unit = (graph_frontier_count >> 3ull) / schedule_num_units;
    const uint64_t schedule_work_extra = (graph_frontier_count >> 3ull) % schedule_num_units;
    uint64_t* const schedule_dynamic_counter = graph_scheduler_dynamic_counter_numa[spindleGetTaskID()];
    
    // initialize shared variables with a single thread
    if (0 == spindleGetLocalThreadID())
        *schedule_dynamic_counter = spindleGetLocalThreadCount();
    
    spindleBarrierLocal();
    
    // iterate through this thread's dynamically-assigned units of work
    for (uint64_t schedule_current_unit = (uint64_t)spindleGetLocalThreadID(); schedule_current_unit < schedule_num_units; schedule_current_unit = __sync_fetch_and_add(schedule_dynamic_counter, 1ull))
    {
        const uint64_t schedule_unit_base = (schedule_work_per_unit * schedule_current_unit) + ((schedule_current_unit < schedule_work_extra) ? schedule_current_unit : schedule_work_extra);
        const uint64_t schedule_unit_max = schedule_unit_base + schedule_work_per_unit + ((schedule_current_unit < schedule_work_extra) ? 1ull : 0ull);
        
        for (uint64_t i = schedule_unit_base; i < schedule_unit_max; ++i)
        {
            const uint64_t frontier_base_idx = (i << 3ull);
            const __m256i frontier_bits_vec1 = _mm256_load_si256((__m256i*)(&graph_frontier_active_vertices[frontier_base_idx + 0ull]));
            const __m256i frontier_bits_vec2 = _mm256_load_si256((__m256i*)(&graph_frontier_active_vertices[frontier_base_idx + 4ull]));
            
            if (0 == _mm256_testc_si256(_mm256_setzero_si256(), frontier_bits_vec1))
            {
                uint64_t frontier_bits = _mm256_extract_epi64(frontier_bits_vec1, 0);
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    const uint64_t vertex_to_activate = ((frontier_base_idx + 0ull) << 6ull) + _tzcnt_u64(frontier_bits);
                    wedge_internal_activate_vertex(wedge_frontier, vertex_to_activate, csri_index, csri_edges);
                }
                
                frontier_bits = _mm256_extract_epi64(frontier_bits_vec1, 1);
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    const uint64_t vertex_to_activate = ((frontier_base_idx + 1ull) << 6ull) + _tzcnt_u64(frontier_bits);
                    wedge_internal_activate_vertex(wedge_frontier, vertex_to_activate, csri_index, csri_edges);
                }
                
                frontier_bits = _mm256_extract_epi64(frontier_bits_vec1, 2);
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    const uint64_t vertex_to_activate = ((frontier_base_idx + 2ull) << 6ull) + _tzcnt_u64(frontier_bits);
                    wedge_internal_activate_vertex(wedge_frontier, vertex_to_activate, csri_index, csri_edges);
                }
                
                frontier_bits = _mm256_extract_epi64(frontier_bits_vec1, 3);
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    const uint64_t vertex_to_activate = ((frontier_base_idx + 3ull) << 6ull) + _tzcnt_u64(frontier_bits);
                    wedge_internal_activate_vertex(wedge_frontier, vertex_to_activate, csri_index, csri_edges);
                }
            }
            
            if (0 == _mm256_testc_si256(_mm256_setzero_si256(), frontier_bits_vec2))
            {
                uint64_t frontier_bits = _mm256_extract_epi64(frontier_bits_vec2, 0);
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    const uint64_t vertex_to_activate = ((frontier_base_idx + 4ull) << 6ull) + _tzcnt_u64(frontier_bits);
                    wedge_internal_activate_vertex(wedge_frontier, vertex_to_activate, csri_index, csri_edges);
                }
                
                frontier_bits = _mm256_extract_epi64(frontier_bits_vec2, 1);
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    const uint64_t vertex_to_activate = ((frontier_base_idx + 5ull) << 6ull) + _tzcnt_u64(frontier_bits);
                    wedge_internal_activate_vertex(wedge_frontier, vertex_to_activate, csri_index, csri_edges);
                }
                
                frontier_bits = _mm256_extract_epi64(frontier_bits_vec2, 2);
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    const uint64_t vertex_to_activate = ((frontier_base_idx + 6ull) << 6ull) + _tzcnt_u64(frontier_bits);
                    wedge_internal_activate_vertex(wedge_frontier, vertex_to_activate, csri_index, csri_edges);
                }
                
                frontier_bits = _mm256_extract_epi64(frontier_bits_vec2, 3);
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    const uint64_t vertex_to_activate = ((frontier_base_idx + 7ull) << 6ull) + _tzcnt_u64(frontier_bits);
                    wedge_internal_activate_vertex(wedge_frontier, vertex_to_activate, csri_index, csri_edges);
                }
            }
        }
    }
}
