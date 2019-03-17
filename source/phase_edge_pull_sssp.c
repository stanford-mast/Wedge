/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* phase_edge_pull_sssp.c
*      Implementation of the Edge-Pull phase for Single-Source Shortest Path.
*      This is a simulated version that computes edge weights instead of
*      having them stored directly with the graph.  Thus, the graph data
*      structures are all unweighted.
*****************************************************************************/

#include "configuration.h"
#include "constants.h"
#include "graphdata.h"
#include "graphtypes.h"
#include "intrinhelper.h"

#include <spindle.h>
#include <stdint.h>


#define extract_destination_vertex(filtered_vec) ((_mm256_extract_epi64(filtered_vec, 0) >> 48ull) | (_mm256_extract_epi64(filtered_vec, 1) >> 33ull) | (_mm256_extract_epi64(filtered_vec, 2) >> 18ull) | (_mm256_extract_epi64(filtered_vec, 3) >> 3ull))


// atomically set the specified bit in the specified bitmask
static inline void set_bit(void* const bitmask, const uint64_t bitindex)
{
    uint8_t* const addr = &(((uint8_t*)bitmask)[bitindex >> 3ull]);
    const uint8_t mask = (1 << ((uint8_t)bitindex & 7));
    
    __asm__("lock or BYTE PTR [%1], %2" : "=m"(*addr) : "r"(addr), "r"(mask));
}

// atomically set the specified bit in the specified bitmask if it is not already set
// returns original status of the bit (either 1 or 0)
static inline uint8_t cond_set_bit(void* const bitmask, const uint64_t bitindex)
{
    uint8_t* const addr = &(((uint8_t*)bitmask)[bitindex >> 3ull]);
    const uint8_t mask = (1 << ((uint8_t)bitindex & 7));
    
    if (0 != (*addr & mask))
        return 1;
    
    __asm__("lock or BYTE PTR [%1], %2" : "=m"(*addr) : "r"(addr), "r"(mask));
    return 0;
}


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "phases.h" for documentation.

void perform_edge_pull_phase_sssp(const __m256i* const edge_list, const uint64_t edge_list_count)
{
    // load common constants
    const __m256i vec_mask_destination_vertex = const_vid_and_mask;
    const __m256i vec_mask_source_vertices = const_edge_list_and_mask;
    const __m256i vec_mask_valid_bits = const_edge_mask_and_mask;
    const __m256i vec_infinity = _mm256_set1_epi64x(-1ll);
    
    // initialize per-thread variables
    const uint64_t schedule_num_units = (uint64_t)spindleGetLocalThreadCount() << 5ull;
    const uint64_t schedule_work_per_unit = edge_list_count / schedule_num_units;
    const uint64_t schedule_work_extra = edge_list_count % schedule_num_units;
    uint64_t* const schedule_dynamic_counter = graph_scheduler_dynamic_counter_numa[spindleGetTaskID()];
    mergeaccum_t* const merge_buffer_base = graph_vertex_merge_buffer_baseptr_numa[spindleGetTaskID()];
    uint64_t num_edges_changed = 0ull;
    
    // initialize shared variables with a single thread
    if (0 == spindleGetLocalThreadID())
        *schedule_dynamic_counter = spindleGetLocalThreadCount();
    
    spindleBarrierLocal();
    
    // iterate through this thread's dynamically-assigned units of work
    for (uint64_t schedule_current_unit = (uint64_t)spindleGetLocalThreadID(); schedule_current_unit < schedule_num_units; schedule_current_unit = __sync_fetch_and_add(schedule_dynamic_counter, 1ull))
    {
        const uint64_t schedule_unit_base = (schedule_work_per_unit * schedule_current_unit) + ((schedule_current_unit < schedule_work_extra) ? schedule_current_unit : schedule_work_extra);
        const uint64_t schedule_unit_max = schedule_unit_base + schedule_work_per_unit + ((schedule_current_unit < schedule_work_extra) ? 1ull : 0ull);
        
        // initialize per-unit storage elements
        __m128i vertex_accumulator = _mm256_castsi256_si128(vec_infinity);
        uint64_t prev_destination_vertex = UINT64_MAX;
        
        for (uint64_t i = (schedule_unit_base << 1ull); i < (schedule_unit_max << 1ull); i += 2ull)
        {
            // load the vector
            const __m256i edgevec = _mm256_load_si256(&edge_list[i]);
            const __m256i edgeweights = _mm256_load_si256(&edge_list[i + 1]);
            
            // prefetch ahead in the edge list
            _mm_prefetch((char*)(&edge_list[i + 16]), _MM_HINT_NTA);
            
            // extract the various quantities from the edge vector using bitwise-AND operations
            const __m256i edgevec_destination_vertex = _mm256_and_si256(edgevec, vec_mask_destination_vertex);
            const __m256i edgevec_source_vertices = _mm256_and_si256(edgevec, vec_mask_source_vertices);
            const __m256i edgevec_valid_bits = _mm256_and_si256(edgevec, vec_mask_valid_bits);
            
            // gather from the source vertices
            const __m256i source_vertex_properties = _mm256_mask_i64gather_epi64(vec_infinity, (long long int*)&graph_vertex_props[0].u, edgevec_source_vertices, edgevec_valid_bits, sizeof(graph_vertex_props[0].u));
            
            // extract the destination vertex identifier
            const uint64_t destination_vertex = extract_destination_vertex(edgevec_destination_vertex);
            
            // if there is a change to a new destination vertex, potentially update the values in memory
            if (prev_destination_vertex < destination_vertex)
            {
                const uint64_t current_value = graph_vertex_props[prev_destination_vertex].u;
                const uint64_t proposed_value = (uint64_t)_mm_extract_epi64(vertex_accumulator, 0);
                
                // if the value is changing, write it back to memory and update other data structures
                if (proposed_value < current_value)
                {
                    // write the new vertex property value
                    graph_vertex_props[prev_destination_vertex].u = proposed_value;
                    
                    // mark the vertex as active for the next iteration
                    if (!cond_set_bit(graph_frontier_active_vertices, prev_destination_vertex))
                    {
                        // increase the number of edges changed, for convergence detection
                        num_edges_changed += graph_vertex_outdegrees[prev_destination_vertex].u;
                    }
                }
                
                // advance to the new vertex
                vertex_accumulator = _mm256_castsi256_si128(vec_infinity);
            }
            
            prev_destination_vertex = destination_vertex;
            
            // adjust weights if any property values are still +INFINITY in the vertex properties
            const __m256i adjusted_edge_weights = _mm256_castpd_si256(_mm256_blendv_pd(_mm256_castsi256_pd(edgeweights), _mm256_setzero_pd(), _mm256_castsi256_pd(_mm256_cmpeq_epi64(source_vertex_properties, vec_infinity))));
            const __m256i source_properties_plus_weights = _mm256_add_epi64(source_vertex_properties, adjusted_edge_weights);
            
            // perform a reduction on the results of the gather operation to figure out the new value to put into the vertex accumulator
            const __m128i temp1 = _mm_min_epu32(_mm256_extracti128_si256(source_properties_plus_weights, 1), _mm256_castsi256_si128(source_properties_plus_weights));
            const __m128i temp2 = _mm_min_epu32(temp1, _mm_srli_si128(temp1, 8));
            vertex_accumulator = _mm_min_epu32(temp2, vertex_accumulator);
        }
        
        // write to the merge buffer for the current unit of work
        merge_buffer_base[schedule_current_unit].final_vertex_id = prev_destination_vertex;
        
        {
            const uint64_t current_value = graph_vertex_props[prev_destination_vertex].u;
            const uint64_t proposed_value = (uint64_t)_mm_extract_epi64(vertex_accumulator, 0);
            
            merge_buffer_base[schedule_current_unit].final_partial_value.u = (proposed_value < current_value) ? proposed_value : current_value;
            
            // if the value is changing, update other data structures
            if (proposed_value < current_value)
            {
                // mark the vertex as active for the next iteration
                if (!cond_set_bit(graph_frontier_active_vertices, prev_destination_vertex))
                {
                    // increase the number of edges changed, for convergence detection
                    num_edges_changed += graph_vertex_outdegrees[prev_destination_vertex].u;
                }
            }
        }
    }
    
    // write the number of updated edges to the global buffer
    graph_reduce_buffer[spindleGetGlobalThreadID()] = num_edges_changed;
}

void perform_edge_pull_phase_sssp_with_wedge_frontier(const __m256i* const edge_list, const uint64_t edge_list_count, uint64_t* const wedge_frontier, const uint64_t wedge_frontier_count)
{
    // load common constants
    const __m256i vec_mask_destination_vertex = const_vid_and_mask;
    const __m256i vec_mask_source_vertices = const_edge_list_and_mask;
    const __m256i vec_mask_valid_bits = const_edge_mask_and_mask;
    const __m256i vec_infinity = _mm256_set1_epi64x(-1ll);
    
    // initialize per-thread variables
    const uint64_t schedule_num_units = (uint64_t)spindleGetLocalThreadCount() << 5ull;
    const uint64_t schedule_work_per_unit = (wedge_frontier_count >> 3ull) / schedule_num_units;
    const uint64_t schedule_work_extra = (wedge_frontier_count >> 3ull) % schedule_num_units;
    uint64_t* const schedule_dynamic_counter = graph_scheduler_dynamic_counter_numa[spindleGetTaskID()];
    mergeaccum_t* const merge_buffer_base = graph_vertex_merge_buffer_baseptr_numa[spindleGetTaskID()];
    uint64_t num_edges_changed = 0ull;
    
    // initialize shared variables with a single thread
    if (0 == spindleGetLocalThreadID())
        *schedule_dynamic_counter = spindleGetLocalThreadCount();
    
    spindleBarrierLocal();
    
    // iterate through this thread's dynamically-assigned units of work
    for (uint64_t schedule_current_unit = (uint64_t)spindleGetLocalThreadID(); schedule_current_unit < schedule_num_units; schedule_current_unit = __sync_fetch_and_add(schedule_dynamic_counter, 1ull))
    {
        const uint64_t schedule_unit_base = (schedule_work_per_unit * schedule_current_unit) + ((schedule_current_unit < schedule_work_extra) ? schedule_current_unit : schedule_work_extra);
        const uint64_t schedule_unit_max = schedule_unit_base + schedule_work_per_unit + ((schedule_current_unit < schedule_work_extra) ? 1ull : 0ull);
        
        // initialize per-unit storage elements
        __m128i vertex_accumulator = _mm256_castsi256_si128(vec_infinity);
        uint64_t prev_destination_vertex = UINT64_MAX;
        
        for (uint64_t i = schedule_unit_base; i < schedule_unit_max; ++i)
        {
            const uint64_t frontier_base_idx = (i << 3ull);
            const uint64_t frontier_offset_start = (0 != _mm256_testc_si256(_mm256_setzero_si256(), *((__m256i*)(&wedge_frontier[frontier_base_idx])))) ? 4ull : 0ull;
            const uint64_t frontier_offset_end = (0 != _mm256_testc_si256(_mm256_setzero_si256(), *((__m256i*)(&wedge_frontier[frontier_base_idx + 4ull])))) ? 4ull : 8ull;
            
            for (uint64_t j = frontier_offset_start; j < frontier_offset_end; ++j)
            {
                const uint64_t frontier_idx = frontier_base_idx + j;
                
                uint64_t frontier_bits = wedge_frontier[frontier_idx];
                if (0ull == frontier_bits)
                    continue;
                
                wedge_frontier[frontier_idx] = 0ull;
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    // compute the vector index
                    const uint64_t vector_index = (frontier_idx << (7ull + CONFIG_LOG2_FRONTIER_PRECISION)) + (_tzcnt_u64(frontier_bits) << (1ull + CONFIG_LOG2_FRONTIER_PRECISION));
                    
                    for (uint64_t k = 0ull; k < (2ull << CONFIG_LOG2_FRONTIER_PRECISION); k += 2ull)
                    {
                        // load the vector
                        const __m256i edgevec = _mm256_load_si256(&edge_list[vector_index + k]);
                        const __m256i edgeweights = _mm256_load_si256(&edge_list[vector_index + k + 1]);
                        
                        // prefetch ahead in the edge list
                        _mm_prefetch((char*)(&edge_list[vector_index + k + 16]), _MM_HINT_NTA);
                        
                        // extract the various quantities from the edge vector using bitwise-AND operations
                        const __m256i edgevec_destination_vertex = _mm256_and_si256(edgevec, vec_mask_destination_vertex);
                        const __m256i edgevec_source_vertices = _mm256_and_si256(edgevec, vec_mask_source_vertices);
                        const __m256i edgevec_valid_bits = _mm256_and_si256(edgevec, vec_mask_valid_bits);
                        
                        // gather from the source vertices
                        const __m256i source_vertex_properties = _mm256_mask_i64gather_epi64(vec_infinity, (long long int*)&graph_vertex_props[0].u, edgevec_source_vertices, edgevec_valid_bits, sizeof(graph_vertex_props[0].u));
                        
                        // extract the destination vertex identifier
                        const uint64_t destination_vertex = extract_destination_vertex(edgevec_destination_vertex);
                        
                        // if there is a change to a new destination vertex, potentially update the values in memory
                        if (prev_destination_vertex < destination_vertex)
                        {
                            const uint64_t current_value = graph_vertex_props[prev_destination_vertex].u;
                            const uint64_t proposed_value = (uint64_t)_mm_extract_epi64(vertex_accumulator, 0);
                            
                            // if the value is changing, write it back to memory and update other data structures
                            if (proposed_value < current_value)
                            {
                                // write the new vertex property value
                                graph_vertex_props[prev_destination_vertex].u = proposed_value;
                                
                                // mark the vertex as active for the next iteration
                                if (!cond_set_bit(graph_frontier_active_vertices, prev_destination_vertex))
                                {
                                    // increase the number of edges changed, for convergence detection
                                    num_edges_changed += graph_vertex_outdegrees[prev_destination_vertex].u;
                                }
                            }
                            
                            // advance to the new vertex
                            vertex_accumulator = _mm256_castsi256_si128(vec_infinity);
                        }
                        
                        prev_destination_vertex = destination_vertex;
                        
                        // adjust weights if any property values are still +INFINITY in the vertex properties
                        const __m256i adjusted_edge_weights = _mm256_castpd_si256(_mm256_blendv_pd(_mm256_castsi256_pd(edgeweights), _mm256_setzero_pd(), _mm256_castsi256_pd(_mm256_cmpeq_epi64(source_vertex_properties, vec_infinity))));
                        const __m256i source_properties_plus_weights = _mm256_add_epi64(source_vertex_properties, adjusted_edge_weights);
                        
                        // perform a reduction on the results of the gather operation to figure out the new value to put into the vertex accumulator
                        const __m128i temp1 = _mm_min_epu32(_mm256_extracti128_si256(source_properties_plus_weights, 1), _mm256_castsi256_si128(source_properties_plus_weights));
                        const __m128i temp2 = _mm_min_epu32(temp1, _mm_srli_si128(temp1, 8));
                        vertex_accumulator = _mm_min_epu32(temp2, vertex_accumulator);
                    }
                }
            }
        }
        
        // write to the merge buffer for the current unit of work
        if (UINT64_MAX != prev_destination_vertex)
        {
            const uint64_t current_value = graph_vertex_props[prev_destination_vertex].u;
            const uint64_t proposed_value = (uint64_t)_mm_extract_epi64(vertex_accumulator, 0);
            
            merge_buffer_base[schedule_current_unit].final_vertex_id = prev_destination_vertex;
            merge_buffer_base[schedule_current_unit].final_partial_value.u = (proposed_value < current_value) ? proposed_value : current_value;
            
            // if the value is changing, update other data structures
            if (proposed_value < current_value)
            {
                // mark the vertex as active for the next iteration
                if (!cond_set_bit(graph_frontier_active_vertices, prev_destination_vertex))
                {
                    // increase the number of edges changed, for convergence detection
                    num_edges_changed += graph_vertex_outdegrees[prev_destination_vertex].u;
                }
            }
        }
    }
    
    // write the number of updated edges to the global buffer
    graph_reduce_buffer[spindleGetGlobalThreadID()] = num_edges_changed;
}
