/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* phase_edge_pull_pr.c
*      Implementation of the Edge-Pull phase for PageRank.
*****************************************************************************/

#include "constants.h"
#include "graphdata.h"
#include "intrinhelper.h"

#include <spindle.h>
#include <stdint.h>


#define extract_destination_vertex(filtered_vec) ((_mm256_extract_epi64(filtered_vec, 0) >> 48ull) | (_mm256_extract_epi64(filtered_vec, 1) >> 33ull) | (_mm256_extract_epi64(filtered_vec, 2) >> 18ull) | (_mm256_extract_epi64(filtered_vec, 3) >> 3ull))


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "phases.h" for documentation.

void perform_edge_pull_phase_pr(const __m256i* const edge_list, const uint64_t edge_list_count)
{
    // load common constants
    const __m256i vec_mask_destination_vertex = const_vid_and_mask;
    const __m256i vec_mask_source_vertices = const_edge_list_and_mask;
    const __m256i vec_mask_valid_bits = const_edge_mask_and_mask;
    
    // initialize per-thread variables
    const uint64_t schedule_num_units = (uint64_t)spindleGetLocalThreadCount() << 5ull;
    const uint64_t schedule_work_per_unit = edge_list_count / schedule_num_units;
    const uint64_t schedule_work_extra = edge_list_count % schedule_num_units;
    uint64_t* const schedule_dynamic_counter = graph_scheduler_dynamic_counter_numa[spindleGetTaskID()];
    mergeaccum_t* const merge_buffer_base = graph_vertex_merge_buffer_baseptr_numa[spindleGetTaskID()];
    __m128d total_rank = _mm_setzero_pd();
    
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
        __m256d vertex_accumulator = _mm256_setzero_pd();
        uint64_t prev_destination_vertex = extract_destination_vertex(_mm256_and_si256(_mm256_load_si256(&edge_list[schedule_unit_base]), vec_mask_destination_vertex));
        merge_buffer_base[schedule_current_unit].initial_vertex_id = prev_destination_vertex;
        
        for (uint64_t i = schedule_unit_base; i < schedule_unit_max; ++i)
        {
            // load the vector
            const __m256i edgevec = _mm256_load_si256(&edge_list[i]);
            
            // prefetch ahead in the edge list
            _mm_prefetch((char*)(&edge_list[i + 8]), _MM_HINT_NTA);
            
            // extract the various quantities from the edge vector using bitwise-AND operations
            const __m256i edgevec_destination_vertex = _mm256_and_si256(edgevec, vec_mask_destination_vertex);
            const __m256i edgevec_source_vertices = _mm256_and_si256(edgevec, vec_mask_source_vertices);
            const __m256i edgevec_valid_bits = _mm256_and_si256(edgevec, vec_mask_valid_bits);
            
            // gather from the source vertices
            const __m256d source_vertex_properties = _mm256_mask_i64gather_pd(_mm256_setzero_pd(), &graph_vertex_props[0].d, edgevec_source_vertices, _mm256_castsi256_pd(edgevec_valid_bits), sizeof(graph_vertex_props[0].d));
            
            // extract the destination vertex identifier
            const uint64_t destination_vertex = extract_destination_vertex(edgevec_destination_vertex);
            
            // if there is a change to a new destination vertex, potentially update the values in memory
            if (prev_destination_vertex < destination_vertex)
            {
                // horizontally sum up the vertex accumulator and write the result to the master accumulator data structure
                const __m256d temp1 = _mm256_hadd_pd(vertex_accumulator, vertex_accumulator);
                const __m128d temp2 = _mm256_extractf128_pd(temp1, 1);
                const __m128d new_partial_value = _mm_add_pd(_mm256_castpd256_pd128(temp1), temp2);
                
                graph_vertex_accumulators[prev_destination_vertex].u = _mm_cvtsi128_si64x(_mm_castpd_si128(new_partial_value));
                total_rank = _mm_add_pd(total_rank, new_partial_value);
                
                // advance to the new vertex
                vertex_accumulator = _mm256_setzero_pd();
            }
            
            prev_destination_vertex = destination_vertex;
            
            // record the just-gathered values into the accumulator for the present vertex
            vertex_accumulator = _mm256_add_pd(vertex_accumulator, source_vertex_properties);
        }
        
        // write to the merge buffer for the current unit of work
        merge_buffer_base[schedule_current_unit].final_vertex_id = prev_destination_vertex;
        
        {
            // horizontally sum up the vertex accumulator and write the result to the master accumulator data structure
            const __m256d temp1 = _mm256_hadd_pd(vertex_accumulator, vertex_accumulator);
            const __m128d temp2 = _mm256_extractf128_pd(temp1, 1);
            const __m128d new_partial_value = _mm_add_pd(_mm256_castpd256_pd128(temp1), temp2);
            
            merge_buffer_base[schedule_current_unit].final_partial_value.u = _mm_cvtsi128_si64x(_mm_castpd_si128(new_partial_value));
            total_rank = _mm_add_pd(total_rank, new_partial_value);
        }
    }
    
    // write the total distributed rank to the global buffer
    graph_reduce_buffer[spindleGetGlobalThreadID()] = _mm_cvtsi128_si64x(_mm_castpd_si128(total_rank));
}
