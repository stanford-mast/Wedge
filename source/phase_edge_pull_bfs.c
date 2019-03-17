/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* phase_edge_pull_bfs.c
*      Implementation of the Edge-Pull phase for Breadth-First Search.
*****************************************************************************/

#include "configuration.h"
#include "constants.h"
#include "graphdata.h"
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

// atomically clear the specified bit in the specified bitmask
static inline void clear_bit(void* const bitmask, const uint64_t bitindex)
{
    uint8_t* const addr = &(((uint8_t*)bitmask)[bitindex >> 3ull]);
    const uint8_t mask = ~(1 << ((uint8_t)bitindex & 7));
    
    __asm__("lock and BYTE PTR [%1], %2" : "=m"(*addr) : "r"(addr), "r"(mask));
}

// test the specified bit in the specified bitmask, returning nonzero if it is set, 0 if not
static inline uint8_t test_bit(void* const bitmask, const uint64_t bitindex)
{
    uint8_t* const addr = &(((uint8_t*)bitmask)[bitindex >> 3ull]);
    const uint8_t mask = (1 << ((uint8_t)bitindex & 7));
    
    return (*addr & mask);
}

// gather the specified four bits into the upper-most bit positions of the resulting vector (all other bits are 0)
// gather operation is subject to mask bits in the cond parameter, any unset bits get 0 in the result
static inline __m256i gather_test_bits(void* const bitmask, const __m256i bitindex, const __m256i cond)
{
    // read from the correct elements in the bitmask
    // anything not read immediately gets set to 0
    const __m256i bitmask_elements = _mm256_mask_i64gather_epi64(_mm256_setzero_si256(), (long long int*)bitmask, _mm256_srli_epi64(bitindex, 6), cond, sizeof(uint64_t));
    
    // compute the bit position within each element
    const __m256i bitmask_positions = _mm256_and_si256(bitindex, _mm256_set1_epi64x(63ll));
    
    // place the specified bits in the upper-most bit positions of the result
    // shift the gather result right by the specified bit position, then shift left by 63
    return _mm256_slli_epi64(_mm256_srlv_epi64(bitmask_elements, bitmask_positions), 63);
}

// searches the specified bitmask starting with the specified bit position for the next bit that is set and returns its bit position
// the number of 64-bit elements that make up the bitmask is passed as a parameter
// if no set bits are found prior to the end of the bitmask, returns UINT64_MAX
static uint64_t find_next_set_bit(const uint64_t* const bitmask, const uint64_t startbitindex, const uint64_t count64)
{
    const uint64_t initialmask = (~((2ull << (startbitindex & 63ull)) - 1ull));
    
    uint64_t current64 = startbitindex >> 6ull;
    uint64_t currentbits = bitmask[current64] & initialmask;
    
    while (0ull == currentbits)
    {
        current64 += 1ull;
        if (current64 == count64)
            return UINT64_MAX;
        
        currentbits = bitmask[current64];
    }
    
    return (current64 << 6ull) + _tzcnt_u64(currentbits);
}


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "phases.h" for documentation.

void perform_edge_pull_phase_bfs(const __m256i* const edge_list, const uint64_t edge_list_count)
{
    // load required buffer pointers
    const uint64_t* const edge_index = graph_edges_gather_list_index_numa[spindleGetTaskID()];
    
    // load common constants
    const __m256i vec_mask_destination_vertex = const_vid_and_mask;
    const __m256i vec_mask_source_vertices = const_edge_list_and_mask;
    const __m256i vec_mask_valid_bits = const_edge_mask_and_mask;
    
    // initialize per-thread variables
    const uint64_t schedule_num_units = (uint64_t)spindleGetLocalThreadCount() << 5ull;
    const uint64_t schedule_work_per_unit = edge_list_count / schedule_num_units;
    const uint64_t schedule_work_extra = edge_list_count % schedule_num_units;
    uint64_t* const schedule_dynamic_counter = graph_scheduler_dynamic_counter_numa[spindleGetTaskID()];
    uint64_t num_edges_changed = 0ull;
    uint64_t last_active_vector_found = 0ull;
    
    // initialize shared variables with a single thread
    if (0 == spindleGetLocalThreadID())
        *schedule_dynamic_counter = spindleGetLocalThreadCount();
    
    spindleBarrierLocal();
    
    // iterate through this thread's dynamically-assigned units of work
    for (uint64_t schedule_current_unit = (uint64_t)spindleGetLocalThreadID(); schedule_current_unit < schedule_num_units; schedule_current_unit = __sync_fetch_and_add(schedule_dynamic_counter, 1ull))
    {
        const uint64_t schedule_unit_base = (schedule_work_per_unit * schedule_current_unit) + ((schedule_current_unit < schedule_work_extra) ? schedule_current_unit : schedule_work_extra);
        const uint64_t schedule_unit_max = schedule_unit_base + schedule_work_per_unit + ((schedule_current_unit < schedule_work_extra) ? 1ull : 0ull);
        
        // if, during a search of the unconverged vertex frontier, the next possible vector is beyond the end of this unit of work, skip this unit of work
        if (last_active_vector_found > schedule_unit_max)
            continue;
        
        // if, during a search of the unconverged vertex frontier, the next possible vector is somewhere in this unit of work, skip right to it
        const uint64_t schedule_unit_start = (last_active_vector_found > schedule_unit_base) ? last_active_vector_found : schedule_unit_base;
        
        for (uint64_t i = schedule_unit_start; i < schedule_unit_max; ++i)
        {
            // load the vector
            const __m256i edgevec = _mm256_load_si256(&edge_list[i]);
            
            // prefetch ahead in the edge list
            _mm_prefetch((char*)(&edge_list[i + 8]), _MM_HINT_NTA);
            
            // extract the various quantities from the edge vector using bitwise-AND operations
            const __m256i edgevec_destination_vertex = _mm256_and_si256(edgevec, vec_mask_destination_vertex);
            const __m256i edgevec_source_vertices = _mm256_and_si256(edgevec, vec_mask_source_vertices);
            const __m256i edgevec_valid_bits = _mm256_and_si256(edgevec, vec_mask_valid_bits);
            
            // extract the destination vertex identifier
            const uint64_t destination_vertex = extract_destination_vertex(edgevec_destination_vertex);
            
            // verify that the destination vertex identifier is still unvisited
            if (UINT64_MAX == graph_vertex_props[destination_vertex].u)
            {
                // vertex has not been visited
                // see if the present vector contains a valid BFS parent by checking if source vertices are active this iteration
                const __m256i sources_valid = gather_test_bits(graph_frontier_active_vertices_snapshot, edgevec_source_vertices, edgevec_valid_bits);
                
                // stop here if none of the sources are valid
                if (_mm256_testz_pd(_mm256_castsi256_pd(sources_valid), _mm256_castsi256_pd(sources_valid)))
                    continue;
                
                // filter out impossible parents by using the valid bits to create a mask for bitwise-AND
                const __m256i possible_parents = _mm256_and_si256(_mm256_cmpgt_epi64(_mm256_setzero_si256(), sources_valid), edgevec_source_vertices);
                
                // pick any valid parent; use max so that the above filtering impossible parents by replacement with 0 is algorithmically correct
                // write the selected parent to the master vertex properties
                const __m128d temp1 = _mm_max_pd(_mm_castsi128_pd(_mm256_extractf128_si256(possible_parents, 1)), _mm256_castpd256_pd128(_mm256_castsi256_pd(possible_parents)));
                const __m128d temp2 = _mm_max_pd(temp1, _mm_castsi128_pd(_mm_srli_si128(_mm_castpd_si128(temp1), 8)));
                graph_vertex_props[destination_vertex].u = _mm_extract_epi64(_mm_castpd_si128(temp2), 0);
                
                // mark the vertex as active for the next iteration and converged effective immediately
                set_bit(graph_frontier_active_vertices, destination_vertex);
                clear_bit(graph_frontier_unconverged_vertices, destination_vertex);
                
                // increase the number of edges changed, for convergence detection
                num_edges_changed += graph_vertex_outdegrees[destination_vertex].u;
            }
            else
            {
                // vertex has been visited and is no longer unconverged
                // find the next unvisited vertex and consult the index to figure out where the next active vector is located
                const uint64_t next_unconverged_vertex = find_next_set_bit(graph_frontier_unconverged_vertices, destination_vertex, graph_frontier_count);
                
                if (next_unconverged_vertex >= graph_num_vertices)
                {
                    // this phase is done, so bail early
                    graph_reduce_buffer[spindleGetGlobalThreadID()] = num_edges_changed;
                    return;
                }
                
                last_active_vector_found = edge_index[next_unconverged_vertex];
                i = last_active_vector_found - 1ull;
            }
        }
    }
    
    // write the number of updated vertices to the global buffer
    graph_reduce_buffer[spindleGetGlobalThreadID()] = num_edges_changed;
}

void perform_edge_pull_phase_bfs_with_wedge_frontier(const __m256i* const edge_list, const uint64_t edge_list_count, uint64_t* const wedge_frontier, const uint64_t wedge_frontier_count)
{
    // load required buffer pointers
    const uint64_t* const edge_index = graph_edges_gather_list_index_numa[spindleGetTaskID()];
    
    // load common constants
    const __m256i vec_mask_destination_vertex = const_vid_and_mask;
    const __m256i vec_mask_source_vertices = const_edge_list_and_mask;
    const __m256i vec_mask_valid_bits = const_edge_mask_and_mask;
    
    // initialize per-thread variables
    const uint64_t schedule_num_units = (uint64_t)spindleGetLocalThreadCount() << 5ull;
    const uint64_t schedule_work_per_unit = (wedge_frontier_count >> 3ull) / schedule_num_units;
    const uint64_t schedule_work_extra = (wedge_frontier_count >> 3ull) % schedule_num_units;
    uint64_t* const schedule_dynamic_counter = graph_scheduler_dynamic_counter_numa[spindleGetTaskID()];
    uint64_t num_edges_changed = 0ull;
    uint64_t last_active_vector_found = 0ull;
    
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
            const uint64_t frontier_offset_start = (0 != _mm256_testc_si256(_mm256_setzero_si256(), *((__m256i*)(&wedge_frontier[frontier_base_idx])))) ? 4ull : 0ull;
            const uint64_t frontier_offset_end = (0 != _mm256_testc_si256(_mm256_setzero_si256(), *((__m256i*)(&wedge_frontier[frontier_base_idx + 4ull])))) ? 4ull : 8ull;
            
            for (uint64_t j = frontier_offset_start; j < frontier_offset_end; ++j)
            {
                const uint64_t frontier_idx = frontier_base_idx + j;
                
                uint64_t frontier_bits = wedge_frontier[frontier_idx];
                if (0ull == frontier_bits)
                    continue;
                
                wedge_frontier[frontier_idx] = 0ull;
                
                if (((frontier_idx + 64ull) << (6ull + CONFIG_LOG2_FRONTIER_PRECISION)) < last_active_vector_found)
                    continue;
                
                for (; 0ull != frontier_bits; frontier_bits = _blsr_u64(frontier_bits))
                {
                    // compute the vector index
                    const uint64_t vector_index = (frontier_idx << (6ull + CONFIG_LOG2_FRONTIER_PRECISION)) + (_tzcnt_u64(frontier_bits) << (0ull + CONFIG_LOG2_FRONTIER_PRECISION));
                    
                    for (uint64_t k = 0ull; k < (1ull << CONFIG_LOG2_FRONTIER_PRECISION); ++k)
                    {
                        if ((vector_index + k) < last_active_vector_found)
                            continue;
                        
                        // load the vector
                        const __m256i edgevec = _mm256_load_si256(&edge_list[vector_index + k]);
                        
                        // extract the various quantities from the edge vector using bitwise-AND operations
                        const __m256i edgevec_destination_vertex = _mm256_and_si256(edgevec, vec_mask_destination_vertex);
                        const __m256i edgevec_source_vertices = _mm256_and_si256(edgevec, vec_mask_source_vertices);
                        const __m256i edgevec_valid_bits = _mm256_and_si256(edgevec, vec_mask_valid_bits);
                        
                        // extract the destination vertex identifier
                        const uint64_t destination_vertex = extract_destination_vertex(edgevec_destination_vertex);
                        
                        // verify that the destination vertex identifier is still unvisited
                        if (UINT64_MAX == graph_vertex_props[destination_vertex].u)
                        {
                            // vertex has not been visited
                            // see if the present vector contains a valid BFS parent by checking if source vertices are active this iteration
                            const __m256i sources_valid = gather_test_bits(graph_frontier_active_vertices_snapshot, edgevec_source_vertices, edgevec_valid_bits);
                            
                            // stop here if none of the sources are valid
                            if (_mm256_testz_pd(_mm256_castsi256_pd(sources_valid), _mm256_castsi256_pd(sources_valid)))
                                continue;
                            
                            // filter out impossible parents by using the valid bits to create a mask for bitwise-AND
                            const __m256i possible_parents = _mm256_and_si256(_mm256_cmpgt_epi64(_mm256_setzero_si256(), sources_valid), edgevec_source_vertices);
                            
                            // pick any valid parent; use max so that the above filtering impossible parents by replacement with 0 is algorithmically correct
                            // write the selected parent to the master vertex properties
                            const __m128d temp1 = _mm_max_pd(_mm_castsi128_pd(_mm256_extractf128_si256(possible_parents, 1)), _mm256_castpd256_pd128(_mm256_castsi256_pd(possible_parents)));
                            const __m128d temp2 = _mm_max_pd(temp1, _mm_castsi128_pd(_mm_srli_si128(_mm_castpd_si128(temp1), 8)));
                            graph_vertex_props[destination_vertex].u = _mm_extract_epi64(_mm_castpd_si128(temp2), 0);
                            
                            // mark the vertex as active for the next iteration and converged effective immediately
                            set_bit(graph_frontier_active_vertices, destination_vertex);
                            clear_bit(graph_frontier_unconverged_vertices, destination_vertex);
                            
                            // increase the number of edges changed, for convergence detection
                            num_edges_changed += graph_vertex_outdegrees[destination_vertex].u;
                        }
                        else
                        {
                            // vertex has been visited
                            // find the next unvisited vertex and consult the index to figure out where the next active vector is located
                            const uint64_t next_unconverged_vertex = find_next_set_bit(graph_frontier_unconverged_vertices, destination_vertex, graph_frontier_count);
                            
                            if (next_unconverged_vertex >= graph_num_vertices)
                            {
                                // this phase is done, so bail early
                                graph_reduce_buffer[spindleGetGlobalThreadID()] = num_edges_changed;
                                return;
                            }
                            
                            last_active_vector_found = edge_index[next_unconverged_vertex];
                        }
                    }
                }
            }
        }
    }
    
    // write the number of updated vertices to the global buffer
    graph_reduce_buffer[spindleGetGlobalThreadID()] = num_edges_changed;
}
