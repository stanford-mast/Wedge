/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* phase_vertex_pr.c
*      Implementation of the Vertex phase for PageRank.
*****************************************************************************/

#include "constants.h"
#include "graphdata.h"
#include "intrinhelper.h"
#include "phases.h"

#include <spindle.h>
#include <stdint.h>


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "phases.h" for documentation.

void perform_vertex_phase_pr(const uint64_t vertex_start, const uint64_t vertex_count)
{
    // initialize per-thread variables
    vertexprop_t* const vertex_prop_base = &graph_vertex_props[vertex_start];
    vertexprop_t* const vertex_accumulators_base = &graph_vertex_accumulators[vertex_start];
    vertexprop_t* const vertex_outdegrees_base = &graph_vertex_outdegrees[vertex_start];
    
    // load the number of vertices to an AVX vector
    const __m256d vec_num_vertices = _mm256_set1_pd((double)graph_num_vertices);
    
    // load the damping factor
    const __m256d vec_damping = const_damping_factor;
    
    // calculate (1 - damping) / numvertices, a constant offset factor
    const __m256d vec_1_minus_d_by_v = _mm256_div_pd(_mm256_sub_pd(_mm256_set1_pd(1.0), vec_damping), vec_num_vertices);
    
    // compute the PageRank correction factor, which is added to all ranks to account for sink vertices (i.e. vertices with outdegree 0)
    // this is (1 - total_rank) / numvertices, where total_rank is the total distributed PageRank from the Edge phase
    // the value of total_rank was written in parts to the reduce buffer during the Edge phase
    const __m256d vec_total_rank = _mm256_set1_pd(phase_op_combine_global_var_from_buf_double((double*)graph_reduce_buffer));
    const __m256d vec_correction_factor = _mm256_div_pd(_mm256_sub_pd(_mm256_set1_pd(1.0), vec_total_rank), vec_num_vertices);
    
    // initialize per-thread variables
    const uint64_t schedule_num_units = (uint64_t)spindleGetLocalThreadCount();
    const uint64_t schedule_num_iterations = (vertex_count >> 2ull) + ((vertex_count & 3ull) ? 1ull : 0ull);
    const uint64_t schedule_work_per_unit = schedule_num_iterations / schedule_num_units;
    const uint64_t schedule_work_extra = schedule_num_iterations % schedule_num_units;
    
    // the work is extremely regular, so each thread gets a single chunk of work statically assigned to it
    const uint64_t schedule_current_unit = (uint64_t)spindleGetLocalThreadID();
    const uint64_t schedule_unit_base = ((schedule_work_per_unit * schedule_current_unit) + ((schedule_current_unit < schedule_work_extra) ? schedule_current_unit : schedule_work_extra));
    const uint64_t schedule_unit_max = (schedule_unit_base + schedule_work_per_unit + ((schedule_current_unit < schedule_work_extra) ? 1ull : 0ull));
    
    for (uint64_t i = schedule_unit_base; i < schedule_unit_max; i += 1ull)
    {
        // load the vertex outdegrees
        const __m256d vec_raw_outdegrees = _mm256_castsi256_pd(_mm256_stream_load_si256((__m256i*)&vertex_outdegrees_base[i << 2ull]));
        
        // compare each with 0 and, if equal, replace the element with the number of vertices in the graph
        const __m256d vec_outdegrees_equal_zero = _mm256_castsi256_pd(_mm256_cmpeq_epi64(_mm256_castpd_si256(vec_raw_outdegrees), _mm256_setzero_si256()));
        const __m256d vec_outdegrees = _mm256_blendv_pd(vec_raw_outdegrees, vec_num_vertices, vec_outdegrees_equal_zero);
        
        // load from the vertex accumulator
        const __m256d vec_accumulators = _mm256_castsi256_pd(_mm256_load_si256((__m256i*)&vertex_accumulators_base[i << 2ull]));
        
        // add the sink correction factor, multiply by damping, add the constant ((1 - damping) / numvertices), and divide by the outdegree
        const __m256d vec_new_ranks = _mm256_div_pd(_mm256_add_pd(_mm256_mul_pd(_mm256_add_pd(vec_accumulators, vec_correction_factor), vec_damping), vec_1_minus_d_by_v), vec_outdegrees);
        
        // write the new ranks to the master vertex properties
        _mm256_stream_pd(&vertex_prop_base[i << 2ull].d, vec_new_ranks);
    }
}
