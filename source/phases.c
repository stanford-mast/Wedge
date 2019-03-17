/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* phases.c
*      Implementation of operations used throughout both phases of execution.
*      Note that most of the implementation is in assembly.
*****************************************************************************/

#include "execution.h"
#include "graphtypes.h"
#include "graphdata.h"

#include <spindle.h>
#include <stdint.h>


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "phases.h" for documentation.

uint64_t phase_op_combine_global_var_from_buf(uint64_t* reduce_buffer)
{
    uint64_t value = 0ull;
    
    for (uint32_t i = 0; i < spindleGetGlobalThreadCount(); ++i)
    {
        value += reduce_buffer[i];
    }
    
    return value;
}

// --------

double phase_op_combine_global_var_from_buf_double(double* reduce_buffer)
{
    double value = 0.0;
    
    for (uint32_t i = 0; i < spindleGetGlobalThreadCount(); ++i)
    {
        value += reduce_buffer[i];
    }
    
    return value;
}

// --------

void edge_pull_op_idempotent_merge_with_merge_buffer(mergeaccum_t* merge_buffer, uint64_t count, vertexprop_t* vertex_accumulators, vertexprop_t (*scalar_reduce_op)(vertexprop_t, vertexprop_t))
{
    for (uint64_t i = 0ull; i < count; ++i)
    {
        if (merge_buffer[i].final_vertex_id & 0x8000000000000000ull)
            continue;
        
        // operator is idempotent, so just merge everything in unconditionally
        vertex_accumulators[merge_buffer[i].final_vertex_id] = scalar_reduce_op(vertex_accumulators[merge_buffer[i].final_vertex_id], merge_buffer[i].final_partial_value);
        merge_buffer[i].final_vertex_id = 0x8000000000000000ull;
    }
}

// --------

void edge_pull_op_merge_with_merge_buffer(mergeaccum_t* merge_buffer, uint64_t count, vertexprop_t* vertex_accumulators, vertexprop_t (*scalar_reduce_op)(vertexprop_t, vertexprop_t))
{
    vertexprop_t proposed_value;
    uint64_t last_final_vertex_id = UINT64_MAX;
    
    for (uint64_t i = 0ull; i < count; ++i)
    {
        if (merge_buffer[i].final_vertex_id & 0x8000000000000000ull)
            continue;
        
        if (UINT64_MAX == last_final_vertex_id)
        {
            // no merge buffers have yet been read
            // directly capture the first one encountered
            last_final_vertex_id = merge_buffer[i].final_vertex_id;
            proposed_value = merge_buffer[i].final_partial_value;
        }
        else if (last_final_vertex_id == merge_buffer[i].final_vertex_id)
        {
            // merging for the same vertex as last time
            // just accumulate into the stashed proposed value for that vertex
            proposed_value = scalar_reduce_op(proposed_value, merge_buffer[i].final_partial_value);
        }
        else
        {
            // merging for a different vertex
            // a write is required to the accumulators
            
            // if there are outstanding values for the same vertex as last time, that value would be in the accumulators
            // in this case, a merge is required with the value already present before performing the final write
            if (last_final_vertex_id == merge_buffer[i].initial_vertex_id)
                proposed_value = scalar_reduce_op(proposed_value, vertex_accumulators[last_final_vertex_id]);
            
            vertex_accumulators[last_final_vertex_id] = proposed_value;
            
            // move on to the vertex represented in the current merge buffer
            last_final_vertex_id = merge_buffer[i].final_vertex_id;
            proposed_value = merge_buffer[i].final_partial_value;
        }
        
        merge_buffer[i].final_vertex_id = 0x8000000000000000ull;
    }
    
    if (UINT64_MAX != last_final_vertex_id)
        vertex_accumulators[last_final_vertex_id] = proposed_value;
}
