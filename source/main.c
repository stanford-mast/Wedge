/*****************************************************************************
* Wedge
*      High performance, hardware-optimized graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* main.c
*      Program entry point. Causes command-line options to be parsed and
*      orchestrates the execution of the rest of the program.
*****************************************************************************/

#include "application.h"
#include "benchmark.h"
#include "cmdline.h"
#include "execution.h"
#include "graphdata.h"
#include "hwexperiments.h"
#include "versioninfo.h"

#include <spindle.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>


/* -------- FUNCTIONS ------------------------------------------------------ */

// Program entry point.
int main(int argc, char* argv[])
{
    const cmdline_opts_t* cmdline_settings = NULL;
    
    // parse command-line options
    cmdline_parse_options_or_die(argc, argv);
    cmdline_settings = cmdline_get_current_settings();
    
#ifdef EXPERIMENT_STR
    // identify any active experiments
    printf("Experiments: %s\n", EXPERIMENT_STR);
#endif

#ifdef ENABLE_HARDWARE_EXPERIMENTS
    hwexperiments_initialize();
#endif
    
    // print out some frontier configuration information
    printf("Frontier:    precision set to %llu edge vector%s/bit\n", (long long unsigned int)CONFIG_FRONTIER_PRECISION, (1 == CONFIG_FRONTIER_PRECISION) ? "" : "s");
    printf("Frontier:    threshold set to %llu%%\n", (long long unsigned int)cmdline_settings->frontier_threshold_pct);
    
    // attempt to set the application
    if (true == application_select(cmdline_settings->application))
    {
        printf("Application: %s with %llu thread%s on NUMA node%s %u", execution_app_name, (long long unsigned int)cmdline_settings->num_threads, (cmdline_settings->num_threads > 1 ? "s" : ""), (cmdline_settings->num_numa_nodes > 1 ? "s" : ""), (unsigned int)cmdline_settings->numa_nodes[0]);
        
        for (uint32_t i = 1; i < cmdline_settings->num_numa_nodes; ++i)
            printf(", %u", (unsigned int)cmdline_settings->numa_nodes[i]);
    
        printf("\n");
    }
    else
    {
        printf("Application: ERROR: unable to run \"%s\"\n", cmdline_settings->application);
        return __LINE__;
    }
    
    // load the input graph
    if (false == graph_data_read_from_file(cmdline_settings->graph_input_filename, cmdline_settings->num_numa_nodes, cmdline_settings->numa_nodes))
        return __LINE__;
    
    // initialize the remaining data structures
    graph_data_initialize_data_structures(cmdline_settings->num_threads, cmdline_settings->num_numa_nodes, cmdline_settings->numa_nodes);
    
    // initialize execution parameters from the command-line
    graph_param_num_static_iterations = cmdline_settings->num_iterations;
    graph_param_root_vertex = cmdline_settings->root_vertex;
    graph_param_frontier_threshold_pct = cmdline_settings->frontier_threshold_pct;
    
    if (graph_param_root_vertex >= graph_num_vertices)
    {
        printf("Application: WARNING: root vertex %llu is out of range, using %llu instead\n", (long long unsigned int)graph_param_root_vertex, (long long unsigned int)CMDLINE_DEFAULT_ROOT_VERTEX);
        graph_param_root_vertex = (uint64_t)CMDLINE_DEFAULT_ROOT_VERTEX;
    }
    
    // run the application
    printf("Application: starting execution\n");
    
    {
        SSpindleTaskSpec execution_task_spec[4];
        
        for (uint32_t i = 0; i < cmdline_settings->num_numa_nodes; ++i)
        {
            execution_task_spec[i].func = &application_execute;
            execution_task_spec[i].arg = NULL;
            execution_task_spec[i].numaNode = cmdline_settings->numa_nodes[i];
            execution_task_spec[i].numThreads = cmdline_settings->num_threads / cmdline_settings->num_numa_nodes;
            execution_task_spec[i].smtPolicy = SpindleSMTPolicyPreferPhysical;
        }
        
        spindleThreadsSpawn(&execution_task_spec[0], cmdline_settings->num_numa_nodes, true);
    }
    
    printf("Application: completed execution\n");
    
#ifdef ENABLE_HARDWARE_EXPERIMENTS
    hwexperiments_uninitialize();
#endif
    
    // write application output if requested
    if (NULL != cmdline_settings->graph_application_output_filename)
    {
        FILE* output_file = NULL;
        
        output_file = fopen(cmdline_settings->graph_application_output_filename, "w");
        
        if (NULL != output_file)
        {
            double output_time = 0.0;
            printf("Application: writing output to \"%s\"\n", cmdline_settings->graph_application_output_filename);
            
            benchmark_start(&output_time);
            
            for (uint64_t i = 0ull; (i < graph_num_vertices) && (!ferror(output_file)); ++i)
            {
                char vertex_value_string[256];
                
                execution_get_vertex_prop_string(i, vertex_value_string, sizeof(vertex_value_string));
                fprintf(output_file, "%llu %s\n", (long long unsigned int)i, vertex_value_string);
            }
            
            if (ferror(output_file))
            {
                printf("Application: ERROR: file I/O error during write\n");
            }
            else
            {
                output_time = benchmark_stop(&output_time);
                printf("Application: writing completed in %.2lf msec\n", output_time);
            }
            
            fclose(output_file);
        }
        else
        {
            printf("Application: ERROR: unable to write to \"%s\"\n", cmdline_settings->graph_application_output_filename);
        }
    }
    
    // print statistics
    {
        const double total_processing_time = total_processing_time_nowedge + total_processing_time_wedge;
        const double running_time = total_processing_time + total_activation_time;
        
        printf("\n--------------- EXECUTION STATISTICS ---------------\n");
        printf("%-28s = %.2lf msec\n", "Running Time", running_time);
        printf("%-28s = %.4lf Bedges/sec\n", "Effective Throughput", (double)graph_num_edges * (double)(total_iterations_executed) / (double)running_time / 1000000.0);
        printf("----------------------------------------------------\n");
        printf("%-28s = %.2lf msec\n", "Pull Time", total_processing_time);
        printf("%-28s = %.2lf msec\n", "Frontier Pull Time", total_processing_time_wedge);
        printf("%-28s = %.2lf msec\n", "No Frontier Pull Time", total_processing_time_nowedge);
        printf("%-28s = %.2lf msec\n", "Wedge Time", total_activation_time);
        printf("----------------------------------------------------\n");
        printf("%-28s = %llu\n", "Total Iterations", (long long unsigned int)total_iterations_executed);
        printf("%-28s = %llu\n", "Iterations with Wedge", (long long unsigned int)total_wedge_iterations_executed);
        printf("%-28s = %llu\n", "Iterations without Wedge", (long long unsigned int)(total_iterations_executed - total_wedge_iterations_executed));
       
        printf("----------------------------------------------------\n");
        
        if (execution_has_special_output_stat)
        {
            char special_stat_value_string[28];
            
            execution_special_stat_string_pr(special_stat_value_string, sizeof(special_stat_value_string));
            printf("%-28s = %s\n", execution_special_stat_name(), special_stat_value_string);
            printf("----------------------------------------------------\n");
        }
        
        printf("\n");
    }
    
    return 0;
}
