/*****************************************************************************
* Wedge
*      High performance, hardware-optimized graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* cmdline.c
*      Implementation of functions for parsing command-line arguments.
*****************************************************************************/

#include "cmdline.h"
#include "configuration.h"
#include "versioninfo.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <topo.h>


/* -------- PLATFORM-SPECIFIC MACROS --------------------------------------- */

#ifdef GRAZELLE_WINDOWS
#define CMDLINE_SWITCH_CHAR                     '/'
#define CMDLINE_HELP_OPTION                     '?'
#else
#define CMDLINE_SWITCH_CHAR                     '-'
#define CMDLINE_HELP_OPTION                     'h'
#endif


/* -------- LOCALS --------------------------------------------------------- */

static cmdline_opts_t cmdline_opts;


/* -------- INTERNAL FUNCTIONS --------------------------------------------- */

// Determines if the specified character is acceptable as a command-line switch starting character.
// The accepted command-line switch starting characters vary by platform, but '-' is always accepted.
// Returns 0 for NO, 1 for YES.
uint32_t cmdline_helper_is_char_supported_as_switch(char check)
{
    switch (check)
    {
    case '-':
#ifdef GRAZELLE_WINDOWS
    case '/':
#endif
        return 1;
    
    default:
        return 0;
    }
}

// Determines if the specified option is recognized.
// May vary by platform, depending on the option's semantic meaning.
// Returns 0 for NO, 1 for YES.
uint32_t cmdline_helper_is_recognized_option(char check)
{
    switch (check)
    {
    case 'a':
    case 'h':
    case 'i':
    case 'n':
    case 'N':
    case 'o':
    case 'r':
    case 't':
    case 'V':
	case 'u':
#ifdef GRAZELLE_WINDOWS
    case '?':
#endif
        return 1;

    default:
        return 0;
    }
}

// Determines if the specified option requires a value.
// Returns 0 for NO, 1 for YES.
uint32_t cmdline_helper_option_requires_value(char check)
{
    switch (check)
    {
    case 'a':
    case 'i':
    case 'n':
    case 'N':
	case 'u':
    case 'o':
    case 'r':
    case 't':
        return 1;

    default:
        return 0;
    }
}

// Determines if the specified option can accept a value.
// Returns 0 for NO, 1 for YES.
uint32_t cmdline_helper_option_accepts_value(char check)
{
    switch (check)
    {
    case 'h':
#ifdef GRAZELLE_WINDOWS
    case '?':
#endif
        return 0;

    default:
        return cmdline_helper_option_requires_value(check);
    }
}

// Prints an error about an unknown command line option and exits the program.
void cmdline_helper_print_error_unknown_option_and_exit(char* argv0, char* cmdline_option)
{
    printf("%s: Unrecognized option `%s'.\n", argv0, cmdline_option);
    printf("Try `%s %c%c' for more information.\n", argv0, CMDLINE_SWITCH_CHAR, CMDLINE_HELP_OPTION);
    
    exit(1);
}

// Prints an error about an unknown command line value and exits the program.
void cmdline_helper_print_error_invalid_value_and_exit(char* argv0, char* cmdline_option, char* cmdline_value)
{
    printf("%s: Invalid value `%s' for option `%s'.\n", argv0, cmdline_value, cmdline_option);
    printf("Try `%s %c%c' for more information.\n", argv0, CMDLINE_SWITCH_CHAR, CMDLINE_HELP_OPTION);
    
    exit(2);
}

// Prints an error about an unknown command line value and exits the program.
void cmdline_helper_print_error_missing_value_and_exit(char* argv0, char* cmdline_option)
{
    printf("%s: Missing argument for option `%s'.\n", argv0, cmdline_option);
    printf("Try `%s %c%c' for more information.\n", argv0, CMDLINE_SWITCH_CHAR, CMDLINE_HELP_OPTION);
    
    exit(3);
}

// Prints an error about an extraneous command line value and exits the program.
void cmdline_helper_print_error_extraneous_value_and_exit(char* argv0, char* cmdline_option)
{
    printf("%s: Option `%s' does not accept an argument.\n", argv0, cmdline_option);
    printf("Try `%s %c%c' for more information.\n", argv0, CMDLINE_SWITCH_CHAR, CMDLINE_HELP_OPTION);
    
    exit(4);
}

// Prints an error about a missing command line option and exits the program.
void cmdline_helper_print_error_missing_option_and_exit(char* argv0, char* cmdline_option)
{
    printf("%s: Missing required option `%c%s'.\n", argv0, CMDLINE_SWITCH_CHAR, cmdline_option);
    printf("Try `%s %c%c' for more information.\n", argv0, CMDLINE_SWITCH_CHAR, CMDLINE_HELP_OPTION);
    
    exit(5);
}

// Prints an error about an imcompatible combination of options and exits the program.
void cmdline_helper_print_error_incompatible_options_and_exit(char* argv0)
{
    printf("%s: Specified option combination is incompatible.\n", argv0);
    printf("Try `%s %c%c' for more information.\n", argv0, CMDLINE_SWITCH_CHAR, CMDLINE_HELP_OPTION);
    
    exit(6);
}

// Prints version information and exits the program.
void cmdline_helper_print_version_and_exit()
{
    printf("%s v%s for %s\nCompiled on %s at %s\n", GRAZELLE_PROGRAM_NAME, GRAZELLE_PROGRAM_VERSION, GRAZELLE_PLATFORM_NAME, __DATE__, __TIME__);
    
    exit(0);
}

// Prints usage information and exits the program.
void cmdline_helper_print_usage_and_exit(char* argv0)
{
    printf("Usage: %s [options] %ca application-name %ci input-graph\n", argv0, CMDLINE_SWITCH_CHAR, CMDLINE_SWITCH_CHAR);
    if (cmdline_helper_is_recognized_option('h'))
    {
        if (cmdline_helper_is_recognized_option('?'))
        {
            printf("       %s %ch | %c?\n", argv0, CMDLINE_SWITCH_CHAR, CMDLINE_SWITCH_CHAR);
        }
        else
        {
            printf("       %s %ch\n", argv0, CMDLINE_SWITCH_CHAR);
        }
    }
    if (cmdline_helper_is_recognized_option('V'))
    {
        printf("       %s %cV\n", argv0, CMDLINE_SWITCH_CHAR);
    }
    printf("\n");
    printf("Required:\n");
    
    if (cmdline_helper_is_recognized_option('a'))
    {
        printf("  %ca application-name\n", CMDLINE_SWITCH_CHAR);
        printf("        Name of the application to execute.\n");
    }
    
    if (cmdline_helper_is_recognized_option('i'))
    {
        printf("  %ci input-graph\n", CMDLINE_SWITCH_CHAR);
        printf("        Path of the file containing the input graph.\n");
    }
    
    printf("\n");
    printf("Options:\n");
    
    if (cmdline_helper_is_recognized_option('h'))
    {
        if (cmdline_helper_is_recognized_option('?'))
        {
            printf("  %ch | %c?\n", CMDLINE_SWITCH_CHAR, CMDLINE_SWITCH_CHAR);
        }
        else
        {
            printf("  %ch\n", CMDLINE_SWITCH_CHAR);
        }
        printf("        Prints this information and exits.\n");
    }
    
    if (cmdline_helper_is_recognized_option('n'))
    {
        printf("  %cn num-threads\n", CMDLINE_SWITCH_CHAR);
        printf("        Number of threads to use when executing.\n");
        printf("        Must be a multiple of the number of NUMA nodes.\n");
        printf("        Specify 0 to use all available threads on the requested NUMA nodes.\n");
        printf("        Defaults to %llu.\n", (long long unsigned int)(CMDLINE_DEFAULT_NUM_THREADS));
    }
    
    if (cmdline_helper_is_recognized_option('N'))
    {
        printf("  %cN num-iterations\n", CMDLINE_SWITCH_CHAR);
        printf("        Number of iterations of the algorithm to execute.\n");
        printf("        Ignored for algorithms that dynamically converge.\n");
        printf("        Defaults to %llu.\n", (long long unsigned int)(CMDLINE_DEFAULT_NUM_ITERATIONS));
    }
    
    if (cmdline_helper_is_recognized_option('o'))
    {
        printf("  %co output-file\n", CMDLINE_SWITCH_CHAR);
        printf("        Path of the file to write as output. Will contain vertex ranks.\n");
    }
	
	if (cmdline_helper_is_recognized_option('r'))
    {
        printf("  %cr root-vertex\n", CMDLINE_SWITCH_CHAR);
        printf("        Root vertex to use for searches and traversals.\n");
        printf("        Useful only for certain algorithms.\n");
        printf("        Defaults to %llu.\n", (long long unsigned int)CMDLINE_DEFAULT_ROOT_VERTEX);
    }
    
    if (cmdline_helper_is_recognized_option('t'))
    {
        printf("  %ct root-vertex\n", CMDLINE_SWITCH_CHAR);
        printf("        Frontier fullness threshold.\n");
        printf("        Percentage of active edges above which Wedge is skipped.\n");
        printf("        Defaults to %llu.\n", (long long unsigned int)CMDLINE_DEFAULT_FRONTIER_THRESHOLD_PCT);
    }
    
    if (cmdline_helper_is_recognized_option('u'))
    {
        printf("  %cu node1[,node2[,node3[...]]]\n", CMDLINE_SWITCH_CHAR);
        printf("        Comma-delimited list of NUMA nodes for worker threads.\n");
		printf("        Worker threads will be distributed across and bound to each NUMA node.\n");
		printf("        Values from 0 to (# NUMA nodes in the system - 1) are accepted.\n");
		printf("        Maximum number of values is min(%llu, # NUMA nodes in the system).\n", (long long unsigned int)CONFIG_NUMA_MAX_NODES);
		printf("        Nodes must be specified in ascending order.\n");
        printf("        Specifying a node multiple times is not allowed.\n");
        printf("        Default behavior is to use only the first NUMA node.\n");
    }
    
    if (cmdline_helper_is_recognized_option('V'))
    {
        printf("  %cV\n", CMDLINE_SWITCH_CHAR);
        printf("        Prints version information and exits.\n");
    }
    
    exit(0);
}

// Parses a single command-line option, returns on success, or prints and terminates on failure.
void cmdline_parse_single_option_or_die(char* argv0, char* cmdline_option, char* cmdline_value)
{
    // Validate the format of the command-line option
    // The first character must be acceptable as a switch starting character and the third must be '\0'
    if (!cmdline_helper_is_char_supported_as_switch(cmdline_option[0]) || '\0' != cmdline_option[2])
    {
        cmdline_helper_print_error_unknown_option_and_exit(argv0, cmdline_option);
    }
    
    // Filter out options that do not properly specify (or omit) a value and those that are not recognized
    if (!cmdline_helper_is_recognized_option(cmdline_option[1]))
    {
        cmdline_helper_print_error_unknown_option_and_exit(argv0, cmdline_option);
    }
    
    if (cmdline_helper_option_requires_value(cmdline_option[1]) && NULL == cmdline_value)
    {
        cmdline_helper_print_error_missing_value_and_exit(argv0, cmdline_option);
    }
    
    if (!cmdline_helper_option_accepts_value(cmdline_option[1]) && NULL != cmdline_value)
    {
        cmdline_helper_print_error_extraneous_value_and_exit(argv0, cmdline_option);
    }
    
    // Handle each individual supported command-line option
    switch (cmdline_option[1])
    {
    case 'h':
    case '?':
        cmdline_helper_print_usage_and_exit(argv0);
        break;
    
    case 'a':
        strncpy(cmdline_opts.application, cmdline_value, (sizeof(cmdline_opts.application) / sizeof(char)) - (1 * sizeof(char)));
        break;
    
    case 'i':
        strncpy(cmdline_opts.graph_input_filename, cmdline_value, (sizeof(cmdline_opts.graph_input_filename) / sizeof(char)) - (1 * sizeof(char)));
        break;
        
    case 'n':
        {
            char* endptr;
            uint32_t cmdline_num_threads = strtoul(cmdline_value, &endptr, 10);
            
            if ('\0' != *endptr)
            {
                cmdline_helper_print_error_invalid_value_and_exit(argv0, cmdline_option, cmdline_value);
            }
            
            cmdline_opts.num_threads = cmdline_num_threads;
        }
        break;
    
    case 'N':
        {
            char* endptr;
            uint32_t cmdline_num_iterations = strtoul(cmdline_value, &endptr, 10);
            
            if ('\0' != *endptr || cmdline_num_iterations < 1)
            {
                cmdline_helper_print_error_invalid_value_and_exit(argv0, cmdline_option, cmdline_value);
            }
            
            cmdline_opts.num_iterations = cmdline_num_iterations;
        }
        break;
    
    case 'o':
        cmdline_opts.graph_application_output_filename = cmdline_value;
        break;
	
	case 'r':
        {
            char* endptr;
            uint64_t cmdline_root_vertex = strtoull(cmdline_value, &endptr, 10);
            
            if ('\0' != *endptr)
            {
                cmdline_helper_print_error_invalid_value_and_exit(argv0, cmdline_option, cmdline_value);
            }
            
            cmdline_opts.root_vertex = cmdline_root_vertex;
        }
        break;
        
    case 't':
        {
            char* endptr;
            uint64_t cmdline_frontier_threshold_pct = strtoull(cmdline_value, &endptr, 10);
            
            if ('\0' != *endptr)
            {
                cmdline_helper_print_error_invalid_value_and_exit(argv0, cmdline_option, cmdline_value);
            }
            
            cmdline_opts.frontier_threshold_pct = cmdline_frontier_threshold_pct;
        }
        break;
    
    case 'u':
        {
            char* endptr = cmdline_value;
            int32_t cmdline_node;
            uint32_t node_idx = 0;
            
            while ('\0' != *endptr)
            {
                if (!isdigit(*endptr) && !cmdline_helper_is_char_supported_as_switch(*endptr))
                {
                    cmdline_helper_print_error_invalid_value_and_exit(argv0, cmdline_option, cmdline_value);
                }
                
                cmdline_node = strtol(endptr, &endptr, 10);
                
                if (cmdline_node < 0 || (uint32_t)cmdline_node >= topoGetSystemNUMANodeCount() || node_idx >= topoGetSystemNUMANodeCount() || ('\0' != *endptr && ',' != *endptr))
                {
                    cmdline_helper_print_error_invalid_value_and_exit(argv0, cmdline_option, cmdline_value);
                }
                
                if ('\0' != *endptr && '\0' != *(endptr + 1))
                {
                    endptr += 1;
                }
                
                cmdline_opts.numa_nodes[node_idx] = (uint32_t)cmdline_node;
                node_idx += 1;
            }
			
			cmdline_opts.num_numa_nodes = node_idx;
            
            for (uint32_t i = 1; i < cmdline_opts.num_numa_nodes; ++i)
            {
                if (cmdline_opts.numa_nodes[i] <= cmdline_opts.numa_nodes[i - 1])
                    cmdline_helper_print_error_invalid_value_and_exit(argv0, cmdline_option, cmdline_value);
            }
        }
        break;
    
    case 'V':
        cmdline_helper_print_version_and_exit();
        break;
    
    default:
        cmdline_helper_print_error_unknown_option_and_exit(argv0, cmdline_option);
        break;
    }
}

// Validates the command-line settings structure, returns on success, or prints and terminates on failure.
void cmdline_validate_or_die(char* argv0)
{
    // Verify that an application has been supplied
    if ('\0' == cmdline_opts.application[0])
    {
        cmdline_helper_print_error_missing_option_and_exit(argv0, "a");
    }
    
    // Verify that an input filename has been supplied
    if ('\0' == cmdline_opts.graph_input_filename[0])
    {
        cmdline_helper_print_error_missing_option_and_exit(argv0, "i");
    }
    
    // Calculate the number of threads if 0 is specified.
    if (0 == cmdline_opts.num_threads)
    {
        cmdline_opts.num_threads = cmdline_opts.num_numa_nodes * topoGetNUMANodeLogicalCoreCount(cmdline_opts.numa_nodes[0]);
    }
    
    // Verify that number of threads is divisible by number of NUMA nodes
    if (0 != cmdline_opts.num_threads % cmdline_opts.num_numa_nodes)
    {
        cmdline_helper_print_error_incompatible_options_and_exit(argv0);
    }
}

// Initializes the command-line settings structure, including setting the default values.
void cmdline_init()
{
    memset((void*)&cmdline_opts, 0, sizeof(cmdline_opts_t));
    for (uint32_t i = 0; i < CONFIG_NUMA_MAX_NODES + 1; ++i)
    {
        cmdline_opts.numa_nodes[i] = i;
    }
    
    // Set the default configuration values
    cmdline_opts.num_threads = CMDLINE_DEFAULT_NUM_THREADS;
	cmdline_opts.num_numa_nodes = CMDLINE_DEFAULT_NUM_NUMA_NODES;
    cmdline_opts.num_iterations = CMDLINE_DEFAULT_NUM_ITERATIONS;
    cmdline_opts.root_vertex = CMDLINE_DEFAULT_ROOT_VERTEX;
    cmdline_opts.frontier_threshold_pct = CMDLINE_DEFAULT_FRONTIER_THRESHOLD_PCT;
}


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "cmdline.h" for documentation.

void cmdline_parse_options_or_die(int argc, char* argv[])
{
    int cmdline_idx = 1;
    
    cmdline_init();
    
    while (cmdline_idx < argc)
    {
        char* cmdline_option = argv[cmdline_idx];
        char* cmdline_value = NULL;
        
        if ((cmdline_idx + 1) < argc)
        {
            char check = argv[cmdline_idx + 1][0];
            
            if (isprint(check) && !cmdline_helper_is_char_supported_as_switch(check))
            {
                cmdline_value = argv[cmdline_idx + 1];
                cmdline_idx += 1;
            }
        }
        
        cmdline_parse_single_option_or_die(argv[0], cmdline_option, cmdline_value);
        
        cmdline_idx += 1;
    }
    
    cmdline_validate_or_die(argv[0]);
}

// ---------

const cmdline_opts_t* const cmdline_get_current_settings()
{
    return &cmdline_opts;
}
