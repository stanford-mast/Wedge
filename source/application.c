/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* application.c
*      Implementation of application selection functionality.
*****************************************************************************/


#include "application.h"
#include "execution.h"
#include "versioninfo.h"

#include <stdbool.h>
#include <string.h>


/* -------- PLATFORM-SPECIFIC MACROS --------------------------------------- */

#ifdef GRAZELLE_WINDOWS
#define strnicmp                                _strnicmp
#endif

#ifdef GRAZELLE_LINUX
#define strnicmp                                strncasecmp
#endif


/* -------- TYPE DEFINITIONS ----------------------------------------------- */

// Enumerates all supported applications.
typedef enum application_t
{
    APPLICATION_BREADTH_FIRST_SEARCH,
    APPLICATION_CONNECTED_COMPONENTS,
    APPLICATION_PAGERANK,
    APPLICATION_SINGLE_SOURCE_SHORTEST_PATH
} application_t;


/* -------- LOCALS --------------------------------------------------------- */

// Strings accepted as names for Breadth-First Search.
static const char* string_bfs[] = { "bfs", "breadth_first_search", "breadthfirstsearch" };

// Strings accepted as names for Connected Components.
static const char* string_cc[] = { "cc", "connected_components", "connectedcomponents" };

// Strings accepted as names for PageRank.
static const char* string_pr[] = { "pr", "pagerank" };

// Strings accepted as names for Single-Source Shortest Path.
static const char* string_sssp[] = { "sssp", "single_source_shortest_path", "singlesourceshortestpath" };

// Canonical name of Breadth-First Search.
static const char name_bfs[] = "Breadth-First Search";

// Canonical name of Connected Components.
static const char name_cc[] = "Connected Components";

// Canonical name of PageRank.
static const char name_pr[] = "PageRank";

// Canonical name of Single-Source Shortest Path.
static const char name_sssp[] = "Single-Source Shortest Path";


/* -------- INTERNAL FUNCTIONS --------------------------------------------- */

// Maps from application name to internal application identifier.
static bool application_from_string(const char* const appname, application_t* app)
{
    for (size_t i = 0; i < (sizeof(string_bfs) / sizeof(string_bfs[0])); ++i)
    {
        if (0 == strnicmp(string_bfs[i], appname, 1 + strlen(string_bfs[i])))
        {
            *app = APPLICATION_BREADTH_FIRST_SEARCH;
            return true;
        }
    }
    
    for (size_t i = 0; i < (sizeof(string_cc) / sizeof(string_cc[0])); ++i)
    {
        if (0 == strnicmp(string_cc[i], appname, 1 + strlen(string_cc[i])))
        {
            *app = APPLICATION_CONNECTED_COMPONENTS;
            return true;
        }
    }
    
    for (size_t i = 0; i < (sizeof(string_pr) / sizeof(string_pr[0])); ++i)
    {
        if (0 == strnicmp(string_pr[i], appname, 1 + strlen(string_pr[i])))
        {
            *app = APPLICATION_PAGERANK;
            return true;
        }
    }
    
    for (size_t i = 0; i < (sizeof(string_sssp) / sizeof(string_sssp[0])); ++i)
    {
        if (0 == strnicmp(string_sssp[i], appname, 1 + strlen(string_sssp[i])))
        {
            *app = APPLICATION_SINGLE_SOURCE_SHORTEST_PATH;
            return true;
        }
    }
    
    return false;
}

static bool application_set(application_t app)
{
    switch (app)
    {
    case APPLICATION_BREADTH_FIRST_SEARCH:
        execution_has_special_output_stat = false;
        execution_needs_vertex_accumulator = false;
        execution_needs_wedge_frontier = true;
        execution_needs_traditional_frontiers = true;
        execution_uses_edge_weights = false;
        execution_get_vertex_prop_string = execution_get_vertex_prop_string_bfs;
        execution_get_vertex_prop_val = execution_get_vertex_prop_val_bfs;
        execution_initialize = execution_initialize_bfs;
        execution_impl = execution_impl_bfs;
        execution_special_stat_name = NULL;
        execution_special_stat_string = NULL;
        execution_app_name = name_bfs;
        return true;

    case APPLICATION_CONNECTED_COMPONENTS:
        execution_has_special_output_stat = false;
        execution_needs_vertex_accumulator = false;
        execution_needs_wedge_frontier = true;
        execution_needs_traditional_frontiers = false;
        execution_uses_edge_weights = false;
        execution_get_vertex_prop_string = execution_get_vertex_prop_string_cc;
        execution_get_vertex_prop_val = execution_get_vertex_prop_val_cc;
        execution_initialize = execution_initialize_cc;
        execution_impl = execution_impl_cc;
        execution_special_stat_name = NULL;
        execution_special_stat_string = NULL;
        execution_app_name = name_cc;
        return true;

    case APPLICATION_PAGERANK:
        execution_has_special_output_stat = true;
        execution_needs_vertex_accumulator = true;
        execution_needs_wedge_frontier = false;
        execution_needs_traditional_frontiers = false;
        execution_uses_edge_weights = false;
        execution_get_vertex_prop_string = execution_get_vertex_prop_string_pr;
        execution_get_vertex_prop_val = execution_get_vertex_prop_val_pr;
        execution_initialize = execution_initialize_pr;
        execution_impl = execution_impl_pr;
        execution_special_stat_name = execution_special_stat_name_pr;
        execution_special_stat_string = execution_special_stat_string_pr;
        execution_app_name = name_pr;
        return true;
        
    case APPLICATION_SINGLE_SOURCE_SHORTEST_PATH:
        execution_has_special_output_stat = false;
        execution_needs_vertex_accumulator = false;
        execution_needs_wedge_frontier = true;
        execution_needs_traditional_frontiers = false;
        execution_uses_edge_weights = true;
        execution_get_vertex_prop_string = execution_get_vertex_prop_string_sssp;
        execution_get_vertex_prop_val = execution_get_vertex_prop_val_sssp;
        execution_initialize = execution_initialize_sssp;
        execution_impl = execution_impl_sssp;
        execution_special_stat_name = NULL;
        execution_special_stat_string = NULL;
        execution_app_name = name_sssp;
        return true;

    default:
        return false;
    }
}


/* -------- FUNCTIONS ------------------------------------------------------ */
// See "application.h" for documentation.

void application_execute(void* arg)
{
    execution_impl();
}

// --------

bool application_select(const char* const appname)
{
    application_t app;
    
    if (false == application_from_string(appname, &app))
        return false;
    
    return application_set(app);
}
