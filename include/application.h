/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* application.h
*      Declaration of application selection functionality.
*****************************************************************************/

#ifndef __GRAZELLE_APPLICATION_H
#define __GRAZELLE_APPLICATION_H


#include <stdbool.h>


/* -------- FUNCTIONS ------------------------------------------------------ */

// Spindle entry point for executing the application.
void application_execute(void* arg);

// Selects the specified application identified by name.
// Must be invoked to initialize internal data structures.
// Returns `true` if the name maps to a valid supported application to set, `false` otherwise.
bool application_select(const char* const appname);


#endif //__GRAZELLE_APPLICATION_H
