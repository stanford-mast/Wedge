/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* versioninfo.h
*      Definitions that provide information on the version of this program.
*      Includes platform, compiler, and optimizations.
*****************************************************************************/

#ifndef __GRAZELLE_VERSIONINFO_H
#define __GRAZELLE_VERSIONINFO_H


/* -------- CONSTANTS ------------------------------------------------------ */

#define GRAZELLE_PROGRAM_NAME                   "Wedge"
#define GRAZELLE_PROGRAM_VERSION                "1.0"


/* -------- PLATFORM-SPECIFIC CHECKS AND MACROS ---------------------------- */

#if defined(__gnu_linux__) && defined(__amd64__)

#define GRAZELLE_PLATFORM_NAME                  "Linux/glibc x64"
#define GRAZELLE_LINUX

#else
#error "Your platform is not supported. Wedge is intended for GNU-compatible Linux on x86-64."
#endif


/* -------- HARDWARE-SPECIFIC CHECKS AND MACROS ---------------------------- */

#ifndef __AVX2__
#error "Your target CPU is not supported. This program requires AVX2 instructions."
#endif


#endif //__GRAZELLE_VERSIONINFO_H
