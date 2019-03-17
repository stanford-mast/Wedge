/*****************************************************************************
* Wedge
*      High performance pull-based graph processing engine.
*      Targets a single machine with one or more x86-based sockets.
*****************************************************************************
* Authored by Samuel Grossman
* Department of Electrical Engineering, Stanford University
* (c) 2015-2018
*****************************************************************************
* intrinhelper.h
*      Helpers for intrinsic operations that differ between platforms.
*****************************************************************************/

#ifndef __GRAZELLE_INTRINHELPER_H
#define __GRAZELLE_INTRINHELPER_H


#include "versioninfo.h"


/* -------- PLATFORM-SPECIFIC MACROS --------------------------------------- */

#ifdef GRAZELLE_WINDOWS

#include <intrin.h>

#define _mm256_extract_epi8(a, index)           _mm_extract_epi8(_mm256_extracti128_si256(a, (index >> 4ull)), (index & 15ull))
#define _mm256_extract_epi16(a, index)          _mm_extract_epi16(_mm256_extracti128_si256(a, (index >> 3ull)), (index & 7ull))
#define _mm256_extract_epi32(a, index)          _mm_extract_epi32(_mm256_extracti128_si256(a, (index >> 2ull)), (index & 3ull))
#define _mm256_extract_epi64(a, index)          _mm_extract_epi64(_mm256_extracti128_si256(a, (index >> 1ull)), (index & 1ull))

#define _mm256_insert_epi8(a, i, index)         _mm256_inserti128_si256(a, _mm_insert_epi8(_mm256_extracti128_si256(a, (index >> 4ull)), i, (index & 15ull)), (index >> 4ull))
#define _mm256_insert_epi16(a, i, index)        _mm256_inserti128_si256(a, _mm_insert_epi16(_mm256_extracti128_si256(a, (index >> 3ull)), i, (index & 7ull)), (index >> 3ull))
#define _mm256_insert_epi32(a, i, index)        _mm256_inserti128_si256(a, _mm_insert_epi32(_mm256_extracti128_si256(a, (index >> 2ull)), i, (index & 3ull)), (index >> 2ull))
#define _mm256_insert_epi64(a, i, index)        _mm256_inserti128_si256(a, _mm_insert_epi64(_mm256_extracti128_si256(a, (index >> 1ull)), i, (index & 1ull)), (index >> 1ull))

#endif


#ifdef GRAZELLE_LINUX

#include <x86intrin.h>

#endif


#endif //__GRAZELLE_INTRINHELPER_H
