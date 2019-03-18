# Wedge

Wedge is a high-performance pull-only graph processing framework targeting a single machine containing or more x86-64-based processors.  It is distinguished from all prior work by introducing an efficient pull-based implementation of the frontier optimization as described in the paper *A New Frontier for Pull-Based Graph Processing*.

The content of this repository is intended to support the results presented in the aforementioned paper.  It contains all the source code and documentation required to build and run Wedge as configured for each experiment presented in the paper as well as instructions on how to obtain the datasets used as input.

Wedge is based on [**Grazelle**](https://github.com/stanford-mast/Grazelle-PPoPP18).  It is implemented entirely using C and C++; all assembly-language files in Grazelle have been promoted to C.


# Requirements

Wedge requires an x86-64-based CPU with support for AVX2 instructions, such as Intel processors of the Haswell generation or later.  NUMA scaling experiments require multiple CPU sockets.  We recommend 256GB DRAM per socket.

Wedge is intended to run on Ubuntu 14.04 or later.  Runtime dependencies include glibc, libnuma (package "libnuma-dev"), and pthreads.  Build dependencies include make and gcc 4.8.4 or later.

External dependencies are provided in binary form in this repository.  These are [**Spindle**](https://github.com/stanford-mast/Spindle), [**Silo**](https://github.com/stanford-mast/Silo), [**Parutil**](https://github.com/stanford-mast/Parutil), [**Topo**](https://github.com/stanford-mast/Topo), and [**hwloc**](https://www.open-mpi.org/projects/hwloc/).  The latter two libraries have build dependencies on `libpciaccess` and `libxml2`, both of which should be available as packages on a Linux distribution.

Experiments that use hardware performance counters additionally require linking with the [**Intel Performance Counter Monitor**](http://www.intel.com/software/pcm) library, also included in binary form.  Note that this library has been deprecated and Wedge has not been updated to support its replacement, so it is not guaranteed that these experiments will work on state-of-the-art or future server platforms.  Furthermore, none of the results shown in the paper make use of this library.


# Building

Wedge uses a Makefile-based build system.  To build Wedge in its default configuration, simply type:

    make

For additional options and customizations, access the build system documentation by typing:

    make help

Builds can be configured to run experiments, such as those used to gather data shown in the paper.  These experiments are enabled by adding an `EXPERIMENTS` variable to the build command:

    make EXPERIMENTS="<experiment1>[=value] <experiment2>[=value]"

All supported experiment flags are documented.  Experiment documentation can be accessed by typing:

    make EXPERIMENTS=HELP


# Datasets

Wedge supports graphs represented using a binary edge list format.  For unweighted graphs, Wedge uses the same format as does Grazelle.  For weighted graphs, each edge is represented by three 64-bit values instead of two:

1. A 64-bit unsigned integer identifying the source vertex

1. A 64-bit unsigned integer identifying the destination vertex

1. A 64-bit unsigned integer or double-precision floating-point value indicating the edge weight

Wedge expects that edges will be grouped by destination vertex such that the destination vertices appear in ascending order.  Unlike Grazelle, Wedge does not require a source-grouped representation, because Wedge processes graphs using only a pull pattern.  Accordingly, Wedge can directly consume as input datasets formatted for Grazelle's pull engine.


## Paper Experiment Datasets

Pre-converted ready-to-use __unweighted__ versions of each of the graphs from the paper are included with the [**artifact for Grazelle's original publication**](https://doi.org/10.5281/zenodo.1169388).  Note that only the versions that end with "-pull" are required.

To obtain __weighted__ versions of all datasets except `dimacs-usa`, use [**GraphTool**](https://github.com/stanford-mast/GraphTool) to add weights using the multiplicative hashing method.  To obtain the weighted version of `dimacs-usa` follow the reference link supplied in the paper for that graph and convert it to the binary edge list format Wedge uses.


## Custom Datasets

[**GraphTool**](https://github.com/stanford-mast/GraphTool) can be used to perform basic format conversion.  Converting graphs to Wedge's binary edge list format can be performed using this tool, although it is likely some manual intervention will also be required.


# Running

Simply type Wedge's executable path and supply the required command-line options.  Assuming the current directory is the directory from which `make` was launched, the executable would normally be located at `output/linux/wedge`.  To get help:

    output/linux/wedge -h

The only required command-line options are `-i`, which is used to specify the location of the input graph, and `-a`, which is used to specify the application to be executed.
