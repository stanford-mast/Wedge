###############################################################################
# Wedge
#      High performance pull-based graph processing engine.
#      Targets a single machine with one or more x86-based sockets.
###############################################################################
# Authored by Samuel Grossman
# Department of Electrical Engineering, Stanford University
# (c) 2015-2018
###############################################################################
# Makefile
#      Build script for GNU-compatible Linux operating systems.
###############################################################################


# --------- PROJECT PROPERTIES ------------------------------------------------

PROJECT_NAME                = wedge
PLATFORM_NAME               = linux

LIBRARY_DEPENDENCIES        = libparutil libsilo libspindle libtopo libhwloc libpciaccess libxml2 librt libpthread libnuma

SOURCE_DIR                  = source
INCLUDE_DIR                 = include

OUTPUT_BASE_DIR             = output
OUTPUT_DOCS_DIR             = $(OUTPUT_BASE_DIR)/docs
OUTPUT_DIR                  = $(OUTPUT_BASE_DIR)/$(PLATFORM_NAME)
OUTPUT_FILE                 = $(PROJECT_NAME)
INTERMEDIATE_DIR            = $(OUTPUT_DIR)/build

C_SOURCE_SUFFIX             = .c
CXX_SOURCE_SUFFIX           = .cpp
ASSEMBLY_SOURCE_SUFFIX      = .s


# --------- TOOL SELECTION AND CONFIGURATION ----------------------------------

CC                          = gcc
CXX                         = g++
LD                          = g++

CCFLAGS                     = -g -O3 -Wall -std=c11 -masm=intel -march=core-avx2 -mno-vzeroupper -pthread -I$(INCLUDE_DIR) -D_GNU_SOURCE
CXXFLAGS                    = -g -O3 -Wall -std=c++11 -masm=intel -march=core-avx2 -mno-vzeroupper -pthread -I$(INCLUDE_DIR)
LDFLAGS                     = -g


# --------- EXPERIMENTS -------------------------------------------------------

ifeq 'HELP' '$(EXPERIMENTS)'

experimenthelp:
	@echo ''
	@echo 'FRONTIER_PRECISION=<number>'
	@echo '    Overrides the frontier precision.'
	@echo '    Measured in number of edge vectors per frontier element.'
	@echo '    Value provided is the base-2 logarithm of the desired precision.'
	@echo '    Defaults to 2.'
	@echo 'HW_MEMBW_PULL'
	@echo '    Hardware experiment.'
	@echo '    Measures memory bandwidth utilization during Pull.'
	@echo '    Prints output to standard error.'
	@echo 'HW_MEMBW_WEDGE'
	@echo '    Hardware experiment.'
	@echo '    Measures memory bandwidth utilization during Wedge.'
	@echo '    Prints output to standard error.'
	@echo 'ITERATION_STATS'
	@echo '    Prints detailed iteration statistics to standard error.'
	@echo 'MEASURE_LOAD_BALANCE'
	@echo '    Causes load balance measurements to be taken.'
	@echo '    Results are printed to standard error.'
	@echo ''

else

SUPPORTED_EXPERIMENTS       = HW_MEMBW_PULL HW_MEMBW_WEDGE ITERATION_STATS MEASURE_LOAD_BALANCE
SUPPORTED_EXPERIMENTS_PARAM = FRONTIER_PRECISION
UNSUPPORTED_EXPERIMENTS     = $(filter-out $(SUPPORTED_EXPERIMENTS) $(SUPPORTED_EXPERIMENTS_PARAM), $(foreach experiment, $(EXPERIMENTS), $(word 1, $(subst =, ,$(experiment)))))
MISSING_PARAM_EXPERIMENTS   = $(filter $(SUPPORTED_EXPERIMENTS_PARAM), $(filter-out %=%, $(EXPERIMENTS)))

HARDWARE_EXPERIMENTS        = $(filter HW_%, $(EXPERIMENTS))
HARDWARE_EXPERIMENT_LIBS    = libintelpcm

ifneq ($(strip $(HARDWARE_EXPERIMENTS)),)
LIBRARY_DEPENDENCIES        += $(HARDWARE_EXPERIMENT_LIBS)
CCFLAGS                     += -DENABLE_HARDWARE_EXPERIMENTS
CXXFLAGS                    += -DENABLE_HARDWARE_EXPERIMENTS
endif

ifneq ($(strip $(UNSUPPORTED_EXPERIMENTS)),)
$(error Invalid experiment(s): $(UNSUPPORTED_EXPERIMENTS).  Try 'EXPERIMENTS=HELP' for more information)
endif

ifneq ($(strip $(MISSING_PARAM_EXPERIMENTS)),)
$(error Missing parameter for experiment(s): $(MISSING_PARAM_EXPERIMENTS).  Try 'EXPERIMENTS=HELP' for more information)
endif

ifneq ($(strip $(EXPERIMENTS)),)

CCFLAGS                     += -DEXPERIMENT_STR="\"$(EXPERIMENTS)\""
CXXFLAGS                    += -DEXPERIMENT_STR="\"$(EXPERIMENTS)\""

endif


PARAMLESS_EXPERIMENTS       = $(filter-out %=%, $(EXPERIMENTS))
PARAM_EXPERIMENTS           = $(filter %=%, $(EXPERIMENTS))

ifneq ($(strip $(PARAMLESS_EXPERIMENTS)),)

CCFLAGS                     += $(foreach EXPERIMENT, $(PARAMLESS_EXPERIMENTS), -DEXPERIMENT_$(EXPERIMENT))
CXXFLAGS                    += $(foreach EXPERIMENT, $(PARAMLESS_EXPERIMENTS), -DEXPERIMENT_$(EXPERIMENT))
ASFLAGS                     += $(foreach EXPERIMENT, $(PARAMLESS_EXPERIMENTS), --defsym EXPERIMENT_$(EXPERIMENT)=1)

endif

ifneq ($(strip $(PARAM_EXPERIMENTS)),)

CCFLAGS                     += $(foreach EXPERIMENT, $(PARAM_EXPERIMENTS), -DEXPERIMENT_$(EXPERIMENT))
CXXFLAGS                    += $(foreach EXPERIMENT, $(PARAM_EXPERIMENTS), -DEXPERIMENT_$(EXPERIMENT))
ASFLAGS                     += $(foreach EXPERIMENT, $(PARAM_EXPERIMENTS), --defsym EXPERIMENT_$(EXPERIMENT))

endif

endif


# --------- FILE ENUMERATION --------------------------------------------------

OBJECT_FILE_SUFFIX          = .o
DEP_FILE_SUFFIX             = .d

C_SOURCE_FILES              = $(filter-out $(wildcard $(SOURCE_DIR)/*-*$(C_SOURCE_SUFFIX)), $(wildcard $(SOURCE_DIR)/*$(C_SOURCE_SUFFIX))) $(wildcard $(SOURCE_DIR)/*-$(PLATFORM_NAME)$(C_SOURCE_SUFFIX))
CXX_SOURCE_FILES            = $(filter-out $(wildcard $(SOURCE_DIR)/*-*$(CXX_SOURCE_SUFFIX)), $(wildcard $(SOURCE_DIR)/*$(CXX_SOURCE_SUFFIX))) $(wildcard $(SOURCE_DIR)/*-$(PLATFORM_NAME)$(CXX_SOURCE_SUFFIX))
ALL_SOURCE_FILES            = $(C_SOURCE_FILES) $(CXX_SOURCE_FILES)

OBJECT_FILES                = $(patsubst $(SOURCE_DIR)/%, $(INTERMEDIATE_DIR)/%$(OBJECT_FILE_SUFFIX), $(ALL_SOURCE_FILES))
DEP_FILES                   = $(patsubst $(SOURCE_DIR)/%, $(INTERMEDIATE_DIR)/%$(DEP_FILE_SUFFIX), $(ALL_SOURCE_FILES))

LINK_LIBRARIES              = $(patsubst lib%, -l%, $(LIBRARY_DEPENDENCIES))


# --------- TOP-LEVEL RULE CONFIGURATION --------------------------------------

.PHONY: wedge help clean


# --------- TARGET DEFINITIONS ------------------------------------------------

wedge: $(OUTPUT_DIR)/$(OUTPUT_FILE)

help:
	@echo ''
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Targets:'
	@echo '    wedge'
	@echo '        Default target. Builds Wedge.'
	@echo '    clean'
	@echo '        Removes all output files, including binary and documentation.'
	@echo '    help'
	@echo '        Shows this information.'
	@echo ''
	@echo 'Variables:'
	@echo '    EXPERIMENTS'
	@echo '        Selects experiments to run or experimental optimizations to enable.'
	@echo '        Defaults to no experiments or experimental optimizations.'
	@echo '        Type `make EXPERIMENTS=HELP'\'' for more information.'
	@echo ''


# --------- BUILDING AND CLEANING RULES ---------------------------------------

$(OUTPUT_DIR)/$(OUTPUT_FILE): $(OBJECT_FILES)
	@echo '   LD        $@'
	@$(LD) $(LDFLAGS) -o $@ $(OBJECT_FILES) $(LINK_LIBRARIES) $(LDEXTRAFLAGS)
	@echo 'Build completed: $(PROJECT_NAME).'

clean:
	@echo '   RM        $(OUTPUT_DIR)'
	@rm -rf $(OUTPUT_DIR)
	@echo 'Clean completed: $(PROJECT_NAME).'


# --------- COMPILING AND ASSEMBLING RULES ------------------------------------

$(INTERMEDIATE_DIR):
	@mkdir -p $(INTERMEDIATE_DIR)

$(OUTPUT_DIR):
	@mkdir -p $(OUTPUT_DIR)

$(OUTPUT_DOCS_DIR):
	@mkdir -p $(OUTPUT_DOCS_DIR)

$(INTERMEDIATE_DIR)/%$(C_SOURCE_SUFFIX)$(OBJECT_FILE_SUFFIX): $(SOURCE_DIR)/%$(C_SOURCE_SUFFIX) | $(INTERMEDIATE_DIR)
	@echo '   CC        $@'
	@$(CC) $(CCFLAGS) -MD -MP -c -o $@ -Wa,-adhlms=$(patsubst %$(OBJECT_FILE_SUFFIX),%$(ASSEMBLY_SOURCE_SUFFIX),$@) $<

$(INTERMEDIATE_DIR)/%$(CXX_SOURCE_SUFFIX)$(OBJECT_FILE_SUFFIX): $(SOURCE_DIR)/%$(CXX_SOURCE_SUFFIX) | $(INTERMEDIATE_DIR)
	@echo '   CXX       $@'
	@$(CXX) $(CXXFLAGS) -MD -MP -c -o $@ -Wa,-adhlms=$(patsubst %$(OBJECT_FILE_SUFFIX),%$(ASSEMBLY_SOURCE_SUFFIX),$@) $<

-include $(DEP_FILES_FROM_SOURCE)
