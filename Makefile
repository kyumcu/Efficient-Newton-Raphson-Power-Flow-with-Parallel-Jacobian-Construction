# Compiler and flags
CXX =    g++ # icpx #
CXXFLAGS = -std=c++17 #-Wall -Wextra  
LDFLAGS = 
LDTESTFLAGS = -lgtest -pthread

PERFFLAGS = -O3 -mavx -mavx2 -msse4.2 -march=native -ffast-math -fopenmp #-qopenmp
INCDIR = -Iinclude -I/usr/include/eigen3


CXXFLAGS += $(PERFFLAGS)
CXXFLAGS += $(INCDIR)

# Source and build directories
SRCDIR = src
BUILDDIR = build
TESTDIR = test
OUTDIR = $(CURDIR)/output/
MPOWERDIR = $(HOME)/matpower7/

MACROS = -DROOTDIR='"$(CURDIR)/"'
MACROS += -DOUTDIR='"$(OUTDIR)"'
MACROS += -DMPOWERDIR='"$(MPOWERDIR)"'

# DEBUG section
MACROS += -DTIME0 # global time
#MACROS += -DTIME # general steps
#MACROS += -DTIME2 # solver detaisl
#MACROS += -DTIME3
#MACROS += -DDEBUG
#CXXFLAGS += -g

#PARAMS
MACROS += -DBATCHSIZE=17

CXXFLAGS += $(MACROS)

# Conditional check if CXX is 'icpx'
ifeq ($(strip $(CXX)),icpx)
MACROS += -DMKL
INCDIR += -I"${MKLROOT}/include"
LDFLAGS +=    -Wl,--start-group ${MKLROOT}/lib/libmkl_intel_lp64.a ${MKLROOT}/lib/libmkl_intel_thread.a ${MKLROOT}/lib/libmkl_core.a -Wl,--end-group -liomp5 -lpthread -lm -ldl
$(info Using Intel oneAPI compiler icpx)
$(info USE)
$(info source /opt/intel/oneapi/setvars.sh)
$(info -----------------------------------)
endif


# Executable names
MAIN = $(BUILDDIR)/main
MAIN_TEST = $(BUILDDIR)/main_test

# Source files
SRC = $(wildcard $(SRCDIR)/*.cpp) $(wildcard $(SRCDIR)/**/*.cpp)
TEST_SRC = $(wildcard $(TESTDIR)/*.cpp) $(wildcard $(TESTDIR)/**/*.cpp) 

# Object files
OBJ = $(SRC:$(SRCDIR)/%.cpp=$(BUILDDIR)/%.o)
TEST_OBJ = $(TEST_SRC:$(TESTDIR)/%.cpp=$(BUILDDIR)/test/%.o) $(filter-out $(BUILDDIR)/main.o, $(OBJ))


# Default and phony targets
.PHONY: all clean test redo rtest format

# Default target
all: $(MAIN) $(MAIN_TEST)

# Main application
$(MAIN): $(OBJ)
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@
	mkdir -p $(OUTDIR)

# Test application
$(MAIN_TEST): $(TEST_OBJ)
	$(CXX) $(CXXFLAGS) $^  $(LDFLAGS) $(LDTESTFLAGS) -o $@
	mkdir -p $(OUTDIR)

# Compile source files into objects
$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile test files into objects
$(BUILDDIR)/test/%.o: $(TESTDIR)/%.cpp
	mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $<  -o $@


# Clean up
clean:
	rm -rf $(BUILDDIR)
	rm -rf $(OUTDIR)*

main:
	@$(MAKE) -j build/main

redo: clean
	@$(MAKE) -j build/main
	./build/main

format:
	clang-format -i $(shell find . -name '*.cpp' -or -name '*.h')


# Parallel build target
test: clean
	@$(MAKE) -j build/main_test
	./build/main_test
