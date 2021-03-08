CFLAGS=-std=c11 -g
CXXFLAGS=-std=c++17 -g

# Define C headers
C_HEADERS= $(wildcard *.h)

# Define C sources
C_SOURCES= $(subst app_root.c,,$(wildcard *.c))
C_OBJECTS= $(patsubst %.c,_%.o, $(C_SOURCES))

# Define C++ sources
CPP_SOURCES= $(subst catch2_lib.cpp,, $(subst tests.cpp,, $(wildcard *.cpp)))
CPP_OBJECTS= $(patsubst %.cpp,_%.opp, $(CPP_SOURCES))

# Surrounding dirs
BIN_DIR=../bin


# Default build target
all: $(BIN_DIR)/promenad $(BIN_DIR)/tests

## Run things

run: $(BIN_DIR)/promenad
	cd $(BIN_DIR) && ./promenad

check: $(BIN_DIR)/tests
	cd $(BIN_DIR) && ./tests

check-windowed: $(BIN_DIR)/tests
	cd $(BIN_DIR) && ./tests "[windowed]"

## Build things

$(BIN_DIR)/promenad: app_root.c $(C_HEADERS) libpromenad.a $(BIN_DIR)
	$(CC) $(CFLAGS) $< libpromenad.a  -lraylib -lstdc++ -o $@

$(BIN_DIR)/tests: tests.cpp $(C_HEADERS) libpromenad.a libcatch2.a $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< libpromenad.a libcatch2.a -lraylib -o $@

# Bundle core object files
libpromenad.a: $(C_OBJECTS) $(CPP_OBJECTS) $(C_HEADERS)
	$(AR) rcs $@ $(C_OBJECTS) $(CPP_OBJECTS)

# Build all C object files
_%.o: %.c $(C_HEADERS)
	$(CC) -c  $(CFLAGS) $< -o $@

# Build all C++ object files
_%.opp: %.cpp $(C_HEADERS) PathFinder.hpp
	$(CXX) -c  $(CXXFLAGS) $< -o $@

# Avoid building catch2 implementation so often
# https://github.com/catchorg/Catch2/blob/master/docs/slow-compiles.md
libcatch2.a: catch2_lib.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

## Dirs
$(BIN_DIR):
	mkdir $@

# Clean output from dir(s)
clean:
	rm -rvf $(BIN_DIR)/*
	rm -vf *.o
	rm -vf *.opp
	rm -vf *.a
