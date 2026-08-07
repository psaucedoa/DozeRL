#!/bin/bash
set -e

# Change to the tests directory
cd "$(dirname "$0")"

# Number of steps to benchmark (default: 1,000,000)
STEPS=${1:-1000000}

echo "Preparing physics header (stripping rendering code)..."
# Find the line where raylib_render.h is included
LINE=$(grep -n '#include "raylib_render.h"' ../dozerl.h | head -1 | cut -d: -f1)

if [ -z "$LINE" ]; then
    echo "Warning: could not find raylib_render.h include in dozerl.h, using the whole file."
    cp ../dozerl.h dz.h
else
    # Strip everything from the include onwards
    { head -n $((LINE-1)) ../dozerl.h; echo "#endif"; } > dz.h
fi

echo "Compiling benchmark..."
gcc -std=c11 -D_GNU_SOURCE -O2 -march=native -Wall -Wextra -o bench bench.c -lm

echo "Running benchmark for $STEPS steps..."
./bench $STEPS

# Clean up
rm dz.h bench
