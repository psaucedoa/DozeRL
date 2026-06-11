#!/bin/bash
set -e

# Change to the project root directory so paths work from anywhere
cd "$(dirname "$0")/.."

# Ensure the out directory exists for the binary file
mkdir -p out

echo "Compiling kinematics test..."
gcc -O3 tests/test_kinematics.c -o tests/test_kinematics -lm

echo "Running kinematics test..."
./tests/test_kinematics

echo "Launching kinematics visualizer in Rerun..."
python viz/kinematics.py
