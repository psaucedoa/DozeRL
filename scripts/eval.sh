#!/bin/bash
# Resolve workspace root directory
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "$1" ]; then
    echo "Usage: $0 <path_to_checkpoint_bin>"
    exit 1
fi

CHECKPOINT_PATH="$1"
SIM_EXEC="/home/rlab/workspaces/RL/pufferlib/dozerl"

# Resolve absolute path for checkpoint
if [[ ! "$CHECKPOINT_PATH" = /* ]]; then
    CHECKPOINT_PATH="$(pwd)/$CHECKPOINT_PATH"
fi

if [ ! -f "$CHECKPOINT_PATH" ]; then
    echo "Error: Checkpoint file '$CHECKPOINT_PATH' not found."
    exit 1
fi

if [ ! -f "$SIM_EXEC" ]; then
    echo "Error: Standalone simulator executable '$SIM_EXEC' not found. Please build it first."
    exit 1
fi

# Ensure out/ directory exists in workspace
mkdir -p "$WORKSPACE_DIR/out"

# Change directory to workspace root so outfile is written to out/sim_out.bin there
cd "$WORKSPACE_DIR"

echo "Running simulation rollout using policy: $CHECKPOINT_PATH"
"$SIM_EXEC" "$CHECKPOINT_PATH"
