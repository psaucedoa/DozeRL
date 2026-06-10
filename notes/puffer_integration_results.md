# PufferLib RL Integration Report for DozeRL

The environment has been refactored, updated with more realistic physics parameters, and set up for Reinforcement Learning training in float32 precision with PufferLib. Sourcing from the virtual environment `/home/rlab/workspaces/RL/venv`, we have verified compilation, standalone execution, and a fast CUDA training loop.

---

## Completed Tasks

### 1. PufferLib Float32 CUDA Bug Fix
To compile the environment with native PyTorch (which strictly requires float32/FP32 precision under PufferLib), we compiled with `./build.sh dozerl --float`. This previously failed due to a duplicate definition of `cast_dispatch` in [kernels.cu](file:///home/rlab/workspaces/RL/pufferlib/src/kernels.cu#L320-L326) when `precision_t` resolves to `float`.
*   **Fix**: Wrapped the duplicate signature of `cast_dispatch(float*, const float*, ...)` in `#ifndef PRECISION_FLOAT` while keeping the underlying `cast<<<...>>>` GPU kernel active for direct calls within the library.
*   **Result**: PufferLib compiles clean in `--float` mode and executes with full PyTorch/CUDA training stability.

### 2. Corrected Center of Rotation (Track-Center Pivot)
*   **Issue**: The machine previously integrated coordinates centered at the blade (`x`, `y`) and derived the chassis position (`loader_x`, `loader_y`) backwards. This caused the chassis to rotate in a massive circle around the blade, creating unrealistic sliding forces and high velocities.
*   **Fix**: Modified [dozerl.h](file:///home/rlab/workspaces/DozeRL/src/dozerl.h) and [sim.c](file:///home/rlab/workspaces/DozeRL/src/sim.c) to integrate the track-center (`loader_x`, `loader_y`) directly. Global blade coordinates (`x`, `y`) are now dynamically derived from the track-center and yaw.
*   **Result**: The crawler rotates about the center of its tracks (spin-turn), and the blade sweeps in a correct, realistic arc relative to the machine.

### 3. Sluggishness & Realistic Inertia (18-Tonne Bulldozer)
Scaled the physics constants in [dozerl.h](file:///home/rlab/workspaces/DozeRL/src/dozerl.h) and [sim.h](file:///home/rlab/workspaces/DozeRL/src/sim.h) to reflect a medium-sized tracked crawler (similar to a Caterpillar D6):
*   `MACHINE_MASS`: Increased from `4500.0f` to **`18000.0f` kg** ($18\text{ tonnes}$).
*   `MACHINE_INERTIA`: Increased from `6000.0f` to **`25000.0f` kg m²**.
*   `MAX_FORCE_LINEAR`: Increased to **`100000.0f` N** ($100\text{ kN}$).
*   `MAX_TORQUE_ROTATIONAL`: Increased to **`80000.0f` Nm** ($80\text{ kNm}$).
*   `LINEAR_DAMPING`: Increased to `30000.0f`.
*   `ROTATIONAL_DAMPING`: Increased to `40000.0f`.
*   `MAX_FORCE_LIFT`: Increased to `120000.0f` N.

### 4. 30Hz Physics-Step Logging
*   Changed the logging frequency in [sim.c](file:///home/rlab/workspaces/DozeRL/src/sim.c#L623) and [dozerl.h](file:///home/rlab/workspaces/DozeRL/src/dozerl.h#L728) to output once *every single physics step* (`env->step_num % 1 == 0` instead of every 5 steps).
*   Provides high-fidelity trajectory logging to `out/sim_out.bin` for detailed Rerun playback at $30\text{Hz}$ (3 steps per $10\text{Hz}$ control action).

### 5. Track Visualizations in Rerun
*   Modified [viz.py](file:///home/rlab/workspaces/DozeRL/viz.py) to add two dark-grey tracks parallel to the chassis block using `rr.Boxes3D` mapped to `world/machine/tracks`.
*   Draws the tracks matching physical dimensions: Length $2.5\text{m}$, Width $0.4\text{m}$, Height $0.8\text{m}$, and Gauge $1.5\text{m}$ (center-to-center).

---

## How to Build & Train

### 1. Compile Environment with Float32 Precision
Activate the virtual environment containing PufferLib, go to the PufferLib repository, and build the extension:

```bash
cd /home/rlab/workspaces/RL/pufferlib
source /home/rlab/workspaces/RL/venv/bin/activate
./build.sh dozerl --float
```

### 2. Compile Standalone Fast Binary
```bash
./build.sh dozerl --fast
```

### 3. Run Standalone Pure C Demo/Benchmark
To build and run the simulation in pure C:
```bash
cd /home/rlab/workspaces/RL/pufferlib
./build.sh dozerl --local
./dozerl
```

### 4. Generate Rollout and Visualize
To run the policy evaluation and inspect in Rerun:
```bash
cd /home/rlab/workspaces/DozeRL
./scripts/eval.sh /home/rlab/workspaces/RL/pufferlib/checkpoints/dozerl/1781035078552/0000000099876864.bin
python3 viz.py
```
