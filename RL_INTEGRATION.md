# RL Integration Strategy: DozeRL & PufferLib

This document outlines the architectural and mathematical changes required to transform the current `sim.c` codebase into a high-performance Reinforcement Learning environment compatible with [PufferLib](https://github.com/PufferAI/PufferLib).

## 1. Architectural Refactoring (Mandatory for Parallelism)
PufferLib (and most RL frameworks) run many environments in parallel. The current "Global Array" approach in `sim.c` is not thread-safe.

*   **Encapsulation:** Move `grid_H`, `grid_L`, and all environment state into a `SoilEnv` struct.
*   **Memory Management:** Allow the RL wrapper to allocate multiple `SoilEnv` instances.
*   **Reset Function:** Implement a `reset()` function that re-randomizes the terrain noise and berms.

## 2. Joint Control & Actuation (6-DOF Control)
The agent needs explicit control over the machine's actuators. The "target cut depth" should be replaced by a physical arm model.

### Action Space (Inputs)
*   **Drive Tracks:** `v_linear`, `v_rotational` (allows for steering/yaw control).
*   **Arm Lift:** `arm_height` (Relative height of the blade hinge above the tracks).
*   **Blade Pitch:** `blade_pitch_rel` (Tilt forward/back, affecting the `rake_angle`).
*   **Blade Roll:** `blade_roll_rel` (Tilt left/right relative to the chassis).

### Kinematic Updates
The `update_kinematics` function must be modified to calculate the final blade edge position based on the combination of:
1.  Chassis Pose (Track height sampling).
2.  Arm Height.
3.  Relative Blade Pitch/Roll.

## 3. Observation Space (State Representation)
For an agent to learn, it needs a "window" into the world.

### Proprioceptive Observations (Vector)
*   **Machine Attitude:** Pitch, Roll, Yaw rate.
*   **Actuator States:** Current arm height, blade pitch, and blade roll.
*   **Force Feedback:** `last_force` (normalized) and `last_yaw_moment`.
*   **Surcharge:** `surcharge_Q` (normalized).

### Spatial Observations (Heightmap Image)
*   **Local Terrain:** A small cropped window (e.g., 32x32 cells) of the heightmap centered immediately in front of the blade. This allows the agent to "see" the pile it is pushing.

## 4. Reward Function Design
To push soil effectively, the agent needs a dense reward signal:
*   **Primary Reward:** Volume of soil moved in the target direction ($+ \Delta Q \cdot v_{forward}$).
*   **Penalty - Stall:** Negative reward if `slip_ratio > 0.8` (tracks spinning but not moving).
*   **Penalty - Energy:** Small penalty proportional to $|Action|^2$ to encourage smooth control.
*   **Penalty - Deviation:** Penalty for yaw drift from the intended path.

## 5. PufferLib C-Binding (Python Wrapper)
Use `ctypes` or `pybind11` to expose the following C functions to Python:
1.  `env_init()`: Allocate the environment.
2.  `env_reset(seed)`: Reset terrain and pose.
3.  `env_step(actions)`: Apply actions, run 1-5 simulation steps (sub-stepping for stability), and return the new state.

## 6. Performance Goal
The current simulation runs at ~5000Hz (single thread). With PufferLib's vectorization, we should aim for **100,000+ SPS (Steps Per Second)** across all cores, enabling complex soil-moving policies to be trained in minutes rather than hours.
