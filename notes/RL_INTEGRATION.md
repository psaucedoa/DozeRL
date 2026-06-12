# RL Integration Strategy: DozeRL & PufferLib

This document outlines the architectural and mathematical changes required to transform the current `sim.c` codebase into a high-performance Reinforcement Learning environment compatible with [PufferLib](https://github.com/PufferAI/PufferLib).

## 1. Architectural Refactoring (Mandatory for Parallelism)
PufferLib (and most RL frameworks) run many environments in parallel. The current "Global Array" approach in `sim.c` is not thread-safe.

*   **Encapsulation:** Move `grid_H`, `grid_L`, and all environment state into a `SoilEnv` struct.
*   **Memory Management:** Allow the RL wrapper to allocate multiple `SoilEnv` instances.
*   **Reset Function:** Implement a `reset()` function that re-randomizes the terrain noise and berms.

## 2. Joint Control & Actuation (6-DOF Control)
The agent needs explicit control over the machine's actuators. The "target cut depth" should be replaced by a physical arm model.

### Action Space (Inputs: -1.0 to 1.0)
The environment uses **Torque/Force Control (Effort)** for all actuators. The agent provides normalized effort values which are integrated through a dynamics model.

*   **Linear Tracks:** `effort_linear` (Accelerates the machine forward/backward).
*   **Rotational Tracks:** `effort_rotational` (Accelerates machine yaw/heading).
*   **Arm Lift:** `effort_lift` (Vertical force on the lift arm; combats gravity and soil resistance).
*   **Blade Pitch:** `effort_pitch` (Torque for tilting the blade forward/back).
*   **Blade Roll:** `effort_roll` (Torque for tilting the blade left/right).
*   **Blade Yaw:** `effort_yaw` (Currently fixed at 0; no relative blade yaw).

### Actuator Dynamics
Each joint follows the simplified dynamics:
$$\text{accel} = \frac{\text{effort} \cdot F_{max} - F_{resist} - \text{damping} \cdot \text{vel}}{\text{mass/inertia}}$$
$$\text{vel}_{t+1} = \text{vel}_t + \text{accel} \cdot \Delta t$$
$$\text{pos}_{t+1} = \text{pos}_t + \text{vel}_{t+1} \cdot \Delta t$$


## 3. Observation Space (State Representation)
For an agent to learn, it needs a "window" into the world.

### Proprioceptive Observations (Vector)
*   **Chassis Attitude:** Pitch, Roll, Yaw rate (from IMU).
*   **Joint Positions:** Current `arm_height`, `blade_pitch_rel`, and `blade_roll_rel`.
*   **Joint Velocities:** Current `vel_arm_height`, `vel_pitch_rel`, and `vel_roll_rel`.
*   **Track State:** Linear velocity `v_linear` and rotational velocity `v_rotational`. (Note: Real tracks only report velocity, not position).
*   **Internal State:** Surcharge `surcharge_Q` and total resistive force `last_force`.

### Spatial Observations (Multi-Channel Heightmap)
A 50x50 cropped window (10m x 10m at 0.2m resolution) centered immediately in front of the blade.
*   **Channel 0 (Terrain):** Current elevation (`grid_H + grid_L`) relative to chassis elevation.
*   **Channel 1 (Goal):** Target elevation (`grid_G`) relative to chassis elevation.

## 4. Reward Function Design
The agent is tasked with transforming the current terrain into a target "Goal Map" consisting of a **Slot** (where soil is removed) and a **Pile** (where soil is deposited).

### Goal Map Generation
*   **Slot:** A rectangular trench of random dimensions, position, and orientation.
*   **Pile:** A mound located at the end of the slot.
*   **Consistency:** The volume of the pile matches the volume of the slot, adjusted by `SWELL_RATIO` (puffiness).

### Reward Components
*   **Alignment Reward (Dense):** $R_{align} = \text{Error}_{t-1} - \text{Error}_t$, where $\text{Error} = \sum |(H+L) - G|$ within the active zone. This rewards any change that brings the terrain closer to the goal.
*   **Stall Penalty:** $R_{stall} = -\max(0, \text{slip\_ratio} - 0.8)$. Penalizes tracks spinning without moving.
*   **Energy Penalty:** $R_{energy} = - \sum |Action|^2 \cdot \lambda$. Encourages efficient and smooth control.

## 5. PufferLib C-Binding (Python Wrapper)
Use `ctypes` or `pybind11` to expose the following C functions to Python:
1.  `env_init()`: Allocate the environment.
2.  `env_reset(seed)`: Reset terrain and pose.
3.  `env_step(actions)`: Apply actions, run 1-5 simulation steps (sub-stepping for stability), and return the new state.

## 6. Performance Goal
The current simulation runs at ~5000Hz (single thread). With PufferLib's vectorization, we should aim for **100,000+ SPS (Steps Per Second)** across all cores, enabling complex soil-moving policies to be trained in minutes rather than hours.
