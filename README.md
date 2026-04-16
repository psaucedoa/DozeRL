# dsm-rl: Bulldozer Simulation for Reinforcement Learning

`dsm-rl` is a lightweight, high-performance bulldozer simulation environment written in C. It is designed specifically for training reinforcement learning (RL) agents (compatible with PufferLib) to perform complex soil manipulation tasks.

The simulation features a heightmap-based terrain where a bulldozer agent can interact with the soil by cutting, carrying, and depositing it across the environment.

## Key Features

- **Procedural Soil Manipulation:** Realistic blade-soil interaction that tracks terrain height changes in real-time.
- **Dynamic Terrain Engine:** Supports erosion, gradient calculations, and heightmap updates.
- **Rigid Body Agent Physics:** 2D agent movement with velocity, heading, and angular velocity controls.
- **Real-time Visualization:** Built-in renderer using [Raylib](https://www.raylib.com/) for immediate visual feedback and debugging.
- **RL-Ready:** Designed with a modular environment-agent structure suitable for integration with RL frameworks.

## Controls

The simulation can be manually controlled using the following keyboard mappings:

| Key | Action |
| --- | --- |
| `W` / `S` | Increase / Decrease Speed |
| `A` / `D` | Turn Left / Right |
| `Q` / `E` | Blade Yaw Left / Right |
| `Up` / `Down` | Raise / Lower Blade Position |
| `Space` | No-op (Continue simulation) |
| `R` | Reset Environment |
| `Esc` | Exit Simulation |

## Getting Started

### Prerequisites

- **GCC** (or any C compiler)
- **Make**
- **Raylib** (included in the `build/external` directory)

### Building the Project

On Linux/OSX, use the provided `build.sh` script:

```bash
chmod +x build.sh run.sh
./build.sh
```

On Windows, use the appropriate `.bat` file for your environment:
- `build-VisualStudio2022.bat`
- `build-MinGW-W64.bat`

### Running the Simulation

Execute the `run.sh` script (Linux/OSX):

```bash
./run.sh
```

Alternatively, run the binary directly from the `bin` directory.

## Project Structure

- `include/dsm.h`: Core simulation logic, environment/agent structs, and interaction math.
- `src/main.c`: Entry point, input handling, and Raylib integration.
- `resources/`: Graphical assets and icons.
- `build/`: Build configuration files and external dependencies.

## Technical Details

### Blade-Soil Interaction
The `blade_interaction` function in `dsm.h` handles the core logic of how the blade interacts with the terrain. It calculates the height difference between the blade and the soil, moving "soil" (height values) from cut locations to deposit locations based on the agent's movement direction and blade yaw.

### Terrain Dynamics
The simulation includes basic erosion (`erode`) and gradient calculation (`gradient`) functions to simulate terrain stability and provide more informative observations for RL agents.

## TODO & Future Work

- [ ] Implement soil friction model.
- [ ] Add delay in blade control for more realistic hydraulic behavior.
- [ ] Introduce variable soil weights/types.
- [ ] Multi-agent support optimization.

---

*This project is part of a larger effort to study complex manipulation tasks in reinforcement learning.*
