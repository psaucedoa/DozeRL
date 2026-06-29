# Summary: In-Situ Soil-Property Estimation and Bayesian Mapping

This document summarizes the mathematical models used to create a soil simulation for a compact track loader, based on the paper [In-Situ Soil-Property Estimation and Bayesian Mapping with a Simulated Compact Track Loader](https://arxiv.org/html/2507.22356v1).

> **AGENT NOTE:** This document must be kept synchronized with the implementation in `sim.c` and `viz.py`. Whenever the mathematical models, soil parameters, or visualization scales are modified, update this file immediately.

## 1. Fundamental Equation of Earthmoving (FEE) - IMPLEMENTED
The resistive force $f$ encountered by a blade is calculated as:
$$f = \gamma d^2 w N_\gamma + c d w N_c + Q N_Q + c_a d w N_a$$

Where:
*   $\gamma$: Moist unit weight of soil (`env_gamma`)
*   $c$: Cohesion (`env_c`)
*   $c_a$: Adhesion (`env_c_a`)
*   $d$: Depth of cut
*   $w$: Width of blade
*   $Q$: Surcharge force (weight of soil accumulated in front of the blade)

The dimensionless factors $N$ are precomputed in `precompute_FEE` based on the blade rake angle and soil internal friction angle $\phi$.

## 2. Terrain Mapping & Soil Displacement - IMPLEMENTED
The simulation uses a Heightmap $M$, storing elevation $H$ and loose soil $L$. 
*   **Cut:** When the blade intersects terrain, height is removed.
*   **Blade Discretization:** The blade is discretized over the grid using Bresenham's Line Algorithm. This prevents double-counting of cells and accurately calculates effective blade width when pushing diagonally (scaling up to $CELL\_SIZE \cdot \sqrt{2}$).
*   **Swell:** Cut volume is converted to loose soil using `SWELL_RATIO` ($\approx 1.2$).
*   **Compaction:** Tracks compress loose soil back into a compacted state. The rate of compaction is dynamically calculated based on the ratio of the machine's ground pressure to the soil's ultimate bearing capacity $q_u$ (using Terzaghi's equation). Compaction is applied strictly to cells falling within the physical 2D rectangular footprint of the left and right tracks.
*   **Surcharge:** $Q$ is dynamically tracked by calculating the weight of loose soil in a 1.5m window in front of the blade.

## 3. Soil Erosion (Stability Model) - IMPLEMENTED
Soil stability is governed by the Infinite Slope Factor of Safety ($F_s$):
$$F_s(\alpha) = \frac{c A + W \cos(\alpha) \tan(\phi)}{W \sin(\alpha)}$$

Where:
*   $A$: Area of the 3D slip plane ($CELL\_SIZE^2 / \cos(\alpha)$)
*   $W$: Weight of the loose soil column ($CELL\_SIZE^2 \cdot L_{val} \cdot \gamma$)
*   $\alpha$: Slip angle

**Implementation Notes:**
1. **3D Grid vs 2D Literature:** Geotechnical literature typically analyzes a 2D cross-section and uses a 1D slip length $L$. To adapt this to a 3D grid, we use the Cell Area $A$ to represent the 2D slip plane cutting through the 3D cell volume. However, because $CELL\_SIZE^2$ appears in the numerator for both $A$ and $W$, the cell footprint cancels out entirely! The stability relies purely on depth ($L_{val}$) and slope angle ($\alpha$).
2. **Algebraic Optimization:** To avoid expensive iterative trigonometric calculations (`sin`, `cos`, `atan`) in a tight RL simulation loop, the C code algebraically solves for the **closed-form critical slope limit** where $F_s = 1$. By setting $F_s = 1$, letting $K = \frac{c}{\gamma \cdot L_{val}}$, and $t = \tan(\alpha)$, the equation is rearranged into a quadratic form:
$$K t^2 - t + (K + \tan(\phi)) = 0$$
The simulator solves this directly using the quadratic formula to find the maximum stable steepness (`t_limit`), instantly shaving off any soil that exceeds this threshold without requiring any iterative solvers.

## 4. Bayesian Mapping - NOT IMPLEMENTED
Soil properties $\Theta_u = [c, \phi, c_a, \delta, \gamma]$ are intended to be updated spatially based on force feedback. This feature is currently pending.

## 5. Visualization - IMPLEMENTED (`viz.py`)
Rerun is used for 3D visualization. 
*   **Elevation Mapping:** Terrain is colored based on a fixed elevation range (0.5m to 2.0m) to ensure visual consistency during replay.
*   **Machine State:** 6DOF kinematics (Pitch, Roll, Yaw) are visualized for both the chassis and the relative blade roll.
*   **Control Model:** Effort-based (Torque/Force) control for 5 active degrees of freedom: Tracks (Linear/Rotational), Arm Lift, Blade Pitch, and Blade Roll. 


## 6. Track Traction & Slip Model (Mohr-Coulomb) - IMPLEMENTED
The maximum tractive effort $T_{max}$ the machine can generate before the tracks slip is calculated using the Mohr-Coulomb failure criterion:
$$T_{max} = A \cdot c + W \cdot \tan(\phi)$$

Where:
*   $A$: Total contact area of the tracks (`2 * TRACK_LENGTH * TRACK_WIDTH`)
*   $c$: Soil cohesion (`env_c`)
*   $W$: Normal load / weight of the machine (`MACHINE_MASS * GRAVITY`)
*   $\phi$: Soil internal friction angle (`env_phi`)

Slip is then calculated as the ratio of resistive force to maximum traction ($F_{resist} / T_{max}$), dynamically linking machine mobility to the underlying soil properties.
