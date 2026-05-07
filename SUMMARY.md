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
*   **Swell:** Cut volume is converted to loose soil using `SWELL_RATIO` ($\approx 1.2$).
*   **Compaction:** Tracks compress loose soil back into a compacted state at a rate of 15% per pass.
*   **Surcharge:** $Q$ is dynamically tracked by calculating the weight of loose soil in a 1.5m window in front of the blade.

## 3. Soil Erosion (Stability Model) - IMPLEMENTED
Soil stability is governed by the Factor of Safety ($F_s$):
$$F_s(\alpha) = \frac{c A + W \cos(\alpha) \tan(\phi)}{W \sin(\alpha)}$$

Where:
*   $A$: Area of the slip plane ($CELL\_SIZE^2 / \cos(\alpha)$)
*   $W$: Weight of the loose soil column
*   $\alpha$: Slip angle

If $F_s < 1$, the soil is unstable and slips toward neighbors until a stable height differential is reached.

## 4. Bayesian Mapping - NOT IMPLEMENTED
Soil properties $\Theta_u = [c, \phi, c_a, \delta, \gamma]$ are intended to be updated spatially based on force feedback. This feature is currently pending.

## 5. Visualization - IMPLEMENTED (`viz.py`)
Rerun is used for 3D visualization. 
*   **Elevation Mapping:** Terrain is colored based on a fixed elevation range (0.5m to 2.0m) to ensure visual consistency during replay.
*   **Machine State:** 6DOF kinematics (Pitch, Roll, Yaw) are visualized for both the chassis and the relative blade roll.
*   **Control Model:** Effort-based (Torque/Force) control for 5 active degrees of freedom: Tracks (Linear/Rotational), Arm Lift, Blade Pitch, and Blade Roll. 
