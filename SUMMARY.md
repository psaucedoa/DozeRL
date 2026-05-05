# Summary: In-Situ Soil-Property Estimation and Bayesian Mapping

This document summarizes the mathematical models used to create a soil simulation for a compact track loader, based on the paper [In-Situ Soil-Property Estimation and Bayesian Mapping with a Simulated Compact Track Loader](https://arxiv.org/html/2507.22356v1).

## 1. Fundamental Equation of Earthmoving (FEE)
The resistive force $f$ encountered by a blade is calculated as:
$$f = \gamma d^2 w N_\gamma + c d w N_c + Q N_Q + c_a d w N_a$$

Where:
*   $\gamma$: Moist unit weight of soil
*   $c$: Cohesion
*   $c_a$: Adhesion
*   $d$: Depth of cut
*   $w$: Width of blade
*   $Q$: Surcharge force (weight of soil accumulated in front of the blade)

The dimensionless factors $N$ are:
*   $N_\gamma = \frac{[\cot(\rho) + \cot(\beta)] \sin(\alpha + \phi + \beta)}{2 \sin(\eta)}$
*   $N_Q = \frac{\sin(\alpha + \phi + \beta)}{\sin(\eta)}$
*   $N_c = \frac{\cos(\phi)}{\sin(\beta) \sin(\eta)}$
*   $N_{c_a} = \frac{-\cos(\rho + \phi + \beta)}{\sin(\rho) \sin(\eta)}$

Angles:
*   $\alpha$: Surface inclination
*   $\rho$: Blade rake angle
*   $\phi$: Internal angle of friction
*   $\delta$: Soil-tool friction angle
*   $\beta$: Soil failure angle ($\approx \frac{\pi}{4} - \frac{1}{2}\phi$)
*   $\eta = \delta + \rho + \phi + \beta$

## 2. Terrain Mapping & Soil Displacement
The simulation uses a Heightmap $M$, storing elevation $H$ and loose soil $L$. When the blade intersects the terrain, soil is displaced:
*   Cut: $H_{new} = H_{old} - \Delta h$
*   Deposit: $H_{deposit} = H_{old} + s \Delta h$ (where $s$ is the swell ratio $\approx 1.2$).
Surcharge $Q$ tracks the volume of loose soil.

## 3. Soil Erosion (Stability Model)
Factor of Safety ($F_s$) determines if soil slips between adjacent cells:
$$F_s(\alpha) = \frac{c L + W \cos(\alpha) \tan(\phi)}{W \sin(\alpha)}$$
If $F_s < 1$, a slip occurs, distributing height to neighbors.

## 4. Bayesian Mapping
Soil properties $\Theta_u = [c, \phi, c_a, \delta, \gamma]$ are updated spatially. For a map cell, combining the prior $(-)$ with a new estimate:
*   Mean: $\Theta_u^+ = \frac{\Sigma_{\Theta_u}^+ \Theta_u^- + \Sigma_{\Theta_u}^- \Theta_u}{\Sigma_{\Theta_u}^- + \Sigma_{\Theta_u}^+}$
*   Covariance: $\Sigma_{\Theta_u}^+ = \frac{\Sigma_{\Theta_u}^+ \Sigma_{\Theta_u}^-}{\Sigma_{\Theta_u}^- + \Sigma_{\Theta_u}^+}$
