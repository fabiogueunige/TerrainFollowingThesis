# Multi-Objective Optimization of SBES Sensor Angle for Terrain-Following AUV Navigation

## Academic Technical Report

**Abstract:** This document presents a comprehensive mathematical framework for optimizing the mounting angle of Single-Beam Echo Sounders (SBES) in a quadruple-sensor configuration for autonomous underwater vehicle (AUV) terrain-following applications. The optimization balances four competing objectives—resolution, coverage, acoustic robustness, and geometric stability—subject to geometric feasibility constraints. The implemented algorithm uses bounded scalar optimization with explicit handling of 3D geometric constraints accounting for robot orientation and terrain slope.

---

## 1. PROBLEM FORMULATION

### 1.1 System Configuration

We consider an AUV equipped with four SBES sensors arranged in an X-configuration:
- **Sensor placement:** North (+pitch), South (-pitch), East (+roll), West (-roll)  
- **Mounting geometry:** Each sensor tilted at angle γ from the robot's vertical axis
- **Operational mode:** Constant-altitude terrain-following at height h above seafloor

**System parameters:**
- γ ∈ [γ_min, γ_max]: SBES mounting angle (optimization variable)  
- h = 3.0 m: Nominal altitude above terrain (constant)
- v = 0.1 m/s: Surge velocity (forward speed)
- t_react = 0.5 s: Control system reaction time
- σ_SBES = 0.005 m: Sensor depth measurement precision
- φ_beam = 4.0°: SBES acoustic beam opening angle
- f_ping = 20 Hz: Measurement update frequency

**Environmental parameters:**
- β_terrain_max = 80°: Maximum terrain pitch slope capability (single-axis model)
- θ_robot: Robot pitch angle (roll neglected in current implementation)
- δ_tracking: Terrain-following tracking error

---

## 2. GEOMETRIC ANALYSIS

### 2.1 Sensor Footprint Geometry

For a sensor mounted at angle γ from vertical, at altitude h:

**Baseline (sensor pair separation):**
$$s(\gamma) = 2h \tan(\gamma)$$

**Look-ahead distance (predictive horizon):**
$$d_{ahead}(\gamma) = h \tan(\gamma)$$

**Slant range (acoustic path length):**
$$r(\gamma) = \frac{h}{\cos(\gamma)}$$

**Seafloor footprint area (for sensor pair):**
$$A_{footprint}(\gamma) \approx s^2(\gamma) = 4h^2 \tan^2(\gamma)$$

### 2.2 Three-Dimensional Ray-Terrain Interaction

The critical geometric constraint arises from the requirement that SBES acoustic rays must intersect the terrain. Consider:

**Robot orientation:** Pitch θ_robot (relative to world frame)
**Terrain orientation:** Pitch slope β_terrain
**SBES ray direction:** Angle γ from robot vertical

For a sensor on the pitch axis (North/South):
$$\vec{r}_{ray}^{robot} = [\sin(\gamma), 0, \cos(\gamma)]^T$$

After rotation by robot pitch (θ), the ray direction in world frame becomes:
$$\vec{r}_{ray}^{world} = R_y(\theta) \, \vec{r}_{ray}^{robot}$$

**Feasibility condition:** The angle θ_eff between ray and terrain normal must satisfy:
$$\theta_{eff} = \arccos(\vec{r}_{ray}^{world} \cdot \vec{n}_{terrain}) < 90°$$

In the implemented algorithm (pitch-only), we use the corrected approximation based on relative orientation:
$$\theta_{eff} \approx \gamma + \big|\theta_{robot} - \beta_{\text{terrain}}\big| + \delta_{tracking} \quad (+\; \tfrac{\phi_{beam}}{2}\;\text{for worst-case beam edge})$$

---

## 3. MULTI-OBJECTIVE OPTIMIZATION FRAMEWORK

The optimization seeks to maximize a weighted combination of four objective functions, each addressing a specific performance criterion:

$$J(\gamma) = w_1 f_{res}(\gamma) + w_2 f_{cov}(\gamma) + w_3 f_{rob}(\gamma) + w_4 f_{unc}(\gamma)$$

Subject to: $\gamma \in [\gamma_{min}, \gamma_{max}]$ and $\sum_{i=1}^4 w_i = 1, \quad w_i > 0$

### 3.1 Objective Component 1: Resolution Quality

**Physical motivation:** Larger baselines reduce the relative impact of sensor noise on slope angle estimation.

**Formulation:**
$$f_{res}(\gamma) = \tanh\left(\frac{s_{eff}(\gamma)}{s_{ref}}\right)$$

where:
- $s_{eff}(\gamma) = 2h \tan(\gamma) \cos(\theta_{robot})$ is the effective baseline projected onto terrain (pitch-only)
- $s_{ref} = 1.0$ m is a normalization constant

**Properties:**
- Monotonically increasing with γ (larger angles → larger baselines)
- Saturates at $f_{res} \to 1$ for very large baselines (tanh limiting behavior)
- Angular resolution: $\sigma_\alpha \approx \sigma_{SBES} / s_{eff}$

**Tuning parameter:**
- $s_{ref}$: Controls saturation point. Larger values delay saturation, encouraging larger angles.

### 3.2 Objective Component 2: Coverage and Anticipation

**Physical motivation:** The control system requires sufficient look-ahead distance to react to terrain variations.

**Formulation:**
$$f_{cov}(\gamma) = 1 - \exp\left(-k_c \left[d_{ahead}(\gamma) - v \cdot t_{react}\right]\right)$$

where:
- $d_{ahead}(\gamma) = h \tan(\gamma)$ is the forward-looking distance
- $v \cdot t_{react}$ is the minimum required distance for control action
- $k_c = 5.0$ is a scaling coefficient

**Properties:**
- Step-like transition around $d_{ahead} = v \cdot t_{react}$
- Penalizes angles that provide insufficient reaction time
- Approaches 1.0 when $d_{ahead} \gg v \cdot t_{react}$

**Tuning parameter:**
- $k_c$: Controls transition sharpness. Larger values create harder threshold behavior.

### 3.3 Objective Component 3: Acoustic Robustness

**Physical motivation:** Empirical evidence suggests optimal acoustic return quality at moderate angles (~25°) due to seafloor backscatter characteristics and grazing angle effects.

**Formulation:**
$$f_{rob}(\gamma) = \exp\left(-\frac{(\gamma - \gamma_{opt})^2}{2\sigma_\gamma^2}\right)$$

where:
- $\gamma_{opt} = 25°$ (empirical optimum from acoustic experiments)
- $\sigma_\gamma = 15°$ (tolerance bandwidth)

**Properties:**
- Gaussian preference centered at $\gamma_{opt}$
- Maximum at γ = 25°, decreasing for both smaller and larger angles
- At γ = 40° or γ = 10°: $f_{rob} \approx 0.37$ (one standard deviation)

**Tuning parameters:**
- $\gamma_{opt}$: Center of acoustic preference (application-dependent)
- $\sigma_\gamma$: Tolerance for deviation. Smaller values enforce stricter preference.

### 3.4 Objective Component 4: Geometric Stability

**Physical motivation:** On steep terrain, large sensor angles risk rays becoming parallel or divergent from terrain surface, causing measurement loss.

**Formulation:**
$$f_{unc}(\gamma) = \tanh\left(\frac{\max(\cos(\theta_{eff}), \epsilon)}{s_{scale}}\right)$$

where (pitch-only):
$$\theta_{eff}(\gamma) = \gamma + \big|\theta_{robot} - \beta_{\text{terrain,max}}\big| + \delta_{tracking}$$

and:
- $\epsilon = 0.01$ (numerical stability threshold)
- $s_{scale} = 0.3$ (scaling factor for tanh response)

**Critical angle:** Ray becomes parallel to terrain when $\theta_{eff} \to 90°$, causing $\cos(\theta_{eff}) \to 0$ and thus $f_{unc} \to 0$.

**Properties:**
- Decreases as γ approaches the critical angle: $\gamma_{crit} = 90° - \big|\theta_{robot} - \beta_{\text{terrain,max}}\big| - \delta_{tracking}$
- Accounts for worst-case scenario: maximum terrain slope + maximum robot orientation error
- Provides smooth penalty rather than hard constraint

**Tuning parameters:**
- $s_{scale}$: Controls penalty severity. Smaller values create earlier/stronger penalties.
- $\epsilon$: Prevents numerical instability when denominator approaches zero.

---

## 4. CONSTRAINT FORMULATION

The feasible region for γ is bounded by two hard constraints derived from physical and geometric considerations:

### 4.1 Lower Bound: Resolution Constraint (γ_min)

**Requirement:** The baseline must be sufficiently large to achieve a target signal-to-noise ratio (SNR) for slope angle estimation.

**Derivation:**
Given sensor precision $\sigma_{SBES}$, the angular estimation error is:
$$\sigma_\alpha = \frac{\sigma_{SBES}}{s(\gamma)} = \frac{\sigma_{SBES}}{2h\tan(\gamma)}$$

For a target SNR ratio $k_{res} = \frac{s}{\sigma_{SBES}}$ (typically 100:1), we require:
$$s(\gamma) \geq k_{res} \cdot \sigma_{SBES}$$

This yields:
$$\gamma_{min} = \arctan\left(\frac{k_{res} \cdot \sigma_{SBES}}{2h}\right)$$

**Numerical example:** With h = 3.0 m, $\sigma_{SBES}$ = 0.005 m, $k_{res}$ = 100:
$$\gamma_{min} = \arctan\left(\frac{100 \times 0.005}{2 \times 3.0}\right) = \arctan(0.0833) \approx 4.76°$$

### 4.2 Upper Bound: Terrain Intersection Constraint (γ_max)

**Requirement:** SBES rays must physically intersect the terrain surface, even on maximum slope.

The constraint depends on the terrain-following control strategy:

#### Scenario 1: Perfect Terrain-Following (Optimistic)
Robot orientation perfectly matches terrain slope: $\phi_{robot} = \alpha_{terrain}$, $\theta_{robot} = \beta_{terrain}$

In this ideal case, rays remain perpendicular to robot frame = perpendicular to terrain:
$$\gamma_{max}^{(1)} = \frac{\pi}{2} - \frac{\phi_{beam}}{2} \approx 88.0°$$

#### Scenario 2: Tracking with Error (Practical)
Robot tracks terrain with typical error $\delta_{tracking}$ (e.g., 5-10°):
$$\gamma_{max}^{(2)} = \frac{\pi}{2} - \delta_{tracking} - \frac{\phi_{beam}}{2}$$

**Example:** $\delta_{tracking}$ = 10° → $\gamma_{max}^{(2)}$ = 78.0°

#### Scenario 3: Fixed Horizontal Orientation (Conservative)
Robot maintains horizontal attitude regardless of terrain slope (worst case):
$$\gamma_{max}^{(3)} = \frac{\pi}{2} - \beta_{\text{terrain,max}} - \frac{\phi_{beam}}{2}$$

**Example:** $\beta_{\text{terrain,max}}$ = 80° → $\gamma_{max}^{(3)}$ = 8.0°

**Note:** This is extremely restrictive and only applies if terrain-following control is absent.

#### Scenario 4: Current Orientation (Adaptive)
Uses actual robot pitch and terrain slope at evaluation time. For typical same-direction pitch (robot follows terrain):
$$\gamma_{max}^{(4)} \approx \frac{\pi}{2} - \big|\theta_{robot} - \beta_{\text{terrain,max}}\big| - \frac{\phi_{beam}}{2} - \delta_{tracking}$$
If robot and terrain are tilted in opposite directions, replace the difference with a sum: $\;\big|\theta_{robot}\big| + \beta_{\text{terrain,max}} + \delta_{tracking}$.

**Implementation:** The algorithm allows selection of scenario via the `SCENARIO` parameter (1-4), with Scenario 2 (tracking with error) recommended for realistic applications.

### 4.3 Feasibility Check

After constraint application, the algorithm verifies:
$$\gamma_{min} < \gamma_{max}$$

If this condition fails, the problem is infeasible with current parameters. Solutions:
1. Reduce $\beta_{\text{terrain,max}}$ (limit operational terrain)
2. Improve tracking performance (reduce $\delta_{tracking}$)
3. Decrease $k_{res}$ requirement (accept lower resolution)
4. Implement active terrain-following control (use Scenario 1 or 2)

---

## 5. WEIGHT SELECTION AND TUNING

### 5.1 Weight Interpretation

The weights $w_1, w_2, w_3, w_4$ encode the relative importance of each objective:

| Weight | Objective | Physical Meaning | Typical Value |
|--------|-----------|------------------|---------------|
| $w_1$ | Resolution | Precision of slope estimation | 0.30 (30%) |
| $w_2$ | Coverage | Anticipation and reaction time | 0.20 (20%) |
| $w_3$ | Robustness | Acoustic signal quality | 0.30 (30%) |
| $w_4$ | Uncertainty | Geometric stability margin | 0.20 (20%) |

**Constraint:** $\sum_{i=1}^4 w_i = 1$ (automatic normalization in implementation)

### 5.2 Application-Specific Tuning Guidelines

**High-speed operations (v > 0.5 m/s):**
- Increase $w_2$ (coverage) to 0.35-0.40
- Decrease $w_4$ (uncertainty) to 0.15
- Rationale: Need more look-ahead distance at higher velocities

**Precision mapping missions:**
- Increase $w_1$ (resolution) to 0.40-0.45
- Decrease $w_2$ (coverage) to 0.15
- Rationale: Prioritize slope estimation accuracy over speed

**Challenging terrain ($\beta_{terrain,max}$ > 45°):**
- Increase $w_4$ (uncertainty) to 0.30-0.35
- Decrease $w_3$ (robustness) to 0.20
- Rationale: Ensure geometric feasibility on steep slopes

**Noisy acoustic environment:**
- Increase $w_3$ (robustness) to 0.40
- Adjust others proportionally
- Rationale: Maximize signal quality in difficult conditions

### 5.3 Sensitivity Analysis

The implemented algorithm automatically generates sensitivity plots showing how the optimal γ* varies with weight combinations. Key observations:

1. **Resolution-Coverage trade-off:** Higher $w_1$ or $w_2$ → larger γ*
2. **Robustness anchor:** Higher $w_3$ → γ* pulled toward 25° (empirical optimum)
3. **Stability limit:** Higher $w_4$ → γ* decreases, especially at high $\beta_{\text{terrain,max}}$

---

## 6. OPTIMIZATION ALGORITHM

### 6.1 Method Selection: Bounded Scalar Optimization

The problem structure—single decision variable with smooth, well-behaved objectives—admits efficient solution via **bounded scalar optimization**.

**Implementation:** MATLAB's `fminbnd` function (or equivalent in other languages)
- Algorithm: Golden section search + parabolic interpolation
- Convergence: Guaranteed for continuous functions on closed intervals
- Computational cost: O(log(ε⁻¹)) function evaluations

**Problem transformation:** Since `fminbnd` minimizes, we optimize:
$$\gamma^* = \arg\min_{\gamma \in [\gamma_{min}, \gamma_{max}]} \left[-J(\gamma)\right]$$

### 6.2 Algorithm Workflow

```
ALGORITHM: SBES_Angle_Optimization
─────────────────────────────────────────────────────────────
INPUT:
  System parameters: h, v, t_react, σ_SBES, φ_beam, f_ping
  Environment: β_terrain_max, θ_robot, δ_tracking  (pitch-only)
  Tuning: w1, w2, w3, w4, s_ref, k_c, γ_opt, σ_γ
  Numerical: tolerance ε = 10⁻⁸

OUTPUT:
  γ*: Optimal SBES mounting angle
  J(γ*): Objective function value
  Metrics: baseline, look-ahead, precision, stability
─────────────────────────────────────────────────────────────

STEP 1: Compute Lower Constraint
  γ_min ← arctan(k_res · σ_SBES / (2h))
  Validate: γ_min ∈ [0°, 45°] (sanity check)

STEP 2: Compute Upper Constraint
  Select scenario ∈ {1, 2, 3, 4} based on control capability
  
  SWITCH scenario:
    CASE 1 (Perfect tracking):
      γ_max ← π/2 - φ_beam/2
    CASE 2 (Tracking with error):
      γ_max ← π/2 - δ_tracking - φ_beam/2
    CASE 3 (Fixed horizontal):
      γ_max ← π/2 - β_terrain_max - φ_beam/2
    CASE 4 (Current orientation):
      γ_max ← π/2 - |θ_robot - β_terrain_max| - δ_tracking - φ_beam/2
  END SWITCH
  
  IF γ_max ≤ 0:
    ERROR: "Infeasible constraints - cannot intersect terrain"
  SUGGEST: Reduce β_terrain_max or improve tracking
  END IF
  
  IF γ_max ≤ γ_min:
    ERROR: "Resolution requirement incompatible with geometry"
    SUGGEST: Decrease k_res or increase h
  END IF

STEP 3: Define Objective Function
  objective_neg(γ) ← -J(γ) where:
    
    // Component 1: Resolution
    baseline_robot ← 2h·tan(γ)
  baseline_eff ← baseline_robot · cos(θ_robot)
    f_res ← tanh(baseline_eff / s_ref)
    
    // Component 2: Coverage
    d_ahead ← h·tan(γ)
    f_cov ← 1 - exp(-k_c·(d_ahead - v·t_react))
    
    // Component 3: Robustness
    f_rob ← exp(-(γ - γ_opt)² / (2σ_γ²))
    
    // Component 4: Uncertainty
  effective_slope ← |θ_robot - β_terrain_max| + δ_tracking
  θ_eff ← γ + effective_slope
  denom ← cos(θ_eff) / cos(effective_slope)
    f_unc ← tanh(max(denom, 0.01) / 0.3)
    
    // Weighted combination
    J ← w1·f_res + w2·f_cov + w3·f_rob + w4·f_unc
    RETURN -J  // Negate for minimization

STEP 4: Bounded Optimization
  options ← {TolX: ε, MaxFunEvals: 500, Display: 'off'}
  [γ*, J_neg*] ← fminbnd(objective_neg, γ_min, γ_max, options)
  J* ← -J_neg*

STEP 5: Compute Operational Metrics
  baseline_opt ← 2h·tan(γ*) · cos(θ_robot)
  d_ahead_opt ← h·tan(γ*)
  σ_α_opt ← σ_SBES / baseline_opt
  SNR_angular ← baseline_opt / σ_SBES
  t_available ← d_ahead_opt / v
  margin_time ← t_available - t_react
  
  // Stability assessment
  θ_eff_opt ← γ* + (|θ_robot - β_terrain_max| + δ_tracking)
  cos_margin ← cos(θ_eff_opt)
  
  IF cos_margin > 0.1:
    status ← "STABLE"
  ELSE IF cos_margin > 0:
    status ← "MARGINAL"
  ELSE:
    status ← "UNSTABLE"
  END IF

STEP 6: Generate Visualization
  FOR γ ∈ linspace(γ_min, γ_max, 200):
    Evaluate J(γ) and all components
  END FOR
  
  Plot figures:
    - Figure 1: Objective function and components vs γ
    - Figure 2: Geometric constraints and feasibility regions
  - Figure 3: Pitch sensitivity analysis

RETURN γ*, J*, metrics, status
─────────────────────────────────────────────────────────────
```

### 6.3 Computational Complexity

**Function evaluations per optimization:** ~50-100 (depends on ε)
**Wall-clock time:** < 1 second on modern hardware
**Memory usage:** O(1) for optimization, O(n) for visualization (n = plot points)

**Scalability:** The algorithm is inherently fast due to:
1. Single decision variable (1D search space)
2. Smooth, continuous objectives (efficient interpolation)
3. Explicit constraint handling (no iterative projection)

---

## 7. IMPLEMENTATION DETAILS

### 7.1 Numerical Stability Considerations

**Division by zero protection:**
- In $f_{unc}$ denominator: Use `max(cos(θ_eff), 0.001)` instead of raw `cos(θ_eff)`
- In baseline calculations: Constraint γ_min > 0 ensures tan(γ) is never zero

**Exponential overflow prevention:**
- Coverage component: Argument of exp() is always negative, preventing overflow
- Robustness component: Gaussian has bounded domain, no overflow risk

**Angle domain conventions:**
- **Internal computations:** All angles in radians
- **User interface:** Display angles in degrees via `rad2deg()`
- **Conversion points:** Input parsing and output formatting only

### 7.2 Code Structure (MATLAB Implementation)

```matlab
%% optimization_script.m
% Main sections:

% SECTION 1-2: Parameter Definition
%   - System parameters (h, v, t_react)
%   - Sensor specifications (σ_SBES, φ_beam, f_ping)
%   - Robot orientation (θ, δ_tracking)  % pitch-only

% SECTION 3-4: Constraint Computation
%   - γ_min from SNR requirement
%   - γ_max from geometric feasibility (scenario selection)

% SECTION 5-6: Objective Function Setup
%   - Weight definition and normalization
%   - Component parameter tuning

% SECTION 7: Optimization Execution
%   - Call fminbnd with objective_function handle
%   - Extract optimal γ* and J(γ*)

% SECTION 8: Metrics Computation
%   - Baseline, look-ahead, precision
%   - Stability assessment
%   - Component contributions

% SECTION 9-11: Visualization
%   - Sensitivity plots (J vs γ)
%   - Geometric constraint visualization
%   - Pitch impact analysis

% LOCAL FUNCTIONS:
%   objective_function(γ, params...) → J(γ)
%   compute_ray_terrain_angle(γ, θ, β, beam, δ) → θ_eff
```

### 7.3 Helper Function: Ray-Terrain Angle Computation (Pitch-only)

This function calculates the effective angle between SBES ray and terrain normal in the pitch axis:

```matlab
function theta_eff = compute_ray_terrain_angle(gamma, theta_robot, beta_terrain, beamwidth, delta_tracking)
  % Account for beam edge (worst case)
  gamma_edge = gamma + beamwidth/2;

  % Ray angle from robot vertical after pitch rotation
  % Relative to terrain: |(gamma + theta) - beta| + δ
  theta_eff = abs((gamma_edge + theta_robot) - beta_terrain) + delta_tracking;
end
```

**Physical interpretation:**
- `gamma_edge`: Outer edge of acoustic beam (worst case)
- `abs((gamma+θ) - β)`: Relative misalignment robot vs terrain
- `+ δ`: Tracking error margin

### 7.4 Objective Function Implementation

```matlab
function J = objective_function(gamma, h, v, t_react, beta_max, ...
        theta, delta, s_ref, k_c, gamma_opt, sigma_gamma, ...
        w1, w2, w3, w4, denom_min, scale_unc)

  % 1. RESOLUTION COMPONENT (pitch-only)
  baseline_robot = 2 * h * tan(gamma);
  baseline_eff = baseline_robot * cos(theta);
  f_res = tanh(baseline_eff / s_ref);

  % 2. COVERAGE COMPONENT
  d_ahead = h * tan(gamma);
  f_cov = 1 - exp(-k_c * (d_ahead - v*t_react));

  % 3. ROBUSTNESS COMPONENT
  f_rob = exp(-(gamma - gamma_opt)^2 / (2*sigma_gamma^2));

  % 4. UNCERTAINTY COMPONENT (geometry)
  effective_slope = abs(theta - beta_max) + delta;
  theta_eff = gamma + effective_slope;
  denom = cos(theta_eff) / cos(effective_slope);
  f_unc = tanh(max(denom, denom_min) / scale_unc);

  % WEIGHTED COMBINATION
  J = w1*f_res + w2*f_cov + w3*f_rob + w4*f_unc;
end
```

**Note:** This function is called ~50-100 times during optimization. Efficiency is achieved through:
- Vectorized operations (no loops)
- Direct mathematical expressions (no iterative solvers)
- Minimal temporary variable allocation

---

## 8. RESULTS AND INTERPRETATION

### 8.1 Typical Optimization Output

**Example configuration (pitch-only):**
- System: h = 3.0 m, v = 0.1 m/s, t_react = 0.5 s
- Sensor: σ_SBES = 5 mm, φ_beam = 4.0°, f_ping = 20 Hz
- Environment: β_terrain_max = 80°, δ_tracking = 10°
- Weights: w1 = 0.30, w2 = 0.20, w3 = 0.30, w4 = 0.20
- Scenario: 2 (tracking with error)

**Constraint computation:**
```
γ_min = arctan(100 × 0.005 / (2 × 3.0)) = 4.76°
γ_max = 90° - 10° - 2° = 78.00°
Feasible range: [4.76°, 78.00°]
```

**Optimization result:**
```
╔════════════════════════════════════════╗
║ Optimal angle: γ* = 25.43° = 0.444 rad ║
║ Objective value: J(γ*) = 0.8043        ║
╚════════════════════════════════════════╝
```

**Operational metrics:**
```
GEOMETRY:
  - Baseline (effective): 2.853 m
  - Look-ahead distance: 1.427 m
  - SBES range (nominal): 3.322 m
  - Footprint area: 8.14 m²

PRECISION:
  - Angular estimation error: σ_α = 1.75 mrad = 0.100°
  - Angular SNR: 571:1

TIMING:
  - Time available: 14.27 s
  - Time required: 0.50 s
  - Safety margin: 2753%

STABILITY:
  - Ray-terrain angle: 115.43°
  - Status: ⚠ UNSTABLE
  - Recommendation: Use Scenario 1 or reduce β_terrain_max
```

**Component contributions:**
```
f_resolution  = 0.9934  →  w1·f1 = 0.2980
f_coverage    = 0.9990  →  w2·f2 = 0.1998
f_robustness  = 0.9996  →  w3·f3 = 0.2999
f_uncertainty = 0.0333  →  w4·f4 = 0.0067
──────────────────────────────────────────
                   Total J = 0.8043
```

### 8.2 Interpretation Guidelines

**Objective value assessment:**
- **High J(γ*) > 0.80:** Well-balanced solution, all objectives reasonably satisfied
- **Moderate J(γ*) = 0.60-0.80:** Acceptable but some objectives compromised
- **Low J(γ*) < 0.60:** Conflicting requirements, consider relaxing constraints or adjusting weights

**Stability status:**
- **STABLE (cos(θ_eff) > 0.1):** Safe operation guaranteed on specified terrain
- **MARGINAL (0 < cos(θ_eff) ≤ 0.1):** Verify in simulation/field tests before deployment
- **UNSTABLE (cos(θ_eff) ≤ 0):** Geometric infeasibility—reduce γ* or improve terrain tracking

---

## 9. PARAMETER TUNING RECOMMENDATIONS

### 9.1 Weight Configuration Examples

**Precision mapping mission:**
```
w1 = 0.40  (resolution)    ← Increased
w2 = 0.15  (coverage)      ← Decreased
w3 = 0.30  (robustness)
w4 = 0.15  (uncertainty)
```
→ Favors larger baselines for better slope estimation

**High-speed survey:**
```
w1 = 0.25  (resolution)
w2 = 0.35  (coverage)      ← Increased
w3 = 0.25  (robustness)
w4 = 0.15  (uncertainty)
```
→ Prioritizes look-ahead distance for faster navigation

**Steep terrain operation:**
```
w1 = 0.25  (resolution)
w2 = 0.20  (coverage)
w3 = 0.20  (robustness)
w4 = 0.35  (uncertainty)   ← Increased
```
→ Emphasizes geometric stability on challenging slopes

### 9.2 Scenario Selection Guidelines

**Scenario 1 (Perfect tracking):** Research/simulation only—idealized case
**Scenario 2 (Tracking with error):** **RECOMMENDED** for practical systems with active control
**Scenario 3 (Fixed horizontal):** Extremely conservative—use only if no terrain-following control
**Scenario 4 (Current orientation):** For real-time adaptation based on measured attitude

---

## 10. CONCLUSIONS

This document has presented a comprehensive framework for SBES angle optimization in AUV terrain-following applications. The key contributions are:

1. **Multi-objective formulation** balancing resolution, coverage, robustness, and stability
2. **Explicit geometric constraints** derived from 3D ray-terrain intersection analysis
3. **Efficient implementation** via bounded scalar optimization (< 1 second runtime)
4. **Practical guidance** for weight tuning and scenario selection
5. **Extensive visualization** for sensitivity analysis and parameter exploration

**Recommended workflow:**
1. Characterize mission parameters (h, v, β_terrain_max)
2. Select terrain-following scenario (1-4) based on control capability
3. Choose weights (w1-w4) based on mission priorities
4. Run optimization to obtain γ*
5. Verify stability status and operational metrics
6. Validate in simulation before field deployment

**Key takeaway:** The optimal angle γ* is **not universal** but depends critically on:
- Terrain characteristics (β_terrain_max)
- Vehicle dynamics (v, t_react)
- Mission objectives (weight configuration w1-w4)
- Control performance (δ_tracking, scenario)

The algorithm provides a principled, computationally efficient method to find the best γ* for each specific application context.

---

## APPENDIX: MATLAB QUICK START

```matlab
% FILE: optimization_script.m
% Minimal example for quick testing

% 1. Set parameters
h = 3.0;                        % Altitude [m]
v = 0.1;                        % Velocity [m/s]
t_react = 0.5;                  % Reaction time [s]
beta_terrain_max = deg2rad(30); % Max terrain pitch slope (use 30° for stable results)
tracking_error = deg2rad(5);     % Tracking error

% 2. Set weights
w1 = 0.30;  % Resolution
w2 = 0.20;  % Coverage
w3 = 0.30;  % Robustness
w4 = 0.20;  % Uncertainty

% 3. Choose scenario
SCENARIO = 2;  % 1=Perfect, 2=Tracking, 3=Fixed, 4=Current

% 4. Run optimization
run('optimization_script.m');

% 5. Results displayed automatically:
%    - Optimal angle γ*
%    - Operational metrics
%    - Three visualization figures
```

**Execution time:** ~1-2 seconds on typical hardware

**Output files:** None (all output to console and figures)

**Dependencies:** None (base MATLAB only, no toolboxes required)

---

*Document version 2.1 | Academic Technical Report | Last updated: November 2, 2025*
