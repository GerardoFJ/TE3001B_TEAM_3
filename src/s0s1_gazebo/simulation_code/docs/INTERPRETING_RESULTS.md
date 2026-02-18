# Interpreting Results

## Quick Lookup: What Does This Mean?

### Steady-State Error (SSE)

**Definition**: The final position error when holding a target under disturbance.

```
SSE = |q_desired - q_measured| at the end of the hold phase
```

**Interpretation**:
- **≈ 0 rad** (excellent): Controller completely compensates bias
  - Requires **non-zero Ki** and long enough integral accumulation
  - P and PD controllers cannot achieve this!
  
- **0.05 rad** (small): Acceptable for many applications
  - P control with moderate Kp typically here
  - Good PD damping helps reduce slightly
  
- **0.15 rad** (moderate): Noticeable error
  - Low Kp or very weak integral action
  - Not suitable for precise positioning
  
- **> 0.3 rad** (large): Poor control
  - Too-low gains or disturbance too strong
  - May indicate saturation or controller failure

**Example**: Under disturbance, shoulder_pan drifts 0.087 rad (≈5°) at rest = **poor tracking**.

### Rise Time

**Definition**: Time for the position to reach 90% of the target.

```
rise_time = t such that |q_error| < 0.1 * |q_initial|
```

**Interpretation**:
- **< 0.3s** (very fast): High Kp; risks overshoot
  - Good for responsive control, but may oscillate
  
- **0.3–0.8s** (fast): Balanced Kp
  - Typical for well-tuned PD control
  - Smooth transient
  
- **0.8–1.5s** (moderate): Kp reduced to avoid overshoot
  - Smooth but slower to respond to errors
  
- **> 1.5s** (slow): Very low Kp or high Kd damping
  - Conservative, stable, but sluggish behavior
  - May lose to disturbances

**Example**: "Elbow reaches 90% target in 0.47s" = **fast response, acceptable if damped well**.

### Overshoot

**Definition**: Peak deviation beyond the target as percentage of the target change.

```
overshoot_% = 100 * max(|q_error|) / |q_target - q_start|
```

**Interpretation**:
- **0%** (none): No ringing; very damped (high Kd or low Kp)
- **2–5%**: Tiny overshoot; excellent damping
  - Indicates good PD tuning
  
- **5–15%** (small): Acceptable industrial-grade control
  - One small peak above target, then settles
  
- **15–30%** (moderate): Noticeable but controlled
  - Might be acceptable depending on mechanical limits
  - Indicates under-damping
  
- **> 30%** (large): Significant overshoot
  - Under-damped; may trigger limits or fail
  - Reduce Kp or increase Kd

**Example**: 
- Wrist_roll overshoots by 22% → "Moderate overshoot, watch for mechanical damage"
- Shoulder_pan has 0.8% overshoot → "Excellent damping, minimal ringing"

### Oscillation Count & Frequency

**Definition**: Number of zero-crossings in the error signal → cycles; and dominant frequency via FFT.

**Interpretation**:
- **0 cycles** (no oscillation): Heavily damped; slow settling
  - Good for safety, bad for speed
  
- **1–2 cycles** (light ringing): Well-damped response
  - One or two small wiggles after reaching target
  - Typical of good PD control
  
- **3–5 cycles** (moderate oscillation): Under-damped
  - Multiple bounces; settling takes time
  - May indicate overshoot or weak Kd
  
- **> 5 cycles** (persistent**: Marginal stability
  - Energy not dissipated; could grow if disturbance aligns with oscillation
  - Dangerous; increase Kd immediately

**Frequency**:
- **Low (< 1 Hz)**: Slow oscillation; structural mode or heavy inertia
  - Common when Kp too high for joint mass
  
- **1–3 Hz**: Medium; typical of robot arm control loops
  - Normal range for PD control
  
- **> 5 Hz**: High-frequency; sensor noise or Kd too high
  - Derivative amplifies measurement noise
  - Reduce Kd or add low-pass filter

**Example**: "Oscillation: 2 cycles @ 1.8 Hz" → "Light ringing at typical control frequency; acceptable."

### Disturbance Rejection

**Definition**: A robustness metric; ratio of stable error response to disturbance amplitude.

```
rejection = 1 / (1 + overall_error)
```

- **0.85–1.0**: Excellent disturbance rejection
  - Controller quickly suppresses perturbations
  - High stiffness (Kp) and damping (Kd)
  
- **0.70–0.85**: Good rejection; moderate disturbance effects visible
  - Standard well-tuned controller
  
- **0.50–0.70**: Weak rejection; disturbance significantly affects motion
  - Low gain or weak integral action; accumulates error
  
- **< 0.50**: Poor rejection; controller barely tracks under disturbance
  - Insufficient stiffness; gains too low or stability marginal

**Example**: "Disturbance rejection: 0.81" → "Good robustness; controller handles perturbations well."

### Stability Classification

Determined by overshoot + oscillation count:

| Overshoot | Oscillation | Stability | Meaning |
|-----------|-------------|-----------|---------|
| < 10% | 0–1 | **Stable** | Excellent; no instability risk |
| 10–25% | 1–2 | **Stable** | Good; minor ringing |
| 25–40% | 2–4 | **Questionable** | Watch for growth; may need tuning |
| > 40% | > 4 | **Marginal** | High risk; likely unstable under worse conditions |

## Reading a Qualitative Table

### Example Table for P Family

| Combo ID | Name | Rise & Speed | Overshoot | Oscillation | Steady-State Error | Disturbance Rejection | Stability | Notes |
|----------|------|--------------|-----------|-------------|--------------------|-----------------------|-----------|-------|
| P-1 | Very Low Kp | Very slow | None | None | Large (>0.3 rad) | 0.65 | Stable | Compliant; barely responds to errors |
| P-2 | Low Kp | Slow | None | None | Moderate (~0.15) | 0.74 | Stable | Acceptable compromise; smooth motion |
| P-3 | Medium Kp | Moderate | Small (8%) | Light (1 cycle) | Small (~0.08) | 0.82 | Stable | Good speed & damping; near P-4 stability |
| P-4 | High Kp | Fast | Moderate (18%) | Moderate (2–3) | Small (~0.05) | 0.87 | Stable | Fast but ringing; borderline overshoot |
| P-5 | Very High Kp | Very fast | Large (35%) | Persistent (4+) | ≈ 0 | 0.91 | **Marginal** | Near instability; avoid in practice |

### Reading Trends

**Across the table**:
1. **Rise time decreases**: As Kp increases (P-1 → P-5), faster response
2. **Overshoot increases**: Stiffness without damping creates overshoot
3. **SSE decreases**: Higher Kp reduces steady-state error
4. **Oscillation increases**: Under-damped behavior at high Kp
5. **Stability degrades**: By P-5, marginal; risk of instability

**Insight**: P control trades off speed vs. stability; **cannot remove steady-state error**.

## Comparing Controller Families

### Example Cross-Family Comparison

| Family | Best Combo | SSE | Rise Time | Oscillation | Disturbance | Stability | Best For |
|--------|-----------|-----|-----------|-------------|-------------|-----------|----------|
| **P** | P-3 | 0.08 rad | 0.68s | 1 cycle | 0.82 | Stable | Speed with simplicity |
| **PD** | PD-3 | 0.08 rad | 0.52s | 0 cycles | 0.84 | Stable | **Smooth, fast, damped** |
| **PI** | PI-3 | ≈ 0.01 rad | 1.2s | 2 cycles | 0.78 | Stable | Precision positioning |
| **PID** | PID-2 | ≈ 0.01 rad | 0.65s | 1 cycle | 0.86 | Stable | **All-around best; balanced** |

### Key Observations

1. **P alone cannot reach zero error**: P-3 and P-4 leave ~0.05–0.08 rad residual error
2. **PD adds smoothness**: No overshoot in PD-3 vs. P-3 (1 cycle at P-3 becomes 0)
3. **PI removes bias**: PI-3 reaches near-zero error, but takes longer (1.2s rise time)
4. **PID combines all benefits**: PID-2 gets speed (0.65s), zero error, and stability

**Recommendation**: If system allows integrator (no extreme disturbances), **use PID**. Otherwise, **use PD** for robustness.

## Plotting and Visual Inspection

### Position vs. Time Plot

```
q_measured (orange line)
q_desired  (blue dashed)

┌─────────────────────────────────────┐
│    ___                              │  ← q_desired = 0°
│   /   \___            /\            │
│  /         \         /  \___        │  ← smooth approach, small overshoot
│ /           \       /       \___    │
└─────────────────────────────────────┘
  move_to    hold_zero  return  hold
  zero     (2.0s)       start  start
  (2.0s)               (2.0s)  (2.0s)
```

**Read it**:
1. **Sharpness of approach**: How fast does orange catch blue?
   - Steep → fast response (high Kp)
   - Gradual → slow response (low Kp)

2. **Peak overshoot**: Does orange cross blue and overshoot?
   - No crossing → very damped (high Kd)
   - Small crossing → well-damped
   - Large crossing → under-damped (low Kd)

3. **Settlement time**: How long for orange to stay on blue?
   - < 0.5s → fast settling
   - 0.5–1.0s → normal
   - > 1.0s → slow settling (too much damping or low Kp)

4. **Disturbance effects**: In hold phases, does orange wander?
   - Stays on blue ± 0.01 rad → good rejection
   - Drifts by 0.1+ rad → weak rejection

### Error vs. Time Plot

```
e_pos (orange = error)
qd (purple = velocity)

┌──────────────────────────────────┐
│   0.3─┐                          │ ← large initial error
│       │     ╱‾‾╲              ╱‾ │   (hump = overshoot)
│   0.1─┤    ╱    ╲             ╱  │
│       │   ╱      ╲    ╱\     ╱   │
│   0.0─┼──────────────────────────│ ← zero error
│  -0.1─┤  ╱ purple = velocity    │ (purple dips below
│       │ ╱                      ╲ │  zero = overshoot reversal)
│
└──────────────────────────────────┘
```

**Read it**:
1. **Error magnitude**: How high does orange spike?
   - < 0.05 rad → small error
   - 0.05–0.15 rad → moderate
   - > 0.15 rad → large error (poor tracking)

2. **Velocity ringing**: How many cycles does purple oscillate?
   - Smooth purple curve → well-damped
   - Wiggly purple → under-damped, multiple reversals

3. **Settling behavior**: Does orange approach zero asymptotically or overshoot?
   - Smooth approach → good Kd
   - Oscillatory approach → weak Kd

### Torque Plot

```
tau_pid (green)
tau_dist (red = disturbance)
tau_total (black = sum)

┌────────────────────────────────────┐
│  8  ┌─┐                            │ ← PID saturates (i_limit)
│  6  │ │  ╱──┐              ╱─     │
│  4  │ │ ╱    ╲_╱──┐      ╱_│_     │ ← disturbance spikes
│  2  │ │╱        ╲ │    ╱    │    │
│  0  ├─┼─────────────┼───────────┤ ← at equilibrium
│ -2  │ │             │  ╲        │
│ -4  │ │             │   ╲      │
│ -6  └─┴─────────────┴────╲────────│
│ -8                         ╲      │
└────────────────────────────────────┘
   move      hold       return  hold
```

**Read it**:
1. **PID magnitude**: How much torque is needed?
   - < 2 N⋅m → low effort, conservative control
   - 2–5 N⋅m → typical for SO101
   - > 5 N⋅m → aggressive, risk of saturation

2. **Saturation**: Does tau_total plateau flat?
   - No plateau → controller not limiting
   - Flat tops → torque limits engaged (may reduce effectiveness)

3. **Disturbance response**: Does tau_pid spike with tau_dist?
   - Rapid counter-spikes → good disturbance rejection (high Kp, Kd)
   - Sluggish response → weak rejection (low gains)

4. **Oscillation in torque**: Does tau_pid oscillate post-move?
   - Smooth decay → well-damped
   - Ringing → under-damped or sensor noise (too much Kd)

## Tips for Writing Your Report

### What NOT to Do

❌ "Kp = 25 produces an error of 0.087 rad."  
✓ "With Kp = 25, the steady-state error is 0.087 rad (≈5°), indicating that proportional control alone insufficient for precision positioning without integral action."

❌ "The plots show oscillation."  
✓ "The position traces exhibit 2–3 oscillation cycles with a dominant frequency of 1.8 Hz, consistent with an under-damped second-order system. Increasing Kd by 50% (Case PD-3) reduces this to light ringing, demonstrating the stabilizing effect of derivative damping."

### What TO Do

✅ Connect gains to physics:
- "Higher Kp increases stiffness → faster rise time → reduced steady-state error but increased overshoot as damping is insufficient."
- "Increasing Kd damps oscillations by dissipating kinetic energy → smoother settlement but at the cost of slower response if overdone."
- "Ki integrates error over time → eventually removes constant bias (e.g., load offset) but must be limited to prevent windup."

✅ Reference the data:
- "Table 3 shows that P-family combinations achieve similar steady-state errors (~0.08–0.15 rad) but differ dramatically in rise time (0.68–2.1s) and oscillation (0–4 cycles), illustrating the speed-stability tradeoff."

✅ Extract trends:
- "Across all PD combinations, rise time decreases monotonically with Kp (0.72 → 0.38s) while Kd prevents the oscillation growth seen in pure P control."

✅ Justify your choice:
- "PID-2 (balanced baseline) offers a practical compromise: 0.65s response time, zero steady-state error, and stable disturbance rejection (0.86) suitable for both speed and precision."

---

**Next**: See `CONTROLLER_THEORY.md` for deeper understanding of PID mechanics.
