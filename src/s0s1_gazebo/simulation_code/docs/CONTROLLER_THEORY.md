# PID Control Theory for Robot Arms

## Core Equation

A PID controller produces torque based on error feedback:

$$\tau = K_p \cdot e + K_i \cdot \int e \, dt + K_d \cdot \frac{de}{dt}$$

Where:
- $e = q_{\text{desired}} - q_{\text{measured}}$ = position error (rad)
- $K_p$ = proportional gain (N⋅m/rad)
- $K_i$ = integral gain (N⋅m/(rad⋅s))
- $K_d$ = derivative gain (N⋅m⋅s/rad)

Physically, this means:
1. **Proportional term**: Act now (stiffness)
2. **Integral term**: Accumulate error (bias removal)
3. **Derivative term**: Predict & damp (smoothing)

## P: Proportional Control Only

### Equation

$$\tau = K_p \cdot e$$

### Physics

**Stiffness Model**:
- Think of $K_p$ as the stiffness of an invisible spring pulling toward the target
- Higher $K_p$ = stiffer spring = faster response but more overshoot

### Characteristic Behavior

**Static Case** (holding a position):
- If the robot experiences a constant external load (e.g., gravity), it will drift
- There will always be a residual error to balance the load
- Error at steady-state: $e_{\infty} = F_{\text{load}} / K_p$
- **Example**: Under 1 N external force, if $K_p = 10$ N⋅m/rad, you get 0.1 rad error

**Dynamic Case** (moving to a new position):
- Faster rise time with higher $K_p$
- But if $K_p$ is too high, the spring "overshoots" and bounces back
- No damping → oscillations never stop (marginally stable)

**Why P Alone Fails**:
1. Cannot eliminate steady-state error under constant load
2. Cannot damp oscillations (energy just bounces)
3. Risk of saturation (control torque > motor limit)

### Tuning Tradeoff

```
        Low Kp              High Kp
Rise time:    Slow            Fast
Steady-state error: Large      Small
Overshoot:    None            Large
Oscillation:  None            Persistent
Stability:    Safe            Risky
Robustness:   Weak            Strong
```

## PD: Proportional-Derivative Control

### Equation

$$\tau = K_p \cdot e + K_d \cdot \frac{de}{dt}$$

### Physics

**Spring-Damper Analogy**:
- $K_p$: spring stiffness (restores to target)
- $K_d$: damper coefficient (dissipates energy)
- Together: 2nd-order mechanical system

### How Derivative Damping Works

Imagine error oscillating around zero:

```
e(t) = A * sin(ωt)     Position error (sine wave)
de/dt = Aω * cos(ωt)   Velocity of error (cosine wave, leads by 90°)

τ_p = Kp * A * sin(ωt)           Proportional (follows error)
τ_d = Kd * Aω * cos(ωt)          Derivative (opposes velocity of error)
```

When $e > 0$ (position lags target):
- $\tau_p > 0$ pulls toward target
- If $de/dt > 0$ (error growing), $\tau_d > 0$ also pulls, **accelerating recovery**
- If $de/dt < 0$ (error shrinking), $\tau_d < 0$ opposes motion, **preventing overshoot**

**Energy Dissipation**:
- Damping converts kinetic energy of the error oscillation into heat
- Energy per cycle dissipated: $E = \int \tau_d \cdot \dot{q} \, dt$
- Higher $K_d$ → more damping → faster energy loss → fewer oscillations

### Characteristic Behavior

**Well-tuned PD**:
1. Strong $K_p$ for fast response
2. Moderate $K_d$ to prevent overshoot
3. Result: Smooth, quick approach to target with minimal ringing

**Under-damped** ($K_d$ too low):
- Despite high $K_p$, error oscillates multiple times
- Takes longer to settle than optimally damped

**Over-damped** ($K_d$ too high):
- Response sluggish; takes forever to reach target
- But does so smoothly with no overshoot
- **Problem**: Tries to follow every disturbance slowly → poor disturbance rejection

### Tuning Principle: Critical Damping

For a 2nd-order system, optimal damping is **critical damping**:

$$K_d = 2 \sqrt{K_p \cdot m}$$

Where $m$ is the joint inertia (kg⋅m²).

- **ζ = 1** (critical): Fastest response without overshoot ✓
- **ζ < 1** (under-damped): Oscillates; lower ζ = more oscillation
- **ζ > 1** (over-damped): Slow; higher ζ = more sluggish

**Practice**: Robot inertia varies per joint → per-joint tuning of $(K_p, K_d)$ ratio needed.

## PI: Proportional-Integral Control

### Equation

$$\tau = K_p \cdot e + K_i \cdot \int_0^t e(\tau) \, d\tau$$

### Physics

**Slow Bias Correction**:
- Proportional term reacts immediately
- Integral term accumulates error over time
- If there's persistent error, integral slowly increases torque until error disappears

**Time-Constant Interpretation**:
- Recall: $\frac{1}{K_i / K_p}$ = approximate time constant for integral to "take over"
- Small $K_i$: Integral acts slowly (safe but sluggish error removal)
- Large $K_i$: Integral acts quickly (aggressive but risk of overshoot)

### Problem: Integral Windup

**Scenario**:
1. You command the robot to move to an unreachable position (e.g., beyond mechanical limit)
2. Error stays positive: $e > 0$ always
3. Integral keeps accumulating: $\int e \, dt \to \infty$
4. Control torque saturates at maximum: $\tau = \tau_{\max}$
5. Motor hits the limit and can't move further
6. But integral doesn't know this → keeps accumulating
7. When you finally move elsewhere, the integrator has a huge "debt" to pay off
8. Result: Overshoot, oscillation, instability

**Solution: Anti-Windup**:
```python
# Clamp integral state to prevent unbounded growth
self.i_state[jn] = np.clip(self.i_state[jn], -i_limit, i_limit)

# When torque is saturated, freeze integral accumulation (more advanced)
if abs(tau) >= tau_limit:
    self.i_state[jn] = self.i_state[jn]  # Don't update
```

### Characteristic Behavior

**Strong Ki**:
- Quickly removes constant bias (e.g., gravity load)
- But can overshoot because integral overshoots before realizing it

**Weak Ki**:
- Slowly builds up → minimal overshoot risk
- But takes a long time to eliminate error

**No damping (Kd = 0)**:
- P term reacts immediately
- I term catches up later
- Can oscillate if Ki too aggressive
- Usually needs moderate-to-high Kp to avoid explosion

### When PI is Used

- **Robotic arms**: Usually NOT alone; need damping
- **Temperature control**: Perfect (slow, thermal system, no oscillation risk)
- **Industrial setpoint regulation**: Common (constant load compensation)

## PID: Full Control

### Equation

$$\tau = K_p \cdot e + K_i \cdot \int e \, dt + K_d \cdot \frac{de}{dt}$$

### The Complete Picture

1. **$K_p$**: Fast response, creates stiffness
2. **$K_i$**: Removes bias, compensates load
3. **$K_d$**: Damps oscillations, prevents overshoot

**Ideal Behavior**:
- Fast rise time (from $K_p$ and $K_d$)
- No overshoot (from $K_d$ and anti-windup)
- Zero steady-state error (from $K_i$)
- Smooth motion (from $K_d$ damping & $K_i$ gradualness)

### Tuning Strategy

**1. Start with P**:
- Set desired rise time; adjust $K_p$ until you get 20–30% overshoot
- This is your "baseline" stiffness

**2. Add D to damp**:
- Set $K_d \approx 2\sqrt{K_p \cdot m}$ (critical damping estimate)
- Adjust down if response too sluggish
- Adjust up if still oscillating

**3. Add I for bias removal**:
- Set $K_i$ low (e.g., $0.1 K_p$)
- Slowly increase until steady-state error disappears
- Stop before overshoot appears

**Example for shoulder_lift** (estimated inertia $m \approx 0.3$):
```
Step 1: Kp = 30 → rise_time ~0.9s, 25% overshoot
Step 2: Kd = 2*sqrt(30*0.3) = 4.24 → still some ringing
        Try Kd = 2.0 → acceptable compromise
Step 3: Ki = 0.3 (start) → slow zero error buildup
        Increase to Ki = 0.6 → faster but still stable
        Stop at Ki = 0.6; higher causes overshoot
```

Result: **Kp=30, Kd=2.0, Ki=0.6** for this joint.

## Disturbance Rejection

### The Robustness Question

**How well does the controller reject disturbances?**

Consider a sinusoidal disturbance:
$$F_{\text{dist}} = A \sin(\omega t)$$

Applied to the robot:
$$\ddot{q} = \frac{1}{J} (\tau_{\text{PID}} + F_{\text{dist}})$$

The PID responds:
$$\tau_{\text{PID}} = K_p e + K_d \dot{e} + K_i \int e \, dt$$

**Magnitude of error under disturbance**:
- At **low frequencies** ($\omega \ll K_p/J$): Error ~ $\frac{F}{K_p}$ (stiffness matters)
  - Higher $K_p$ → smaller error ✓
  
- At **high frequencies** ($\omega \gg K_p/J$): Error ~ $\frac{F}{J\omega^2}$ (inertia matters)
  - Derivative damping helps: $K_d \dot{e}$ fights velocity of error
  - But ultimately can't overcome inertia at very high frequencies

**Practical Insight**:
- **Strong $K_p$ + $K_d$**: Good disturbance rejection across frequency range
- **Weak $K_p$**: Robot sags under load / disturbance
- **No $K_d$**: Oscillates excessively when disturbed

## Perturbation Profile in This Experiment

### Scientifically Designed Disturbance

Three components:

1. **Sinusoidal** (low-frequency fatigue):
   - $\tau_{\sin} = 0.8 \sin(2\pi \cdot 0.5 \cdot t)$ N⋅m
   - 0.5 Hz (2-second period) = **slow, sustained load variation**
   - Tests steady-state & integral action
   
2. **Colored Noise** (broadband):
   - Low-pass filtered white noise (time constant 0.25s)
   - Std dev 0.25 N⋅m
   - Tests high-frequency damping
   
3. **Impulses** (sudden shocks):
   - 12% probability per second; 2 N⋅m, 50ms duration
   - Random timing: **unpredictable "hits"**
   - Tests transient response & stability

### Why This Combination?

- **Sinusoidal**: Realistic load from gravity / mechanism
- **Noise**: Realistic sensor/actuator imperfections
- **Impulses**: Worst-case sudden disturbances

Together: Forces controllers to prove **robustness**, not just tracking under ideal conditions.

## Expected Results

### P Family Trends

| Kp | Rise Time | SSE | Oscillation | Disturbance Rejection |
|----|-----------|-----|-------------|----------------------|
| Low | Slow | Large | None | Weak (gets blown around) |
| Med | Fast | Moderate | Medium | Moderate (stiff but ringing) |
| High | Very Fast | Small | Large | Strong (stiff but unstable) |

**Insight**: Pure proportional control is brittle—you must choose between speed and stability.

### PD Family Trends

| Kp + Kd | Rise Time | SSE | Oscillation | Disturbance Rejection |
|---------|-----------|-----|-------------|----------------------|
| Conservative | Slow | Moderate | None | Weak |
| **Balanced** | **Fast** | **Moderate** | **Light** | **Good** |  ← Sweet spot
| Over-damped | Very Slow | Moderate | None | Weak (sluggish) |

**Insight**: PD adds a "shock absorber"—most important gain improvement over P alone.

### PI Family Trends

| Kp + Ki | Rise Time | SSE | Oscillation | Disturbance Rejection |
|---------|-----------|-----|-------------|----------------------|
| Low Ki | Fast | Slow decay to zero | Light | Moderate (eventual zero) |
| **Moderate Ki** | Fast | **Medium (settles)** | Medium | Moderate |  ← Practical choice
| High Ki | Fast | Overshoot settling to zero | Large | Risky (integral overshoot) |

**Insight**: Integral action is powerful but must be limited to avoid instability and windup.

### PID Family Trends

| Balanced | Rise Time | SSE | Oscillation | Disturbance Rejection |
|----------|-----------|-----|-------------|----------------------|
| All low | Slow | Moderate | None | Weak |
| Conservative | Fast | Good | Light | Good |
| **Balanced** | **Fast** | **Zero** | **Light** | **Excellent** |  ← Best overall
| Aggressive | Very Fast | Zero | Moderate | Strong but risky |

**Insight**: PID is the "Swiss Army knife"—if tuned well, beats all others on all metrics.

## References & Further Reading

### Theory
- K.J. Åström & R.M. Murray: "Feedback Systems" (Princeton)
  - Chapter 11: PID Control, excellent for depth
  
- B. Wie: "Space Vehicle Dynamics and Control" 
  - Chapter 3: Classical control & tuning rules
  
- S. Skogestad: "Simple Analytic Rules for Model Reduction and PID Tuning"
  - Practical step-by-step rules for your own robots

### Practice
- BrainVCR: "Tuning PID Loops" (YouTube series)
  - Visual demonstrations of overshoot, windup, etc.

---

**Next**: Return to `INTERPRETING_RESULTS.md` to apply this theory to your actual experiment data!
