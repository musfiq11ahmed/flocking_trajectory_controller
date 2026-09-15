# Code Review — Differential-Drive Target Tracking Controllers

**Files reviewed:** `unicycle_model_arc.py`, `unicycle_model_turn.py`, `README.md`
**System:** ESP32 differential-drive robot (track width 0.074 m, N20 encoder gear motors), overhead webcam + ArUco vision (4 corner tags define the arena grid via homography, 1 robot tag for pose), PC dispatcher → UDP → firmware PID (50 ms).
**Method:** static analysis by two independent reviewers (control/kinematics + systems/integration), cross-validated against closed-loop kinematic simulation (50 Hz Euler, ring-buffer vision latency).

**Scope note:** the vision pipeline, dispatcher, and firmware were *not* provided — only described in the README. Findings are split into **confirmed defects** (provable from the supplied code) and **integration risks** (require the missing code or hardware to confirm).

---

## Part 1 — Confirmed defects

### 1. CRITICAL — `arc` variant can orbit the target forever and never report arrival

`unicycle_model_arc.py:14–25` — the robot always drives forward at fixed v = 0.15 m/s with only proportional heading correction `w = 0.4·heading_error`. The closed-loop error dynamics are

```
ṙ = −v·cos(e)          ė = v·sin(e)/r − w
```

At e = ±90° the robot neither approaches nor retreats (ṙ = 0), and a **stable circular equilibrium** exists at r* = v/w. Simulation confirms the eigenvalues of the linearized system are stable (≈ −0.093 ± j0.30) — perturbations spiral *back onto the orbit*.

- With the deadband as written (defect #2 halves the gain): **r\* ≈ 0.51 m** — verified in simulation; the robot circles the target at ~0.5 m indefinitely, `done` never fires.
- Even with the deadband removed: r\* = 0.15 / (0.4·π/2) ≈ **0.24 m**, still 2.4× the 0.10 m stop tolerance.
- Raising K_w alone cannot fix this: keeping r* inside tolerance needs K_w > 0.95, but the wheel-speed headroom (0.05 m/s differential) caps K_w at ~0.43.

**Fix:** add a turn-in-place (or near-zero-v) phase gated below π/2, e.g.:

```python
HEADING_GATE = 0.50
if abs(heading_error) > HEADING_GATE:
    w = max(-W_TURN_MAX, min(W_TURN_MAX, 1.5 * heading_error))
    return -w * TRACK_WIDTH / 2.0, w * TRACK_WIDTH / 2.0, False
```

and/or scale forward speed down with heading error: `v = MIN_SPEED * max(0.0, math.cos(heading_error))`. Re-verify over a full (pose × target) grid, not just straight-ahead cases.

### 2. CRITICAL — the "minimum speed" deadband breaks the turning radius for *every* nonzero heading error (README claim is false)

`unicycle_model_arc.py:24, 36–39`; `README.md:12`. Because v is set to exactly MIN_SPEED = 0.15, *any* nonzero w pushes the inner wheel to 0.15 − |w|·0.037 < MIN_SPEED, so the deadband block **always fires** for any heading error ≠ 0. It bumps only the inner wheel back to 0.15, leaving the differential on the outer wheel alone:

| heading error | commanded w | **effective w** | intended radius | **actual radius** | actual v |
|---:|---:|---:|---:|---:|---:|
| 0.25 rad | 0.100 | 0.050 | 1.50 m | 3.04 m | 0.152 |
| 1.00 rad | 0.400 | 0.200 | 0.375 m | 0.787 m | 0.157 |
| π rad | 1.257 | 0.628 | 0.119 m | 0.276 m | 0.173 |

Effective K_w is **0.2, not 0.4**; turning radius is >2× intended everywhere; forward speed drifts up to 15% high mid-turn. This is the direct cause of defect #1's enlarged orbit, and it falsifies the README's claim of deadband scaling "without breaking the unicycle turning radius."

**Fix (preserves radius, keeps both wheels ≥ MIN_SPEED):**

```python
D_MAX = (MAX_SPEED - MIN_SPEED) / 2.0
w = max(-2*D_MAX/TRACK_WIDTH, min(2*D_MAX/TRACK_WIDTH, K_w * heading_error))
delta = w * TRACK_WIDTH / 2.0
v_center = MIN_SPEED + abs(delta)     # boost BOTH wheels equally
v_left, v_right = v_center - delta, v_center + delta
```

### 3. HIGH — NaN vision input → full-speed forward, in both files

Python's `min(0.20, nan)` returns 0.20, so the "clamp to safe range" **absorbs** NaN instead of propagating it. Verified: `calculate_wheel_velocities(nan, nan, 0, 1, 0)` → `(0.2, 0.2, False)` in **both** variants (in `turn`, `abs(nan) > 0.25` is False, so it silently takes the drive phase; the 10 cm stop check is also NaN-poisoned). A single lost-tag/garbage-homography frame commands maximum speed indefinitely. Infinite theta raises `ValueError` in `math.sin` instead.

**Fix:**

```python
if not all(math.isfinite(z) for z in (curr_x, curr_y, curr_theta, targ_x, targ_y)):
    return 0.0, 0.0, False
```

plus a staleness timeout in the dispatcher and a UDP watchdog in firmware (risk #2).

### 4. HIGH — `turn` variant's "slow enough not to overshoot" turn is 201°/s; oscillates once total feedback lag exceeds ~142 ms

`unicycle_model_turn.py:10–12, 25–35`. Turn-in-place rate = 2·0.13 / 0.074 = **3.51 rad/s = 201°/s**. Rotation per control step vs. the ±14.3° acceptance half-window:

| loop period | rotation/step | fraction of 14.3° window |
|---:|---:|---:|
| 20 ms | 4.0° | 28% |
| 50 ms | 10.1° | 70% |
| +100 ms lag | +20.1° | overshoots |
| +200 ms lag | +40.3° | limit cycle |

The full half-window is traversed in 71 ms; after one 50 ms command period only **21 ms of latency budget remains**. EMA ramp lag alone is (1−α)/α frames — α = 0.3 at 30 fps ≈ 78 ms, α = 0.2 ≈ 133 ms — before adding frame grab, processing, PC loop, UDP, and the 50 ms firmware PID. Simulation confirms convergence at τ ≤ 100 ms and sustained bang-bang oscillation at τ ≥ 200 ms. The code comment's assumption is only valid for near-zero-latency feedback.

**Fix:** make the turn proportional near the boundary (`w = clamp(1.5·heading_error, ±W_MAX)`), halve TURN_SPEED (doubles lag margin to ~284 ms), add enter/exit hysteresis (e.g., enter at 0.30 rad, exit at 0.18 rad), and measure end-to-end latency before deploying.

### 5. MEDIUM — `turn` drive-phase correction is below actuation noise → sawtooth realignment near the target

`unicycle_model_turn.py:13, 40`. At the phase boundary, w = 0.3·0.25 = 0.075 rad/s → wheel differential of **0.0028 m/s** (1.9% of cruise) — below encoder quantization and stiction asymmetry. Meanwhile the bearing-rotation term destabilizes heading inside r < v/K_w = **0.5 m**: at r = 0.20 m, e = 0.25 rad, ė = +0.11 rad/s — the error *grows* back past the gate, forcing repeated turn/drive realignment cycles. Under ideal fresh measurements it still converges (verified in sim), but it is inefficient and becomes genuinely oscillatory with latency.

**Fix:** add bearing-rate feedforward (cancels the destabilizing term exactly):

```python
w = K_w * heading_error + MOVE_SPEED * math.sin(heading_error) / max(dist_error, TOLERANCE)
```

or raise drive-mode K_w above 1.48 (with wheel-headroom clamping).

### 6. MEDIUM — no deceleration; both variants stop at full 0.15 m/s approach speed

Both files command ~0.15 m/s until the *measured* position is inside the 10 cm circle. Per-step overshoot is small (3–9 mm), but `done` is evaluated on the EMA-lagged pose, so the true stop point lands up to v·τ past it (1.5–4.5 cm at τ = 100–300 ms), plus physical coast. Acceptable margin exists only while τ_total ≲ 600 ms.

**Fix:** add a slowdown radius — `v = MOVE_SPEED * min(1.0, dist_error / 0.30)` — or a predictive stop based on measured braking distance.

### 7. LOW — dead/misleading code and unverified constants

- `MAX_SPEED` clamp is unreachable in both variants (arc outer wheel maxes at 0.1965, turn drive-phase at 0.1528) — dead safety code.
- The `abs(v) > 0.01` guard in the arc deadband is unreachable (inner wheel min is 0.1035).
- If 0.15 m/s is a real motor deadband (as the arc file implies), the `turn` file violates it: ±0.13 m/s pivot commands and inner-wheel 0.147 m/s in drive mode may stall. Verify with encoder step-response tests which value is truthful, and reconcile the two files.
- TRACK_WIDTH = 0.074 assumed exact; a 10% error scales ω and the defect-#4 latency budget by 10%. Measure the real turn rate (time a 360°).

---

## Part 2 — What is correct

- Inverse kinematics `v_left = v − wT/2`, `v_right = v + wT/2` — correct in both files.
- `turn` phase-1 sign convention — correct (positive heading error → CCW → right wheel faster).
- Angle wrapping via `atan2(sin, cos)` — correct in both files.
- Target exactly behind: `turn` handles it; `arc` picks a valid direction but then hits defect #1.

---

## Part 3 — Integration risks (require the vision/dispatcher/firmware code or hardware to confirm)

| # | Severity | Risk | Key test |
|---|---|---|---|
| R1 | HIGH | **y-flip / θ sign inversion** in homography or tag-corner math: image frame is y-down, arena math y-up. If θ is computed in pixel coordinates, θ_meas = −θ_true → steering becomes positive feedback; robot spins/diverges | Physically rotate robot CCW → reported θ must increase; drive toward +x_arena → ẏ ≈ 0 |
| R2 | HIGH | **No stale-frame/dropout watchdog** anywhere in the loop; on vision loss the last pose is reused and motion continues (compounds defect #3) | Occlude the robot tag → wheel commands must zero within ~250 ms |
| R3 | HIGH | **Firmware unit/type mismatch**: controllers emit float m/s per wheel; firmware runs a "positional PID" on encoder ticks. m-vs-mm, missing wheel-radius conversion, or position-vs-velocity setpoint confusion → saturation/stall; integral windup while arc holds one wheel ≥ 0.15 | Bench step test: command (0.15, 0.15), measure actual m/s with ruler + timer |
| R4 | MEDIUM | **Slew-rate limiter vs ±0.13 m/s instant reversal**: a 0.26 m/s/wheel step through a slow ramp adds >142 ms effective lag (destabilizes defect #4) and mid-ramp both wheels briefly co-rotate, translating while "turning in place" | Log commanded vs ramped velocities during a turn phase |
| R5 | MEDIUM | **EMA hazards**: scalar EMA over θ glitches at ±π wrap (apparent 180° flips); EMA lag consumes the defect-#4 budget; init transient feeds garbage pose for ~5 frames | Vector EMA (sin/cos) or unwrapped angle; step-move settle-time measurement |
| R6 | MEDIUM | **Off-center robot tag**: the controller drives the *tag* to the target, not the wheelbase center; during turn-in-place the reported position sweeps a circle of radius e, flapping the phase logic near the 10 cm tolerance | Rotate robot in place → reported (x, y) must stay within ~1 cm, else apply tag→center transform in arena frame |
| R7 | MEDIUM | **Lens distortion**: homography absorbs tilt but not radial distortion; 1–2% barrel distortion = 2–4 cm error at arena corners (20–40% of tolerance) | Static pose at ≥9 known grid points incl. corners → residual ≪ 5 cm, else undistort before homography |
| R8 | MEDIUM | **Calibration gate / tag-ID hygiene**: nothing shown prevents commands before 'c' locks the homography, or robot-tag vs corner-tag ID confusion | Command inhibit until calibration-valid; ID whitelist; per-frame plausibility gate (reject \|Δpose\| > v_max·Δt) |
| R9 | LOW | Boundary cases: start inside tolerance counts as instant arrival (dangerous during EMA warm-up); out-of-arena/unreachable target has no timeout (with defect #1 even *reachable* targets can fail) | Per-waypoint timeout in dispatcher; don't count arrival on frame 1 |
| R10 | LOW | README's "optical dead reckoning" is unsupported by any provided code; if the dispatcher extrapolates pose from commanded velocities during dropouts, defect #2 makes that model up to 2× wrong in curvature | Verify or remove the claim |

---

## Part 4 — Recommended pre-flight verification suite

1. Static accuracy: known grid points × 8 headings → < 2 cm, < 2°.
2. Sign test: physical CCW rotation → θ increases (R1).
3. In-place rotation → reported position fixed (R6).
4. End-to-end latency: step-move settle time → compare against the ~142 ms budget (defect #4).
5. Dropout drill: occlude tag → motion stops ≤ 250 ms (R2, defect #3).
6. NaN injection → outputs (0, 0) (defect #3).
7. Pre-'c' run → no motion (R8).
8. Firmware bench step test: commanded vs measured m/s per wheel (R3, defect #7).
9. End-to-end square path with per-waypoint arrival/overshoot logging — including waypoints **behind and beside** the robot, which is exactly where the `arc` variant currently fails (defect #1).

---

## Bottom line

- **`unicycle_model_turn.py` is the sounder baseline** — correct conventions and guaranteed convergence under fresh measurements — but its turn phase is only safe if total vision+transport latency stays under ~100–140 ms, which the described EMA + 50 ms firmware loop likely does not guarantee. Add proportional turning, hysteresis, and a NaN guard.
- **`unicycle_model_arc.py` should not be used as-is**: the deadband silently halves the steering gain and doubles every turning radius (contradicting the README), and the controller has a mathematically stable limit cycle that prevents it from ever reaching targets that start more than ~0.3 rad off its heading. It needs a turn-in-place gate and a radius-preserving deadband before any field use.
- The highest-severity *system* risks are the frame-sign check (R1), the missing vision-loss watchdog (R2/defect #3), and the unverified m/s→encoder interface (R3). All three are cheap to test before first motion and expensive to debug after.
